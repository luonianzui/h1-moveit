import copy
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest
from rosidl_runtime_py.convert import message_to_ordereddict

from alignment_scene import make_alignment_scene
import planner_resources
from planner_resources import ResourceCache, StaticResources, make_resource_scene, signature
from prefetch_cycle import PlanningPrefetchMixin, drift_within_bounds, next_stationary_plan, snapshot_drift
from planner_client import PlannerClient
from speed_policy import PROFILE

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r347'


def fixture():
    return json.loads((PARENT / 'run/4_load_transport_left_request.json').read_text())


def test_shared_model_but_fresh_state_scene_and_acm():
    request = fixture()
    original = make_alignment_scene(request['positions'], request['poses'], ROOT / 'inputs/robot_dual.urdf')
    first = make_resource_scene(request['positions'], request['poses'], ROOT / 'inputs/robot_dual.urdf')
    assert message_to_ordereddict(original.scene.planning_scene_message) == message_to_ordereddict(first.scene.planning_scene_message)
    first.scene.allowed_collision_matrix.set_entry('box48_left', 'table_link_surface', True)
    first.state.joint_positions = dict(first.initial, torso_joint=.2)
    first.state.update()
    second = make_resource_scene(request['positions'], request['poses'], ROOT / 'inputs/robot_dual.urdf')
    assert first.model is second.model
    assert first.state is not second.state and first.scene is not second.scene
    assert message_to_ordereddict(original.scene.planning_scene_message) == message_to_ordereddict(second.scene.planning_scene_message)
    assert first.check(request['positions']) == second.check(request['positions'])


def test_cached_file_mutation_fails_closed(tmp_path):
    path = tmp_path / 'model'
    path.write_bytes(b'first')
    resources = StaticResources.__new__(StaticResources)
    resources.dependencies = {str(path): dict(signature=signature(path))}
    resources.verify()
    path.write_bytes(b'other')
    with pytest.raises(RuntimeError, match='changed'):
        resources.verify()


def test_mixed_identity_not_silently_cached(tmp_path):
    first, other = tmp_path / 'first', tmp_path / 'other'
    first.write_bytes(b'first')
    other.write_bytes(b'other')
    cache = ResourceCache()
    cache.resources = SimpleNamespace(robot_path=first)
    with pytest.raises(ValueError, match='mix'):
        cache.get(other)


@pytest.mark.parametrize('hand', ['left', 'right'])
def test_only_existing_stationary_holds_prefetch(hand):
    held = {hand: {'payload': hand}}
    assert next_stationary_plan(hand + '_seq_load_static_grip_hold', 10, held) == ('load_transport', hand, hand)
    assert next_stationary_plan('seq_retrieval_single_pilot', 1., held) == ('retrieval_transport', hand, hand)
    for phase in ('machine_running_steady', 'paired_retrieval_setdown_fine', 'seq_load_empty_park', 'unknown'):
        assert next_stationary_plan(phase, 1., held) is None
    assert next_stationary_plan('seq_retrieval_single_pilot', .5, held) is None
    assert next_stationary_plan('seq_retrieval_single_pilot', 1., {}) is None
    assert next_stationary_plan('seq_retrieval_single_pilot', 1., {'left': {}, 'right': {}}) is None


@pytest.mark.parametrize('field,limit', [
    ('joint_drift_rad', 'snapshot_joint_rad'), ('pose_drift_m', 'snapshot_pose_m'),
    ('angle_drift_deg', 'snapshot_angle_deg'), ('finger_drift_m', 'snapshot_finger_m')])
def test_prefetch_rechecks_original_limits_and_nonfinite(field, limit):
    request = fixture()
    values = snapshot_drift(request, request['positions'], request['poses'])
    assert drift_within_bounds(values)
    values[field] = PROFILE[limit] + 1e-10
    assert not drift_within_bounds(values)
    for invalid in (float('nan'), float('inf')):
        values[field] = invalid
        assert not drift_within_bounds(values)


def test_async_ticket_overlaps_observation_and_cannot_be_reused(tmp_path):
    (tmp_path / 'run').mkdir()
    (tmp_path / 'persistent_worker.py').write_text(
        'import sys,json,time\nfrom pathlib import Path\nsys.path.insert(0,' + repr(str(ROOT)) + ')\n'
        'from persistent_worker import serve\n'
        'def handle(source,destination):\n'
        '    time.sleep(.15)\n'
        '    with destination.open("x") as stream: json.dump(json.loads(source.read_text()),stream)\n'
        'serve(Path(__file__).parent,sys.argv[1],handle,sys.stdin)\n')
    source = tmp_path / 'run/1_load_transport_left_request.json'
    output = source.with_name(source.name.replace('_request', '_result'))
    source.write_text('{}')
    client = PlannerClient(tmp_path)
    try:
        began = time.monotonic()
        ticket = client.submit(source, output)
        assert time.monotonic() - began < .1 and not output.exists()
        with pytest.raises(RuntimeError, match='unconsumed'):
            client.submit(source, output)
        time.sleep(.25)
        receipt = client.finish(ticket, lambda: time.sleep(.001))
        assert receipt['source_sha256'] == hashlib.sha256(b'{}').hexdigest()
        with pytest.raises(RuntimeError, match='consumed'):
            client.finish(ticket, lambda: None)
    finally:
        client.close()


def test_observation_duration_not_shortened_and_begin_precedes_it():
    calls = []
    class Parent:
        def dwell(self, seconds):
            calls.append(('original_hold', seconds))
    class Cycle(PlanningPrefetchMixin, Parent):
        def begin_stationary_plan(self, *args):
            calls.append(('submit', args))
    cycle = Cycle()
    cycle.phase = 'left_seq_load_static_grip_hold'
    cycle.holders = SimpleNamespace(snapshot=lambda: {'left': {'payload': 'left'}})
    cycle.planning_worker = object()
    cycle.dwell(10)
    assert calls == [('submit', ('load_transport', 'left', 'left')), ('original_hold', 10)]
    cycle.prefetch_pending = {}
    with pytest.raises(RuntimeError, match='unconsumed'):
        cycle.dwell(10)


def test_wrong_hand_pending_plan_rejected_before_finish():
    cycle = PlanningPrefetchMixin()
    cycle.prefetch_pending = {'key': ('load_transport', 'left', 'left')}
    with pytest.raises(RuntimeError, match='different'):
        cycle.single_plan('load_transport', 'right', 'right')
