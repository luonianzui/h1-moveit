import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import numpy as np

from gripper_return_geometry import FINGERS, MODES, plan_post_retreat_gripper, verify_request, verify_return_margins, verify_finger_snapshot
from contact_phase_policy import allow_planning_grasp_touch
from inherited_source_identity import restore_inherited_source
from pickup_approach_cycle import PickupApproachCycle
from post_retreat_gripper_cycle import PostRetreatGripperCycle, RETREAT_PHASES
from test_retrieval_grip import candidate


def test_actual_two_ros_actions_only_after_three_original_retreats_and_FCL_before_fresh_replan():
    cycle, goals = candidate()
    cycle.__class__ = PostRetreatGripperCycle
    calls = []
    stages = [dict(phase=phase, samples=[dict(step=index)])
              for index, phase in enumerate(RETREAT_PHASES + ('empty_return',))]

    def plan(mode, assignment):
        calls.append(('plan', mode))
        if mode == 'empty_return':
            assert not goals
            return dict(stages=stages)
        assert tuple(cycle.completed_empty_retreat_stages) == RETREAT_PHASES
        if mode == MODES[0]:
            assert not goals
            return dict(status='PASS')
        assert set(goals) == {'left', 'right'}
        assert cycle.positions == dict.fromkeys(cycle.positions, -.019)
        assert cycle.accepted_gripper_efforts == dict(left=0., right=0.)
        return dict(stages=[dict(phase='empty_return', samples=[dict(fresh=True)])])

    cycle.actual_plan = plan
    cycle.move = lambda samples, phase, rate: calls.append(('move', phase, rate))
    cycle.return_neutral()
    assert calls == [('plan', 'empty_return')] + [
        ('move', 'machine_paired_' + phase, .13 if index == 0 else .8)
        for index, phase in enumerate(RETREAT_PHASES)] + [
        ('plan', MODES[0]), ('plan', MODES[1]), ('move', 'machine_paired_empty_return', .8)]
    assert cycle.motion_count == 2
    for goal in goals.values():
        assert list(goal.trajectory.points[-1].positions) == [-.019, -.019]
        assert list(goal.trajectory.points[-1].effort) == [0., 0.]
        assert all(tolerance.position == .0002 for tolerance in goal.goal_tolerance)
    assert len([row for row in cycle.events if row[0] == 'original_gripper_endpoint_fresh_feedback_pass']) == 2


@pytest.mark.parametrize('failure', ['first_retreat', 'stroke_FCL', 'endpoint'])
def test_runtime_stage_FCL_or_native_endpoint_failure_cannot_dispatch_final_return(failure):
    cycle, goals = candidate(endpoint_error=.00026 if failure == 'endpoint' else 0.)
    cycle.__class__ = PostRetreatGripperCycle
    cycle.spin = lambda: setattr(cycle, 'sim', cycle.sim + .5)
    calls = []

    def plan(mode, assignment):
        calls.append(mode)
        if mode == 'empty_return':
            return dict(stages=[dict(phase=phase, samples=[]) for phase in RETREAT_PHASES + ('empty_return',)])
        if failure == 'stroke_FCL':
            raise RuntimeError('Original full-state collision')
        return dict(status='PASS')

    def move(samples, phase, rate):
        if failure == 'first_retreat':
            raise RuntimeError('Original endpoint fault')

    cycle.actual_plan, cycle.move = plan, move
    with pytest.raises(RuntimeError):
        cycle.return_neutral()
    assert MODES[1] not in calls
    if failure != 'endpoint':
        assert not goals


def request(mode=MODES[0]):
    return dict(mode=mode, current_run_guide_identity='fresh', original_runtime_holder_references={},
                completed_empty_retreat_stages=list(RETREAT_PHASES), accepted_gripper_efforts=dict(left=0., right=0.),
                source_sim=15., positions=dict(torso_joint=0., **dict.fromkeys(FINGERS, -.0199)))


@pytest.mark.parametrize('field,value', [('current_run_guide_identity', 'old'),
                                       ('original_runtime_holder_references', dict(left=dict(payload='left'))),
                                       ('completed_empty_retreat_stages', list(RETREAT_PHASES[:2])),
                                       ('accepted_gripper_efforts', dict(left=16., right=0.)),
                                       ('source_sim', float('nan')), ('mode', 'other')])
def test_worker_rejects_wrong_identity_incomplete_retreat_loaded_or_nonfinite_state(field, value):
    data = request()
    data[field] = value
    with pytest.raises(ValueError):
        verify_request(data, 'fresh')


class FakePlanner:
    def __init__(self, data, references, collision=False):
        self.names = ['torso_joint']
        self.lower, self.upper = np.array([-1.]), np.array([1.])
        self.checked = 0
        self.data = data
        self.calls = []
        self.planner = SimpleNamespace(check=lambda values: self.check(values, collision))

    def check(self, values, collision):
        self.calls.append(dict(values))
        return dict(collision=collision)

    def empty_connector(self, first, last):
        assert self.data['positions'] == dict(torso_joint=0., **dict.fromkeys(FINGERS, -.019))
        self.checked = 2
        return [dict(first), dict(last)]


def test_actual_full_four_finger_stroke_is_collision_checked_without_mutating_request():
    data = request()
    original = copy.deepcopy(data)
    planner = FakePlanner(data, {})
    result = plan_post_retreat_gripper(lambda *arguments: planner, data, dict(torso_joint=.1), 'fresh')
    assert data == original and result['checked_samples'] == len(planner.calls) == 31
    assert all(set(values) == set(data['positions']) for values in planner.calls)
    assert planner.calls[0] == data['positions']
    assert planner.calls[-1] == dict(torso_joint=0., **dict.fromkeys(FINGERS, -.019))


def test_collision_in_original_full_model_rejects_before_any_command():
    data = request()
    with pytest.raises(RuntimeError, match='full-state FCL collision'):
        plan_post_retreat_gripper(lambda *arguments: FakePlanner(data, {}, True), data, {}, 'fresh')


def test_final_connector_requires_actual_endpoint_and_original_neutral_seed():
    data = request(MODES[1])
    with pytest.raises(ValueError, match='fresh actual endpoint'):
        verify_request(data, 'fresh')
    data['positions'].update(dict.fromkeys(FINGERS, -.019))
    planner = FakePlanner(data, {})
    result = plan_post_retreat_gripper(lambda *arguments: planner, data, dict(torso_joint=.1), 'fresh')
    assert result['stages'] == [dict(phase='empty_return', samples=[dict(torso_joint=0.), dict(torso_joint=.1)])]


def test_original_lifecycle_physics_and_modified_source_seams_are_exact():
    for name in ('run_complete', 'retrieve', 'paired_grippers', 'gripper_trajectory_points',
                 'capture_pair', 'latest_native_guard', 'safety', 'motion_limits', 'hold'):
        assert getattr(PostRetreatGripperCycle, name) is getattr(PickupApproachCycle, name)
    root = Path(__file__).resolve().parent
    records = {row['relative']: row for row in json.loads((root / 'derivation.json').read_text())['records']}
    for name in ('execute_cycle.py', 'execute_dual.py', 'dual_worker.py', 'contact_phase_policy.py', 'retrieval_grip_cycle.py'):
        source = restore_inherited_source(name, (root / name).read_text())
        assert hashlib.sha256(source.encode()).hexdigest() == records[name]['derived_sha256']


@pytest.mark.parametrize('mode', MODES)
def test_new_known_empty_roles_do_not_exempt_loose_payload_finger_contacts(mode):
    assert allow_planning_grasp_touch(mode) is False


@pytest.mark.parametrize('sample', [{}, dict(torso_joint=float('nan')), dict(torso_joint=1.01),
                                   dict(torso_joint=-1.01)])
def test_original_complete_finite_joint_margins_cannot_be_omitted(sample):
    planner = FakePlanner(request(), {})
    with pytest.raises((ValueError, RuntimeError)):
        verify_return_margins(planner, [sample])


@pytest.mark.parametrize('drift', [0., .000299, .000301, float('nan')])
@pytest.mark.parametrize('finger', FINGERS)
def test_new_mode_four_finger_snapshot_cannot_silently_drift_during_planning(drift, finger):
    initial = dict.fromkeys(FINGERS, -.019)
    current = dict(initial)
    current[finger] += drift
    if math.isfinite(drift) and drift < .0003:
        assert verify_finger_snapshot(current, initial) < .0003
    else:
        with pytest.raises(RuntimeError):
            verify_finger_snapshot(current, initial)
