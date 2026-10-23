import copy
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

from joined_stages import joined_phase, joined_samples, joined_timing, stage_groups
from persistent_worker import request_paths, serve
from pipeline_source import restore_pipeline_source
from planner_client import PlannerClient
from planning_pipeline import PlanningPipelineMixin
from speed_policy import PROFILE, common_timing


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r333'


def actual_groups():
    for path in sorted((PARENT / 'run').glob('*transport*_result.json')):
        planned = json.loads(path.read_text())
        for group in stage_groups(planned['stages']):
            if len(group) == 2:
                yield group


@pytest.mark.parametrize('group', list(actual_groups()))
def test_join_keeps_every_dense_point_and_segment_ramp(group):
    initial = group[0]['samples'][0]
    before = copy.deepcopy(group)
    timing = joined_timing(initial, group, .8)
    first = common_timing(initial, group[0]['samples'], .8)
    second = common_timing(group[0]['samples'][-1], group[1]['samples'], .8)
    assert timing['points'] == first['points'] + second['points'][1:]
    assert timing['points'][1:] == joined_samples(group)
    assert timing['times'][:len(first['times'])] == first['times']
    assert timing['times'][len(first['times']):] == [first['duration_sim_s'] + stamp - second['times'][0] for stamp in second['times'][1:]]
    assert all(right > left for left, right in zip(timing['times'], timing['times'][1:]))
    assert group == before
    assert timing['duration_sim_s'] == pytest.approx(first['duration_sim_s'] + second['duration_sim_s'] - .04)
    from audit_speed_paths import verify_axis_exit_timing, verify_published
    command = dict(phase='seq_load_' + joined_phase(group), payload=dict(
        samples=joined_samples(group), joined_segments=group, holders={'left': {}},
        initial=initial, times=timing['times'], groups=timing['groups'], actual_rate_rad_s=.8))
    planned = dict(mode='load_transport', stages=group)
    verify_published([command], planned)
    verify_axis_exit_timing(command)
    forged = copy.deepcopy(command)
    forged['payload']['joined_segments'][0]['samples'][0]['torso_joint'] += .001
    with pytest.raises(ValueError):
        verify_published([forged], planned)


@pytest.mark.parametrize('phase', ['retrieve_lift', 'retrieve_axis_exit', 'load_lower', 'retrieve_setdown', 'single_pilot'])
def test_observation_contact_and_low_speed_stages_never_join(phase):
    group = next(actual_groups())
    group[0]['phase'] = phase
    assert len(stage_groups(group)) == 2
    with pytest.raises(ValueError):
        joined_samples(group)


def test_discontinuous_join_rejected():
    group = next(actual_groups())
    name = next(iter(group[1]['samples'][0]))
    group[1]['samples'][0][name] += .00001
    with pytest.raises(ValueError, match='waypoint'):
        stage_groups(group)


def test_join_dispatch_has_one_holder_and_preserves_single_stage_hooks():
    calls = []
    class Parent:
        def execute_stages(self, planned, prefix):
            calls.append(('original', planned['stages'][0]['phase']))
    class Candidate(PlanningPipelineMixin, Parent):
        def record(self, *args):
            pass
        def move(self, samples, phase, rate):
            calls.append(('joined', phase))
            assert self.joined_stage_segments
    cycle = Candidate()
    cycle.holders = SimpleNamespace(rows={'left': {}})
    group = next(actual_groups())
    cycle.execute_stages({'stages': [dict(phase='retrieve_lift', samples=[]), *group]}, 'seq_load')
    assert calls == [('original', 'retrieve_lift'), ('joined', 'seq_load_' + joined_phase(group))]
    assert cycle.joined_stage_segments is None
    assert cycle.phase == 'seq_load_' + group[-1]['phase']
    cycle.holders.rows = {}
    with pytest.raises(RuntimeError, match='one actual holder'):
        cycle.execute_stages({'stages': group}, 'seq_load')


def test_original_profile_and_only_declared_runtime_source_changes():
    original = (PARENT / 'sequential_speed_cycle.py').read_text().replace(PARENT.name, ROOT.name)
    actual = (ROOT / 'sequential_speed_cycle.py').read_text()
    assert restore_pipeline_source('sequential_speed_cycle.py', actual) == original
    assert restore_pipeline_source('sequential_speed_cycle.py', actual.replace('snapshot_joint_rad', 'wrong_limit')) != original
    assert PROFILE == json.loads((PARENT / 'identity.json').read_text())['speed_profile']
    assert PROFILE['steady_spin_sim_s'] == 30


def test_request_identity_duplicate_paths_and_sequence(tmp_path):
    run = tmp_path / 'run'
    run.mkdir()
    source, output = run / '1_park_left_request.json', run / '1_park_left_result.json'
    source.write_text('{}')
    message = dict(token='unique', sequence=1, source=str(source), destination=str(output))
    assert request_paths(tmp_path, message, 'unique', 1)[:2] == (source, output)
    for changed in (dict(message, token='old'), dict(message, sequence=2), dict(message, destination=str(tmp_path / output.name))):
        with pytest.raises(ValueError):
            request_paths(tmp_path, changed, 'unique', 1)
    output.write_text('{}')
    with pytest.raises(FileExistsError):
        request_paths(tmp_path, message, 'unique', 1)


def test_real_process_reuses_pid_and_validates_fresh_results(tmp_path):
    (tmp_path / 'run').mkdir()
    worker = tmp_path / 'persistent_worker.py'
    worker.write_text('import sys, json\nfrom pathlib import Path\n'
        + 'sys.path.insert(0, ' + repr(str(ROOT)) + ')\n'
        + 'from persistent_worker import serve\n'
        + 'def handle(source, destination):\n'
        + '    with destination.open("x") as stream: json.dump(json.loads(source.read_text()), stream)\n'
        + 'serve(Path(__file__).parent, sys.argv[1], handle, sys.stdin)\n')
    client = PlannerClient(tmp_path)
    try:
        for sequence in (1, 2):
            source = tmp_path / 'run' / f'{sequence}_park_left_request.json'
            output = source.with_name(source.name.replace('_request', '_result'))
            source.write_text(json.dumps(dict(sequence=sequence)))
            receipt = client.execute(source, output, lambda: time.sleep(.002))
            assert receipt['pid'] == client.process.pid
            assert receipt['sequence'] == sequence
            assert json.loads(output.read_text()) == {'sequence': sequence}
            with pytest.raises(FileExistsError):
                client.execute(source, output, lambda: None)
    finally:
        client.close()
    assert client.process.returncode == 0
    assert len(client.receipts) == 2


def test_planner_cleanup_remains_after_original_executor_hold(monkeypatch):
    calls = []
    class Parent:
        def run_complete(self):
            raise ValueError('original guard failed')
        def close(self):
            calls.append('original_close')
    class Candidate(PlanningPipelineMixin, Parent):
        pass
    monkeypatch.setattr('planning_pipeline.PlannerClient', lambda root: SimpleNamespace(
        close=lambda: calls.append('planner_close')))
    cycle = Candidate()
    with pytest.raises(ValueError, match='original guard failed'):
        cycle.run_complete()
    assert calls == []
    calls.append('original_executor_stop_and_hold')
    cycle.close()
    assert calls == ['original_executor_stop_and_hold', 'planner_close', 'original_close']
