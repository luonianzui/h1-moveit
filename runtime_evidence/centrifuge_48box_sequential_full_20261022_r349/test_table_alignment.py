import copy
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from sequential_geometry import pose_matrix
from table_alignment import alignment_metrics, correction_target, align_supported_return


def poses(x=.43776, y=-.37981, z=.84475, angle=0.):
    quaternion = Rotation.from_euler('z', angle, degrees=True).as_quat()
    return dict(box48_right=dict(position=dict(x=x, y=y, z=z),
        orientation=dict(zip(('x', 'y', 'z', 'w'), quaternion.tolist()))))


def test_horizontal_step_preserves_height_rotation_and_source():
    source = poses()
    original = copy.deepcopy(source)
    metrics, target = correction_target(source, 'right', 'right')
    first = pose_matrix(source['box48_right'])
    assert metrics['position_error_m'] > .002
    assert np.linalg.norm(target[:2, 3] - first[:2, 3]) == pytest.approx(.001)
    assert np.array_equal(target[:3, :3], first[:3, :3])
    assert target[2, 3] == first[2, 3]
    assert source == original


@pytest.mark.parametrize('source', [poses(x=.43), poses(z=.845), poses(angle=.3)])
def test_rejects_unbounded_or_unlevel_snapshot(source):
    with pytest.raises(ValueError):
        alignment_metrics(source, 'right', 'right')


def make_cycle(positive=True):
    holder = dict(right=dict(payload='right', source_sim=2., native_reference='original', fk_reference='original'))
    cycle = SimpleNamespace(holders=SimpleNamespace(snapshot=lambda: copy.deepcopy(holder)),
        order=SimpleNamespace(phase='retrieval', locations=dict(right='held')),
        return_hands=dict(right='right'), pair_clearance_payloads=set(), poses=poses(),
        pose_time=3., latest=dict(right_contacts=time.monotonic()),
        contacts=dict(right=[dict(first='box48_right::body', second='table::link::surface', force=1. if positive else 0.)]),
        safety=lambda: None, record=lambda *args: None, dwell=lambda seconds: None)
    cycle.single_plan = lambda mode, hand, payload: dict(mode=mode)
    return cycle, holder


def test_correction_uses_fresh_observation_and_preserves_anchor():
    cycle, holder = make_cycle()
    calls = []
    def execute(planned, prefix):
        calls.append(planned)
        cycle.poses = poses(x=.43876, y=-.3799)
    cycle.execute_stages = execute
    align_supported_return(cycle, 'right', 'right')
    assert len(calls) == 1
    assert cycle.holders.snapshot() == holder


def test_absent_positive_support_cannot_trigger_motion():
    cycle, unused = make_cycle(False)
    cycle.execute_stages = lambda *args: pytest.fail('No unsupported correction allowed')
    with pytest.raises(RuntimeError, match='positive table support'):
        align_supported_return(cycle, 'right', 'right')


def test_nonconvergence_stops_after_two_commands():
    cycle, unused = make_cycle()
    calls = []
    cycle.execute_stages = lambda *args: calls.append(args)
    with pytest.raises(RuntimeError, match='converge'):
        align_supported_return(cycle, 'right', 'right')
    assert len(calls) == 2


def test_success_already_in_original_gate_needs_no_motion():
    cycle, unused = make_cycle()
    cycle.poses = poses(x=.439)
    cycle.execute_stages = lambda *args: pytest.fail('Unneeded motion')
    align_supported_return(cycle, 'right', 'right')


def test_profile_service_only_adds_new_observer_not_control():
    from install_profile_plugin import request_command
    command = request_command(Path('/unique/fresh/run-root'))
    assert command[command.index('-s') + 1] == '/world/h1_world/entity/system/add'
    assert 'gz.msgs.EntityPlugin_V' in command
    assert 'TaskCpuProfile' in command[-1]
    assert 'process_identities.json' in command[-1] and 'result.json' in command[-1]
    assert 'world/control' not in ' '.join(command)


def test_runtime_hook_only_after_completed_setdown(monkeypatch):
    import table_alignment_cycle as runtime
    calls = []
    monkeypatch.setattr(runtime.ParentSpeedPriorityCycle, 'execute_stages', lambda *args: None)
    monkeypatch.setattr(runtime.ParentSpeedPriorityCycle, 'dwell', lambda *args: None)
    monkeypatch.setattr(runtime, 'align_supported_return', lambda *args: calls.append(args[1:]))
    cycle = runtime.SpeedPriorityCycle.__new__(runtime.SpeedPriorityCycle)
    cycle.phase = 'paired_retrieval_setdown_fine'
    cycle.execute_stages(dict(mode='retrieval_transport', stages=[dict(phase='retrieve_setdown')],
                             single_hand='right', single_payload='right'), 'seq_retrieval')
    cycle.dwell(.05)
    assert not calls
    cycle.dwell(.5)
    cycle.dwell(.5)
    assert calls == [('right', 'right')]
