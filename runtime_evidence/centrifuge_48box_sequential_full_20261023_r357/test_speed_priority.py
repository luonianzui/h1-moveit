import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from dual_motion import TASK_JOINTS, common_arm_timing
from sequential_order import SequentialOrder
from speed_order import SpeedPriorityOrder
from speed_policy import PROFILE, common_timing, configure_simulation_profile, dwell_duration, motion_limits
from speed_source import restore_speed_source


def samples():
    initial = dict.fromkeys(sorted(TASK_JOINTS), 0.)
    path = [dict(initial, left_elbow_joint=.002 * index) for index in range(100)]
    return initial, path


def test_retiming_preserves_all_points_and_controller_groups():
    initial, path = samples()
    saved = copy.deepcopy(path)
    original = common_arm_timing(initial, path, .35)
    accelerated = common_timing(initial, path, 1.2)
    assert path == saved
    assert accelerated['points'] == original['points']
    assert accelerated['groups'] == original['groups']
    assert accelerated['duration_sim_s'] < original['duration_sim_s'] / 3
    for before, after, first, last in zip(accelerated['times'], accelerated['times'][1:],
                                          accelerated['points'], accelerated['points'][1:]):
        assert after - before >= .002 - 1e-12
        assert max(abs(last[name] - first[name]) for name in first) / (after - before) <= 1.2 + 1e-10


@pytest.mark.parametrize('rate', [0., -1., float('nan'), float('inf'), 1.21])
def test_invalid_rates_rejected(rate):
    initial, path = samples()
    with pytest.raises(ValueError):
        common_timing(initial, path, rate)


def test_invalid_geometry_and_stationary_path():
    initial, path = samples()
    path[0]['left_elbow_joint'] = float('nan')
    with pytest.raises(ValueError):
        common_timing(initial, path, .4)
    result = common_timing(initial, [initial, initial], .4)
    assert all(math.isfinite(value) for value in result['times'])
    assert result['times'] == sorted(set(result['times']))


@pytest.mark.parametrize('phase,loaded,rate', [
    ('seq_load_empty_transfer', False, 1.2), ('seq_load_transfer', True, .8),
    ('seq_load_lower', True, .4), ('seq_load_pilot', True, .15),
    ('seq_retrieval_retrieve_axis_exit', True, .15),
    ('paired_retrieval_setdown', True, .4), ('paired_retrieval_setdown_fine', True, .15),
])
def test_explicit_phase_caps(phase, loaded, rate):
    assert motion_limits(phase, loaded)[0] == rate


def test_only_static_grip_dwell_shortened_spin_not_shortened():
    assert dwell_duration('left_seq_load_static_grip_hold', 10) == 1.
    assert dwell_duration('seq_retrieval_static_grip_hold', 10) == 1.
    assert dwell_duration('machine_spin', 30) == 30.
    assert dwell_duration('machine_spin', 10) == 10.
    assert dwell_duration('leg_initialization', 5) == 5.


def test_relaxed_grip_configuration_is_explicit_and_legacy_default_unchanged(monkeypatch):
    import grip_limits
    assert grip_limits.CARRY_SLIP_LIMIT_M == .002 and grip_limits.CARRY_ANGLE_LIMIT_DEG == 5.
    monkeypatch.setattr(grip_limits, 'CARRY_SLIP_LIMIT_M', .002)
    monkeypatch.setattr(grip_limits, 'CARRY_ANGLE_LIMIT_DEG', 5.)
    configure_simulation_profile()
    assert not grip_limits.grip_exceeds_limits(.004, 9.)
    assert grip_limits.grip_exceeds_limits(.006, 0)
    assert grip_limits.grip_exceeds_limits(.001, 10)
    assert grip_limits.grip_exceeds_limits(float('nan'), 0)


def test_order_requires_both_boxes_and_actual_release():
    order = SpeedPriorityOrder()
    for hand in ('left', 'right'):
        order.begin(hand, hand, {hand: dict(payload=hand)})
        with pytest.raises(ValueError):
            order.finish_loading(hand, hand, {}, .49)
        with pytest.raises(ValueError):
            order.finish_loading(hand, hand, {hand: dict(payload=hand)}, 1)
        order.finish_loading(hand, hand, {}, .5)
    assert order.phase == 'machine'
    machine = dict(rotor_rpm=0, rotor_rad_s=0, lid_rad=math.radians(120),
                   rotor_enabled=False, can_open=True, starts=1, stops=1)
    with pytest.raises(ValueError):
        order.finish_machine({}, 29.99, machine, dict(left='right', right='left'))
    order.finish_machine({}, 30., machine, dict(left='right', right='left'))
    for hand, payload in [('left', 'right'), ('right', 'left')]:
        order.begin(hand, payload, {hand: dict(payload=payload)})
        order.finish_return(hand, payload, {}, .5)
    with pytest.raises(ValueError):
        order.finish_terminal({}, .99)
    order.finish_terminal({}, 1.)
    assert order.snapshot()['locations'] == dict(left='table', right='table')
    assert order.phase == 'complete'
    original = SequentialOrder()
    original.begin('left', 'left', {'left': dict(payload='left')})
    with pytest.raises(ValueError):
        original.finish_loading('left', 'left', {}, .5)


@pytest.mark.parametrize('name', ['execute_loading.py', 'execute_sequential.py'])
def test_only_declared_legacy_seams_changed(name):
    root = Path(__file__).resolve().parent
    parent = root.parent / 'centrifuge_48box_sequential_full_20261020_r320'
    original = (parent / name).read_text().replace('centrifuge_48box_sequential_full_r320', 'centrifuge_48box_sequential_full_r349')
    assert restore_speed_source(name, (root / name).read_text()) == original
    changed = (root / name).read_text().replace('import ', 'import unrelated_', 1)
    assert restore_speed_source(name, changed) != original


def test_runtime_subclass_dispatch_and_limits():
    from sequential_speed_cycle import SpeedPriorityCycle
    cycle = object.__new__(SpeedPriorityCycle)
    cycle.holders = SimpleNamespace(rows={})
    assert cycle.motion_limits('empty', .35) == (1.2, .08)
    cycle.joint_time = 10.
    cycle.positions = dict(joint=.0019)
    assert cycle.paired_endpoint_stamp(dict(joint=0.), 10.) == 10.
    assert cycle.paired_endpoint_stamp(dict(joint=0.), 11.) is None
    cycle.positions['joint'] = .0021
    assert cycle.paired_endpoint_stamp(dict(joint=0.), 10.) is None
    assert cycle.pose_clock_skew_limit_sim_s == .75


def test_worker_requires_explicit_profile_and_sealed_source(tmp_path):
    from speed_worker import SOURCE_NAMES, require_profile
    names = SOURCE_NAMES
    hashes = {}
    for name in names:
        (tmp_path / name).write_text(name)
        hashes[name] = hashlib.sha256(name.encode()).hexdigest()
    identity = dict(actual_runtime_subclass='SpeedPriorityCycle', speed_profile=PROFILE, sha256=hashes)
    (tmp_path / 'identity.json').write_text(json.dumps(identity))
    require_profile(tmp_path)
    (tmp_path / 'grip_limits.py').write_text('changed')
    with pytest.raises(ValueError, match='source changed'):
        require_profile(tmp_path)
    identity['speed_profile'] = {}
    (tmp_path / 'identity.json').write_text(json.dumps(identity))
    with pytest.raises(ValueError, match='identity required'):
        require_profile(tmp_path)
