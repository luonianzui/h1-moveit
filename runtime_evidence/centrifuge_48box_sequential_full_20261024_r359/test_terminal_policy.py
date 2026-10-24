import inspect
import math

import pytest

from execute_cycle import FullCycle
from terminal_policy import skip_final_parking, table_terminal_metrics, table_support_valid


def pose(position=None, yaw=math.pi):
    position = position or [.44, .38, .84475]
    return dict(position=dict(zip('xyz', position)), orientation=dict(x=0, y=0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))


def pair(**changes):
    value = dict(first='box48_right::body::collision', second='table::surface::collision', force=.33, depth=.00001)
    value.update(changes)
    return value


def metrics(poses=None, pairs=None, reference=None):
    return table_terminal_metrics(poses or {'box48_right': pose()}, 'right', pairs if pairs is not None else [pair()],
                                  [.44, .38, .84475], math.pi, reference or [.44, .38, .84475])


@pytest.mark.parametrize('phase', ['approach', 'descend', 'pilot_lift', 'lift', 'transfer', 'setdown'])
def test_required_material_motion_is_never_skipped(phase):
    assert not skip_final_parking(True, phase)
    assert not skip_final_parking(False, phase)


@pytest.mark.parametrize('phase', ['retreat', 'return'])
def test_only_last_payload_optional_parking_is_skipped(phase):
    assert skip_final_parking(True, phase)
    assert not skip_final_parking(False, phase)


def test_verified_table_pose_and_positive_support_pass():
    result = metrics()
    assert result['geometry_valid'] and result['contacts_valid'] and result['supported']
    assert result['no_finger_contact']


@pytest.mark.parametrize('changes', [dict(force=math.nan), dict(depth=.00101),
    dict(second='h1_2::left_dex1_finger_1::collision'), dict(second='centrifuge::bucket_right::collision')])
def test_nonfinite_excess_depth_finger_or_other_support_fails(changes):
    assert not metrics(pairs=[pair(**changes)])['contacts_valid']


def test_empty_contacts_and_position_drift_are_rejected():
    assert not metrics(pairs=[])['contacts_valid']
    assert not metrics({'box48_right': pose([.440501, .38, .84475])})['geometry_valid']
    assert not metrics({'box48_right': pose(yaw=math.pi + math.radians(2.01))})['geometry_valid']


def test_original_twenty_ms_support_window_is_not_relaxed():
    result = metrics(pairs=[pair(force=0)])
    assert not result['supported']
    assert table_support_valid(result, 1.01, 1)['valid']
    assert not table_support_valid(result, 1.021, 1)['valid']
    assert not table_support_valid(result, .9, 1)['valid']
    assert not table_support_valid(result, 1, None)['valid']


def test_terminal_hold_keeps_full_health_gate_and_is_not_early_exit():
    source = inspect.getsource(FullCycle.terminal_hold)
    assert 'self.spin()' in source and 'table_support_valid' in source
    assert 'while self.sim - start < duration_sim_s:' in source and '.025' in source
    assert 'neutral_arms_not_claimed=True' in source
    source = inspect.getsource(FullCycle.retrieve)
    assert 'self.terminal_hold(10)' in source
    assert 'skip_final_parking(payload == order[-1], phase)' in source

