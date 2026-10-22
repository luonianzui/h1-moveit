import hashlib
import json
from pathlib import Path

import pytest

from startup_ramp_gate import verify_startup_status
from startup_ramp_model import restore_startup_model, verify_model_fork, PLUGIN_SHA256


ROOT = Path(__file__).resolve().parent


def valid_status():
    return dict(ready=True, rotor_enabled=False, can_start=True, startup_ramp_rad_s2=.8,
                stopped_velocity_gain=80, stopped_position_gain=8000, starts=0, stops=0,
                lid_rad=0., rotor_rpm=0.)


def test_exact_current_stopped_initial_status_accepts_tested_plugin():
    assert verify_startup_status(valid_status())['status'] == 'PASS'


@pytest.mark.parametrize('name,value', [('startup_ramp_rad_s2', .9), ('startup_ramp_rad_s2', True),
                                      ('stopped_velocity_gain', 70), ('stopped_position_gain', 0),
                                      ('rotor_enabled', True), ('ready', False), ('starts', 1), ('stops', 1),
                                      ('can_start', False), ('lid_rad', .01), ('rotor_rpm', .1)])
def test_wrong_live_field_or_nonfresh_state_rejected(name, value):
    state = valid_status()
    state[name] = value
    with pytest.raises(ValueError):
        verify_startup_status(state)


def test_missing_old_plugin_field_cannot_satisfy_current_gate():
    state = valid_status()
    state.pop('startup_ramp_rad_s2')
    with pytest.raises(ValueError):
        verify_startup_status(state)


def original_model():
    records = json.loads((ROOT / 'derivation.json').read_text())['records']
    record = next(row for row in records if row['relative'] == 'inputs/centrifuge_brake.sdf')
    current = (ROOT / 'inputs/centrifuge_brake.sdf').read_bytes()
    restored = restore_startup_model(current)
    assert hashlib.sha256(current).hexdigest() == record['derived_sha256']
    return restored, current


def test_only_plugin_binding_changes_not_model_geometry_or_lid_limits():
    original, current = original_model()
    assert verify_model_fork(original, current)['status'] == 'PASS'


@pytest.mark.parametrize('before,after', [(b'<velocity>0.8</velocity>', b'<velocity>0.9</velocity>'),
                                        (b'<effort>30</effort>', b'<effort>31</effort>'),
                                        (b'<damping>.003</damping>', b'<damping>.004</damping>')])
def test_any_undeclared_joint_or_basket_change_is_rejected(before, after):
    original, current = original_model()
    assert before in current
    with pytest.raises(ValueError):
        verify_model_fork(original, current.replace(before, after, 1))


def test_private_compiled_plugin_matches_frozen_prototype():
    assert hashlib.sha256((ROOT / 'inputs/libCentrifugeInterlockStartupRampProbeR1.so').read_bytes()).hexdigest() == PLUGIN_SHA256
    proof = json.loads((ROOT / 'derivation.json').read_text())
    assert proof['probe_frozen_members_verified'] == 51 and proof['parent_full_frozen_members_verified'] == 563
