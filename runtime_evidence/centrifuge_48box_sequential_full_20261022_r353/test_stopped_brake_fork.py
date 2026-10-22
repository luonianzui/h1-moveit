import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
BENCH = ROOT / 'runtime_evidence/centrifuge_48box_position_brake_20261010_r189'


def normalized(element):
    return element.tag, sorted(element.attrib.items()), (element.text or '').strip(), [normalized(child) for child in element]


def test_machine_fork_only_changes_motor_plugin():
    current = ET.parse(EVIDENCE / 'inputs/centrifuge_brake.sdf').getroot()
    original = ET.parse(ROOT / 'src/h1_2_description/models/centrifuge/model.sdf').getroot()
    motor = current.find("./model/plugin[@name='CentrifugeInterlockStartupRampProbeR1']")
    assert motor.find('stopped_velocity_gain').text == '80'
    assert motor.find('stopped_position_gain').text == '8000'
    assert motor.get('filename') == str(EVIDENCE / 'inputs/libCentrifugeInterlockStartupRampProbeR1.so')
    motor.remove(motor.find('stopped_velocity_gain'))
    motor.remove(motor.find('stopped_position_gain'))
    motor.set('name', 'CentrifugeInterlock')
    motor.set('filename', 'CentrifugeInterlock')
    current_lid_velocity = current.find("./model/joint[@name='lid_hinge_joint']/axis/limit/velocity")
    original_lid_velocity = original.find("./model/joint[@name='lid_hinge_joint']/axis/limit/velocity")
    assert current_lid_velocity.text == '0.8' and original_lid_velocity.text == '0.35'
    current_lid_velocity.text = original_lid_velocity.text
    assert normalized(current) == normalized(original)


def test_inherited_legacy_motor_library_bytes_preserved_not_active_plugin():
    source = ROOT / 'install_centrifuge_brake_20261010_r189/h12_centrifuge_brake_r189/lib/libCentrifugeInterlockBrakeR189.so'
    assert hashlib.sha256(source.read_bytes()).digest() == hashlib.sha256((EVIDENCE / 'inputs/libCentrifugeInterlockBrakeR189.so').read_bytes()).digest()


def test_original_safety_header_unchanged():
    assert (BENCH / 'plugin_src/systems/CentrifugeSafety.hh').read_bytes() == (ROOT / 'src/h1_2_moveit_config/systems/CentrifugeSafety.hh').read_bytes()


def test_launch_binds_only_new_model():
    source = (EVIDENCE / 'inputs/lab_effort.launch.py').read_text()
    assert repr(str(EVIDENCE / 'inputs/centrifuge_brake.sdf')) in source
    assert "model_path = os.path.join(description, 'models', 'centrifuge', 'model.sdf')" not in source


def test_machine_still_requires_physical_torque_and_old_static_gate():
    source = (BENCH / 'plugin_src/systems/CentrifugeInterlockBrakeR189.cc').read_text()
    assert 'JointForceCmd' in source
    assert 'JointVelocityCmd' not in source
    assert 'StoppedMotorTorque' in source
    guard = (EVIDENCE / 'loading_machine_guard_original.py').read_text()
    assert '.02' in guard


def test_full_cycle_fails_closed_on_missing_or_wrong_brake_config():
    source = (EVIDENCE / 'execute_cycle.py').read_text()
    assert "machine.get('stopped_velocity_gain') != 80" in source
    assert "abs(machine['motor_torque_nm']) > 2.000001" in source
    assert 'min(rate, .012)' not in source

