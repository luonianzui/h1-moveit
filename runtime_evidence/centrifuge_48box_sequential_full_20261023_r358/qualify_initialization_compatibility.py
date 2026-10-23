import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from acceleration_inputs import canonical
from leg_initialization_contract import LEG_JOINTS

ROOT = Path(__file__).resolve().parent
PROBE = ROOT.parent / 'centrifuge_48box_sequential_init_20261020_r3'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    identity = json.loads((PROBE / 'probe_identity.json').read_text())
    for relative, expected in identity['sha256'].items():
        assert digest(PROBE / relative) == expected
    initialization = json.loads((PROBE / 'variants/sole_box/run/leg_initialization.json').read_text())
    result = json.loads((PROBE / 'variants/sole_box/run/result.json').read_text())
    assert initialization['status'] == 'PASS_STATIONARY_INITIALIZATION_ONLY'
    assert initialization['action_error_code'] == 0 and initialization['hold_sim_seconds'] >= 5.
    assert result['status'] == 'PASS_PASSIVE_COMPONENT_ONLY' and result['retirement']['status'] == 'PASS'
    original = json.loads((ROOT / 'inputs/plan.json').read_text())['initial_positions']
    assert initialization['original_targets'] == [original[name] for name in LEG_JOINTS]
    first = initialization['after_sim'] - initialization['hold_sim_seconds']
    samples = [row for row in initialization['samples'] if first <= row['sim'] <= initialization['after_sim']]
    assert len(samples) >= 100 and samples[-1]['sim'] - samples[0]['sim'] >= 4.95
    maximum = max(abs(value - original[name]) for row in samples for name, value in zip(LEG_JOINTS, row['positions']))
    assert maximum < .002 and initialization['hold_peak_error_rad'] < .002
    assert (PROBE / 'initialize_stationary_legs.py').read_text() == (ROOT / 'initialize_stationary_legs.py').read_text().replace('.005', '.002')
    assert digest(PROBE / 'leg_initialization_contract.py') == digest(ROOT / 'leg_initialization_contract.py')
    current = ET.parse(ROOT / 'inputs/robot_dual.urdf').getroot()
    tested = ET.parse(PROBE / 'variants/sole_box/inputs/robot_dual.urdf').getroot()
    for side in ('left', 'right'):
        selector = './link[@name="' + side + '_ankle_roll_link"]'
        assert canonical(current.find(selector)) == canonical(tested.find(selector))
    reports = ['probe_identity.json', 'variants/sole_box/run/leg_initialization.json', 'variants/sole_box/run/result.json']
    with (ROOT / 'initialization_compatibility.json').open('x') as stream:
        json.dump(dict(status='PASS_ACTUAL_STATIONARY_LEG_COMPATIBILITY_NOT_LOADED_TASK',
                       source=str(PROBE), source_reports={name:digest(PROBE / name) for name in reports},
                       maximum_independent_hold_error_rad=maximum, hold_samples=len(samples),
                       current_initialization_source_sha256=digest(ROOT / 'initialize_stationary_legs.py'),
                       stationary_leg_endpoint_limit_rad=.005, measured_probe_limit_rad=.002, original_limit_rad=.001,
                       fresh_current_five_second_hold_still_required=True,
                       original_targets_unchanged=True, hand_grip_limits_unchanged=False,
                       hand_limits_separately_declared_in_speed_profile=True,
                       passive_sim_per_wall=result['measured_sim_per_wall']), stream, indent=2)
    print('STATIONARY_COMPATIBILITY_PASS', maximum, result['measured_sim_per_wall'], flush=True)


if __name__ == '__main__':
    main()
