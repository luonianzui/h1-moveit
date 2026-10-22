import json
import math
from pathlib import Path
import sys

import numpy as np

from audit_buttons import add_box
from button_planner_routed_generated import PressPlanner
from handover_policy import require_stop_pre


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]


def audit(request):
    if request.get('original_runtime_holder_references') or request.get('mode') != 'stop_handover':
        raise ValueError('Only actual empty-handed stop handover is supported')
    if not math.isfinite(request['source_sim']) or request['source_sim'] < 0:
        raise ValueError('Missing actual source timestamp')
    positions = request['positions']
    plan = json.loads((EVIDENCE / 'inputs/button_plan.json').read_text())
    require_stop_pre(positions, plan['arms']['right'])
    planner = PressPlanner(ROOT, EVIDENCE / 'inputs/robot_dual.urdf')
    if not planner.initial.keys() <= positions.keys():
        raise ValueError('Actual snapshot does not cover the complete robot model')
    planner.initial = {name: float(positions[name]) for name in planner.initial}
    if not all(math.isfinite(value) for value in planner.initial.values()):
        raise ValueError('Nonfinite native handover state')
    add_box(planner, 'loaded_rotor_swept_envelope', [.550571051126, 0, 1.060], [.440, .440, .140])
    arm = plan['arms']['left']
    if max(abs(positions[name] - value) for name, value in zip(arm['joints'], arm['approach'][0])) >= .005:
        raise ValueError('Left actual hand not at the original opening-path start')
    cap = 'centrifuge_assembly_' + arm['button'] + '_button_collision_0'
    planner.scene.allowed_collision_matrix.set_entry(cap, arm['contact_link'], True)
    records = []
    for phase, samples in (('approach', arm['approach']), ('push', arm['push'])):
        previous = np.array(samples[0])
        minimum, checked = float('inf'), 0
        for target in samples:
            target = np.array(target)
            count = max(2, int(np.ceil(np.max(np.abs(target - previous)) / .002)) + 1)
            for vector in np.linspace(previous, target, count):
                collision, distance = planner.collision('left', vector)
                checked += 1
                minimum = min(minimum, distance)
                if collision or distance < .005:
                    raise ValueError('Original full-model handover collision/5mm clearance gate failed')
            previous = target
        records.append(dict(phase=phase, checked=checked, minimum_nonallowed_distance_m=minimum))
    return dict(status='PASS', records=records, maximum_joint_step_rad=.002,
                minimum_original_nonallowed_clearance_m=.005, source_sim=request['source_sim'],
                original_push_and_approach_reverse_paths_identical_geometry=True,
                original_five_lid_angle_and_loaded_rotor_envelope_preserved=True,
                whole_robot_actual_positions_used=True, only_target_left_fingertip_button_contact_exempt=True,
                right_stop_pre_not_replaced_by_old_home=True, no_physical_attachment_or_robot_command=True)


if __name__ == '__main__':
    request_path, output_path = map(Path, sys.argv[1:3])
    try:
        result = audit(json.loads(request_path.read_text()))
    except (ValueError, KeyError) as error:
        result = dict(status='FAIL', error=str(error), no_robot_command_issued=True)
    with output_path.open('x') as output:
        json.dump(result, output, indent=2)
    print(result['status'], result.get('error', ''), flush=True)
    assert result['status'] == 'PASS'
