import hashlib
import json
from pathlib import Path

import numpy as np
from moveit.core.collision_detection import CollisionResult

from audit_buttons import add_box
from button_planner_routed_generated import PressPlanner


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]


def main():
    path = EVIDENCE / 'inputs/button_plan.json'
    plan = json.loads(path.read_text())
    rows = []
    for arrangement in ('table', 'loaded'):
        for opening in (-.019, -.0199):
            planner = PressPlanner(ROOT, EVIDENCE / 'inputs/robot_dual.urdf',
                ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/passive_leg_baseline.json')
            for side in ('left', 'right'):
                for number in (1, 2):
                    planner.initial[side + f'_dex1_finger_{number}_joint'] = opening
            if arrangement == 'table':
                for side in ('left', 'right'):
                    add_box(planner, 'box48_' + side, [.44, .38 if side == 'left' else -.38, .84475],
                            [.1187450051085, .08814010043, .0595])
            else:
                add_box(planner, 'loaded_rotor_swept_envelope', [.550571051126, 0, 1.060], [.440, .440, .140])
            for side in ('left', 'right'):
                arm = plan['arms'][side]
                cap = 'centrifuge_assembly_' + arm['button'] + '_button_collision_0'
                planner.scene.allowed_collision_matrix.set_entry(cap, arm['contact_link'], True)
                for phase, samples in (('approach', arm['approach']), ('withdraw', arm['approach'][::-1]), ('push', arm['push'])):
                    previous = np.array(samples[0])
                    checked, collision, minimum = 0, False, float('inf')
                    for target in samples:
                        target = np.array(target)
                        count = max(2, int(np.ceil(np.max(np.abs(target - previous)) / .002)) + 1)
                        for vector in np.linspace(previous, target, count):
                            bad, distance = planner.collision(side, vector)
                            checked += 1
                            minimum = min(minimum, distance)
                            if bad:
                                collision = True
                                break
                        previous = target
                        if collision:
                            break
                    row = dict(arrangement=arrangement, gripper_opening=opening, side=side, phase=phase,
                               checked=checked, collision=collision, minimum_nonallowed_distance_m=minimum,
                               original_five_lid_angle_envelope_present=True)
                    rows.append(row)
                    print(row, flush=True)
                planner.scene.allowed_collision_matrix.set_entry(cap, arm['contact_link'], False)
    result = dict(status='PASS' if all(not row['collision'] and row['minimum_nonallowed_distance_m'] >= .005 for row in rows) else 'FAIL',
                  records=rows, maximum_joint_step_rad=.002, minimum_original_nonallowed_clearance_m=.005,
                  only_target_fingertip_button_contact_exempt=True, source_plan_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  actual_shifted_robot_and_machine_reviewed=True, old_r119_press_plan_not_executed=True)
    with (EVIDENCE / 'buttons_collision_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    assert result['status'] == 'PASS', result


if __name__ == '__main__':
    main()
