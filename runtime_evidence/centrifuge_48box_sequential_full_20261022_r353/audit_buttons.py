import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from moveit.core.collision_detection import CollisionResult
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'src/h1_2_moveit_config/scripts'))
from centrifuge_press_plan import PressPlanner
sys.path.insert(0, str(ROOT / 'runtime_evidence/centrifuge_48box_loading_20260923_r121'))
from probe_loading_collision import message_pose


def add_box(planner, name, centre, size):
    item = CollisionObject(id=name, operation=CollisionObject.ADD)
    item.header.frame_id = 'world'
    item.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
    matrix = np.eye(4)
    matrix[:3, 3] = centre
    item.primitive_poses = [message_pose(matrix)]
    planner.scene.apply_collision_object(item)


def main():
    original = ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119'
    plan = json.loads((original / 'press_plan.json').read_text())
    loading = json.loads((EVIDENCE / 'inputs/plan.json').read_text())
    records = []
    for arrangement in ('table', 'loaded'):
        planner = PressPlanner(ROOT, EVIDENCE / 'inputs/robot_effort.urdf', original / 'passive_leg_baseline.json')
        if arrangement == 'table':
            for side in ('left', 'right'):
                add_box(planner, 'box48_' + side, [.44, .38 if side == 'left' else -.38, .84475],
                        [.1187450051085, .08814010043, .0595])
        else:
            add_box(planner, 'loaded_rotor_swept_envelope', [.630571051126, 0, 1.060], [.440, .440, .140])
        for side in ('left', 'right'):
            arm = plan['arms'][side]
            cap = 'centrifuge_assembly_' + arm['button'] + '_button_collision_0'
            planner.scene.allowed_collision_matrix.set_entry(cap, arm['contact_link'], True)
            paths = [('approach', [dict(zip(arm['joints'], values)) for values in arm['approach']]),
                     ('push', [dict(zip(arm['joints'], values)) for values in arm['push']])]
            if arrangement == 'loaded':
                park = next(stage for stage in loading['stages'] if stage['side'] == side and stage['phase'] == 'park')
                paths.append(('park_to_neutral', park['samples'][::-1]))
            for phase, samples in paths:
                previous = samples[0]
                checked = 0
                collisions = []
                for index, sample in enumerate(samples):
                    count = max(2, int(np.ceil(max(abs(sample[name] - previous[name]) for name in sample) / .002)) + 1)
                    for fraction in np.linspace(0, 1, count):
                        configuration = dict(plan['initial_positions'])
                        configuration.update({name: previous[name] + fraction * (sample[name] - previous[name]) for name in sample})
                        planner.state.joint_positions = configuration
                        planner.state.update()
                        planner.request.contacts = False
                        result = CollisionResult()
                        planner.scene.check_collision(planner.request, result, planner.state)
                        checked += 1
                        if result.collision:
                            collisions.append(dict(index=index, fraction=float(fraction)))
                            break
                    previous = sample
                    if collisions:
                        break
                records.append(dict(arrangement=arrangement, side=side, phase=phase,
                                    checked=checked, collisions=collisions))
                print(records[-1], flush=True)
    audit = dict(status='PASS' if not any(row['collisions'] for row in records) else 'FAIL',
                 maximum_joint_step_rad=.002, lid_swept_degrees=[0, 30, 60, 90, 120], records=records,
                 loaded_swept_envelope_m=[.440, .440, .140],
                 only_target_fingertip_button_contact_exempt=True,
                 source_plan_sha256=hashlib.sha256((original / 'press_plan.json').read_bytes()).hexdigest())
    with (EVIDENCE / 'buttons_collision_audit.json').open('x') as stream:
        json.dump(audit, stream, indent=2)
    assert audit['status'] == 'PASS'
    with (EVIDENCE / 'inputs/button_plan.json').open('x') as stream:
        json.dump(plan, stream, indent=2)


if __name__ == '__main__':
    main()

