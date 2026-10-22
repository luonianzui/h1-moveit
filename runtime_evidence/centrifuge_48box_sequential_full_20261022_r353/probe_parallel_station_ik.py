import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as element_tree

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
ORIGINAL = ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/robot.urdf'
STATION = ROOT / 'runtime_evidence/centrifuge_48box_dual_sync_20261012_r227/inputs/robot_station.urdf'
SEMANTIC = ROOT / 'src/h1_2_moveit_config/config/h1_2.srdf'


def main():
    assert hashlib.sha256(STATION.read_bytes()).hexdigest() == '4ee37cfc32e62845cc7d3223110950bae31189e6fff2ebf2180e471a2394e20b'
    plan = json.loads((ROOT / 'runtime_evidence/centrifuge_48box_coherent_feedback_20261012_r230/inputs/plan.json').read_text())
    station_solutions = json.loads((STATION.parents[1] / 'working_station_pose_probe.json').read_text())
    original = RobotState(RobotModel(str(ORIGINAL), str(SEMANTIC)))
    state = RobotState(RobotModel(str(STATION), str(SEMANTIC)))
    tree = element_tree.fromstring(STATION.read_bytes())
    generator = np.random.default_rng(232)
    references, records = {}, []
    for side in ('left', 'right'):
        stage = next(stage for stage in plan['stages'] if stage['side'] == side and stage['phase'] == 'descend')
        original.joint_positions = dict(plan['initial_positions'], **stage['samples'][-1])
        original.update()
        body = np.eye(4)
        body[:3, 3] = [.44, .38 if side == 'left' else -.38, .84475]
        references[side] = np.linalg.inv(original.get_global_link_transform(side + '_wrist_yaw_link')) @ body
        names = list(state.robot_model.get_joint_model_group(side + '_arm').active_joint_model_names)
        limits = [tree.find(f"./joint[@name='{name}']/limit") for name in names]
        lower = np.array([float(limit.get('lower')) + .02 for limit in limits])
        upper = np.array([float(limit.get('upper')) - .02 for limit in limits])
        initial = next(row['joints'] for row in station_solutions if row['side'] == side and row['shift'] == .12)
        seed = np.array([initial[name] for name in names])
        for phase, centre, yaw in (
                ('table_above', [.44, .38 if side == 'left' else -.38, 1.00475], 0.),
                ('table_grip', [.44, .38 if side == 'left' else -.38, .84475], 0.),
                ('table_pilot', [.44, .38 if side == 'left' else -.38, .85975], 0.),
                ('table_lift', [.44, .38 if side == 'left' else -.38, 1.180], 0.),
                ('basket_above', [.603178339 if side == 'left' else .657963763,
                                 .085730038 if side == 'left' else -.085730037, 1.180], .309270119),
                ('basket_release', [.603178339 if side == 'left' else .657963763,
                                   .085730038 if side == 'left' else -.085730037, 1.07525], .309270119)):
            body[:3, 3] = centre
            body[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
            target = body @ np.linalg.inv(references[side])

            def residual(values):
                configuration = dict(plan['initial_positions'])
                configuration.update(zip(names, values))
                configuration['torso_joint'] = 0.
                state.joint_positions = configuration
                state.update()
                wrist = state.get_global_link_transform(side + '_wrist_yaw_link')
                return np.r_[20 * (wrist[:3, 3] - target[:3, 3]),
                             Rotation.from_matrix(target[:3, :3] @ wrist[:3, :3].T).as_rotvec()]

            best, score = None, math.inf
            seeds = [seed] + [generator.uniform(lower, upper) for unused in range(5)]
            for candidate in seeds:
                solution = least_squares(residual, np.clip(candidate, lower, upper), bounds=(lower, upper),
                                         max_nfev=300, ftol=1e-10, xtol=1e-10, gtol=1e-10)
                error = residual(solution.x)
                quality = float(np.linalg.norm(error))
                if quality < score:
                    best, score = solution, quality
                if np.linalg.norm(error[:3]) / 20 <= .00002 and np.linalg.norm(error[3:]) <= math.radians(.01):
                    break
            error = residual(best.x)
            passed = np.linalg.norm(error[:3]) / 20 <= .00002 and np.linalg.norm(error[3:]) <= math.radians(.01)
            records.append(dict(side=side, phase=phase, status='IK_PASS_NOT_FCL' if passed else 'FAIL',
                                position_error_m=float(np.linalg.norm(error[:3]) / 20),
                                angle_error_deg=math.degrees(float(np.linalg.norm(error[3:]))),
                                shared_torso_rad=0., body_target=body.tolist(), wrist_target=target.tolist(),
                                joints=dict(zip(names, best.x.tolist()))))
            print(side, phase, records[-1]['status'], records[-1]['position_error_m'], records[-1]['angle_error_deg'], flush=True)
            if passed:
                seed = best.x
    with (EVIDENCE / 'parallel_station_ik_probe.json').open('x') as stream:
        json.dump(dict(status='OFFLINE_IK_DIAGNOSTIC_NOT_PHYSICAL', records=records,
                       references={side:matrix.tolist() for side, matrix in references.items()},
                       runtime_station_not_selected=True, previous_initial_paired_path_FCL_failure_preserved=True,
                       no_robot_command=True, no_formal_asset_changed=True), stream, indent=2)


if __name__ == '__main__':
    main()
