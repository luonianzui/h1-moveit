import json
import math
from pathlib import Path
import xml.etree.ElementTree as element_tree

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

from probe_parallel_station_ik import EVIDENCE, ROOT, STATION, SEMANTIC


def main():
    prior = json.loads((EVIDENCE / 'parallel_station_ik_probe.json').read_text())
    references = {side:np.array(value) for side, value in prior['references'].items()}
    initial = json.loads((ROOT / 'runtime_evidence/centrifuge_48box_coherent_feedback_20261012_r230/inputs/plan.json').read_text())['initial_positions']
    model = RobotModel(str(STATION), str(SEMANTIC))
    state = RobotState(model)
    names = ['torso_joint'] + [name for side in ('left', 'right')
                             for name in model.get_joint_model_group(side + '_arm').active_joint_model_names]
    tree = element_tree.fromstring(STATION.read_bytes())
    limits = [tree.find(f"./joint[@name='{name}']/limit") for name in names]
    lower = np.array([float(limit.get('lower')) + .02 for limit in limits])
    upper = np.array([float(limit.get('upper')) - .02 for limit in limits])
    lower[0], upper[0] = np.deg2rad([-90, 90])
    start = dict(initial)
    for side in ('left', 'right'):
        start.update(next(row['joints'] for row in prior['records'] if row['side'] == side and row['phase'] == 'table_grip'))
    generator = np.random.default_rng(233)
    records = []
    for virtual_station in (.12, .18, .20):
        seed = np.array([start[name] for name in names])
        for phase, height, yaw in (('table_grip', .84475, 0.), ('basket_above', 1.180, .309270119),
                                   ('basket_release', 1.07525, .309270119)):
            targets = {}
            for side in ('left', 'right'):
                body = np.eye(4)
                body[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
                body[:3, 3] = ([.44, .38 if side == 'left' else -.38, height] if phase == 'table_grip' else
                    [.603178339 if side == 'left' else .657963763,
                     .085730038 if side == 'left' else -.085730037, height])
                targets[side] = body @ np.linalg.inv(references[side])
                targets[side][0, 3] -= virtual_station - .12

            def residual(values):
                state.joint_positions = dict(initial, **dict(zip(names, values)))
                state.update()
                errors = []
                for side in ('left', 'right'):
                    wrist = state.get_global_link_transform(side + '_wrist_yaw_link')
                    target = targets[side]
                    errors.extend(20 * (wrist[:3, 3] - target[:3, 3]))
                    errors.extend(Rotation.from_matrix(target[:3, :3] @ wrist[:3, :3].T).as_rotvec())
                return np.array(errors)

            candidates = [seed] + [np.clip(seed + generator.normal(0, .7, len(names)), lower, upper) for unused in range(5)]
            best, score = None, math.inf
            for candidate in candidates:
                solved = least_squares(residual, candidate, bounds=(lower, upper), max_nfev=400,
                                       ftol=1e-10, xtol=1e-10, gtol=1e-10)
                error = residual(solved.x)
                if np.linalg.norm(error) < score:
                    best, score = solved, float(np.linalg.norm(error))
                if all(np.linalg.norm(error[index:index + 3]) / 20 <= .00002 and
                       np.linalg.norm(error[index + 3:index + 6]) <= math.radians(.01) for index in (0, 6)):
                    break
            error = residual(best.x)
            metrics = {side:dict(position_error_m=float(np.linalg.norm(error[index:index + 3]) / 20),
                                angle_error_deg=math.degrees(float(np.linalg.norm(error[index + 3:index + 6]))))
                       for side, index in (('left', 0), ('right', 6))}
            passed = all(row['position_error_m'] <= .00002 and row['angle_error_deg'] <= .01 for row in metrics.values())
            records.append(dict(virtual_station_x_m=virtual_station, phase=phase,
                                status='PAIR_IK_PASS_NOT_FCL' if passed else 'FAIL', errors=metrics,
                                joints=dict(zip(names, best.x.tolist()))))
            print(virtual_station, phase, records[-1]['status'], best.x[0], metrics, flush=True)
            if passed:
                seed = best.x
    with (EVIDENCE / 'shared_torso_pair_probe.json').open('x') as stream:
        json.dump(dict(status='OFFLINE_DIAGNOSTIC_NOT_RUNTIME_SELECTION', records=records,
                       virtual_station_translation_is_target_transform_only=True,
                       no_new_robot_model_created=True, dense_paths_and_FCL_not_done=True,
                       no_robot_commands_or_physical_certificate=True), stream, indent=2)


if __name__ == '__main__':
    main()
