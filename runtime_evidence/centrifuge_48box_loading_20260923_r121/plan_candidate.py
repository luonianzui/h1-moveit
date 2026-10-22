import json
import math
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, RobotState
from shape_msgs.msg import SolidPrimitive

from probe_loading_collision import LoadingPlanner, ROOT, EVIDENCE, message_pose


class CandidatePlanner(LoadingPlanner):
    def update_scene_state(self):
        message = RobotState()
        message.joint_state.name = list(self.state.joint_positions)
        message.joint_state.position = list(self.state.joint_positions.values())
        self.scene.set_current_state(message)

    def solve_tcp(self, side, target, yaw, seed):
        names = ['torso_joint'] + list(self.model.get_joint_model_group(side + '_arm').active_joint_model_names)
        limits = [self.urdf.find(f"./joint[@name='{name}']/limit") for name in names]
        lower = np.array([float(limit.get('lower')) + .02 for limit in limits])
        upper = np.array([float(limit.get('upper')) - .02 for limit in limits])
        lower[0], upper[0] = np.deg2rad([-40, 40])
        vector = np.array([seed[name] for name in names])
        rotation = Rotation.from_euler('z', yaw).as_matrix() @ Rotation.from_euler('y', 60, degrees=True).as_matrix()

        def residual(positions):
            state = dict(self.initial)
            state.update(zip(names, positions))
            self.state.joint_positions = state
            self.state.update()
            pose = self.state.get_global_link_transform(side + '_dex1_tcp')
            wrist = self.state.get_global_link_transform(side + '_wrist_yaw_link')
            return np.r_[20 * (pose[:3, 3] - target),
                         Rotation.from_matrix(rotation @ wrist[:3, :3].T).as_rotvec(),
                         .0001 * (positions - vector)]

        result = least_squares(residual, np.clip(vector, lower, upper), bounds=(lower, upper),
                               max_nfev=300, ftol=1e-10, xtol=1e-10, gtol=1e-10)
        error = residual(result.x)
        if np.linalg.norm(error[:3]) / 20 > .00001 or np.linalg.norm(error[3:6]) > .0001:
            raise ValueError(f'IK failed side={side} target={target} error={error[:6]}')
        return dict(zip(names, result.x.tolist()))

    def attach_for_planning(self, side, centre):
        self.update_scene_state()
        attached = AttachedCollisionObject()
        attached.link_name = side + '_wrist_yaw_link'
        attached.touch_links = [side + f'_dex1_finger_{number}_{suffix}'
                                for number in (1, 2) for suffix in ('link', 'pad_link')]
        attached.object.id = 'box48_' + side
        attached.object.header.frame_id = 'world'
        attached.object.operation = CollisionObject.ADD
        attached.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1187450051085, .08814010043, .0595])]
        matrix = np.eye(4)
        matrix[:3, 3] = centre
        attached.object.primitive_poses = [message_pose(matrix)]
        self.scene.process_attached_collision_object(attached)
        self.state = self.scene.current_state


def interpolate(first, last, count):
    return [{name: first[name] + fraction * (last[name] - first[name]) for name in first}
            for fraction in np.linspace(0, 1, count)]


def free_path(planner, first, last):
    names = list(first)
    origin = np.array([first[name] for name in names])
    goal = np.array([last[name] for name in names])
    generator = np.random.default_rng(480121)
    checks = 0

    def valid(vector):
        nonlocal checks
        checks += 1
        return not planner.check(dict(zip(names, vector)))['collision']

    def edge(start, end):
        count = max(2, int(np.ceil(np.max(np.abs(end - start)) / .035)) + 1)
        return all(valid(vector) for vector in np.linspace(start, end, count))

    if not valid(origin) or not valid(goal):
        raise ValueError('RRT endpoint in collision')
    trees = [[(origin, None)], [(goal, None)]]
    lower = np.array([float(planner.urdf.find(f"./joint[@name='{name}']/limit").get('lower')) + .02 for name in names])
    upper = np.array([float(planner.urdf.find(f"./joint[@name='{name}']/limit").get('upper')) - .02 for name in names])
    lower[0], upper[0] = np.deg2rad([-40, 40])

    def extend(tree, target):
        index = int(np.argmin([np.linalg.norm(node[0] - target) for node in tree]))
        start = tree[index][0]
        distance = np.linalg.norm(target - start)
        if distance < 1e-9:
            return index, True
        end = start + min(1, .25 / distance) * (target - start)
        if not edge(start, end):
            return None, False
        tree.append((end, index))
        return len(tree) - 1, distance <= .25

    def trace(tree, index):
        path = []
        while index is not None:
            point, index = tree[index]
            path.append(point)
        return path[::-1]

    deadline = time.monotonic() + 120
    for iteration in range(2000):
        if time.monotonic() > deadline:
            break
        active = iteration % 2
        if iteration % 5 == 0:
            target = trees[1 - active][0][0]
        elif iteration % 3:
            target = np.clip(origin + generator.random() * (goal - origin) + generator.normal(0, .65, len(names)), lower, upper)
        else:
            target = generator.uniform(lower, upper)
        index, reached = extend(trees[active], target)
        if index is None:
            continue
        meeting = trees[active][index][0]
        for attempt in range(60):
            other_index, reached = extend(trees[1 - active], meeting)
            if other_index is None:
                break
            if reached:
                paths = [None, None]
                paths[active] = trace(trees[active], index)
                paths[1 - active] = trace(trees[1 - active], other_index)
                vectors = paths[0] + paths[1][::-1][1:]
                for attempt in range(40):
                    first_index, last_index = sorted(generator.choice(len(vectors), 2, replace=False))
                    if last_index > first_index + 1 and edge(vectors[first_index], vectors[last_index]):
                        vectors = vectors[:first_index + 1] + vectors[last_index:]
                samples = []
                for start, end in zip(vectors, vectors[1:]):
                    count = max(2, int(np.ceil(np.max(np.abs(end - start)) / .02)) + 1)
                    samples.extend(dict(zip(names, point.tolist())) for point in np.linspace(start, end, count)[1:])
                print('RRT found', iteration, checks, len(samples), flush=True)
                return [first] + samples
    raise ValueError(f'RRT no path after {checks} collision queries')


def main():
    planner = CandidatePlanner(ROOT, ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/robot.urdf',
                               ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/passive_leg_baseline.json')
    probes = json.loads((EVIDENCE / 'ik_probe.json').read_text())
    result = dict(status='FAIL', planning_only_attachment=True, stages=[], failure=None,
                  initial_positions=dict(planner.initial), grasp_tcp_height_above_centre_m=.04,
                  gripper_open_m=-.02, gripper_grasp_m=-.01899,
                  grasp_pitch_deg=60, release_floor_gap_m=.011)
    for side in ('left', 'right'):
        planner.payload('box48_' + side, [.44, .38 if side == 'left' else -.38, .84475])
    try:
        old_plan = json.loads((ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/press_plan.json').read_text())
        for side in ('left', 'right'):
            names = old_plan['arms'][side]['joints']
            park = dict(zip(names, old_plan['arms'][side]['approach'][80]))
            origin = {name: planner.initial[name] for name in names}
            path = interpolate(origin, park, 101)
            planner.set_arm(side, list(park.values()))
            finger = side + '_dex1_finger_' + ('1' if side == 'left' else '2') + '_link'
            finger_pose = planner.state.get_global_link_transform(finger)
            rotation = planner.state.get_global_link_transform(side + '_wrist_yaw_link')[:3, :3]
            local = np.array(old_plan['arms'][side]['local_contact_point_m'])
            target = finger_pose[:3, :3] @ local + finger_pose[:3, 3] + [-.12, 0, .05]
            deeper = dict(zip(names, planner.solve(side, rotation, target, local, np.array(list(park.values())))))
            path += interpolate(park, deeper, 101)[1:]
            park = deeper
            collisions = [index for index, sample in enumerate(path) if planner.check(sample)['collision']]
            result['stages'].append(dict(side=side, phase='park', samples=path, collisions=collisions))
            print(side, 'park', collisions, flush=True)
            if collisions:
                raise ValueError('Parking collision')
            planner.initial.update(park)
        for side in ('left', 'right'):
            selected = next(record for record in probes if record['side'] == side and record['phase'] == 'pick' and record['pitch'] == 60 and record['flip'] == 0)
            seed = selected['joints']
            names = list(seed)
            home = {name: planner.initial[name] for name in names}
            pick = np.array([.44, .38 if side == 'left' else -.38, .88475])
            above = pick + [0, 0, .16]
            grasp = planner.solve_tcp(side, pick, 0, seed)
            pre = planner.solve_tcp(side, above, 0, grasp)
            stages = [('approach', free_path(planner, home, pre))]
            descend = []
            seed = pre
            for fraction in np.linspace(0, 1, 81):
                seed = planner.solve_tcp(side, above + fraction * (pick - above), 0, seed)
                descend.append(seed)
            stages.append(('descend', descend))
            for name, path in stages:
                checks = [planner.check(positions) for positions in path]
                entry = dict(side=side, phase=name, samples=path, collisions=[index for index, check in enumerate(checks) if check['collision']])
                result['stages'].append(entry)
                print(side, name, len(path), entry['collisions'], flush=True)
                if entry['collisions']:
                    planner.check(path[entry['collisions'][0]], verbose=True)
                    raise ValueError('Approach collision')
            planner.check(descend[-1])
            planner.attach_for_planning(side, pick - [0, 0, .04])
            seed = descend[-1]
            target = np.array([.603178339 if side == 'left' else .657963763,
                               .085730038 if side == 'left' else -.085730037, 1.10325])
            waypoints = [(pick + [0, 0, .35], 0), (target + [0, 0, .15], .309270119), (target, .309270119)]
            previous = pick
            previous_yaw = 0
            for label, (endpoint, yaw) in zip(('lift', 'transfer', 'lower'), waypoints):
                path = []
                for fraction in np.linspace(0, 1, 101):
                    seed = planner.solve_tcp(side, previous + fraction * (endpoint - previous),
                                             previous_yaw + fraction * (yaw - previous_yaw), seed)
                    sample = dict(seed)
                    for number in (1, 2):
                        sample[f'{side}_dex1_finger_{number}_joint'] = -.01899
                    path.append(sample)
                checks = []
                for index, positions in enumerate(path):
                    if label == 'lift' and index == 0:
                        planner.scene.allowed_collision_matrix.set_entry('box48_' + side, 'table_link_surface', True)
                    checks.append(planner.check(positions))
                    planner.scene.allowed_collision_matrix.set_entry('box48_' + side, 'table_link_surface', False)
                entry = dict(side=side, phase=label, samples=path, collisions=[index for index, check in enumerate(checks) if check['collision']])
                result['stages'].append(entry)
                print(side, label, len(path), entry['collisions'], flush=True)
                if entry['collisions']:
                    planner.check(path[entry['collisions'][0]], verbose=True)
                    raise ValueError('Attached payload collision')
                previous, previous_yaw = endpoint, yaw
            planner.state.clear_attached_bodies()
            planner.update_scene_state()
            planner.payload('box48_' + side, target - [0, 0, .051], .309270119)
            retreat = [{name: value for name, value in sample.items() if 'dex1_finger' not in name}
                       for sample in path[::-1]]
            for label, path in [('retreat', retreat), ('return', free_path(planner, retreat[-1], home))]:
                collisions = [index for index, sample in enumerate(path) if planner.check(sample)['collision']]
                result['stages'].append(dict(side=side, phase=label, samples=path, collisions=collisions))
                print(side, label, collisions, flush=True)
                if collisions:
                    raise ValueError('Retreat collision')
            result['status'] = 'LEFT_ONLY' if side == 'left' else 'OFFLINE_PASS'
    except Exception as error:
        result['failure'] = str(error)
        print('FAIL', error, flush=True)
    with (EVIDENCE / sys.argv[1]).open('x') as output:
        json.dump(result, output, indent=2)


if __name__ == '__main__':
    main()
