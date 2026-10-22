import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from alignment_scene import make_alignment_scene, message_pose, pose_matrix
from retrieval_rrt import free_path

EVIDENCE = Path(__file__).resolve().parent


class RetrievalPlanner:
    def __init__(self, request):
        self.request = request
        self.hand = request['hand']
        self.payload = request['payload']
        self.planner = make_alignment_scene(request['positions'], request['poses'])
        self.names = ['torso_joint'] + list(self.planner.model.get_joint_model_group(self.hand + '_arm').active_joint_model_names)
        limits = [self.planner.urdf.find(f"./joint[@name='{name}']/limit") for name in self.names]
        self.lower = np.array([float(limit.get('lower')) + .02 for limit in limits])
        self.upper = np.array([float(limit.get('upper')) - .02 for limit in limits])
        self.lower[0], self.upper[0] = np.deg2rad([-90, 90])
        self.checked = 0
        self.fingers = [self.hand + f'_dex1_finger_{number}_{suffix}' for number in (1, 2)
                        for suffix in ('link', 'pad_link')]
        for finger in self.fingers:
            self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload, finger, True)

    def solve(self, target, seed):
        origin = np.array([seed[name] for name in self.names])

        def residual(vector):
            configuration = dict(self.planner.initial)
            configuration.update(zip(self.names, vector))
            self.planner.state.joint_positions = configuration
            self.planner.state.update()
            wrist = self.planner.state.get_global_link_transform(self.hand + '_wrist_yaw_link')
            return np.r_[20 * (wrist[:3, 3] - target[:3, 3]),
                         Rotation.from_matrix(target[:3, :3] @ wrist[:3, :3].T).as_rotvec(),
                         .0001 * (vector - origin)]

        solved = least_squares(residual, np.clip(origin, self.lower, self.upper),
                               bounds=(self.lower, self.upper), max_nfev=500,
                               ftol=1e-11, xtol=1e-11, gtol=1e-11)
        error = residual(solved.x)
        if np.linalg.norm(error[:3]) / 20 > .00002 or np.linalg.norm(error[3:6]) > math.radians(.01):
            raise RuntimeError('Retrieval IK residual exceeds 20 um / .01 degree: ' +
                               json.dumps(dict(target=target.tolist(), error=error.tolist(), names=self.names,
                                   solution=solved.x.tolist(), lower=self.lower.tolist(), upper=self.upper.tolist())))
        return dict(zip(self.names, solved.x.tolist()))

    def checked_path(self, samples, table_endpoint=False, basket_start=False, carry_reference=None):
        previous = samples[0]
        for index, target in enumerate(samples):
            count = max(2, int(np.ceil(max(abs(target[name] - previous[name]) for name in target) / .002)) + 1)
            for fraction in np.linspace(0, 1, count):
                sample = {name: previous[name] + fraction * (target[name] - previous[name]) for name in target}
                table_touch = table_endpoint and index == len(samples) - 1
                self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload, 'table_link_surface', table_touch)
                self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload,
                    'centrifuge_bucket_' + self.payload + '_bucket_' + self.payload + '_collision', basket_start and index <= 1)
                self.checked += 1
                if self.planner.check(sample)['collision']:
                    self.planner.check(sample, verbose=True)
                    raise RuntimeError('Retrieval dense FCL collision at ' + json.dumps(dict(index=index, fraction=float(fraction), joints=sample)))
                if carry_reference is not None:
                    expected_body = self.planner.state.get_global_link_transform(self.hand + '_wrist_yaw_link') @ carry_reference
                    if expected_body[2, 2] <= math.cos(math.radians(2)):
                        raise RuntimeError('Commanded carry payload tilt exceeds 2 degrees')
            previous = target
        self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload, 'table_link_surface', False)
        self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload,
            'centrifuge_bucket_' + self.payload + '_bucket_' + self.payload + '_collision', False)
        self.planner.initial.update(samples[-1])
        return samples

    def cartesian(self, first, last, seed, count=81):
        rotation = Slerp([0, 1], Rotation.from_matrix(np.array([first[:3, :3], last[:3, :3]])))
        samples = []
        for fraction in np.linspace(0, 1, count):
            target = np.eye(4)
            target[:3, 3] = first[:3, 3] + fraction * (last[:3, 3] - first[:3, 3])
            target[:3, :3] = rotation(float(fraction)).as_matrix()
            seed = self.solve(target, seed)
            samples.append(seed)
        return samples

    def attached(self, body):
        self.planner.state.joint_positions = self.planner.initial
        self.planner.state.update()
        self.planner.update_scene_state()
        attached = AttachedCollisionObject()
        attached.link_name = self.hand + '_wrist_yaw_link'
        attached.touch_links = self.fingers
        attached.object.id = 'box48_' + self.payload
        attached.object.header.frame_id = 'world'
        attached.object.operation = CollisionObject.ADD
        attached.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1187450051085, .08814010043, .0595])]
        attached.object.primitive_poses = [message_pose(body)]
        self.planner.scene.process_attached_collision_object(attached)
        self.planner.state = self.planner.scene.current_state

    def pickup(self):
        loading = json.loads((EVIDENCE / 'inputs/plan.json').read_text())
        descend = next(stage for stage in loading['stages'] if stage['side'] == self.hand and stage['phase'] == 'descend')
        configuration = dict(loading['initial_positions'])
        configuration.update(descend['samples'][-1])
        self.planner.state.joint_positions = configuration
        self.planner.state.update()
        original_body = np.eye(4)
        original_body[:3, 3] = [.44, .38 if self.hand == 'left' else -.38, .84475]
        reference = np.linalg.inv(self.planner.state.get_global_link_transform(self.hand + '_wrist_yaw_link')) @ original_body
        body = pose_matrix(self.request['poses']['box48_' + self.payload])
        home = {name: self.request['positions'][name] for name in self.names}
        align = next(stage for stage in loading['stages'] if stage['side'] == self.hand and stage['phase'] == 'align')
        diagnostics = []
        body_yaw = math.atan2(body[1, 0], body[0, 0])
        preferred = self.request.get('preferred_pickup_flip_deg', 180 if abs(body_yaw) > math.pi / 2 else 0)
        for flip in (preferred, 180 - preferred):
            shifted = body.copy()
            shifted[:3, 3] += body[:3, :3] @ [0, 0, .020]
            shifted[:3, :3] = body[:3, :3] @ Rotation.from_euler('z', flip, degrees=True).as_matrix()
            grip = shifted @ np.linalg.inv(reference)
            above = grip.copy()
            above[2, 3] += .12
            try:
                target = self.solve(grip, align['samples'][-1])
                pre = self.solve(above, target)
                if self.planner.check(target)['collision'] or self.planner.check(pre)['collision']:
                    raise RuntimeError('Retrieval endpoint collision')
                approach = free_path(self.planner, home, pre)
                self.checked_path(approach)
                descend_path = self.cartesian(above, grip, pre)
                self.checked_path(descend_path)
                return dict(status='PASS', stages=[dict(phase='approach', samples=approach),
                                                  dict(phase='descend', samples=descend_path)],
                            contact_height_offset_m=.020, flip_deg=flip, diagnostics=diagnostics,
                            checked_samples=self.checked, maximum_joint_sampling_rad=.002,
                            planning_only_attachment=False)
            except (RuntimeError, ValueError) as error:
                diagnostics.append(dict(flip_deg=flip, error=str(error)))
                self.planner.initial = dict(self.request['positions'])
        raise RuntimeError('No collision-free retrieval approach: ' + json.dumps(diagnostics))

    def transport(self):
        body = pose_matrix(self.request['poses']['box48_' + self.payload])
        reference = np.array(self.request['grasp_transform'])
        self.attached(body)
        seed = {name: self.request['positions'][name] for name in self.names}
        first_wrist = body @ np.linalg.inv(reference)
        pilot_body = body.copy()
        bucket = pose_matrix(self.request['poses']['centrifuge']) @ pose_matrix(self.request['poses']['bucket_' + self.payload])
        relative = np.linalg.inv(bucket) @ body
        pilot_body[:3, 3] -= bucket[:3, 0] * relative[0, 3]
        pilot_body[2, 3] += .015
        pilot_wrist = pilot_body @ np.linalg.inv(reference)
        pilot = self.cartesian(first_wrist, pilot_wrist, seed, count=21)
        self.checked_path(pilot, basket_start=True, carry_reference=reference)
        above_body = body.copy()
        carry_height = self.request.get('carry_body_height_m', 1.180)
        above_body[2, 3] = carry_height
        above_wrist = above_body @ np.linalg.inv(reference)
        lift = self.cartesian(pilot_wrist, above_wrist, pilot[-1])
        self.checked_path(lift, carry_reference=reference)
        table_body = np.eye(4)
        body_yaw = math.atan2(body[1, 0], body[0, 0])
        table_yaw = math.pi if abs(body_yaw) > math.pi / 2 else 0.0
        table_body[:3, :3] = Rotation.from_euler('z', table_yaw).as_matrix()
        table_body[:3, 3] = [.44, .38 if self.hand == 'left' else -.38, 1.14475]
        table_wrist = table_body @ np.linalg.inv(reference)
        table_pre = self.solve(table_wrist, lift[-1])
        if 'carry_waypoint_xy' in self.request:
            waypoint = above_body.copy()
            waypoint[:2, 3] = self.request['carry_waypoint_xy']
            first_target = waypoint @ np.linalg.inv(reference)
            first_leg = self.cartesian(above_wrist, first_target, lift[-1])
            self.checked_path(first_leg)
            waypoint[:3, :3] = table_body[:3, :3]
            second_target = waypoint @ np.linalg.inv(reference)
            yaw_leg = self.cartesian(first_target, second_target, first_leg[-1], count=161)
            self.checked_path(yaw_leg)
            final_leg = self.cartesian(second_target, table_wrist, yaw_leg[-1])
            transfer = first_leg + yaw_leg[1:] + final_leg[1:]
        else:
            transfer = self.cartesian(above_wrist, table_wrist, lift[-1], count=161)
        self.checked_path(transfer, carry_reference=reference)
        down_body = table_body.copy()
        down_body[2, 3] = .84475
        down_wrist = down_body @ np.linalg.inv(reference)
        setdown = self.cartesian(table_wrist, down_wrist, transfer[-1], count=121)
        self.checked_path(setdown, table_endpoint=True, carry_reference=reference)
        self.planner.state.clear_attached_bodies()
        self.planner.update_scene_state()
        item = CollisionObject(id='box48_' + self.payload, operation=CollisionObject.ADD)
        item.header.frame_id = 'world'
        item.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1187450051085, .08814010043, .0595])]
        item.primitive_poses = [message_pose(down_body)]
        self.planner.scene.apply_collision_object(item)
        self.planner.state = self.planner.scene.current_state
        for number in (1, 2):
            self.planner.initial[self.hand + f'_dex1_finger_{number}_joint'] = -.0199
        retreat = list(reversed(setdown))
        self.checked_path(retreat)
        loading = json.loads((EVIDENCE / 'inputs/plan.json').read_text())
        park = next(stage for stage in loading['stages'] if stage['side'] == self.hand and stage['phase'] == 'park')
        home = dict(park['samples'][-1], torso_joint=0.0)
        returning = free_path(self.planner, retreat[-1], home)
        self.checked_path(returning)
        return dict(status='PASS', checked_samples=self.checked, maximum_joint_sampling_rad=.002,
                    planning_only_attachment=True, carry_method='cartesian_upright', transfer_body_height_m=carry_height,
                    return_table_yaw_rad=table_yaw, pilot_method='world_vertical_radial_centered', pilot_distance_m=.015,
                    stages=[dict(phase=phase, samples=samples) for phase, samples in (
                        ('pilot_lift', pilot), ('lift', lift), ('transfer', transfer),
                        ('setdown', setdown), ('retreat', retreat), ('return', returning))])


def main():
    request = json.loads(Path(sys.argv[1]).read_text())
    planner = RetrievalPlanner(request)
    result = planner.pickup() if request['mode'] == 'pickup' else planner.transport()
    result.update(hand=request['hand'], payload=request['payload'],
                  simulation_ground_truth_not_vision=True)
    with Path(sys.argv[2]).open('x') as stream:
        json.dump(result, stream, indent=2)
    print('PASS', result['checked_samples'], flush=True)


if __name__ == '__main__':
    main()
