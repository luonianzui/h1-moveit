import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as element_tree

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
PARENT = EVIDENCE.parent / 'centrifuge_48box_parallel_transport_20261013_r238'


def validate_body(matrix):
    value = np.asarray(matrix, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError('Body transform must be finite 4x4')
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
        raise ValueError('Invalid homogeneous transform')
    if not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-9, rtol=0):
        raise ValueError('Nonorthogonal body rotation')
    if abs(np.linalg.det(value[:3, :3]) - 1) > 1e-9:
        raise ValueError('Improper body rotation')
    return value.copy()


def paired_interpolation(first, last, count):
    if first.keys() != last.keys() or set(first) != {'left', 'right'} or count < 2:
        raise ValueError('A paired path requires both unchanged payload identities')
    origins = {side: validate_body(body) for side, body in first.items()}
    goals = {side: validate_body(body) for side, body in last.items()}
    rotations = {side: Slerp([0, 1], Rotation.from_matrix([origins[side][:3, :3], goals[side][:3, :3]]))
                 for side in origins}
    for fraction in np.linspace(0, 1, count):
        targets = {}
        for side in origins:
            matrix = np.eye(4)
            matrix[:3, 3] = origins[side][:3, 3] + fraction * (goals[side][:3, 3] - origins[side][:3, 3])
            matrix[:3, :3] = rotations[side](float(fraction)).as_matrix()
            targets[side] = matrix
        yield targets


def dense_edge(first, last, maximum_step=.002):
    if first.keys() != last.keys() or not first or not 0 < maximum_step <= .002:
        raise ValueError('Dense edge cannot omit joints or loosen the 2mrad check')
    if not all(math.isfinite(value) for value in (*first.values(), *last.values())):
        raise ValueError('Nonfinite measured or planned joints')
    count = max(2, math.ceil(max(abs(last[name] - first[name]) for name in first) / maximum_step) + 1)
    for fraction in np.linspace(0, 1, count):
        yield float(fraction), {name: first[name] + fraction * (last[name] - first[name]) for name in first}


class CoupledPlanner:
    def __init__(self, probe, scene, request, robot_path, references, initial, seed):
        self.probe = probe
        self.scene = scene
        self.references = {side: validate_body(value) for side, value in references.items()}
        self.planner = scene.make_alignment_scene(dict(request['positions'], **seed), request['poses'], robot_path)
        self.initial = dict(initial)
        self.names = ['torso_joint'] + [name for side in ('left', 'right')
                     for name in self.planner.model.get_joint_model_group(side + '_arm').active_joint_model_names]
        tree = element_tree.parse(robot_path).getroot()
        limits = [tree.find(f"./joint[@name='{name}']/limit") for name in self.names]
        self.lower = np.array([float(limit.get('lower')) + .02 for limit in limits])
        self.upper = np.array([float(limit.get('upper')) - .02 for limit in limits])
        self.lower[0], self.upper[0] = math.radians(-44), math.radians(44)
        self.checked = 0
        self.maximum_position_error = 0.
        self.maximum_angle_error = 0.
        self.minimum_joint_margin = math.inf

    def solve(self, bodies, seed):
        targets = {side: validate_body(body) @ np.linalg.inv(self.references[side]) for side, body in bodies.items()}
        origin = np.array([seed[name] for name in self.names])

        def residual(vector):
            self.planner.state.joint_positions = dict(self.initial, **dict(zip(self.names, vector)))
            self.planner.state.update()
            errors = []
            for side in ('left', 'right'):
                wrist = self.planner.state.get_global_link_transform(side + '_wrist_yaw_link')
                errors.extend(20 * (wrist[:3, 3] - targets[side][:3, 3]))
                errors.extend(Rotation.from_matrix(targets[side][:3, :3] @ wrist[:3, :3].T).as_rotvec())
            return np.r_[errors, .0001 * (vector - origin)]

        solved = least_squares(residual, np.clip(origin, self.lower, self.upper), bounds=(self.lower, self.upper),
                               max_nfev=500, ftol=1e-11, xtol=1e-11, gtol=1e-11)
        errors = residual(solved.x)
        metrics = {side: dict(position_error_m=float(np.linalg.norm(errors[index:index + 3]) / 20),
                             angle_error_deg=math.degrees(float(np.linalg.norm(errors[index + 3:index + 6]))))
                   for side, index in (('left', 0), ('right', 6))}
        if any(value['position_error_m'] > .00002 or value['angle_error_deg'] > .01 for value in metrics.values()):
            raise RuntimeError('Paired IK rejected: ' + json.dumps(dict(metrics=metrics, solution=solved.x.tolist())))
        self.maximum_position_error = max(self.maximum_position_error, *(value['position_error_m'] for value in metrics.values()))
        self.maximum_angle_error = max(self.maximum_angle_error, *(value['angle_error_deg'] for value in metrics.values()))
        return dict(zip(self.names, solved.x.tolist()))

    def attach(self, bodies, joints):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        from shape_msgs.msg import SolidPrimitive

        self.planner.initial.update(joints)
        self.planner.state.joint_positions = self.planner.initial
        self.planner.state.update()
        self.planner.update_scene_state()
        for side, body in bodies.items():
            fingers = [side + f'_dex1_finger_{number}_' + suffix
                       for number in (1, 2) for suffix in ('link', 'pad_link')]
            item = AttachedCollisionObject()
            item.link_name = side + '_wrist_yaw_link'
            item.touch_links = fingers
            item.object.id = 'box48_' + side
            item.object.header.frame_id = 'world'
            item.object.operation = CollisionObject.ADD
            item.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=self.probe.SIZE)]
            item.object.primitive_poses = [self.scene.message_pose(body)]
            self.planner.scene.process_attached_collision_object(item)
            for finger in fingers:
                self.planner.scene.allowed_collision_matrix.set_entry(item.object.id, finger, True)
        self.planner.state = self.planner.scene.current_state

    def check_edge(self, first, last, phase, waypoint):
        for fraction, joints in dense_edge(first, last):
            values = np.array([joints[name] for name in self.names])
            if np.any(values < self.lower - 1e-12) or np.any(values > self.upper + 1e-12):
                raise RuntimeError('Paired edge crosses a joint margin')
            self.minimum_joint_margin = min(self.minimum_joint_margin, float(np.min(values - self.lower)),
                                            float(np.min(self.upper - values)))
            initial_support_contact = phase == 'table_vertical_lift' and (waypoint == 0 or waypoint == 1 and fraction == 0)
            for side in ('left', 'right'):
                self.planner.scene.allowed_collision_matrix.set_entry('box48_' + side, 'table_link_surface', initial_support_contact)
            self.checked += 1
            if self.planner.check(joints)['collision']:
                self.planner.check(joints, verbose=True)
                raise RuntimeError('Paired dense FCL collision: ' + json.dumps(dict(phase=phase, waypoint=waypoint,
                                                                                  fraction=fraction, joints=joints)))

    def stage(self, first, last, seed, phase, count=61):
        samples = []
        for index, bodies in enumerate(paired_interpolation(first, last, count)):
            solved = self.solve(bodies, seed)
            self.check_edge(seed, solved, phase, index)
            samples.append(solved)
            seed = solved
        self.planner.initial.update(seed)
        return dict(phase=phase, samples=samples, first_bodies={side: body.tolist() for side, body in first.items()},
                    last_bodies={side: body.tolist() for side, body in last.items()})


def main():
    sys.path.insert(0, str(PARENT))
    import probe_pair_transport as probe

    for name in ('parallel_station_ik_probe.json', 'shared_torso_pair_probe.json'):
        probe.verified(probe.REFERENCE / name, probe.REFERENCE / 'SHA256SUMS')
    for name in ('alignment_scene.py', 'dual_pickup_request.json'):
        probe.verified(probe.SCENE_REFERENCE / name, probe.SCENE_REFERENCE / 'SHA256SUMS')
    prior = json.loads((PARENT / 'pair_transport_probe.json').read_text())
    model = prior['models']['0.12']
    robot_path = Path(model['path'])
    assert probe.digest(robot_path) == model['sha256']
    identity = json.loads((probe.ACTIVE / 'identity.json').read_text())['sha256']
    plan_path = probe.ACTIVE / 'inputs/plan.json'
    assert probe.digest(plan_path) == identity['inputs/plan.json']
    plan = json.loads(plan_path.read_text())
    release_height = plan['release_centre_z_m'] - .045
    assert abs(release_height - 1.07525) < 1e-12
    references = {side: np.array(matrix) for side, matrix in
                  json.loads((probe.REFERENCE / 'parallel_station_ik_probe.json').read_text())['references'].items()}
    specification = importlib.util.spec_from_file_location('coupled_observed_scene', probe.SCENE_REFERENCE / 'alignment_scene.py')
    scene = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(scene)
    original_request = json.loads((probe.SCENE_REFERENCE / 'dual_pickup_request.json').read_text())
    grip_seed = next(row['joints'] for row in prior['records'] if row['station_x_m'] == .12 and row['phase'] == 'table_grip')
    records = []
    for shift, height, pitch in ((-.10, 1.15, -10.), (-.08, 1.16975, -10.), (-.08, 1.14475, 0.)):
        started = time.monotonic()
        request = copy.deepcopy(original_request)
        request['poses']['centrifuge']['position']['x'] += shift
        planner = CoupledPlanner(probe, scene, request, robot_path, references, plan['initial_positions'], grip_seed)
        table = probe.body_targets(.84475, 0., table=True)
        upper = probe.body_targets(height, 0., table=True)
        transport = probe.body_targets(height, pitch)
        release = probe.body_targets(release_height, 0.)
        for bodies in (transport, release):
            for body in bodies.values():
                body[0, 3] += shift
        stages = []
        record = dict(station_x_m=.12, device_shift_m=shift, transport_height_m=height,
                      transport_pitch_deg=pitch, release_height_m=release_height)
        try:
            seed = planner.solve(table, grip_seed)
            planner.attach(table, seed)
            for phase, first, last in (('table_vertical_lift', table, upper),
                                       ('paired_transfer', upper, transport),
                                       ('paired_level_and_lower', transport, release)):
                stage = planner.stage(first, last, seed, phase)
                stages.append(stage)
                seed = stage['samples'][-1]
                print(shift, phase, 'DENSE_FCL_PASS', planner.checked, flush=True)
            record['status'] = 'COMPLETE_CARRY_GEOMETRY_PASS_NOT_STARTUP_OR_PHYSICS'
        except (RuntimeError, ValueError) as error:
            record.update(status='REJECTED', error=str(error))
            print(shift, 'REJECTED', str(error), flush=True)
        record.update(stages=stages, checked_samples=planner.checked,
                      maximum_ik_position_error_m=planner.maximum_position_error,
                      maximum_ik_angle_error_deg=planner.maximum_angle_error,
                      minimum_margin_inside_already_shrunk_limits_rad=planner.minimum_joint_margin,
                      wall_s=time.monotonic() - started)
        records.append(record)
        with (EVIDENCE / f'carry_case_{len(records)}.json').open('x') as stream:
            json.dump(record, stream, indent=2)
    result = dict(status='OFFLINE_PAIRED_CARRY_PATH_SEARCH', records=records,
                  full_joint_edge_step_rad=.002, ik_position_tolerance_m=.00002, ik_angle_tolerance_deg=.01,
                  only_own_fingers_and_initial_table_support_contact_allowed=True,
                  stationary_robot_table_and_experimental_device_pose=True,
                  models_and_sealed_inputs_unchanged=True, no_gazebo_attach_detach_or_control=True,
                  startup_empty_approach_button_plans_and_retrieval_still_required=True,
                  actual_two_hand_grip_and_under_180_not_certified=True,
                  dependencies=dict(robot=model, original_plan_sha256=identity['inputs/plan.json'],
                                    corrected_probe_sha256=probe.digest(PARENT / 'corrected_device_shift_probe.json')))
    with (EVIDENCE / 'paired_carry_paths.json').open('x') as stream:
        json.dump(result, stream, indent=2)


if __name__ == '__main__':
    main()
