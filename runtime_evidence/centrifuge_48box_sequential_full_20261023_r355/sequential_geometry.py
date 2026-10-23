import copy
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as element_tree

import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.spatial.transform import Rotation, Slerp
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from alignment_scene import make_alignment_scene, message_pose, pose_matrix
from dual_motion import TASK_JOINTS, rigid_reference
from bounded_connector import bounded_free_path


EVIDENCE = Path(__file__).resolve().parent
SIZE = [.1187450051085, .08814010043, .0595]
CONTRACT = 'sequential_single_holder_elbow_up_v1'
HANDS = ('left', 'right')


def validate_request(request):
    if request.get('sequential_contract') != CONTRACT:
        raise ValueError('Explicit sequential contract required')
    if request.get('hand') not in HANDS or request.get('payload') not in HANDS:
        raise ValueError('Unknown hand or payload')
    required = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions'].keys()
    if not required <= request.get('positions', {}).keys():
        raise ValueError('Complete current robot snapshot required')
    if not all(math.isfinite(value) for value in request['positions'].values()):
        raise ValueError('Nonfinite current joint snapshot')
    holders = request.get('original_runtime_holder_references', {})
    if not isinstance(holders, dict) or len(holders) > 1:
        raise ValueError('At most one actual holder is allowed')
    if holders and (set(holders) != {request['hand']} or holders[request['hand']].get('payload') != request['payload']):
        raise ValueError('Actual holder does not match requested single payload')


def dense(first, last, maximum_step=.002):
    if set(first) != TASK_JOINTS or first.keys() != last.keys() or not 0 < maximum_step <= .002:
        raise ValueError('Complete original-dense paired controller state required')
    if not all(math.isfinite(value) for value in (*first.values(), *last.values())):
        raise ValueError('Nonfinite trajectory endpoint')
    count = max(2, math.ceil(max(abs(last[name] - first[name]) for name in first) / maximum_step) + 1)
    for fraction in np.linspace(0, 1, count):
        yield float(fraction), {name: first[name] + fraction * (last[name] - first[name]) for name in first}


def interpolate(first, last, count):
    rotation = Slerp([0, 1], Rotation.from_matrix([first[:3, :3], last[:3, :3]]))
    for fraction in np.linspace(0, 1, count):
        target = np.eye(4)
        target[:3, 3] = first[:3, 3] + fraction * (last[:3, 3] - first[:3, 3])
        target[:3, :3] = rotation(float(fraction)).as_matrix()
        yield target


def accurate(values):
    values = np.asarray(values)
    return bool(values.shape == (6,) and np.isfinite(values).all()
                and np.linalg.norm(values[:3]) / 20 <= .00002
                and math.degrees(np.linalg.norm(values[3:])) <= .01)


class SinglePlanner:
    def __init__(self, request, reference=None):
        validate_request(request)
        self.request = copy.deepcopy(request)
        self.hand, self.payload = request['hand'], request['payload']
        self.reference = None if reference is None else rigid_reference(reference)
        self.planner = make_alignment_scene(request['positions'], request['poses'], EVIDENCE / 'inputs/robot_dual.urdf')
        self.initial = dict(request['positions'])
        self.names = ['torso_joint'] + [name for hand in HANDS for name in
                       self.planner.model.get_joint_model_group(hand + '_arm').active_joint_model_names]
        tree = element_tree.parse(EVIDENCE / 'inputs/robot_dual.urdf').getroot()
        limits = [tree.find(f"./joint[@name='{name}']/limit") for name in self.names]
        self.lower = np.array([float(value.get('lower')) + .02 for value in limits])
        self.upper = np.array([float(value.get('upper')) - .02 for value in limits])
        self.lower[0], self.upper[0] = math.radians(-44), math.radians(44)
        self.active = [name for name in self.names if name == 'torso_joint' or name.startswith(self.hand + '_')]
        self.indices = [self.names.index(name) for name in self.active]
        self.checked = 0
        self.maximum_position_error = self.maximum_angle_error = 0.
        self.attached = False
        for hand, payload, number, suffix in itertools.product(HANDS, HANDS, (1, 2), ('link', 'pad_link')):
            self.planner.scene.allowed_collision_matrix.set_entry('box48_' + payload,
                hand + f'_dex1_finger_{number}_' + suffix, False)

    def update(self, joints):
        self.planner.state.joint_positions = dict(self.initial, **joints)
        self.planner.state.update()

    def wrist(self, joints):
        self.update(joints)
        return np.array(self.planner.state.get_global_link_transform(self.hand + '_wrist_yaw_link'), copy=True)

    def elbow_metrics(self, joints):
        self.update(joints)
        shoulder = self.planner.state.get_global_link_transform(self.hand + '_shoulder_roll_link')[:3, 3].copy()
        elbow = self.planner.state.get_global_link_transform(self.hand + '_elbow_link')[:3, 3].copy()
        wrist = self.planner.state.get_global_link_transform(self.hand + '_wrist_yaw_link')[:3, 3].copy()
        direction = elbow - wrist
        return dict(from_down_deg=math.degrees(math.acos(float(np.clip(direction[2] / np.linalg.norm(direction), -1, 1)))),
                    shoulder=shoulder.tolist(), elbow=elbow.tolist(), wrist=wrist.tolist())

    def solve(self, body, seed, refine=True):
        if self.reference is None:
            raise ValueError('No declared planning grasp transform')
        target = np.asarray(body) @ np.linalg.inv(self.reference)
        origin = np.array([seed[name] for name in self.active])

        def errors(vector):
            joints = dict(seed, **dict(zip(self.active, vector.tolist())))
            wrist = self.wrist(joints)
            return np.r_[20 * (wrist[:3, 3] - target[:3, 3]),
                         Rotation.from_matrix(target[:3, :3] @ wrist[:3, :3].T).as_rotvec()]

        def residual(vector):
            return np.r_[errors(vector), .0001 * (vector - origin)]

        solved = least_squares(residual, np.clip(origin, self.lower[self.indices], self.upper[self.indices]),
            bounds=(self.lower[self.indices], self.upper[self.indices]), max_nfev=500,
            ftol=1e-11, xtol=1e-11, gtol=1e-11)
        deviation = errors(solved.x)
        if not accurate(deviation):
            raise RuntimeError('Single-holder IK exceeds original 20um/.01deg: ' + json.dumps(deviation.tolist()))
        selected = solved.x
        if refine:
            def objective(vector):
                errors(vector)
                elbow = self.planner.state.get_global_link_transform(self.hand + '_elbow_link')[:3, 3]
                return float(np.sum((elbow - target[:3, 3] - [0, 0, .24]) ** 2) + .00004 * np.sum((vector - solved.x) ** 2))
            lower = np.maximum(self.lower[self.indices], solved.x - .045)
            upper = np.minimum(self.upper[self.indices], solved.x + .045)
            candidate = minimize(objective, solved.x, method='SLSQP', bounds=list(zip(lower, upper)),
                constraints=[dict(type='eq', fun=errors)], options=dict(maxiter=55, ftol=1e-10))
            if (candidate.success and np.isfinite(candidate.x).all() and accurate(errors(candidate.x))
                    and objective(candidate.x) < objective(solved.x)):
                selected = candidate.x
        deviation = errors(selected)
        self.maximum_position_error = max(self.maximum_position_error, float(np.linalg.norm(deviation[:3]) / 20))
        self.maximum_angle_error = max(self.maximum_angle_error, math.degrees(float(np.linalg.norm(deviation[3:]))))
        return dict(seed, **dict(zip(self.active, selected.tolist())))

    def attach(self, body, joints):
        if self.attached or self.reference is None:
            raise ValueError('Only one planning payload attachment is allowed')
        self.update(joints)
        self.planner.initial = dict(self.initial, **joints)
        self.planner.update_scene_state()
        item = AttachedCollisionObject()
        item.link_name = self.hand + '_wrist_yaw_link'
        fingers = [self.hand + f'_dex1_finger_{number}_{suffix}' for number in (1, 2) for suffix in ('link', 'pad_link')]
        item.touch_links = fingers
        item.object.id = 'box48_' + self.payload
        item.object.header.frame_id = 'world'
        item.object.operation = CollisionObject.ADD
        item.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=SIZE)]
        item.object.primitive_poses = [message_pose(body)]
        self.planner.scene.process_attached_collision_object(item)
        for finger in fingers:
            self.planner.scene.allowed_collision_matrix.set_entry(item.object.id, finger, True)
        self.planner.state = self.planner.scene.current_state
        self.attached = True

    def check_edge(self, first, last, phase, table_endpoint=False, initial_support=None):
        for fraction, joints in dense(first, last):
            vector = np.array([joints[name] for name in self.names])
            if np.any(vector < self.lower - 1e-12) or np.any(vector > self.upper + 1e-12):
                raise RuntimeError('Original shrunken joint bound exceeded')
            if self.attached:
                self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload,
                    'table_link_surface', table_endpoint or initial_support == 'table' or
                    initial_support == 'table_start_only' and fraction == 0)
                self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload,
                    'centrifuge_bucket_' + self.payload + '_bucket_' + self.payload + '_collision',
                    initial_support == 'basket')
            self.checked += 1
            if self.planner.check(joints)['collision']:
                self.planner.check(joints, verbose=True)
                raise RuntimeError('Single-holder dense full-scene collision: ' + json.dumps(dict(phase=phase, fraction=fraction, joints=joints)))
            if self.attached:
                body = self.planner.state.get_global_link_transform(self.hand + '_wrist_yaw_link') @ self.reference
                if body[2, 2] <= math.cos(math.radians(2)):
                    raise RuntimeError('Original two-degree commanded payload tilt exceeded')

    def stage(self, first, last, seed, phase, count=81, table_end=False, support_start=None, refine=True):
        samples = [dict(seed)]
        for index, target in enumerate(interpolate(first, last, count)):
            solved = self.solve(target, seed, refine=refine)
            self.check_edge(seed, solved, phase, table_endpoint=table_end and index == count - 1,
                            initial_support=('table_start_only' if support_start == 'table' and index == 1 else support_start)
                            if index <= 1 else None)
            samples.append(solved)
            seed = solved
        return dict(phase=phase, samples=samples, first_body=first.tolist(), last_body=last.tolist())

    def guided_stage(self, first, last, seed, phase, guides, table_end=False, support_start=None):
        if len(guides) < 3 or any(set(row) != TASK_JOINTS for row in guides):
            raise ValueError('Full-state geometric proposal required at every waypoint')
        samples = [dict(seed)]
        offset = {name: seed[name] - guides[0][name] for name in self.active}
        fallbacks = []
        for index, target in enumerate(interpolate(first, last, len(guides))):
            fraction = index / (len(guides) - 1)
            proposal = dict(seed, **{name: guides[index][name] + (1 - fraction) * offset[name] for name in self.active})
            solved = self.solve(target, proposal, refine=False)
            if max(abs(solved[name] - seed[name]) for name in self.active) > .12:
                solved = self.solve(target, seed, refine=False)
                fallbacks.append(index)
            if max(abs(solved[name] - seed[name]) for name in self.active) > .12:
                raise RuntimeError('Single-arm proposal jumps between IK branches: ' + json.dumps(dict(
                    phase=phase, index=index, maximum_jump_rad=max(abs(solved[name] - seed[name]) for name in self.active))))
            support = support_start if index <= 1 else None
            if support == 'table' and index == 1:
                support = 'table_start_only'
            self.check_edge(seed, solved, phase, table_endpoint=table_end and index == len(guides) - 1,
                            initial_support=support)
            samples.append(solved)
            seed = solved
        return dict(phase=phase, samples=samples, first_body=first.tolist(), last_body=last.tolist(),
                    hints_not_measured_state_or_frozen_runtime_reference=True,
                    original_step_and_precision_current_branch_fallback_indices=fallbacks)

    def empty_connector(self, first, last):
        if self.attached or self.request.get('original_runtime_holder_references'):
            raise ValueError('Free-space connector is only for actually empty hands')
        try:
            self.check_edge(first, last, 'empty_direct')
            count = max(2, math.ceil(max(abs(last[name] - first[name]) for name in first) / .02) + 1)
            return [{name: first[name] + fraction * (last[name] - first[name]) for name in first}
                    for fraction in np.linspace(0, 1, count)]
        except RuntimeError:
            path = bounded_free_path(self.planner, first, last)
            for start, end in zip(path, path[1:]):
                self.check_edge(start, end, 'empty_bounded_tree')
            return path
