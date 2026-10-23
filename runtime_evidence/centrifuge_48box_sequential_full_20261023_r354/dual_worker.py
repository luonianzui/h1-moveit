import copy
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from alignment_scene import make_alignment_scene, message_pose, pose_matrix
from dual_motion import TASK_JOINTS
from contact_phase_policy import allow_planning_grasp_touch
from empty_retreat_geometry import plan_empty_guided_retreat


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
GEOMETRY = EVIDENCE.parent / 'centrifuge_48box_coupled_path_20261013_r240'
sys.path.insert(0, str(GEOMETRY))
from coupled_path import CoupledPlanner, paired_interpolation, dense_edge
sys.path.insert(0, str(ROOT / 'runtime_evidence/centrifuge_48box_loading_20260923_r121'))
from plan_candidate import free_path
from dense_tree import select_free_path
from lift_target_provider import stage_body_targets
from micro_alignment import alignment_stage_parameters
from minimum_turn import CONTRACT, branch_seed, choose_pickup
from elbow_posture import ELBOW_CONTRACT, refine, posture_metrics


SIZE = [.1187450051085, .08814010043, .0595]


def verify_assignment(assignment):
    if set(assignment) != {'left', 'right'} or set(assignment.values()) != {'left', 'right'}:
        raise ValueError('A paired plan requires two distinct real payloads and both hands')


class PairedPlanner(CoupledPlanner):
    def solve(self, bodies, seed):
        return refine(self, bodies, seed, super().solve)

    def __init__(self, request, references):
        from types import SimpleNamespace
        import alignment_scene

        verify_assignment(request['assignment'])
        self.assignment = dict(request['assignment'])
        self.request = request
        seed = {name: request['positions'][name] for name in TASK_JOINTS}
        super().__init__(SimpleNamespace(SIZE=SIZE), alignment_scene, request,
                         EVIDENCE / 'inputs/robot_dual.urdf', references, request['positions'], seed)
        for hand, payload in self.assignment.items():
            for number, suffix in itertools.product((1, 2), ('link', 'pad_link')):
                self.planner.scene.allowed_collision_matrix.set_entry('box48_' + payload,
                    hand + f'_dex1_finger_{number}_' + suffix, allow_planning_grasp_touch(request['mode']))

    def attach(self, bodies, joints):
        self.planner.initial.update(joints)
        self.planner.state.joint_positions = self.planner.initial
        self.planner.state.update()
        self.planner.update_scene_state()
        for hand, payload in self.assignment.items():
            fingers = [hand + f'_dex1_finger_{number}_' + suffix
                       for number, suffix in itertools.product((1, 2), ('link', 'pad_link'))]
            item = AttachedCollisionObject()
            item.link_name = hand + '_wrist_yaw_link'
            item.touch_links = fingers
            item.object.id = 'box48_' + payload
            item.object.header.frame_id = 'world'
            item.object.operation = CollisionObject.ADD
            item.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=SIZE)]
            item.object.primitive_poses = [message_pose(bodies[hand])]
            self.planner.scene.process_attached_collision_object(item)
        self.planner.state = self.planner.scene.current_state

    def checked_stage(self, first, last, seed, phase, count=81, guides=None, basket_start=False, table_end=False, table_start=False):
        count, guides = alignment_stage_parameters(self.request, phase, first, last, count, guides)
        samples = [dict(seed)]
        targets = stage_body_targets(self.request, phase, first, last, count, guides, paired_interpolation,
                                     ROOT / 'src/h1_2_description/models/centrifuge/model.sdf')
        for index, bodies in enumerate(targets):
            preferred = branch_seed(self.request, phase, seed, guides, index)
            solved = self.solve(bodies, preferred)
            for fraction, joints in dense_edge(seed, solved):
                values = np.array([joints[name] for name in self.names])
                if np.any(values < self.lower - 1e-12) or np.any(values > self.upper + 1e-12):
                    raise RuntimeError('Paired path crosses original joint margin')
                for hand, payload in self.assignment.items():
                    initial_contact = table_start and (index == 0 or index == 1 and fraction == 0)
                    self.planner.scene.allowed_collision_matrix.set_entry('box48_' + payload, 'table_link_surface',
                                                                          initial_contact or table_end and index == count - 1)
                    self.planner.scene.allowed_collision_matrix.set_entry('box48_' + payload,
                        'centrifuge_bucket_' + payload + '_bucket_' + payload + '_collision', basket_start and index <= 1)
                self.checked += 1
                if self.planner.check(joints)['collision']:
                    self.planner.check(joints, verbose=True)
                    raise RuntimeError('Paired actual-state dense FCL collision: ' + json.dumps(dict(
                        phase=phase, index=index, fraction=fraction, joints=joints)))
                if self.request.get('retrieval'):
                    for hand in self.assignment:
                        carried = self.planner.state.get_global_link_transform(hand + '_wrist_yaw_link') @ self.references[hand]
                        if carried[2, 2] <= math.cos(math.radians(2)):
                            raise RuntimeError('Commanded retrieval payload tilt exceeds original 2 degrees')
            samples.append(solved)
            seed = solved
        self.planner.initial.update(seed)
        return dict(phase=phase, samples=samples)

    def empty_connector(self, first, last):
        self.planner.initial.update(first)
        samples = select_free_path(self.request, free_path)(self.planner, first, last)
        previous = samples[0]
        for target in samples:
            for fraction, joints in dense_edge(previous, target):
                self.checked += 1
                if self.planner.check(joints)['collision']:
                    raise RuntimeError('Paired empty full-state dense FCL collision')
            previous = target
        self.planner.initial.update(samples[-1])
        return samples

    def release_opening(self, joints):
        for fraction in np.linspace(0, 1, 31):
            values = dict(joints)
            for hand in ('left', 'right'):
                for number in (1, 2):
                    name = hand + f'_dex1_finger_{number}_joint'
                    first = self.request['positions'][name]
                    values[name] = first + fraction * (-.0199 - first)
            self.checked += 1
            if self.planner.check(values)['collision']:
                self.planner.check(values, verbose=True)
                raise RuntimeError('Simultaneous release opening collision')


def nominal_references():
    prior = EVIDENCE.parent / 'centrifuge_48box_control_resample_20261012_r232/parallel_station_ik_probe.json'
    return {hand: np.array(matrix) for hand, matrix in json.loads(prior.read_text())['references'].items()}


def legacy_plan_pickup(request):
    refs = nominal_references()
    table_route = json.loads((EVIDENCE / 'inputs/dual_route.json').read_text())
    guided_grip = table_route['forward_stages'][0]['samples'][0]
    diagnostics = []
    for flips in itertools.product((0, 180), repeat=2):
        shifted_refs = {}
        targets = {}
        for hand, flip in zip(('left', 'right'), flips):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_euler('z', -flip, degrees=True).as_matrix()
            shifted_refs[hand] = refs[hand] @ rotation
            targets[hand] = pose_matrix(request['poses']['box48_' + request['assignment'][hand]])
            if request['mode'] == 'retrieval_pickup':
                offset = request.get('retrieval_grip_height_offset_m', .010)
                assert 0 <= offset <= .020
                targets[hand][2, 3] += offset
        planner = PairedPlanner(request, shifted_refs)
        home = {name: request['positions'][name] for name in planner.names}
        above = copy.deepcopy(targets)
        for body in above.values():
            body[2, 3] += .12
        try:
            grip = planner.solve(targets, guided_grip if request['mode'] == 'load_pickup' else home)
            pre = planner.solve(above, grip)
            approach = planner.empty_connector(home, pre)
            descent = planner.checked_stage(above, targets, pre, 'descend', count=81)
            lookahead = None
            if request['mode'] == 'retrieval_pickup':
                future_positions = dict(request['positions'], **descent['samples'][-1])
                planner.planner.state.joint_positions = future_positions
                planner.planner.state.update()
                future_refs = {hand: (np.linalg.inv(planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link')) @
                    pose_matrix(request['poses']['box48_' + payload])).tolist() for hand, payload in request['assignment'].items()}
                pilot_request = dict(request, mode='pilot', positions=future_positions, planning_references=future_refs, retrieval=True)
                pilot = plan_transport(pilot_request)
                future_positions.update(pilot['stages'][-1]['samples'][-1])
                planner.planner.state.joint_positions = future_positions
                planner.planner.state.update()
                future_poses = copy.deepcopy(request['poses'])
                for hand, payload in request['assignment'].items():
                    body = planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link') @ np.array(future_refs[hand])
                    future_poses['box48_' + payload]['position'] = dict(zip('xyz', body[:3, 3].tolist()))
                    future_poses['box48_' + payload]['orientation'] = dict(zip('xyzw', Rotation.from_matrix(body[:3, :3]).as_quat().tolist()))
                carry = plan_transport(dict(request, mode='retrieval_transport', positions=future_positions,
                                            poses=future_poses, planning_references=future_refs, retrieval=True))
                lookahead = dict(status='PROSPECTIVE_RIGID_GEOMETRY_PASS_NOT_ACTUAL_GRASP',
                                 pilot_checks=pilot['checked_samples'], carry_checks=carry['checked_samples'],
                                 no_command_or_native_reference_freeze_from_prediction=True,
                                 actual_post_grasp_plan_and_guards_still_required=True)
            return dict(status='PASS', stages=[dict(phase='approach', samples=approach), descent],
                        checked_samples=planner.checked, maximum_joint_step_rad=.002, planning_only_attachment=False,
                        assignment=request['assignment'], flips_deg=list(flips), diagnostics=diagnostics,
                        exact_actual_full_state_used=True, no_physical_attachment=True,
                        prospective_retrieval_lookahead=lookahead)
        except (RuntimeError, ValueError) as error:
            diagnostics.append(dict(flips_deg=list(flips), error=str(error)))
    raise RuntimeError('No paired actual pickup path: ' + json.dumps(diagnostics))



def plan_pickup(request):
    return choose_pickup(request, sys.modules[__name__])

def plan_transport(request):
    bodies = {hand: pose_matrix(request['poses']['box48_' + payload]) for hand, payload in request['assignment'].items()}
    references = {hand: np.array(matrix) for hand, matrix in request['planning_references'].items()}
    planner = PairedPlanner(request, references)
    seed = {name: request['positions'][name] for name in planner.names}
    planner.attach(bodies, seed)
    stages = []
    if request['mode'] in ('load_transport', 'load_alignment'):
        route = json.loads((EVIDENCE / 'inputs/dual_route.json').read_text())['forward_stages']
        selected = route if request['mode'] == 'load_transport' else route[-1:]
        first = bodies
        for template in selected:
            last = {hand: np.array(matrix) for hand, matrix in
                    json.loads((EVIDENCE / 'inputs/dual_route.json').read_text())['reverse_stages'][
                        3 - route.index(template)]['first_bodies'].items()}
            stage = planner.checked_stage(first, last, seed, template['phase'], guides=template['samples'])
            stages.append(stage)
            first, seed = last, stage['samples'][-1]
        planner.release_opening(seed)
    elif request['mode'] == 'pilot':
        targets = copy.deepcopy(bodies)
        for hand, body in targets.items():
            body[2, 3] += .015
            if request.get('retrieval'):
                payload = request['assignment'][hand]
                bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + payload])
                relative = np.linalg.inv(bucket) @ bodies[hand]
                body[:3, 3] -= bucket[:3, 0] * relative[0, 3]
        stages.append(planner.checked_stage(bodies, targets, seed, 'pilot_lift', count=21,
                                             basket_start=bool(request.get('retrieval')),
                                             table_start=not request.get('retrieval')))
    elif request['mode'] == 'retrieval_transport':
        high = copy.deepcopy(bodies)
        for body in high.values():
            body[2, 3] = request.get('carry_body_height_m', 1.15975)
        stage = planner.checked_stage(bodies, high, seed, 'lift')
        stages.append(stage)
        seed = stage['samples'][-1]
        outboard = copy.deepcopy(high)
        for hand, body in outboard.items():
            body[1, 3] = .32 if hand == 'left' else -.32
        stage = planner.checked_stage(high, outboard, seed, 'outboard')
        stages.append(stage)
        seed = stage['samples'][-1]
        table = {}
        yaws = {}
        for hand, body in outboard.items():
            yaw = math.atan2(body[1, 0], body[0, 0])
            yaw = math.pi if abs(yaw) > math.pi / 2 else 0.
            yaws[request['assignment'][hand]] = yaw
            target = np.eye(4)
            target[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
            target[:3, 3] = [.44, .38 if hand == 'left' else -.38, 1.14475]
            table[hand] = target
        stage = planner.checked_stage(outboard, table, seed, 'transfer', count=121)
        stages.append(stage)
        seed = stage['samples'][-1]
        down = copy.deepcopy(table)
        for body in down.values():
            body[2, 3] = .84475
        stage = planner.checked_stage(table, down, seed, 'setdown', count=121, table_end=True)
        stage['setdown_contract'] = dict(cartesian_samples=121, extra_initial_seed=True,
                                        vertical_span_by_hand_m={hand: float(table[hand][2, 3] - down[hand][2, 3])
                                                                 for hand in table})
        stages.append(stage)
        return dict(status='PASS', stages=stages, table_yaws=yaws, checked_samples=planner.checked,
                    maximum_joint_step_rad=.002, planning_only_attachment=True, no_physical_attachment=True,
                    assignment=request['assignment'], actual_post_spin_poses_used=True,
                    runtime_grip_references_not_rebased=True)
    else:
        raise ValueError('Unknown paired transport mode')
    return dict(status='PASS', stages=stages, checked_samples=planner.checked, maximum_joint_step_rad=.002,
                planning_only_attachment=True, assignment=request['assignment'], no_physical_attachment=True,
                runtime_grip_references_not_rebased=True, simultaneous_opening_dense_checked=request['mode'].startswith('load_'))


def plan_empty_return(request):
    if request.get('current_run_guide_identity') != EVIDENCE.name:
        raise ValueError('Empty retreat guides must belong to this current fresh run')
    guides = request.get('current_run_executed_loading_guides')
    if not isinstance(guides, list) or len(guides) < 2:
        raise ValueError('No qualified current-run loading guides for empty retreat')
    neutral = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions']
    return plan_empty_guided_retreat(PairedPlanner, request, neutral, guides)


def main():
    request_path, output_path = map(Path, sys.argv[1:3])
    request = json.loads(request_path.read_text())
    request['minimum_turn_contract'] = CONTRACT
    request['elbow_posture_contract'] = ELBOW_CONTRACT
    verify_assignment(request['assignment'])
    try:
        if request['mode'] in ('load_pickup', 'retrieval_pickup'):
            result = plan_pickup(request)
        elif request['mode'] == 'empty_return':
            result = plan_empty_return(request)
        elif request['mode'] in ('empty_gripper_after_retreat', 'empty_return_after_gripper'):
            from gripper_return_geometry import plan_post_retreat_gripper
            neutral = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions']
            result = plan_post_retreat_gripper(PairedPlanner, request, neutral, EVIDENCE.name)
        else:
            result = plan_transport(request)
    except (RuntimeError, ValueError) as error:
        result = dict(status='FAIL', error=str(error), no_robot_command_issued=True)
    with output_path.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(result['status'], result.get('error', ''), flush=True)
    assert result['status'] == 'PASS'


if __name__ == '__main__':
    main()
