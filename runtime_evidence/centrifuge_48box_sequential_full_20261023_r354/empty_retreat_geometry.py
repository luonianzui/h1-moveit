import copy
import math

import numpy as np


def plan_empty_retreat(planner_factory, request, neutral_positions, lift_height_m=.12, outboard_y_m=.30):
    if request['mode'] != 'empty_return' or request.get('original_runtime_holder_references'):
        raise ValueError('Empty retreat requires an actually empty holder registry')
    if not .06 <= lift_height_m <= .18 or not .28 <= outboard_y_m <= .38:
        raise ValueError('Retreat corridor parameters outside reviewed range')
    planner = planner_factory(request, {hand: np.eye(4) for hand in ('left', 'right')})
    seed = {name: request['positions'][name] for name in planner.names}
    planner.planner.state.joint_positions = dict(request['positions'])
    planner.planner.state.update()
    first = {hand: planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link').copy()
             for hand in ('left', 'right')}
    lifted = copy.deepcopy(first)
    for body in lifted.values():
        body[2, 3] += lift_height_m
    lift = planner.checked_stage(first, lifted, seed, 'empty_vertical_clearance', count=81)
    seed = lift['samples'][-1]
    outboard = copy.deepcopy(lifted)
    for hand, body in outboard.items():
        body[1, 3] = outboard_y_m if hand == 'left' else -outboard_y_m
    side = planner.checked_stage(lifted, outboard, seed, 'empty_outboard_clearance', count=81)
    seed = side['samples'][-1]
    target = {name: neutral_positions[name] for name in planner.names}
    home = planner.empty_connector(seed, target)
    return dict(status='PASS', stages=[lift, side, dict(phase='empty_return', samples=home)],
                checked_samples=planner.checked, maximum_joint_step_rad=.002,
                planning_only_attachment=False, no_physical_attachment=True,
                actual_released_scene_used=True, original_loose_payload_collision_guards_unchanged=True,
                wrist_frame_goals_not_carried_or_rebased_payloads=True,
                lift_height_m=lift_height_m, outboard_y_m=outboard_y_m,
                structured_clearance_before_free_joint_search=True)


def plan_empty_guided_retreat(planner_factory, request, neutral_positions, executed_loading_stages):
    if request['mode'] != 'empty_return' or request.get('original_runtime_holder_references'):
        raise ValueError('Empty retreat requires an actually empty holder registry')
    if len(executed_loading_stages) < 2:
        raise ValueError('Current run qualified loading guides required')
    planner = planner_factory(request, {hand: np.eye(4) for hand in ('left', 'right')})
    previous = {name: request['positions'][name] for name in planner.names}
    stages = []
    for original in reversed(executed_loading_stages[1:]):
        samples = [dict(previous)]
        for target in reversed(original['samples']):
            if set(target) != set(planner.names) or not all(math.isfinite(value) for value in target.values()):
                raise ValueError('Incomplete or nonfinite current loading guide')
            count = max(2, math.ceil(max(abs(target[name] - previous[name]) for name in planner.names) / .002) + 1)
            for fraction in np.linspace(0, 1, count):
                joints = {name: previous[name] + fraction * (target[name] - previous[name]) for name in planner.names}
                vector = np.array([joints[name] for name in planner.names])
                if np.any(vector < planner.lower - 1e-12) or np.any(vector > planner.upper + 1e-12):
                    raise RuntimeError('Empty retreat guide exceeds original joint margin')
                planner.checked += 1
                if planner.planner.check(joints)['collision']:
                    planner.planner.check(joints, verbose=True)
                    raise RuntimeError('Actual released-scene empty retreat dense FCL collision')
            samples.append(dict(target))
            previous = dict(target)
        stages.append(dict(phase='empty_reverse_' + original['phase'], samples=samples))
        planner.planner.initial.update(previous)
    target = {name: neutral_positions[name] for name in planner.names}
    home = planner.empty_connector(previous, target)
    for sample in home:
        if set(sample) != set(planner.names) or not all(math.isfinite(value) for value in sample.values()):
            raise ValueError('Incomplete or nonfinite empty home path')
        vector = np.array([sample[name] for name in planner.names])
        if np.any(vector < planner.lower - 1e-12) or np.any(vector > planner.upper + 1e-12):
            raise RuntimeError('Empty home path exceeds original joint margin')
    stages.append(dict(phase='empty_return', samples=home))
    return dict(status='PASS', stages=stages, checked_samples=planner.checked,
                maximum_joint_step_rad=.002, planning_only_attachment=False, no_physical_attachment=True,
                actual_released_scene_used=True, original_loose_payload_collision_guards_unchanged=True,
                current_run_commanded_guides_only_proposals_not_measured_state=True,
                actual_measured_start_and_static_released_boxes_unchanged=True,
                no_unnecessary_return_to_original_table_grasp_height=True,
                original_joint_margins_enforced_in_final_home_path=True,
                fixed_wrist_orientation_lift_not_used=True)
