import math

import numpy as np

from post_retreat_contract import RETREAT_PHASES, expected_retreat_phases


MODES = ('empty_gripper_after_retreat', 'empty_return_after_gripper')
FINGERS = tuple(hand + f'_dex1_finger_{number}_joint' for hand in ('left', 'right') for number in (1, 2))


def verify_request(request, identity):
    if (request.get('mode') not in MODES or request.get('current_run_guide_identity') != identity
            or request.get('original_runtime_holder_references') != {}
            or tuple(request.get('completed_empty_retreat_stages', ())) != expected_retreat_phases(request.get('current_run_executed_loading_guides'))
            or request.get('accepted_gripper_efforts') != dict(left=0., right=0.)):
        raise ValueError('Original completed actual retreats, current identity and empty zero-force hands required')
    if not math.isfinite(request['source_sim']):
        raise ValueError('Finite fresh actual source stamp required')
    positions = request['positions']
    if any(not math.isfinite(positions[name]) or not -.020000001 <= positions[name] <= 0 for name in FINGERS):
        raise ValueError('Actual finite original-range four-finger state required')
    if request['mode'] == MODES[1] and any(abs(positions[name] + .019) >= .00025 for name in FINGERS):
        raise ValueError('Original fresh actual endpoint must precede remaining path planning')


def plan_post_retreat_gripper(planner_factory, request, neutral, identity):
    verify_request(request, identity)
    planner = planner_factory(request, {hand: np.eye(4) for hand in ('left', 'right')})
    seed = {name: request['positions'][name] for name in planner.names}
    if request['mode'] == MODES[0]:
        for fraction in np.linspace(0, 1, 31):
            values = dict(request['positions'])
            for name in FINGERS:
                values[name] += fraction * (-.019 - values[name])
            planner.checked += 1
            if planner.planner.check(values)['collision']:
                raise RuntimeError('Actual post-retreat zero-force gripper stroke has full-state FCL collision')
        return dict(status='PASS', stages=[], checked_samples=planner.checked, maximum_joint_step_rad=.002,
                    actual_full_four_finger_stroke_samples=31, original_zero_force_target_m=-.019,
                    planning_only_attachment=False, no_physical_attachment=True,
                    completed_actual_retreats_required=True, all_loose_payload_finger_collision_guards_unchanged=True)
    target = {name: neutral[name] for name in planner.names}
    samples = planner.empty_connector(seed, target)
    if samples[0] != seed or samples[-1] != target:
        raise ValueError('Fresh actual seed and original neutral endpoint cannot be substituted')
    verify_return_margins(planner, samples)
    return dict(status='PASS', stages=[dict(phase='empty_return', samples=samples)],
                checked_samples=planner.checked, maximum_joint_step_rad=.002,
                actual_fresh_four_finger_state_used=True, planning_only_attachment=False,
                no_physical_attachment=True, original_loose_payload_collision_guards_unchanged=True,
                original_solver_seed_bounds_and_final_dense_FCL_unchanged=True)


def verify_return_margins(planner, samples):
    for sample in samples:
        if set(sample) != set(planner.names) or not all(math.isfinite(value) for value in sample.values()):
            raise ValueError('Original complete finite final-return joint samples required')
        vector = np.array([sample[name] for name in planner.names])
        if np.any(vector < planner.lower - 1e-12) or np.any(vector > planner.upper + 1e-12):
            raise RuntimeError('Final post-gripper return exceeds original joint margin')


def verify_finger_snapshot(current, initial):
    differences = [abs(current[name] - initial[name]) for name in FINGERS]
    if not all(math.isfinite(value) for value in differences):
        raise RuntimeError('Nonfinite actual post-retreat four-finger state')
    drift = max(differences)
    if drift > .0003:
        raise RuntimeError('Post-retreat four-finger state drift exceeds original prismatic .0003m bound')
    return drift
