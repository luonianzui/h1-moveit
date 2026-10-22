import math
import time


def wait_original_gripper_endpoint(cycle, hand, target, result_clock, maximum_sim_s=1., maximum_wall_s=30.):
    if hand not in ('left', 'right') or not math.isfinite(target) or not math.isfinite(result_clock):
        raise ValueError('Finite actual gripper result clock and original target required')
    if maximum_sim_s != 1. or maximum_wall_s != 30.:
        raise ValueError('Original native-feedback waiting bounds cannot be extended')
    names = tuple(hand + '_dex1_finger_' + str(number) + '_joint' for number in (1, 2))
    begin_sim, begin_wall = cycle.sim, time.monotonic()
    cycle.record('original_gripper_endpoint_fresh_feedback_begin', dict(
        hand=hand, target=target, successful_result_clock_sim=result_clock,
        initial_native_joint_stamp=cycle.joint_time, original_precision_m=.00025,
        maximum_wait_sim_s=maximum_sim_s, maximum_wait_wall_s=maximum_wall_s))
    while True:
        cycle.safety()
        error = max(abs(cycle.positions[name] - target) for name in names)
        if math.isfinite(cycle.joint_time) and cycle.joint_time > result_clock and error < .00025:
            cycle.record('original_gripper_endpoint_fresh_feedback_pass', dict(
                hand=hand, source_sim=cycle.joint_time, successful_result_clock_sim=result_clock,
                actual_maximum_error_m=error, original_precision_m=.00025,
                actual_wait_sim_s=cycle.sim-begin_sim, no_command_value_substitution=True))
            return
        if cycle.sim-begin_sim >= maximum_sim_s or time.monotonic()-begin_wall >= maximum_wall_s:
            cycle.record('original_gripper_endpoint_fresh_feedback_fault', dict(
                hand=hand, native_joint_stamp=cycle.joint_time, result_clock_sim=result_clock,
                actual_maximum_error_m=error, original_precision_m=.00025))
            raise RuntimeError('Fresh gripper native endpoint did not satisfy original .00025m gate')
        cycle.spin()
