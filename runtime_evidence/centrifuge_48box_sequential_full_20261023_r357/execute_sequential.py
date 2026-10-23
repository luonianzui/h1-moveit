from execute_cycle import *


def main():
    establish_interrupt_handler()
    verify_interrupt_delivery(os.getpid())
    read_contract(EVIDENCE)
    manager_rate = json.loads((EVIDENCE / 'run/manager_rate.json').read_text())
    verify_manager_rate_output(manager_rate['stdout'], manager_rate['command_returncode'])
    from gain_contract import verify_gain_output
    gain = json.loads((EVIDENCE / 'run/position_gain.json').read_text())
    verify_gain_output(gain['stdout'], gain['command_returncode'])
    from gripper_damping import verify_damping_output
    damping = json.loads((EVIDENCE / 'run/gripper_damping_readback.json').read_text())
    assert damping['read_only_no_parameter_set'] and len(damping['attempts']) == 4
    for attempt in damping['attempts']:
        verify_damping_output(attempt['stdout'], attempt['command_returncode'])
    from button_ramp import require_spline_output
    interpolation = json.loads((EVIDENCE / 'run/button_interpolation_readback.json').read_text())
    assert interpolation['read_only_no_parameter_set'] and {row['side'] for row in interpolation['attempts']} == {'left', 'right'}
    for attempt in interpolation['attempts']:
        require_spline_output(attempt['stdout'], attempt['command_returncode'])
    assert json.loads((EVIDENCE / 'calibrated_gain_audit.json').read_text())['status'] == 'PASS'
    identity = json.loads((EVIDENCE / 'identity.json').read_text())
    assert identity['actual_runtime_subclass'] == 'SpeedPriorityCycle'
    assert identity['sequential_contract'] == 'sequential_single_holder_elbow_up_v1'
    assert identity['maximum_actual_holders'] == 1 and identity['both_payloads_required'] is True
    for relative, expected in identity['sha256'].items():
        assert hashlib.sha256((EVIDENCE / relative).read_bytes()).hexdigest() == expected, relative
    assert json.loads((EVIDENCE / 'buttons_collision_audit.json').read_text())['status'] == 'PASS'
    dispatch = validate_dispatch(EVIDENCE)
    with (EVIDENCE / 'run/dispatch_gate.json').open('x') as stream:
        json.dump(dispatch, stream, indent=2)
    with (EVIDENCE / 'run/execution.claim').open('x') as stream:
        json.dump(dict(identity_sha256=hashlib.sha256((EVIDENCE / 'identity.json').read_bytes()).hexdigest()), stream)
    plan = json.loads((EVIDENCE / 'inputs/plan.json').read_text())
    buttons = json.loads((EVIDENCE / 'inputs/button_plan.json').read_text())
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    from speed_policy import PROFILE, configure_simulation_profile
    assert identity['speed_profile'] == PROFILE
    configure_simulation_profile()
    from table_alignment_cycle import SpeedPriorityCycle
    cycle = SpeedPriorityCycle(plan, buttons)
    status, error = 'FAIL', None
    try:
        cycle.run_complete()
        status = 'PASS'
    except (Exception, KeyboardInterrupt) as exception:
        error = traceback.format_exc()
        cycle.hold(exception)
    finally:
        with (EVIDENCE / 'run/result.json').open('x') as stream:
            json.dump(dict(status=status, scope='two_box_load_close_spin_open_retrieve', error=error,
                        holding=cycle.holding, held_payload=cycle.held_payload, motions=cycle.motion_count,
                        max_slip_m=cycle.max_slip, max_native_slip_m=cycle.max_native_slip,
                        final_machine=cycle.values.get('machine'), physical_press_events=cycle.events,
                        actual_holder_registry=cycle.holders.snapshot(), order=cycle.order.snapshot(),
                        paired_actual_cycle=False, sequential_actual_cycle=True), stream, indent=2)
        cycle.close()
        rclpy.try_shutdown()
    print(status, error, flush=True)
    assert status == 'PASS'


if __name__ == '__main__':
    main()
