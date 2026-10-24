import json
from pathlib import Path

import psutil


def identity(process):
    return dict(pid=process.pid, created=process.create_time(), command=process.cmdline())


def owned_telemetry_fault(record, process_factory=psutil.Process):
    for role, expected in record['processes'].items():
        try:
            process = process_factory(expected['pid'])
            if (not process.is_running() or process.status() in (psutil.STATUS_ZOMBIE, psutil.STATUS_DEAD)
                    or abs(process.create_time() - expected['created']) >= .02
                    or process.cmdline() != expected['command']):
                return 'owned_' + role + '_lost_or_identity_changed'
        except psutil.Error:
            return 'owned_' + role + '_unavailable'
    return None


def capture_owned_telemetry(root):
    launch = json.loads((root / 'run/launch_pid.json').read_text())
    parent = psutil.Process(launch['pid'])
    assert abs(parent.create_time() - launch['created']) < .02
    source = str(root / 'inputs/robot_joint_state_relay_05ms.py')
    relays = [process for process in parent.children(recursive=True) if source in process.cmdline()]
    assert len(relays) == 1
    server = json.loads((root / 'run/dart_cache_runtime_identity.json').read_text())
    processes = dict(relay=identity(relays[0]), server=identity(psutil.Process(server['pid'])))
    assert abs(processes['server']['created'] - server['created']) < .02
    for row in processes.values():
        environment = psutil.Process(row['pid']).environ()
        assert environment['ROS_DOMAIN_ID'] == '167' and environment['GZ_PARTITION'] == 'h1_seq_full_r359'
        assert environment['GZ_TRANSPORT_RCVHWM'] == environment['GZ_TRANSPORT_SNDHWM'] == '20000'
    record = dict(status='EXACT_OWNED_TELEMETRY_BOUND_BEFORE_EXECUTOR', processes=processes,
        guard_poll_wall_s=1, source_step_and_clock_guards_unchanged=True, finite_transport_queue=20000)
    assert owned_telemetry_fault(record) is None
    with (root / 'run/owned_telemetry.json').open('x') as stream:
        json.dump(record, stream, indent=2)

