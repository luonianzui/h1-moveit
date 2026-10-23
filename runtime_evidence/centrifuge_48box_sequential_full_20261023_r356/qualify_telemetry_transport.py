import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from inputs.robot_joint_state_relay_05ms import CANONICAL_JOINTS, RelayCore


ROOT = Path(__file__).resolve().parent
COUNT = 24000
PERIOD = .0005


def qos():
    return QoSProfile(depth=20000, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def child(role, mode):
    rclpy.init()
    node = rclpy.create_node('r353_transport_' + role)
    started = time.monotonic()
    stem = '/r353_offline_transport_' + mode
    core = RelayCore()
    try:
        if role == 'subscriber':
            def joint_received(message):
                stamp = message.header.stamp
                core.accept_joint(stamp_ns=stamp.sec * 1000000000 + stamp.nanosec,
                                  frame_id=message.header.frame_id, names=message.name,
                                  positions=message.position, velocities=message.velocity, efforts=message.effort)

            def clock_received(message):
                core.accept_clock(message.clock.sec * 1000000000 + message.clock.nanosec)

            node.create_subscription(JointState, stem + '/joints', joint_received, qos())
            node.create_subscription(Clock, stem + '/clock', clock_received, qos())
            with (ROOT / ('transport_' + mode + '_ready.json')).open('x') as stream:
                json.dump(dict(ready=True, pid=os.getpid()), stream)
            while core.source_samples < COUNT or core.clock_samples < COUNT:
                assert time.monotonic() - started < 50, (core.source_samples, core.clock_samples)
                rclpy.spin_once(node, timeout_sec=.05)
            assert core.published_samples == COUNT // 20 and not core.pending
            assert core.last_source_stamp_ns == core.last_clock_stamp_ns == COUNT * 500000
            result = dict(mode=mode, source_samples=core.source_samples, clock_samples=core.clock_samples,
                          exact_output_samples=core.published_samples, skipped_or_duplicate_samples=0,
                          last_stamp_ns=core.last_clock_stamp_ns, wall_seconds=time.monotonic() - started)
            with (ROOT / ('transport_' + mode + '_result.json')).open('x') as stream:
                json.dump(result, stream, indent=2)
        else:
            joint_publisher = node.create_publisher(JointState, stem + '/joints', qos())
            clock_publisher = node.create_publisher(Clock, stem + '/clock', qos())
            while joint_publisher.get_subscription_count() < 1 or clock_publisher.get_subscription_count() < 1:
                assert time.monotonic() - started < 10
                rclpy.spin_once(node, timeout_sec=.02)
            message = JointState()
            message.name = list(CANONICAL_JOINTS)
            message.position = [float(index) / 100 for index in range(len(CANONICAL_JOINTS))]
            message.velocity = [0.] * len(CANONICAL_JOINTS)
            message.effort = [1.] * len(CANONICAL_JOINTS)
            clock = Clock()
            began = time.monotonic()
            for index in range(1, COUNT + 1):
                stamp_ns = index * 500000
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
                clock.clock = message.header.stamp
                clock_publisher.publish(clock)
                joint_publisher.publish(message)
                remaining = began + index * PERIOD - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
            deadline = time.monotonic() + 10
            while not (ROOT / ('transport_' + mode + '_result.json')).exists():
                assert time.monotonic() < deadline
                rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '164'
    records = []
    for mode in ('DEFAULT', 'UDPv4'):
        environment = dict(os.environ, FASTDDS_BUILTIN_TRANSPORTS=mode, RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
        with (ROOT / ('transport_' + mode + '_subscriber.log')).open('x') as output:
            subscriber = subprocess.Popen([sys.executable, __file__, 'subscriber', mode],
                                          env=environment, stdout=output, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 10
                while not (ROOT / ('transport_' + mode + '_ready.json')).exists():
                    assert subscriber.poll() is None and time.monotonic() < deadline
                    time.sleep(.02)
                with (ROOT / ('transport_' + mode + '_publisher.log')).open('x') as publish_output:
                    subprocess.run([sys.executable, __file__, 'publisher', mode], env=environment,
                                   stdout=publish_output, stderr=subprocess.STDOUT, timeout=50, check=True)
                assert subscriber.wait(timeout=10) == 0
            finally:
                if subscriber.poll() is None:
                    subscriber.terminate()
                    subscriber.wait(timeout=10)
        records.append(json.loads((ROOT / ('transport_' + mode + '_result.json')).read_text()))
    result = dict(status='PASS_TWO_PROCESS_DDS_STRICT_SAMPLES_NOT_GAZEBO_PROOF', records=records,
                  expected_source_step_ns=500000, relaxed_guards=False,
                  unchanged_relay_sha256=hashlib.sha256((ROOT / 'inputs/robot_joint_state_relay_05ms.py').read_bytes()).hexdigest(),
                  loss_layer_not_proven=True, no_robot_commands_or_public_clock_published=True)
    with (ROOT / 'telemetry_transport_qualification.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    if len(sys.argv) == 3:
        child(*sys.argv[1:])
    else:
        main()
