#!/usr/bin/python3
"""Print one strictly validated ROS joint-state snapshot with low startup cost."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("robot", "incubator"))
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 0.1 <= args.timeout <= 10.0:
        parser.error("--timeout must be finite and in [0.1, 10.0]")
    return args


def main() -> int:
    args = parse_args()
    topic = "/joint_states" if args.mode == "robot" else "/incubator/joint_states"
    rclpy.init(args=None)
    node = rclpy.create_node(f"h1_snapshot_8251_{args.mode}_{os.getpid()}")
    received: list[JointState] = []
    subscription = node.create_subscription(
        JointState, topic, lambda message: received.append(message), qos_profile_sensor_data
    )
    del subscription
    deadline = time.monotonic() + args.timeout
    try:
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        if not received:
            raise RuntimeError(f"no fresh {topic} sample")
        message = received[-1]
        names = list(message.name)
        positions = list(message.position)
        velocities = list(message.velocity)
        if not names or len(names) != len(positions) or len(set(names)) != len(names):
            raise RuntimeError(f"malformed {topic} name/position arrays")
        if args.mode == "incubator" and len(velocities) != len(names):
            raise RuntimeError(f"malformed {topic} velocity array")
        for index, name in enumerate(names):
            if not name or any(character.isspace() for character in name):
                raise RuntimeError(f"unsafe joint name in {topic}")
            values = [positions[index]]
            if args.mode == "incubator":
                values.append(velocities[index])
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError(f"non-finite joint telemetry in {topic}")
            print(name, *(format(value, ".17g") for value in values))
    except RuntimeError as error:
        print(f"h1_ros_snapshot_8251: ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
