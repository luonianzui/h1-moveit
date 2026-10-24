#!/usr/bin/python3
"""Fail-closed simulator-time relay for candidate-3 Gazebo evidence.

The Gazebo JointStatePublisher is the only source of robot telemetry.  It
publishes one exact sample per 0.5 ms physics step, in Gazebo entity order.  This
relay verifies that source without interpolation, reorders all four joint
arrays into the certificate's fixed order, and publishes only samples whose
original simulator stamp lies on a 10 ms boundary.  It also owns the public
``/clock`` publisher so every public joint sample is causally preceded by the
clock value that covers it.

No controller, Gazebo command, service, or action interface is created here.
Any duplicate, skipped, reset, malformed, or non-finite source sample raises a
fatal error instead of being hidden from the evidence run.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import math
import signal
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence, Tuple


SOURCE_JOINT_TOPIC = "/h1_2/native_joint_states_829"
SOURCE_CLOCK_TOPIC = "/h1_2/native_clock_829"
PUBLIC_JOINT_TOPIC = "/joint_states"
PUBLIC_CLOCK_TOPIC = "/clock"
SOURCE_STEP_NS = 500_000
OUTPUT_PERIOD_NS = 10_000_000
MAX_PENDING_SAMPLES = 2_000

LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
RIGHT_ARM_JOINTS = tuple(
    name.replace("left_", "right_", 1) for name in LEFT_ARM_JOINTS)
LEFT_GRIPPER_JOINTS = (
    "left_dex1_finger_1_joint", "left_dex1_finger_2_joint")
RIGHT_GRIPPER_JOINTS = (
    "right_dex1_finger_1_joint", "right_dex1_finger_2_joint")
LEG_JOINTS = (
    "left_hip_yaw_joint", "left_hip_pitch_joint",
    "left_hip_roll_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_hip_roll_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
)
CANONICAL_JOINTS = (
    "torso_joint", *LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS,
    *LEFT_GRIPPER_JOINTS, *RIGHT_GRIPPER_JOINTS, *LEG_JOINTS,
)
CANONICAL_JOINT_SET = frozenset(CANONICAL_JOINTS)


class RelayError(RuntimeError):
    """A source invariant failed; publishing must stop immediately."""


@dataclass(frozen=True)
class JointSnapshot:
    stamp_ns: int
    frame_id: str
    positions: Tuple[float, ...]
    velocities: Tuple[float, ...]
    efforts: Tuple[float, ...]


def _stamp_ns(sec: int, nanosec: int, status: str) -> int:
    if (type(sec) is not int or type(nanosec) is not int or sec < 0 or
            not 0 <= nanosec < 1_000_000_000):
        raise RelayError(f"{status}_STAMP_FIELDS")
    value = sec * 1_000_000_000 + nanosec
    if value % SOURCE_STEP_NS != 0:
        raise RelayError(f"{status}_STAMP_NOT_05MS_GRID:{value}")
    return value


def _finite_vector(
    values: Sequence[float], expected_size: int, status: str,
) -> Tuple[float, ...]:
    if len(values) != expected_size:
        raise RelayError(f"{status}_SIZE:{len(values)}")
    normalized = []
    for index, value in enumerate(values):
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(float(value))):
            raise RelayError(f"{status}_FINITE:{index}")
        normalized.append(float(value))
    return tuple(normalized)


class RelayCore:
    """ROS-free validation and exact-sample scheduling state machine."""

    def __init__(self) -> None:
        self.last_source_stamp_ns: int | None = None
        self.last_clock_stamp_ns: int | None = None
        self.last_snapshot: JointSnapshot | None = None
        self.pending: list[JointSnapshot] = []
        self.source_samples = 0
        self.clock_samples = 0
        self.published_samples = 0

    @staticmethod
    def _strict_step(previous: int | None, current: int, status: str) -> None:
        if previous is not None and current - previous != SOURCE_STEP_NS:
            raise RelayError(
                f"{status}_STEP:{previous}:{current}:"
                f"{current - previous}")

    def accept_joint(
        self, *, stamp_ns: int, frame_id: str, names: Sequence[str],
        positions: Sequence[float], velocities: Sequence[float],
        efforts: Sequence[float],
    ) -> Tuple[JointSnapshot, ...]:
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise RelayError("JOINT_STAMP")
        if stamp_ns % SOURCE_STEP_NS != 0:
            raise RelayError(f"JOINT_STAMP_NOT_05MS_GRID:{stamp_ns}")
        if type(frame_id) is not str or "\x00" in frame_id:
            raise RelayError("JOINT_FRAME_ID")
        source_names = tuple(names)
        if (len(source_names) != len(CANONICAL_JOINTS) or
                any(type(name) is not str or not name for name in source_names) or
                len(set(source_names)) != len(source_names) or
                frozenset(source_names) != CANONICAL_JOINT_SET):
            raise RelayError("JOINT_NAME_SET")
        count = len(source_names)
        source_positions = _finite_vector(positions, count, "JOINT_POSITION")
        source_velocities = _finite_vector(velocities, count, "JOINT_VELOCITY")
        source_efforts = _finite_vector(efforts, count, "JOINT_EFFORT")
        self._strict_step(self.last_source_stamp_ns, stamp_ns, "JOINT_SOURCE")
        index = {name: offset for offset, name in enumerate(source_names)}
        snapshot = JointSnapshot(
            stamp_ns=stamp_ns,
            frame_id=frame_id,
            positions=tuple(source_positions[index[name]]
                            for name in CANONICAL_JOINTS),
            velocities=tuple(source_velocities[index[name]]
                             for name in CANONICAL_JOINTS),
            efforts=tuple(source_efforts[index[name]]
                          for name in CANONICAL_JOINTS),
        )
        self.last_source_stamp_ns = stamp_ns
        self.last_snapshot = snapshot
        self.source_samples += 1
        if stamp_ns % OUTPUT_PERIOD_NS == 0:
            self.pending.append(snapshot)
            if len(self.pending) > MAX_PENDING_SAMPLES:
                raise RelayError("JOINT_CLOCK_BACKLOG")
        return self._flush()

    def accept_clock(self, stamp_ns: int) -> Tuple[JointSnapshot, ...]:
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise RelayError("CLOCK_STAMP")
        if stamp_ns % SOURCE_STEP_NS != 0:
            raise RelayError(f"CLOCK_STAMP_NOT_05MS_GRID:{stamp_ns}")
        self._strict_step(self.last_clock_stamp_ns, stamp_ns, "CLOCK_SOURCE")
        self.last_clock_stamp_ns = stamp_ns
        self.clock_samples += 1
        return self._flush()

    def _flush(self) -> Tuple[JointSnapshot, ...]:
        if self.last_clock_stamp_ns is None:
            return ()
        ready_count = 0
        for snapshot in self.pending:
            if snapshot.stamp_ns > self.last_clock_stamp_ns:
                break
            ready_count += 1
        if ready_count == 0:
            return ()
        ready = tuple(self.pending[:ready_count])
        del self.pending[:ready_count]
        self.published_samples += len(ready)
        return ready


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-sha256", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    source = Path(__file__).resolve()
    actual_sha = _file_sha256(source)
    if not hmac.compare_digest(args.self_sha256, actual_sha):
        print("H1_ROBOT_JOINT_STATE_RELAY_829=FAIL:SELF_SHA256",
              file=sys.stderr, flush=True)
        return 1

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState

    source_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20_000,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    joint_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1_000,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    clock_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=20_000,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    class RelayNode(Node):
        def __init__(self) -> None:
            super().__init__("h1_robot_joint_state_relay_829")
            self.core = RelayCore()
            self.joint_publisher = self.create_publisher(
                JointState, PUBLIC_JOINT_TOPIC, joint_qos)
            self.clock_publisher = self.create_publisher(
                Clock, PUBLIC_CLOCK_TOPIC, clock_qos)
            self.create_subscription(
                JointState, SOURCE_JOINT_TOPIC, self.on_joint, source_qos)
            self.create_subscription(
                Clock, SOURCE_CLOCK_TOPIC, self.on_clock, source_qos)

        def publish_joint(self, snapshot: JointSnapshot) -> None:
            message = JointState()
            message.header.stamp.sec = snapshot.stamp_ns // 1_000_000_000
            message.header.stamp.nanosec = snapshot.stamp_ns % 1_000_000_000
            message.header.frame_id = snapshot.frame_id
            message.name = list(CANONICAL_JOINTS)
            message.position = list(snapshot.positions)
            message.velocity = list(snapshot.velocities)
            message.effort = list(snapshot.efforts)
            self.joint_publisher.publish(message)

        def on_joint(self, message: JointState) -> None:
            stamp = _stamp_ns(
                message.header.stamp.sec, message.header.stamp.nanosec,
                "JOINT")
            ready = self.core.accept_joint(
                stamp_ns=stamp, frame_id=message.header.frame_id,
                names=message.name, positions=message.position,
                velocities=message.velocity, efforts=message.effort)
            for snapshot in ready:
                self.publish_joint(snapshot)

        def on_clock(self, message: Clock) -> None:
            stamp = _stamp_ns(
                message.clock.sec, message.clock.nanosec, "CLOCK")
            ready = self.core.accept_clock(stamp)
            self.clock_publisher.publish(message)
            for snapshot in ready:
                self.publish_joint(snapshot)

    rclpy.init(args=None)
    node = RelayNode()
    print(
        "H1_ROBOT_JOINT_STATE_RELAY_829=READY:"
        f"sha256={actual_sha}:source_step_ns={SOURCE_STEP_NS}:"
        f"output_period_ns={OUTPUT_PERIOD_NS}", flush=True)
    try:
        rclpy.spin(node)
    except (RelayError, KeyboardInterrupt) as error:
        if isinstance(error, RelayError):
            print(f"H1_ROBOT_JOINT_STATE_RELAY_829=FAIL:{error}",
                  file=sys.stderr, flush=True)
            return 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print(
            "H1_ROBOT_JOINT_STATE_RELAY_829=STOP:"
            f"source={node.core.source_samples}:"
            f"clock={node.core.clock_samples}:"
            f"published={node.core.published_samples}:"
            f"pending={len(node.core.pending)}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
