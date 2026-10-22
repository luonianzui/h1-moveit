#!/usr/bin/env python3
"""Finish the already-supported q60 cube staging run from a sealed position cache.

This deliberately narrow recovery does not plan, reset, teleport, or command the
incubator/right arm.  It is admitted only while the left arm is at the cached
rack-support endpoint and the green cube is stationary in the middle-rack
region.  It detaches the Gazebo pose support, opens the left fingers, and sends
the cache's prevalidated retreat path to the left-arm trajectory controller.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


EXPECTED_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]
ADMITTED_CACHES = {
    # Original 70-degree rack recovery retained for compatibility.
    "37fbd0a66c274079ef8ec04254be545cc8ec7f800a99588f4f85fcdf016cec66": {
        "retreat_points": 299,
        "retreat_boundary": 20,
        "admission_tolerance": 0.022,
    },
    # Candidate-3 826.2 parallel-90 rack cache used by the current sealed plan.
    "dfc45b9ab0ad12e6a6e038ed0a2a3300838ec143ad8dd4c591b423273a352a02": {
        "retreat_points": 413,
        "retreat_boundary": 22,
        "admission_tolerance": 0.003,
    },
}
CUBE_DETACH_TOPIC = "/h1_2/left_pose_grasp/detach"


class RecoveryError(RuntimeError):
    pass


def duration(seconds: float):
    from builtin_interfaces.msg import Duration

    whole = int(math.floor(seconds))
    nanos = int(round((seconds - whole) * 1_000_000_000))
    if nanos == 1_000_000_000:
        whole += 1
        nanos = 0
    return Duration(sec=whole, nanosec=nanos)


def parse_retreat(cache_path: Path) -> tuple[list[list[float]], str, float]:
    raw = cache_path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    specification = ADMITTED_CACHES.get(digest)
    if specification is None:
        raise RecoveryError(
            f"cache SHA-256 is not admitted: {digest}"
        )
    lines = raw.decode("utf-8").splitlines()
    if not lines or lines[0] != "H1_LEFT_RACK_POSITION_CACHE_V1":
        raise RecoveryError("cache magic is wrong")
    if len(lines) < 3 or lines[1] != "joint_count 7":
        raise RecoveryError("cache joint count is wrong")
    if lines[2].split()[1:] != EXPECTED_JOINTS:
        raise RecoveryError("cache joint order is wrong")

    stage_index = next(
        (index for index, line in enumerate(lines) if line.startswith("stage retreat ")),
        None,
    )
    if stage_index is None:
        raise RecoveryError("cache has no retreat stage")
    stage_fields = lines[stage_index].split()
    expected_count = int(specification["retreat_points"])
    expected_boundary = int(specification["retreat_boundary"])
    if stage_fields != [
            "stage", "retreat", str(expected_count), str(expected_boundary)]:
        raise RecoveryError("retreat stage declaration is not the admitted path")
    if lines[stage_index + 4] != "points":
        raise RecoveryError("retreat points marker is missing")
    point_lines = lines[stage_index + 5 : stage_index + 5 + expected_count]
    if len(point_lines) != expected_count:
        raise RecoveryError("retreat path is truncated")
    points: list[list[float]] = []
    for row_index, line in enumerate(point_lines):
        fields = line.split()
        if len(fields) != 7:
            raise RecoveryError(f"retreat row {row_index} has the wrong width")
        point = [float(value) for value in fields]
        if not all(math.isfinite(value) for value in point):
            raise RecoveryError(f"retreat row {row_index} is non-finite")
        points.append(point)
    declared_start = [float(value) for value in lines[stage_index + 2].split()[1:]]
    declared_finish = [float(value) for value in lines[stage_index + 3].split()[1:]]
    if points[0] != declared_start or points[-1] != declared_finish:
        raise RecoveryError("retreat endpoints disagree with the cache declaration")
    return points, digest, float(specification["admission_tolerance"])


POSE_RE = re.compile(
    r"\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]"
)


def read_cube_pose() -> tuple[float, float, float]:
    result = subprocess.run(
        ["gz", "model", "-m", "green_cube", "-p"],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=8,
    )
    if result.returncode != 0:
        raise RecoveryError("could not read the live green_cube pose")
    matches = POSE_RE.findall(result.stdout)
    if len(matches) < 2:
        raise RecoveryError("green_cube pose output is malformed")
    xyz = tuple(float(value) for value in matches[-2])
    if not all(math.isfinite(value) for value in xyz):
        raise RecoveryError("green_cube position is non-finite")
    return xyz


def verify_cube_region_and_stability() -> tuple[float, float, float]:
    first = read_cube_pose()
    time.sleep(0.35)
    second = read_cube_pose()
    displacement = math.dist(first, second)
    x, y, z = second
    if not (0.57 <= x <= 0.66 and -0.09 <= y <= 0.03 and 0.975 <= z <= 1.025):
        raise RecoveryError(
            "green_cube is not in the relaxed middle-rack staging region: "
            f"xyz={second}"
        )
    if displacement > 0.003:
        raise RecoveryError(
            f"green_cube is not stationary before detach: displacement={displacement:.6f} m"
        )
    return second


class JointMonitor(Node):
    def __init__(self) -> None:
        super().__init__("h1_left_rack_release_retreat_recovery")
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_state, 20)
        self.arm_publisher = self.create_publisher(
            JointTrajectory, "/left_arm_controller/joint_trajectory", 10
        )
        self.gripper_publisher = self.create_publisher(
            JointTrajectory, "/left_gripper_controller/joint_trajectory", 10
        )

    def _on_state(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        with self._lock:
            self._positions = dict(zip(message.name, message.position))

    def positions(self, joints: list[str]) -> list[float] | None:
        with self._lock:
            if not all(joint in self._positions for joint in joints):
                return None
            return [self._positions[joint] for joint in joints]


def wait_for_telemetry(node: JointMonitor, timeout_seconds: float = 8.0) -> None:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if node.positions(EXPECTED_JOINTS) is not None:
            return
        time.sleep(0.05)
    raise RecoveryError("fresh /joint_states telemetry was not received")


def endpoint_gate(
    node: JointMonitor,
    joints: list[str],
    target: list[float],
    tolerance: float,
    timeout_seconds: float,
    label: str,
) -> float:
    deadline = time.monotonic() + timeout_seconds
    last_error = math.inf
    stable_since: float | None = None
    while time.monotonic() < deadline:
        actual = node.positions(joints)
        if actual is not None:
            last_error = max(abs(a - b) for a, b in zip(actual, target))
            if last_error <= tolerance:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 0.30:
                    return last_error
            else:
                stable_since = None
        time.sleep(0.03)
    raise RecoveryError(
        f"{label} endpoint was not reached: max_joint_error={last_error:.6f} rad"
    )


def wait_for_subscribers(node: JointMonitor) -> None:
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if (
            node.arm_publisher.get_subscription_count() > 0
            and node.gripper_publisher.get_subscription_count() > 0
        ):
            return
        time.sleep(0.05)
    raise RecoveryError("left controller trajectory subscribers are unavailable")


def detach_cube_support() -> None:
    state_listener = subprocess.Popen(
        ["timeout", "8", "gz", "topic", "-e", "-t", "/h1_2/left_pose_grasp/state"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    try:
        time.sleep(0.35)
        result = subprocess.run(
            [
                "gz",
                "topic",
                "-t",
                CUBE_DETACH_TOPIC,
                "-m",
                "gz.msgs.Empty",
                "-p",
                "",
            ],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=8,
        )
        listener_output, _unused = state_listener.communicate(timeout=9)
    except BaseException:
        state_listener.terminate()
        try:
            state_listener.wait(timeout=2)
        except subprocess.TimeoutExpired:
            state_listener.kill()
        raise
    if result.returncode != 0:
        raise RecoveryError(f"Gazebo cube detach command failed: {result.stdout.strip()}")
    if not re.search(r'^\s*data:\s*["\']?detached["\']?\s*$', listener_output, re.MULTILINE):
        raise RecoveryError("Gazebo cube support did not confirm a fresh detached state")


def publish_gripper_open(node: JointMonitor) -> None:
    joints = ["left_dex1_finger_1_joint", "left_dex1_finger_2_joint"]
    current = node.positions(joints)
    if current is None:
        raise RecoveryError("left-finger telemetry is unavailable")
    message = JointTrajectory()
    message.joint_names = joints
    start = JointTrajectoryPoint()
    start.positions = current
    start.time_from_start = duration(0.05)
    finish = JointTrajectoryPoint()
    finish.positions = [-0.019, -0.019]
    finish.time_from_start = duration(1.0)
    message.points = [start, finish]
    node.gripper_publisher.publish(message)


def publish_retreat(node: JointMonitor, points: list[list[float]]) -> None:
    message = JointTrajectory()
    message.joint_names = EXPECTED_JOINTS
    for index, positions in enumerate(points):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = duration(0.10 * (index + 1))
        message.points.append(point)
    node.arm_publisher.publish(message)


def run(cache_path: Path, execute: bool, detach_only: bool) -> None:
    points, cache_sha256, admission_tolerance = parse_retreat(cache_path)
    print(
        "RECOVERY_CACHE=PASS "
        f"sha256={cache_sha256} retreat_points={len(points)}",
        flush=True,
    )
    if not execute:
        print("RECOVERY_DRY_RUN=PASS", flush=True)
        return

    cube_pose = verify_cube_region_and_stability()
    print(f"RECOVERY_CUBE_STAGING=PASS xyz={cube_pose}", flush=True)
    rclpy.init()
    node = JointMonitor()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    try:
        wait_for_telemetry(node)
        wait_for_subscribers(node)
        current = node.positions(EXPECTED_JOINTS)
        if current is None:
            raise RecoveryError("left-arm telemetry disappeared")
        admission_error = max(abs(a - b) for a, b in zip(current, points[0]))
        if admission_error > admission_tolerance:
            raise RecoveryError(
                "left arm is not at the cached rack-support endpoint: "
                f"max_joint_error={admission_error:.6f} rad"
            )
        print(
            "RECOVERY_LEFT_SUPPORT_ENDPOINT=PASS "
            f"max_joint_error={admission_error:.6f} rad",
            flush=True,
        )

        started = time.monotonic()
        detach_cube_support()
        print("RECOVERY_GAZEBO_DETACH=PASS", flush=True)
        if detach_only:
            final_cube_pose = verify_cube_region_and_stability()
            print(
                "RECOVERY_DETACH_ONLY=PASS "
                f"gripper=clamped xyz={final_cube_pose}",
                flush=True,
            )
            return
        time.sleep(0.35)
        publish_gripper_open(node)
        gripper_error = endpoint_gate(
            node,
            ["left_dex1_finger_1_joint", "left_dex1_finger_2_joint"],
            [-0.019, -0.019],
            0.003,
            10.0,
            "left gripper open",
        )
        print(
            f"RECOVERY_LEFT_GRIPPER_OPEN=PASS max_joint_error={gripper_error:.6f} rad",
            flush=True,
        )
        publish_retreat(node, points)
        retreat_error = endpoint_gate(
            node,
            EXPECTED_JOINTS,
            points[-1],
            0.006,
            55.0,
            "left retreat",
        )
        final_cube_pose = verify_cube_region_and_stability()
        elapsed = time.monotonic() - started
        print(
            "RECOVERY_LEFT_RETREAT=PASS "
            f"max_joint_error={retreat_error:.6f} rad wall_seconds={elapsed:.3f}",
            flush=True,
        )
        print(f"RECOVERY_FINAL_CUBE_STABLE=PASS xyz={final_cube_pose}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=2.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("cache", type=Path)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--detach-only", action="store_true")
    args = parser.parse_args()
    if not args.cache.is_absolute() or args.cache.is_symlink() or not args.cache.is_file():
        raise RecoveryError("cache must be an absolute, non-symlink regular file")
    if args.detach_only and not args.execute:
        raise RecoveryError("--detach-only requires --execute")
    run(args.cache, args.execute, args.detach_only)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RecoveryError, OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"h1_left_rack_release_retreat_recovery: ERROR: {error}", file=sys.stderr)
        sys.exit(1)
