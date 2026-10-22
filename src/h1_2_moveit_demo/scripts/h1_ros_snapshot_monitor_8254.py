#!/usr/bin/python3
"""Persist validated read-only joint snapshots without publishing ROS data."""

from __future__ import annotations

import argparse
import fcntl
import math
import os
from pathlib import Path
import re
import stat
import sys
import time
from typing import Final

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


CACHE_MAGIC: Final = "H1_ROS_SNAPSHOT_CACHE_8254"
READY_MAGIC: Final = "H1_ROS_SNAPSHOT_MONITOR_8254"
FORMAT_VERSION: Final = "v1"
ROBOT_TOPIC: Final = "/joint_states"
INCUBATOR_TOPIC: Final = "/incubator/joint_states"
ROBOT_CACHE: Final = "robot.cache"
INCUBATOR_CACHE: Final = "incubator.cache"
READY_FILE: Final = "ready"
LOCK_FILE: Final = ".monitor.lock"
SAFE_JOINT_NAME = re.compile(r"[!-~]+\Z")


class SnapshotMonitorError(RuntimeError):
    """A fail-closed monitor setup, validation, or persistence error."""


def default_cache_dir() -> Path:
    override = os.environ.get("H1_ROS_SNAPSHOT_CACHE_DIR")
    if override:
        return Path(override)
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    return Path(runtime_dir) / "h1_ros_snapshot_8254"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache-dir", type=Path, default=default_cache_dir())
    parser.add_argument("--minimum-write-period", type=float, default=0.05)
    args = parser.parse_args()
    if not args.cache_dir.is_absolute():
        parser.error("--cache-dir must be absolute")
    if (
        not math.isfinite(args.minimum_write_period)
        or not 0.01 <= args.minimum_write_period <= 1.0
    ):
        parser.error("--minimum-write-period must be finite and in [0.01, 1.0]")
    return args


def open_cache_directory(path: Path) -> int:
    try:
        path.mkdir(mode=0o700, parents=False, exist_ok=True)
    except OSError as error:
        raise SnapshotMonitorError(f"cannot create cache directory {path}: {error}") from error
    try:
        metadata = path.lstat()
    except OSError as error:
        raise SnapshotMonitorError(f"cannot inspect cache directory {path}: {error}") from error
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or stat.S_ISLNK(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or stat.S_IMODE(metadata.st_mode) != 0o700
    ):
        raise SnapshotMonitorError(
            f"cache directory must be an owned, non-symlink mode-0700 directory: {path}"
        )
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        directory_fd = os.open(path, flags)
    except OSError as error:
        raise SnapshotMonitorError(f"cannot securely open cache directory {path}: {error}") from error
    opened = os.fstat(directory_fd)
    if (
        opened.st_dev != metadata.st_dev
        or opened.st_ino != metadata.st_ino
        or opened.st_uid != os.getuid()
        or stat.S_IMODE(opened.st_mode) != 0o700
    ):
        os.close(directory_fd)
        raise SnapshotMonitorError("cache directory changed during secure open")
    return directory_fd


def lock_monitor(directory_fd: int) -> int:
    flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        lock_fd = os.open(LOCK_FILE, flags, 0o600, dir_fd=directory_fd)
        metadata = os.fstat(lock_fd)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != os.getuid()
            or stat.S_IMODE(metadata.st_mode) != 0o600
            or metadata.st_nlink != 1
        ):
            raise SnapshotMonitorError("monitor lock must be an owned mode-0600 regular file")
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        os.close(lock_fd)
        raise SnapshotMonitorError("another snapshot monitor owns the cache directory") from error
    except OSError as error:
        if "lock_fd" in locals():
            os.close(lock_fd)
        raise SnapshotMonitorError(f"cannot acquire monitor lock: {error}") from error
    return lock_fd


def atomic_write(directory_fd: int, name: str, content: str) -> None:
    if not content.endswith("\n") or "\x00" in content:
        raise SnapshotMonitorError(f"invalid atomic cache payload for {name}")
    temporary_name = f".{name}.tmp.{os.getpid()}.{time.monotonic_ns()}"
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    file_fd = -1
    try:
        file_fd = os.open(temporary_name, flags, 0o400, dir_fd=directory_fd)
        os.fchmod(file_fd, 0o400)
        payload = content.encode("ascii", errors="strict")
        written = 0
        while written < len(payload):
            count = os.write(file_fd, payload[written:])
            if count <= 0:
                raise SnapshotMonitorError(f"short write while updating {name}")
            written += count
        os.fsync(file_fd)
        os.close(file_fd)
        file_fd = -1
        os.replace(
            temporary_name,
            name,
            src_dir_fd=directory_fd,
            dst_dir_fd=directory_fd,
        )
        os.fsync(directory_fd)
    except (OSError, UnicodeError) as error:
        raise SnapshotMonitorError(f"cannot atomically update {name}: {error}") from error
    finally:
        if file_fd >= 0:
            os.close(file_fd)
        try:
            os.unlink(temporary_name, dir_fd=directory_fd)
        except FileNotFoundError:
            pass


def serialize_snapshot(
    mode: str, message: JointState, monitor_pid: int, received_monotonic_ns: int
) -> str:
    if mode not in ("robot", "incubator"):
        raise SnapshotMonitorError(f"unsupported snapshot mode: {mode}")
    names = list(message.name)
    positions = list(message.position)
    velocities = list(message.velocity)
    if not names or len(names) != len(positions) or len(set(names)) != len(names):
        raise SnapshotMonitorError(f"malformed {mode} name/position arrays")
    if len(names) > 512:
        raise SnapshotMonitorError(f"{mode} snapshot has too many joints")
    if mode == "incubator" and len(velocities) != len(names):
        raise SnapshotMonitorError("malformed incubator velocity array")
    rows: list[str] = []
    for index, name in enumerate(names):
        if (
            not name
            or not name.isascii()
            or not SAFE_JOINT_NAME.fullmatch(name)
            or any(character.isspace() for character in name)
        ):
            raise SnapshotMonitorError(f"unsafe joint name in {mode} snapshot")
        values = [float(positions[index])]
        if mode == "incubator":
            values.append(float(velocities[index]))
        if not all(math.isfinite(value) for value in values):
            raise SnapshotMonitorError(f"non-finite telemetry in {mode} snapshot")
        rows.append(" ".join([name, *(format(value, ".17g") for value in values)]))
    header = " ".join(
        (
            CACHE_MAGIC,
            FORMAT_VERSION,
            mode,
            str(monitor_pid),
            str(received_monotonic_ns),
            str(len(rows)),
        )
    )
    return "\n".join((header, *rows, ""))


class SnapshotMonitor(Node):
    def __init__(self, directory_fd: int, minimum_write_period: float) -> None:
        super().__init__(
            f"h1_ros_snapshot_monitor_8254_{os.getpid()}",
            enable_rosout=False,
            start_parameter_services=False,
            enable_logger_service=False,
        )
        # rclpy creates /parameter_events internally for every Node.  This
        # monitor has no parameters and must have zero publishers, so remove
        # that automatic publisher before creating the two subscriptions.
        parameter_publisher = self._parameter_event_publisher
        if parameter_publisher is not None:
            self.destroy_publisher(parameter_publisher)
            self._parameter_event_publisher = None
        if tuple(self.publishers):
            raise SnapshotMonitorError("snapshot monitor unexpectedly owns a publisher")
        self._directory_fd = directory_fd
        self._minimum_period_ns = int(round(minimum_write_period * 1_000_000_000.0))
        self._last_write_ns = {"robot": 0, "incubator": 0}
        self._seen: set[str] = set()
        self.failure: SnapshotMonitorError | None = None
        self._snapshot_subscriptions = (
            self.create_subscription(
                JointState,
                ROBOT_TOPIC,
                lambda message: self._snapshot_callback("robot", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                JointState,
                INCUBATOR_TOPIC,
                lambda message: self._snapshot_callback("incubator", message),
                qos_profile_sensor_data,
            ),
        )
        atomic_write(
            self._directory_fd,
            READY_FILE,
            f"{READY_MAGIC} {FORMAT_VERSION} STARTING {os.getpid()} {time.monotonic_ns()}\n",
        )

    def _snapshot_callback(self, mode: str, message: JointState) -> None:
        received_ns = time.monotonic_ns()
        if (
            self._last_write_ns[mode] != 0
            and received_ns - self._last_write_ns[mode] < self._minimum_period_ns
        ):
            return
        try:
            content = serialize_snapshot(mode, message, os.getpid(), received_ns)
            cache_name = ROBOT_CACHE if mode == "robot" else INCUBATOR_CACHE
            atomic_write(self._directory_fd, cache_name, content)
            self._last_write_ns[mode] = received_ns
            self._seen.add(mode)
            if self._seen == {"robot", "incubator"}:
                atomic_write(
                    self._directory_fd,
                    READY_FILE,
                    f"{READY_MAGIC} {FORMAT_VERSION} READY {os.getpid()} {time.monotonic_ns()}\n",
                )
        except (SnapshotMonitorError, ValueError, OverflowError) as error:
            self.failure = SnapshotMonitorError(str(error))
            self.get_logger().error(str(error))
            self.context.try_shutdown()

    def mark_stopped(self) -> None:
        try:
            atomic_write(
                self._directory_fd,
                READY_FILE,
                f"{READY_MAGIC} {FORMAT_VERSION} STOPPED {os.getpid()} {time.monotonic_ns()}\n",
            )
        except SnapshotMonitorError as error:
            print(f"h1_ros_snapshot_monitor_8254: ERROR: {error}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    directory_fd = -1
    lock_fd = -1
    node: SnapshotMonitor | None = None
    initialized = False
    try:
        directory_fd = open_cache_directory(args.cache_dir)
        lock_fd = lock_monitor(directory_fd)
        rclpy.init(args=None)
        initialized = True
        node = SnapshotMonitor(directory_fd, args.minimum_write_period)
        rclpy.spin(node)
        if node.failure is not None:
            raise node.failure
    except KeyboardInterrupt:
        pass
    except (SnapshotMonitorError, OSError) as error:
        print(f"h1_ros_snapshot_monitor_8254: ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.mark_stopped()
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()
        if lock_fd >= 0:
            os.close(lock_fd)
        if directory_fd >= 0:
            os.close(directory_fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
