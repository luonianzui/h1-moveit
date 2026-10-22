#!/usr/bin/python3
"""Persistent fail-closed support adapter for the fast 831 execution.

The daemon is deliberately narrower than ``h1_support_orchestrator_828.py``.
The released 828 adapter still performs plugin preload and creates its sealed
same-Gazebo ownership record.  After preload, this process verifies that exact
adapter, manifest, certificate, ownership record, snapshot tool, and plugin;
then it keeps the three ROS telemetry subscriptions and the two native Gazebo
support-state listeners alive for the rest of the flow.

Only a mode-0600 Unix-domain socket accepts the following newline-delimited
commands, from the same uid, in the released order::

    door-attach, cube-attach, cube-detach, door-detach, finalize, shutdown

The default remains that released single-cycle protocol.  ``serve
--roundtrip`` explicitly selects a two-cycle extension: the four released
transitions, a locked cycle checkpoint, and four distinct ``*-return``
transitions.  The resident plugins are still preloaded only once from the
fresh origin scene.

Every support transition publishes exactly one ``gz.msgs.Empty`` command and
requires a newer native state acknowledgement.  The daemon creates no ROS
publisher and never sends a robot trajectory, joint command, or pose command.
Any malformed input, telemetry failure, order violation, or failed gate
permanently locks out all commands except ``shutdown``.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import stat
import subprocess
import sys
import threading
import time
import types
from typing import Any, Callable, Mapping, Sequence


DEFAULT_WORKSPACE = Path(
    "/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
DEFAULT_SUPPORT_SOURCE = (
    DEFAULT_WORKSPACE /
    "src/h1_2_moveit_demo/scripts/h1_support_orchestrator_828.py")

COMMANDS = (
    "door-attach", "cube-attach", "cube-detach", "door-detach",
    "finalize", "shutdown",
)
TRANSITION_COMMANDS = COMMANDS[:4]
ROUNDTRIP_TRANSITION_COMMANDS = (
    *TRANSITION_COMMANDS,
    "door-attach-return", "cube-attach-return",
    "cube-detach-return", "door-detach-return",
)
ROUNDTRIP_PROTOCOL = (
    *TRANSITION_COMMANDS, "cycle-checkpoint",
    *ROUNDTRIP_TRANSITION_COMMANDS[4:],
)
ROUNDTRIP_COMMANDS = (
    *ROUNDTRIP_PROTOCOL, "finalize", "shutdown",
)
ALL_COMMANDS = tuple(dict.fromkeys((*COMMANDS, *ROUNDTRIP_COMMANDS)))
INTERNAL_NATIVE_LISTENER_MODE = "_pdeath-native-state-listener"
INTERNAL_PDEATH_GZ_MODE = "_pdeath-gz-support-mutation"
MAX_REQUEST_BYTES = 256
MAX_RESPONSE_BYTES = 4096
MAX_NATIVE_LINE_BYTES = 4096
MAX_SUPPORT_STATE_AGE_SECONDS = 3.0

PASS_MARKERS = {
    "door-attach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-attach "
        "door=attached cube=detached boundary=E0_ENTRY"),
    "cube-attach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-attach "
        "door=attached cube=attached boundary=CUBE_CONTACT"),
    "cube-detach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-detach "
        "door=attached cube=detached boundary=CUBE_RELEASE"),
    "door-detach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-detach "
        "door=detached cube=detached boundary=E0_RETURN"),
    "finalize": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize door=q0 "
        "handle=q0 latch=locked supports=detached robot=launch publishers=0"),
    "shutdown": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=shutdown execution=none"),
}

ROUNDTRIP_PASS_MARKERS = {
    "door-attach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-attach cycle=1 "
        "door=attached cube=detached boundary=E0_ENTRY"),
    "cube-attach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-attach cycle=1 "
        "door=attached cube=attached boundary=CUBE_CONTACT_ORIGIN"),
    "cube-detach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-detach cycle=1 "
        "door=attached cube=detached boundary=CUBE_RELEASE_INSIDE"),
    "door-detach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-detach cycle=1 "
        "door=detached cube=detached boundary=E0_RETURN"),
    "cycle-checkpoint": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cycle-checkpoint cycle=1 "
        "door=q0 handle=q0 latch=locked supports=detached robot=launch "
        "cube=inside boundary=CUBE_RELEASE_INSIDE publishers=0"),
    "door-attach-return": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-attach-return "
        "cycle=2 door=attached cube=detached boundary=E0_ENTRY"),
    "cube-attach-return": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-attach-return "
        "cycle=2 door=attached cube=attached "
        "boundary=CUBE_RELEASE_INSIDE"),
    "cube-detach-return": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-detach-return "
        "cycle=2 door=attached cube=detached "
        "boundary=CUBE_CONTACT_ORIGIN"),
    "door-detach-return": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-detach-return "
        "cycle=2 door=detached cube=detached boundary=E0_RETURN"),
    "finalize": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize mode=roundtrip "
        "cycle=2 door=q0 handle=q0 latch=locked supports=detached "
        "robot=launch cube=origin boundary=CUBE_CONTACT_ORIGIN publishers=0"),
    "shutdown": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=shutdown "
        "mode=roundtrip execution=none"),
}


class DaemonError(RuntimeError):
    """Raised for a fail-closed daemon admission or runtime failure."""


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise DaemonError(status)


def _digest_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _is_digest(value: Any) -> bool:
    return (isinstance(value, str) and
            re.fullmatch(r"[0-9a-f]{64}", value) is not None)


def _identity(value: os.stat_result) -> tuple[int, int, int, int, int]:
    return (value.st_dev, value.st_ino, value.st_size,
            value.st_mtime_ns, value.st_ctime_ns)


def _verified_socket_path_metadata(
    socket_path: Path, expected: os.stat_result,
) -> os.stat_result:
    """Return the bound filesystem socket identity, rejecting replacement.

    ``fstat(server.fileno())`` describes the kernel socket object and has a
    different device/inode pair from the pathname created by ``bind``.  The
    client can authenticate only the latter, so pin that pathname from READY
    through the one-shot claim and return the same identity to the client.
    """
    try:
        current = socket_path.lstat()
    except OSError as error:
        raise DaemonError(f"SOCKET_PATH_IDENTITY:{error}") from error
    _require(
        _identity(current) == _identity(expected) and
        stat.S_ISSOCK(current.st_mode) and
        not socket_path.is_symlink() and
        current.st_uid == os.geteuid() and
        stat.S_IMODE(current.st_mode) == 0o600,
        "SOCKET_PATH_IDENTITY")
    return current


def _process_start_ticks(pid: int, status: str) -> int:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = raw.rsplit(") ", 1)[1].split()
        value = int(fields[19])
    except (OSError, IndexError, ValueError) as error:
        raise DaemonError(f"{status}:{error}") from error
    _require(value > 0, status)
    return value


def _arm_owner_death(owner_pid: int, owner_start_ticks: int) -> None:
    """Kill this daemon if its exact flow owner disappears."""
    _require(
        (owner_pid == 0 and owner_start_ticks == 0) or
        (owner_pid > 1 and owner_start_ticks > 0),
        "OWNER:ARGUMENT_PAIR")
    if owner_pid == 0:
        owner_pid = os.getppid()
        owner_start_ticks = _process_start_ticks(
            owner_pid, "OWNER:INFERRED_IDENTITY")
    try:
        import ctypes

        libc = ctypes.CDLL(None, use_errno=True)
        result = libc.prctl(1, int(signal.SIGKILL), 0, 0, 0)
        if result != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number))
    except (ImportError, OSError) as error:
        raise DaemonError(f"OWNER:PDEATHSIG:{error}") from error
    # Close both the pre-prctl death race and a possible PID-reuse alias.
    _require(os.getppid() == owner_pid, "OWNER:PARENT_REPLACED")
    _require(
        _process_start_ticks(owner_pid, "OWNER:IDENTITY") ==
        owner_start_ticks,
        "OWNER:START_TICKS")


def _read_exact_fd(descriptor: int, size: int, status: str) -> bytes:
    os.lseek(descriptor, 0, os.SEEK_SET)
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        block = os.read(descriptor, min(1024 * 1024, remaining))
        _require(bool(block), f"{status}:SHORT_READ")
        chunks.append(block)
        remaining -= len(block)
    _require(not os.read(descriptor, 1), f"{status}:GREW")
    return b"".join(chunks)


class FailureLatch:
    """Thread-safe first-failure latch; the first reason is authoritative."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._reason: str | None = None

    def fail(self, reason: str) -> None:
        clean = str(reason).replace("\n", "_").replace("\r", "_")[:1000]
        with self._condition:
            if self._reason is None:
                self._reason = clean or "UNKNOWN"
            self._condition.notify_all()

    @property
    def reason(self) -> str | None:
        with self._condition:
            return self._reason

    def require_clear(self) -> None:
        reason = self.reason
        _require(reason is None, f"LOCKED:{reason}")


class FilePin:
    """Keep one verified regular file open and recheck it without aliases."""

    def __init__(self, path: Path, *, expected_sha256: str | None = None,
                 expected_raw: bytes | None = None, expected_mode: int | None = None,
                 label: str):
        _require(path.is_absolute(), f"{label}:PATH_NOT_ABSOLUTE")
        _require((expected_sha256 is None) != (expected_raw is None),
                 f"{label}:EXPECTED_BINDING")
        if expected_sha256 is not None:
            _require(_is_digest(expected_sha256), f"{label}:EXPECTED_SHA256")
        try:
            metadata = path.lstat()
        except OSError as error:
            raise DaemonError(f"{label}:LSTAT:{error}") from error
        _require(stat.S_ISREG(metadata.st_mode) and not path.is_symlink(),
                 f"{label}:NOT_REGULAR")
        _require(metadata.st_uid == os.geteuid(), f"{label}:OWNER")
        _require(metadata.st_nlink == 1 and metadata.st_size > 0,
                 f"{label}:IDENTITY")
        if expected_mode is not None:
            _require(stat.S_IMODE(metadata.st_mode) == expected_mode,
                     f"{label}:MODE")
        flags = (os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) |
                 getattr(os, "O_NOFOLLOW", 0))
        try:
            descriptor = os.open(path, flags)
        except OSError as error:
            raise DaemonError(f"{label}:OPEN:{error}") from error
        self.path = path
        self.label = label
        self.descriptor = descriptor
        self.metadata = metadata
        self.expected_sha256 = expected_sha256
        self.expected_raw = expected_raw
        try:
            self.check()
        except BaseException:
            os.close(descriptor)
            raise

    def check(self) -> None:
        try:
            path_metadata = self.path.lstat()
            fd_metadata = os.fstat(self.descriptor)
        except OSError as error:
            raise DaemonError(f"{self.label}:STAT:{error}") from error
        _require(_identity(path_metadata) == _identity(self.metadata) and
                 _identity(fd_metadata) == _identity(self.metadata),
                 f"{self.label}:RACE")
        raw = _read_exact_fd(
            self.descriptor, self.metadata.st_size, self.label)
        if self.expected_sha256 is not None:
            _require(_digest_bytes(raw) == self.expected_sha256,
                     f"{self.label}:SHA256")
        else:
            _require(raw == self.expected_raw, f"{self.label}:CONTENT")

    def read(self) -> bytes:
        self.check()
        return _read_exact_fd(
            self.descriptor, self.metadata.st_size, self.label)

    def close(self) -> None:
        if self.descriptor >= 0:
            os.close(self.descriptor)
            self.descriptor = -1


def _load_bound_support_module(
    path: Path, expected_sha256: str,
) -> tuple[types.ModuleType, FilePin]:
    """Compile the exact bytes already read from an owned mode-0500 file."""
    pin = FilePin(
        path, expected_sha256=expected_sha256, expected_mode=0o500,
        label="SUPPORT_SOURCE")
    raw = pin.read()
    try:
        source = raw.decode("utf-8")
    except UnicodeDecodeError as error:
        pin.close()
        raise DaemonError(f"SUPPORT_SOURCE:UTF8:{error}") from error
    module_name = f"_h1_bound_support_828_{expected_sha256}"
    module = types.ModuleType(module_name)
    module.__file__ = str(path)
    module.__package__ = ""
    sys.modules[module_name] = module
    try:
        exec(compile(source, str(path), "exec"), module.__dict__)
    except BaseException:
        sys.modules.pop(module_name, None)
        pin.close()
        raise
    for name in (
            "load_sealed_json", "validate_manifest_and_certificate",
            "GazeboRuntime", "DOOR_STATE_TOPIC", "DOOR_ATTACH_TOPIC",
            "DOOR_DETACH_TOPIC", "CUBE_STATE_TOPIC", "CUBE_ATTACH_TOPIC",
            "CUBE_DETACH_TOPIC", "ALL_ROBOT_JOINTS", "LEFT_JOINTS",
            "TRAJECTORY_TOPICS"):
        _require(hasattr(module, name), f"SUPPORT_SOURCE:API:{name}")
    return module, pin


class SecurityPins:
    """Continuously bind this daemon to the admitted immutable execution."""

    def __init__(self, support: Any, runtime: Any, source_pin: FilePin,
                 manifest_path: Path, manifest_sha256: str,
                 certificate_path: Path, certificate_sha256: str):
        self._pins: list[FilePin] = [source_pin]
        self._pins.append(FilePin(
            manifest_path, expected_sha256=manifest_sha256,
            expected_mode=0o400, label="MANIFEST"))
        self._pins.append(FilePin(
            certificate_path, expected_sha256=certificate_sha256,
            expected_mode=0o400, label="CERTIFICATE"))
        snapshot = runtime._bound_snapshot_tool()
        self._pins.append(FilePin(
            snapshot, expected_sha256=runtime.contract.snapshot_sha256,
            expected_mode=0o500, label="SNAPSHOT"))

        plugin = runtime.plugin.resolve(strict=True)
        self._pins.append(FilePin(
            plugin, expected_sha256=runtime.contract.plugin_sha256,
            label="PLUGIN"))

        runtime._require_ownership()
        ownership_path, ownership_raw = runtime._ownership_path_and_payload()
        self._pins.append(FilePin(
            ownership_path, expected_raw=ownership_raw,
            expected_mode=0o400, label="OWNERSHIP"))
        try:
            ownership = json.loads(ownership_raw.decode("utf-8"))
            self._server_pid = int(ownership["gazebo_server_pid"])
            self._server_start_ticks = int(
                ownership["gazebo_server_start_ticks"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise DaemonError(f"OWNERSHIP:SERVER_BINDING:{error}") from error
        self.check()

    def _check_server(self) -> None:
        try:
            raw = Path(f"/proc/{self._server_pid}/stat").read_text(
                encoding="utf-8")
            fields = raw.rsplit(") ", 1)[1].split()
            start_ticks = int(fields[19])
        except (OSError, IndexError, ValueError) as error:
            raise DaemonError(f"GAZEBO_SERVER:IDENTITY:{error}") from error
        _require(start_ticks == self._server_start_ticks,
                 "GAZEBO_SERVER:REPLACED")

    def check(self) -> None:
        self._check_server()
        for pin in self._pins:
            pin.check()

    def close(self) -> None:
        for pin in reversed(self._pins):
            pin.close()


@dataclass(frozen=True)
class JointSample:
    sequence: int
    received_at: float
    positions: Mapping[str, float]
    velocities: Mapping[str, float]


@dataclass(frozen=True)
class LatchSample:
    sequence: int
    received_at: float
    state: str


class RosTelemetry:
    """Three persistent, read-only ROS subscriptions and no publishers."""

    def __init__(self, latch: FailureLatch, maximum_age: float):
        self._latch = latch
        self._maximum_age = maximum_age
        self._condition = threading.Condition()
        self._robot: JointSample | None = None
        self._incubator: JointSample | None = None
        self._latch_sample: LatchSample | None = None
        self._sequence = {"robot": 0, "incubator": 0, "latch": 0}
        self._thread: threading.Thread | None = None
        self._context: Any = None
        self._executor: Any = None
        self.node: Any = None
        self._subscriptions: list[Any] = []

    def start(self, timeout: float) -> None:
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
        except ImportError as error:
            raise DaemonError(f"ROS_IMPORT:{error}") from error

        context = Context()
        rclpy.init(
            args=None, context=context,
            signal_handler_options=SignalHandlerOptions.NO)
        node = Node(
            f"h1_fast_support_daemon_831_{os.getpid()}", context=context,
            use_global_arguments=False, enable_rosout=False,
            start_parameter_services=False, enable_logger_service=False)
        # rclpy creates /parameter_events even when all parameter services are
        # disabled.  This daemon never declares or changes a parameter, so
        # remove that framework publisher and make the read-only property an
        # executable runtime invariant, not merely a source-code convention.
        parameter_events = getattr(node, "_parameter_event_publisher", None)
        _require(parameter_events is not None and
                 node.destroy_publisher(parameter_events),
                 "ROS_READ_ONLY:PARAMETER_EVENT_PUBLISHER")
        _require(not list(node.publishers), "ROS_READ_ONLY:PUBLISHERS")
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        self._context = context
        self._executor = executor
        self.node = node

        def robot_callback(message: Any) -> None:
            self._joint_callback("robot", message, require_velocity=False)

        def incubator_callback(message: Any) -> None:
            self._joint_callback("incubator", message, require_velocity=True)

        def latch_callback(message: Any) -> None:
            try:
                state = str(message.data)
                _require(state in ("locked", "unlocked"),
                         f"ROS_LATCH:STATE:{state}")
                with self._condition:
                    self._sequence["latch"] += 1
                    self._latch_sample = LatchSample(
                        self._sequence["latch"], time.monotonic(), state)
                    self._condition.notify_all()
            except BaseException as error:
                self._latch.fail(f"ROS_LATCH:{error}")

        self._subscriptions = [
            node.create_subscription(
                JointState, "/joint_states", robot_callback,
                qos_profile_sensor_data),
            node.create_subscription(
                JointState, "/incubator/joint_states", incubator_callback,
                qos_profile_sensor_data),
            node.create_subscription(
                String, "/incubator/latch/state", latch_callback,
                qos_profile_sensor_data),
        ]
        self._thread = threading.Thread(
            target=self._spin, name="h1-fast-831-ros", daemon=True)
        self._thread.start()
        deadline = time.monotonic() + timeout
        with self._condition:
            while (self._robot is None or self._incubator is None or
                   self._latch_sample is None):
                self._latch.require_clear()
                remaining = deadline - time.monotonic()
                _require(remaining > 0.0, "ROS_TELEMETRY:START_TIMEOUT")
                self._condition.wait(min(remaining, 0.1))

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except BaseException as error:
            self._latch.fail(f"ROS_EXECUTOR:{error}")

    def _joint_callback(
        self, kind: str, message: Any, *, require_velocity: bool,
    ) -> None:
        try:
            names = list(message.name)
            positions = list(message.position)
            velocities = list(message.velocity)
            _require(bool(names) and len(names) == len(positions) and
                     len(set(names)) == len(names),
                     f"ROS_{kind.upper()}:ARRAYS")
            if require_velocity:
                _require(len(velocities) == len(names),
                         f"ROS_{kind.upper()}:VELOCITIES")
            elif len(velocities) not in (0, len(names)):
                raise DaemonError(f"ROS_{kind.upper()}:VELOCITIES")
            _require(all(
                isinstance(name, str) and name and
                not any(character.isspace() for character in name)
                for name in names), f"ROS_{kind.upper()}:NAMES")
            _require(all(math.isfinite(float(value)) for value in positions),
                     f"ROS_{kind.upper()}:POSITIONS")
            if velocities:
                _require(all(math.isfinite(float(value)) for value in velocities),
                         f"ROS_{kind.upper()}:VELOCITIES_FINITE")
            position_map = {
                name: float(positions[index]) for index, name in enumerate(names)}
            velocity_map = ({
                name: float(velocities[index]) for index, name in enumerate(names)}
                if velocities else {})
            with self._condition:
                self._sequence[kind] += 1
                sample = JointSample(
                    self._sequence[kind], time.monotonic(),
                    position_map, velocity_map)
                if kind == "robot":
                    self._robot = sample
                else:
                    self._incubator = sample
                self._condition.notify_all()
        except BaseException as error:
            self._latch.fail(f"ROS_{kind.upper()}:{error}")

    def _wait_joint(self, kind: str, after_sequence: int | None,
                    timeout: float) -> JointSample:
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                self._latch.require_clear()
                sample = self._robot if kind == "robot" else self._incubator
                now = time.monotonic()
                if (sample is not None and
                        (after_sequence is None or
                         sample.sequence > after_sequence) and
                        now - sample.received_at <= self._maximum_age):
                    return sample
                remaining = deadline - now
                _require(remaining > 0.0,
                         f"ROS_{kind.upper()}:FRESH_TIMEOUT")
                self._condition.wait(min(remaining, 0.05))

    def next_robot(self, after_sequence: int | None, timeout: float) -> JointSample:
        return self._wait_joint("robot", after_sequence, timeout)

    def next_incubator(
        self, after_sequence: int | None, timeout: float,
    ) -> JointSample:
        return self._wait_joint("incubator", after_sequence, timeout)

    def latch_state(self, timeout: float) -> LatchSample:
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                self._latch.require_clear()
                sample = self._latch_sample
                now = time.monotonic()
                if (sample is not None and
                        now - sample.received_at <= max(
                            self._maximum_age, 1.25)):
                    return sample
                remaining = deadline - now
                _require(remaining > 0.0, "ROS_LATCH:FRESH_TIMEOUT")
                self._condition.wait(min(remaining, 0.05))

    def sequences(self) -> tuple[int, int, int]:
        with self._condition:
            return (self._sequence["robot"], self._sequence["incubator"],
                    self._sequence["latch"])

    def require_trajectory_topics_quiescent(
        self, topics: Sequence[str], timeout: float,
    ) -> None:
        deadline = time.monotonic() + timeout
        while True:
            self._latch.require_clear()
            unsafe = []
            incomplete = []
            for topic in topics:
                publishers = self.node.get_publishers_info_by_topic(topic)
                subscribers = self.node.get_subscriptions_info_by_topic(topic)
                if publishers:
                    unsafe.append(f"{topic}:{len(publishers)}")
                if not subscribers:
                    incomplete.append(topic)
            _require(not unsafe,
                     f"FINAL_TRAJECTORY_PUBLISHERS:{','.join(unsafe)}")
            if not incomplete:
                return
            _require(time.monotonic() < deadline,
                     f"FINAL_TRAJECTORY_DISCOVERY:{','.join(incomplete)}")
            time.sleep(0.05)

    def close(self) -> None:
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except BaseException:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.node is not None:
            try:
                self.node.destroy_node()
            except BaseException:
                pass
        if self._context is not None:
            try:
                self._context.try_shutdown()
            except BaseException:
                pass


class NativeStateListener:
    """One persistent native Gazebo state stream with transition guarding."""

    def __init__(self, topic: str, latch: FailureLatch):
        self.topic = topic
        self._latch = latch
        self._condition = threading.Condition()
        self._state: str | None = None
        self._sequence = 0
        self._received_at = 0.0
        self._allowed = {"attached", "detached"}
        self._stopping = False
        try:
            self._process = subprocess.Popen(
                [sys.executable, str(Path(__file__).resolve(strict=True)),
                 INTERNAL_NATIVE_LISTENER_MODE, str(os.getpid()), topic],
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, text=False, bufsize=0,
                start_new_session=True)
        except OSError as error:
            raise DaemonError(f"STATE_LISTENER:{topic}:{error}") from error
        self._thread = threading.Thread(
            target=self._read, name=f"h1-fast-831-state-{topic.rsplit('/', 1)[-1]}",
            daemon=True)
        self._thread.start()

    @staticmethod
    def _parse_line(line: str) -> str | None:
        match = re.fullmatch(
            r"\s*data:\s*(?:\"([A-Za-z0-9_+-]+)\"|"
            r"'([A-Za-z0-9_+-]+)'|([A-Za-z0-9_+-]+))\s*", line)
        if match is not None:
            value = next(item for item in match.groups() if item is not None)
            _require(value in ("attached", "detached"),
                     f"STATE_VALUE:{value}")
            return value
        clean = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", line).strip()
        _require(
            not clean or re.match(
                r"^(?:\[(?:INFO|WARN|WARNING)\]|INFO:|WARN(?:ING)?(?::|\s|\[))",
                clean, re.IGNORECASE) is not None,
            "STATE_MALFORMED_LINE")
        return None

    def _read(self) -> None:
        try:
            assert self._process.stdout is not None
            while True:
                raw = self._process.stdout.readline(MAX_NATIVE_LINE_BYTES + 1)
                if not raw:
                    break
                _require(len(raw) <= MAX_NATIVE_LINE_BYTES,
                         f"STATE_LISTENER:{self.topic}:LINE_LIMIT")
                try:
                    line = raw.rstrip(b"\r\n").decode("utf-8")
                except UnicodeDecodeError as error:
                    raise DaemonError(
                        f"STATE_LISTENER:{self.topic}:UTF8:{error}") from error
                value = self._parse_line(line)
                if value is None:
                    continue
                with self._condition:
                    _require(value in self._allowed,
                             f"STATE_LISTENER:{self.topic}:UNADMITTED:{value}")
                    self._sequence += 1
                    self._state = value
                    self._received_at = time.monotonic()
                    self._condition.notify_all()
            if not self._stopping:
                raise DaemonError(
                    f"STATE_LISTENER:{self.topic}:EXIT:{self._process.poll()}")
        except BaseException as error:
            self._latch.fail(str(error))
            with self._condition:
                self._condition.notify_all()

    def wait_initial(self, timeout: float) -> str:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._state is None:
                self._latch.require_clear()
                remaining = deadline - time.monotonic()
                _require(remaining > 0.0,
                         f"STATE_LISTENER:{self.topic}:START_TIMEOUT")
                self._condition.wait(min(remaining, 0.1))
            return self._state

    @property
    def state(self) -> str:
        with self._condition:
            _require(self._state is not None,
                     f"STATE_LISTENER:{self.topic}:NO_STATE")
            return self._state

    def begin_transition(self, before: str, after: str) -> int:
        with self._condition:
            self._latch.require_clear()
            _require(self._state == before,
                     f"STATE_LISTENER:{self.topic}:BEFORE:{self._state}")
            self._allowed = {before, after}
            return self._sequence

    def finish_transition(self, after: str, after_sequence: int,
                          timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self._condition:
            while not (self._sequence > after_sequence and self._state == after):
                self._latch.require_clear()
                remaining = deadline - time.monotonic()
                _require(remaining > 0.0,
                         f"STATE_LISTENER:{self.topic}:ACK_TIMEOUT:{after}")
                self._condition.wait(min(remaining, 0.05))
            self._allowed = {after}

    @property
    def sequence(self) -> int:
        with self._condition:
            return self._sequence

    def wait_new_state(
        self, expected: str, timeout: float,
        after_sequence: int | None = None,
    ) -> None:
        """Require a new heartbeat, not merely a cached expected value."""
        deadline = time.monotonic() + timeout
        with self._condition:
            _require(self._state == expected,
                     f"STATE_LISTENER:{self.topic}:EXPECTED:{expected}:"
                     f"{self._state}")
            sequence = (self._sequence if after_sequence is None
                        else after_sequence)
            self._allowed = {expected}
            while self._sequence <= sequence:
                self._latch.require_clear()
                remaining = deadline - time.monotonic()
                _require(remaining > 0.0,
                         f"STATE_LISTENER:{self.topic}:HEARTBEAT_TIMEOUT")
                self._condition.wait(min(remaining, 0.05))
            _require(self._state == expected,
                     f"STATE_LISTENER:{self.topic}:HEARTBEAT_STATE:"
                     f"{self._state}")

    def enforce(self, expected: str) -> None:
        with self._condition:
            _require(self._state == expected,
                     f"STATE_LISTENER:{self.topic}:EXPECTED:{expected}:"
                     f"{self._state}")
            _require(time.monotonic() - self._received_at <=
                     MAX_SUPPORT_STATE_AGE_SECONDS,
                     f"STATE_LISTENER:{self.topic}:STALE")
            self._allowed = {expected}

    def close(self) -> None:
        self._stopping = True
        try:
            os.killpg(self._process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            self._process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            pass
        self._thread.join(timeout=2.0)


class OperationState:
    """Pure sequence contract shared by live execution and offline tests."""

    def __init__(self, roundtrip: bool = False) -> None:
        _require(type(roundtrip) is bool, "PROTOCOL:ROUNDTRIP_TYPE")
        self.roundtrip = roundtrip
        self.protocol = (ROUNDTRIP_PROTOCOL if roundtrip else
                         TRANSITION_COMMANDS)
        self._next = 0
        self.finalized = False

    def admit(self, command: str) -> None:
        commands = ROUNDTRIP_COMMANDS if self.roundtrip else COMMANDS
        _require(command in commands, f"PROTOCOL:COMMAND:{command}")
        if command in self.protocol:
            expected = (self.protocol[self._next]
                        if self._next < len(self.protocol) else None)
            _require(command == expected,
                     f"PROTOCOL:ORDER:{command}:EXPECTED:{expected}")
        elif command == "finalize":
            _require(self._next == len(self.protocol),
                     "PROTOCOL:FINALIZE_BEFORE_TRANSITIONS")
            _require(not self.finalized, "PROTOCOL:FINALIZE_ALREADY_DONE")
        # Shutdown is always admissible.  In particular, a latched failure
        # must still have a bounded cleanup path; shutdown never mutates a
        # support or robot state and does not falsely imply finalization.

    def commit(self, command: str) -> None:
        if command in self.protocol:
            self._next += 1
        elif command == "finalize":
            self.finalized = True


class SupportEngine:
    """In-process equivalent of the four old hook CLIs plus finalization."""

    def __init__(self, support: Any, runtime: Any, pins: SecurityPins,
                 telemetry: RosTelemetry, door: NativeStateListener,
                 cube: NativeStateListener, latch: FailureLatch,
                 timeout: float, roundtrip: bool = False,
                 require_ready_claim: bool = False):
        self.support = support
        self.runtime = runtime
        self.pins = pins
        self.telemetry = telemetry
        self.door = door
        self.cube = cube
        self.latch = latch
        self.timeout = timeout
        self.roundtrip = roundtrip
        self.require_ready_claim = require_ready_claim
        self.ready_claimed = False
        self.state = OperationState(roundtrip=roundtrip)
        self._command_counts = {
            support.DOOR_ATTACH_TOPIC: 0,
            support.DOOR_DETACH_TOPIC: 0,
            support.CUBE_ATTACH_TOPIC: 0,
            support.CUBE_DETACH_TOPIC: 0,
        }

    def _pre_operation(self) -> None:
        self.latch.require_clear()
        self.pins.check()

    def claim_ready(self) -> str:
        """One-shot, non-mutating proof immediately before timed motion."""
        _require(self.roundtrip and self.require_ready_claim,
                 "READY_CLAIM_NOT_ADMITTED")
        _require(not self.ready_claimed, "READY_CLAIM_REPLAY")
        _require(all(count == 0 for count in self._command_counts.values()),
                 "READY_CLAIM_AFTER_MUTATION")
        self._pre_operation()
        self.door.enforce("detached")
        self.cube.enforce("detached")
        door_sequence = self.door.sequence
        cube_sequence = self.cube.sequence
        self.door.wait_new_state(
            "detached", self.timeout, door_sequence)
        self.cube.wait_new_state(
            "detached", self.timeout, cube_sequence)
        self._mechanism_triplet_gate(
            0.0, 0.0, self.support.DOOR_CLOSED_TOLERANCE,
            "locked", "READY_CLAIM")
        self._boundary_gate("FINAL_ROBOT")
        pose = self.runtime.contract.boundaries["CUBE_CONTACT"]["cube_pose"]
        self._cube_stability_gate(pose, "READY_CLAIM")
        self.telemetry.require_trajectory_topics_quiescent(
            self.support.TRAJECTORY_TOPICS, self.timeout)
        self.pins.check()
        self.runtime._require_ownership()
        _ownership_path, ownership_raw = (
            self.runtime._ownership_path_and_payload())
        self.ready_claimed = True
        return hashlib.sha256(ownership_raw).hexdigest()

    def _boundary_gate(self, name: str) -> None:
        boundary = self.runtime.contract.boundaries[name]
        before = self.telemetry.sequences()[0]
        sample = self.telemetry.next_robot(before, self.timeout)
        for joint, expected in boundary["robot_positions"].items():
            _require(joint in sample.positions,
                     f"{name}_JOINT_MISSING:{joint}")
            if ((name == "FINAL_ROBOT" and "dex1_finger" in joint) or
                    joint.startswith("left_dex1_finger_")):
                tolerance = 0.001
            elif joint.startswith("right_dex1_finger_"):
                tolerance = self.support.RIGHT_FINGER_ENDPOINT_TOLERANCE
            else:
                tolerance = self.support.ROBOT_ENDPOINT_TOLERANCE
            _require(abs(sample.positions[joint] - float(expected)) <= tolerance,
                     f"{name}_JOINT_ENDPOINT:{joint}")

    def _settled_boundary_gate(self, name: str) -> None:
        """Require three fresh compliant samples at the return handoff."""
        boundary = self.runtime.contract.boundaries[name]
        deadline = time.monotonic() + min(3.0, self.timeout)
        sequence = self.telemetry.sequences()[0]
        consecutive = 0
        worst_joint = "unknown"
        while True:
            remaining = deadline - time.monotonic()
            _require(remaining > 0.0,
                     f"{name}_JOINT_ENDPOINT:{worst_joint}")
            sample = self.telemetry.next_robot(sequence, remaining)
            sequence = sample.sequence
            compliant = True
            worst_ratio = -1.0
            for joint, expected in boundary["robot_positions"].items():
                _require(joint in sample.positions,
                         f"{name}_JOINT_MISSING:{joint}")
                if joint.startswith("left_dex1_finger_"):
                    tolerance = 0.001
                elif joint.startswith("right_dex1_finger_"):
                    tolerance = self.support.RIGHT_FINGER_ENDPOINT_TOLERANCE
                else:
                    tolerance = self.support.ROBOT_ENDPOINT_TOLERANCE
                error = abs(sample.positions[joint] - float(expected))
                ratio = error / tolerance
                if ratio > worst_ratio:
                    worst_ratio = ratio
                    worst_joint = joint
                if error > tolerance:
                    compliant = False
            consecutive = consecutive + 1 if compliant else 0
            if consecutive >= 3:
                return

    def _left_contact_gate(self) -> None:
        before = self.telemetry.sequences()[0]
        sample = self.telemetry.next_robot(before, self.timeout)
        q1 = sample.positions.get("left_dex1_finger_1_joint", math.nan)
        q2 = sample.positions.get("left_dex1_finger_2_joint", math.nan)
        gap = 0.050061 - (q1 + q2)
        center = (q2 - q1) / 2.0
        _require(-0.0195 <= q1 <= -0.0100 and
                 -0.0195 <= q2 <= -0.0100 and
                 0.077 <= gap <= 0.083 and -0.003 <= center <= 0.003,
                 "LEFT_CUBE_CONTACT")

    def _mechanism_gate(self, door_expected: float, handle_expected: float,
                        door_tolerance: float, latch_expected: str,
                        label: str, after_sequence: int | None) -> int:
        sample = self.telemetry.next_incubator(after_sequence, self.timeout)
        _require("door_hinge_joint" in sample.positions and
                 "handle_latch_joint" in sample.positions and
                 "door_hinge_joint" in sample.velocities and
                 "handle_latch_joint" in sample.velocities,
                 f"{label}_MECHANISM_ROWS")
        door_position = sample.positions["door_hinge_joint"]
        handle_position = sample.positions["handle_latch_joint"]
        door_velocity = sample.velocities["door_hinge_joint"]
        handle_velocity = sample.velocities["handle_latch_joint"]
        _require(
            abs(door_position - door_expected) <= door_tolerance and
            abs(handle_position - handle_expected) <=
            self.support.HANDLE_TOLERANCE and
            abs(door_velocity) <= self.support.MECHANISM_VELOCITY_TOLERANCE and
            abs(handle_velocity) <= self.support.MECHANISM_VELOCITY_TOLERANCE,
            f"{label}_MECHANISM:door_position={door_position:.12g}:"
            f"handle_position={handle_position:.12g}:"
            f"door_velocity={door_velocity:.12g}:"
            f"handle_velocity={handle_velocity:.12g}")
        latch = self.telemetry.latch_state(self.timeout)
        _require(latch.state == latch_expected, f"{label}_LATCH")
        return sample.sequence

    def _mechanism_triplet_gate(
        self, door_expected: float, handle_expected: float,
        door_tolerance: float, latch_expected: str, label: str,
    ) -> None:
        # Short-lived 828 hooks naturally spent several seconds in discovery
        # while a just-commanded mechanism damped.  The resident daemon must
        # model that latency explicitly: wait up to three seconds to enter the
        # exact unchanged position/velocity/latch envelope, then retain the
        # original three fresh consecutive samples as the proof.
        sequence = self.telemetry.sequences()[1]
        deadline = time.monotonic() + min(3.0, self.timeout)
        last_values: tuple[float, float, float, float, str] | None = None
        while True:
            remaining = deadline - time.monotonic()
            _require(remaining > 0.0, f"{label}_CONVERGENCE_TIMEOUT")
            sample = self.telemetry.next_incubator(
                sequence, min(remaining, self.timeout))
            sequence = sample.sequence
            _require(
                "door_hinge_joint" in sample.positions and
                "handle_latch_joint" in sample.positions and
                "door_hinge_joint" in sample.velocities and
                "handle_latch_joint" in sample.velocities,
                f"{label}_CONVERGENCE_ROWS")
            door_position = sample.positions["door_hinge_joint"]
            handle_position = sample.positions["handle_latch_joint"]
            door_velocity = sample.velocities["door_hinge_joint"]
            handle_velocity = sample.velocities["handle_latch_joint"]
            remaining = deadline - time.monotonic()
            _require(remaining > 0.0, f"{label}_CONVERGENCE_TIMEOUT")
            latch_state = self.telemetry.latch_state(
                min(remaining, self.timeout)).state
            last_values = (
                door_position, handle_position, door_velocity,
                handle_velocity, latch_state)
            if (
                abs(door_position - door_expected) <= door_tolerance and
                abs(handle_position - handle_expected) <=
                    self.support.HANDLE_TOLERANCE and
                abs(door_velocity) <=
                    self.support.MECHANISM_VELOCITY_TOLERANCE and
                abs(handle_velocity) <=
                    self.support.MECHANISM_VELOCITY_TOLERANCE and
                latch_state == latch_expected
            ):
                break
            if time.monotonic() >= deadline:
                raise DaemonError(
                    f"{label}_CONVERGENCE:door_position="
                    f"{door_position:.12g}:handle_position="
                    f"{handle_position:.12g}:door_velocity="
                    f"{door_velocity:.12g}:handle_velocity="
                    f"{handle_velocity:.12g}:latch={latch_state}")
        _require(last_values is not None, f"{label}_CONVERGENCE_INTERNAL")
        for index in range(3):
            sequence = self._mechanism_gate(
                door_expected, handle_expected, door_tolerance,
                latch_expected, f"{label}_SAMPLE_{index + 1}", sequence)

    def _cube_pose_gate(self, expected: Mapping[str, Any], label: str) -> None:
        actual = self.runtime._pose_from_entity(
            self.runtime._entity("green_cube"))
        position_error, orientation_error = self.runtime._pose_error(
            actual, expected)
        _require(position_error <= self.support.CUBE_POSITION_TOLERANCE and
                 orientation_error <= self.support.CUBE_ORIENTATION_TOLERANCE,
                 f"{label}_CUBE_POSE:{position_error:.9g}:"
                 f"{orientation_error:.9g}")

    def _cube_stability_gate(
        self, expected: Mapping[str, Any], label: str,
    ) -> None:
        """Retain the released 828 three-sample / 750 ms stability proof."""
        # The removed short-lived ROS/GZ discovery used to give the compliant
        # support several seconds to damp after a fast carry.  Preserve the
        # exact 3 mm / 2 degree gate, but explicitly wait for that same
        # physical convergence instead of depending on process-start latency.
        deadline = time.monotonic() + min(5.0, self.timeout)
        reference = self.runtime._pose_from_entity(
            self.runtime._entity("green_cube"))
        stable_samples = 0
        position_error = math.inf
        orientation_error = math.inf
        movement_position = math.inf
        movement_orientation = math.inf
        while stable_samples < 3:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise DaemonError(
                    f"{label}_CUBE_STABILITY_TIMEOUT:"
                    f"{position_error:.9g}:{orientation_error:.9g}:"
                    f"{movement_position:.9g}:{movement_orientation:.9g}")
            time.sleep(min(0.25, remaining))
            current = self.runtime._pose_from_entity(
                self.runtime._entity("green_cube"))
            position_error, orientation_error = self.runtime._pose_error(
                current, expected)
            movement_position, movement_orientation = (
                self.runtime._pose_error(current, reference))
            compliant = (
                position_error <= self.support.CUBE_POSITION_TOLERANCE and
                orientation_error <=
                self.support.CUBE_ORIENTATION_TOLERANCE and
                movement_position <= self.support.CUBE_POSITION_TOLERANCE and
                movement_orientation <=
                self.support.CUBE_ORIENTATION_TOLERANCE)
            stable_samples = stable_samples + 1 if compliant else 0
            reference = current

    def _transition(self, listener: NativeStateListener, command_topic: str,
                    before: str, after: str, occurrence: int = 1) -> None:
        _require(type(occurrence) is int and occurrence in (1, 2),
                 f"COMMAND_OCCURRENCE:{command_topic}")
        _require(self._command_counts[command_topic] == occurrence - 1,
                 f"COMMAND_OCCURRENCE_ORDER:{command_topic}:"
                 f"{self._command_counts[command_topic] + 1}:"
                 f"EXPECTED:{occurrence}")
        sequence = listener.begin_transition(before, after)
        self._command_counts[command_topic] += 1
        command = [
            "/usr/bin/python3", str(Path(__file__).resolve(strict=True)),
            INTERNAL_PDEATH_GZ_MODE, str(os.getpid()),
            str(_process_start_ticks(os.getpid(), "MUTATION:OWNER")),
            command_topic,
        ]
        try:
            completed = subprocess.run(
                command, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                check=False, timeout=self.timeout, start_new_session=False)
        except (OSError, subprocess.TimeoutExpired) as error:
            raise DaemonError(f"MUTATION:EXEC:{error}") from error
        _require(
            completed.returncode == 0 and
            len(completed.stdout) <= MAX_RESPONSE_BYTES and
            len(completed.stderr) <= MAX_RESPONSE_BYTES,
            f"MUTATION:EXIT:{completed.returncode}")
        listener.finish_transition(after, sequence, self.timeout)

    def door_attach(self, occurrence: int = 1) -> None:
        label = "DOOR_ATTACH" if occurrence == 1 else "DOOR_ATTACH_RETURN"
        self.door.enforce("detached")
        self.cube.enforce("detached")
        self._boundary_gate("E0_ENTRY")
        self._mechanism_triplet_gate(
            0.0, math.pi / 2.0, self.support.DOOR_CLOSED_TOLERANCE,
            "unlocked", f"{label}_PRE")
        self._transition(
            self.door, self.support.DOOR_ATTACH_TOPIC,
            "detached", "attached", occurrence)
        self._boundary_gate("E0_ENTRY")
        self._mechanism_triplet_gate(
            0.0, math.pi / 2.0, self.support.DOOR_CLOSED_TOLERANCE,
            "unlocked", f"{label}_POST")

    def _cube_attach(
        self, boundary: str, occurrence: int, label: str,
    ) -> None:
        _require(boundary in ("CUBE_CONTACT", "CUBE_RELEASE"),
                 f"CUBE_ATTACH_BOUNDARY:{boundary}")
        self.door.enforce("attached")
        self.cube.enforce("detached")
        self._boundary_gate(boundary)
        self._left_contact_gate()
        pose = self.runtime.contract.boundaries[boundary]["cube_pose"]
        self._cube_pose_gate(pose, f"{label}_PRE")
        self._mechanism_gate(
            math.pi / 3.0, math.pi / 2.0,
            self.support.DOOR_OPEN_TOLERANCE, "unlocked",
            f"{label}_PRE", self.telemetry.sequences()[1])
        self._transition(
            self.cube, self.support.CUBE_ATTACH_TOPIC,
            "detached", "attached", occurrence)
        self._boundary_gate(boundary)
        self._left_contact_gate()
        self._cube_pose_gate(pose, f"{label}_POST")

    def cube_attach(self) -> None:
        self._cube_attach("CUBE_CONTACT", 1, "CUBE_ATTACH")

    def cube_attach_return(self) -> None:
        self._cube_attach("CUBE_RELEASE", 2, "CUBE_ATTACH_RETURN")

    def _cube_detach(
        self, boundary: str, occurrence: int, label: str,
    ) -> None:
        _require(boundary in ("CUBE_CONTACT", "CUBE_RELEASE"),
                 f"CUBE_DETACH_BOUNDARY:{boundary}")
        self.door.enforce("attached")
        self.cube.enforce("attached")
        if label == "CUBE_DETACH_RETURN":
            self._settled_boundary_gate(boundary)
        else:
            self._boundary_gate(boundary)
        self._left_contact_gate()
        pose = self.runtime.contract.boundaries[boundary]["cube_pose"]
        self._cube_stability_gate(pose, f"{label}_PRE")
        self._mechanism_gate(
            math.pi / 3.0, math.pi / 2.0,
            self.support.DOOR_OPEN_TOLERANCE, "unlocked",
            f"{label}_PRE", self.telemetry.sequences()[1])
        self._transition(
            self.cube, self.support.CUBE_DETACH_TOPIC,
            "attached", "detached", occurrence)
        self._boundary_gate(boundary)
        self._left_contact_gate()
        self._cube_stability_gate(pose, f"{label}_POST")

    def cube_detach(self) -> None:
        self._cube_detach("CUBE_RELEASE", 1, "CUBE_DETACH")

    def cube_detach_return(self) -> None:
        self._cube_detach("CUBE_CONTACT", 2, "CUBE_DETACH_RETURN")

    def door_detach(self, occurrence: int = 1) -> None:
        label = "DOOR_DETACH" if occurrence == 1 else "DOOR_DETACH_RETURN"
        self.door.enforce("attached")
        self.cube.enforce("detached")
        self._boundary_gate("E0_RETURN")
        self._mechanism_triplet_gate(
            0.0, math.pi / 2.0, self.support.DOOR_CLOSED_TOLERANCE,
            "unlocked", f"{label}_PRE")
        self._transition(
            self.door, self.support.DOOR_DETACH_TOPIC,
            "attached", "detached", occurrence)
        self._boundary_gate("E0_RETURN")
        self._mechanism_triplet_gate(
            0.0, math.pi / 2.0, self.support.DOOR_CLOSED_TOLERANCE,
            "unlocked", f"{label}_POST")

    def roundtrip_checkpoint(self) -> None:
        _require(self.roundtrip, "CHECKPOINT_REQUIRES_ROUNDTRIP")
        self.door.enforce("detached")
        self.cube.enforce("detached")
        _require(
            all(count == 1 for count in self._command_counts.values()),
            "CHECKPOINT_SUPPORT_COMMAND_COUNTS")
        door_sequence = self.door.sequence
        cube_sequence = self.cube.sequence
        self.door.wait_new_state(
            "detached", self.timeout, door_sequence)
        self.cube.wait_new_state(
            "detached", self.timeout, cube_sequence)
        self._mechanism_triplet_gate(
            0.0, 0.0, self.support.DOOR_CLOSED_TOLERANCE,
            "locked", "CHECKPOINT")
        self._boundary_gate("FINAL_ROBOT")
        pose = self.runtime.contract.boundaries["CUBE_RELEASE"]["cube_pose"]
        self._cube_stability_gate(pose, "CHECKPOINT")
        self.telemetry.require_trajectory_topics_quiescent(
            self.support.TRAJECTORY_TOPICS, self.timeout)

    def finalize(self) -> None:
        self.door.enforce("detached")
        self.cube.enforce("detached")
        # The same pinned Gazebo server and plugin binary were topology-checked
        # twice before the readiness socket was published.  Every one of the
        # four exact command endpoints has since produced its unique admitted
        # transition, and the two persistent native listeners below require a
        # newer final heartbeat.  Together those invariants prove the resident
        # plugin is still live without repeating six slow `gz topic -i`
        # discovery processes inside the physical 90-second motion window.
        expected_count = 2 if self.roundtrip else 1
        _require(
            all(count == expected_count
                for count in self._command_counts.values()),
            "FINAL_SUPPORT_COMMAND_COUNTS")
        door_sequence = self.door.sequence
        cube_sequence = self.cube.sequence
        self.door.wait_new_state(
            "detached", self.timeout, door_sequence)
        self.cube.wait_new_state(
            "detached", self.timeout, cube_sequence)
        self._mechanism_triplet_gate(
            0.0, 0.0, self.support.DOOR_CLOSED_TOLERANCE,
            "locked", "FINAL")
        self._boundary_gate("FINAL_ROBOT")
        target_boundary = "CUBE_CONTACT" if self.roundtrip else "CUBE_RELEASE"
        pose = self.runtime.contract.boundaries[target_boundary]["cube_pose"]
        self._cube_stability_gate(
            pose, "FINAL_RETURN" if self.roundtrip else "FINAL")
        self.telemetry.require_trajectory_topics_quiescent(
            self.support.TRAJECTORY_TOPICS, self.timeout)

    def dispatch(self, command: str) -> str:
        if self.require_ready_claim and command != "shutdown":
            _require(self.ready_claimed, "PROTOCOL:READY_CLAIM_REQUIRED")
        self.state.admit(command)
        markers = (ROUNDTRIP_PASS_MARKERS if self.roundtrip else
                   PASS_MARKERS)
        if command == "shutdown":
            self.state.commit(command)
            return markers[command]
        self._pre_operation()
        operation: Callable[[], None] = {
            "door-attach": self.door_attach,
            "cube-attach": self.cube_attach,
            "cube-detach": self.cube_detach,
            "door-detach": self.door_detach,
            "cycle-checkpoint": self.roundtrip_checkpoint,
            "door-attach-return": lambda: self.door_attach(2),
            "cube-attach-return": self.cube_attach_return,
            "cube-detach-return": self.cube_detach_return,
            "door-detach-return": lambda: self.door_detach(2),
            "finalize": self.finalize,
        }[command]
        operation()
        self.pins.check()
        self.state.commit(command)
        return markers[command]


def _parse_request(raw: bytes, roundtrip: bool = False) -> str:
    _require(0 < len(raw) <= MAX_REQUEST_BYTES, "PROTOCOL:SIZE")
    try:
        text = raw.decode("ascii")
    except UnicodeDecodeError as error:
        raise DaemonError(f"PROTOCOL:ASCII:{error}") from error
    _require(text.endswith("\n") and text.count("\n") == 1,
             "PROTOCOL:FRAME")
    command = text[:-1]
    commands = ROUNDTRIP_COMMANDS if roundtrip else COMMANDS
    _require(command in commands, f"PROTOCOL:COMMAND:{command}")
    return command


def _parse_ready_claim(raw: bytes, authority_run_id: str) -> str:
    _require(
        len(authority_run_id) == 64 and
        all(item in "0123456789abcdef" for item in authority_run_id),
        "READY_CLAIM_RUN_ID")
    _require(0 < len(raw) <= MAX_REQUEST_BYTES and
             raw.endswith(b"\n") and raw.count(b"\n") == 1,
             "READY_CLAIM_FRAME")
    try:
        fields = raw[:-1].decode("ascii").split()
    except UnicodeDecodeError as error:
        raise DaemonError(f"READY_CLAIM_ASCII:{error}") from error
    _require(len(fields) == 3 and fields[0] == "claim-ready",
             "READY_CLAIM_FIELDS")
    _require(fields[1] == authority_run_id, "READY_CLAIM_AUTHORITY")
    nonce = fields[2]
    _require(len(nonce) == 64 and
             all(item in "0123456789abcdef" for item in nonce),
             "READY_CLAIM_NONCE")
    return nonce


def _recv_request(connection: socket.socket, timeout: float) -> bytes:
    connection.settimeout(timeout)
    buffer = bytearray()
    while b"\n" not in buffer:
        block = connection.recv(MAX_REQUEST_BYTES + 1 - len(buffer))
        _require(bool(block), "PROTOCOL:EOF")
        buffer.extend(block)
        _require(len(buffer) <= MAX_REQUEST_BYTES, "PROTOCOL:SIZE")
    _require(buffer.endswith(b"\n"), "PROTOCOL:TRAILING_DATA")
    return bytes(buffer)


def _socket_parent_gate(path: Path) -> None:
    _require(path.is_absolute(), "SOCKET_PATH_NOT_ABSOLUTE")
    _require(len(os.fsencode(path)) <= 100, "SOCKET_PATH_TOO_LONG")
    _require(not path.exists() and not path.is_symlink(), "SOCKET_EXISTS")
    parent = path.parent
    try:
        metadata = parent.lstat()
        resolved = parent.resolve(strict=True)
    except OSError as error:
        raise DaemonError(f"SOCKET_PARENT:{error}") from error
    _require(resolved == parent and stat.S_ISDIR(metadata.st_mode) and
             not parent.is_symlink(), "SOCKET_PARENT_ALIAS")
    _require(metadata.st_uid == os.geteuid(), "SOCKET_PARENT_OWNER")
    _require(stat.S_IMODE(metadata.st_mode) & 0o022 == 0,
             "SOCKET_PARENT_WRITABLE")


def _peer_uid(connection: socket.socket) -> int:
    if not hasattr(socket, "SO_PEERCRED"):
        raise DaemonError("SOCKET_PEERCRED_UNAVAILABLE")
    raw = connection.getsockopt(
        socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
    return int.from_bytes(raw[4:8], byteorder=sys.byteorder, signed=True)


def _serve(args: argparse.Namespace) -> int:
    _arm_owner_death(args.owner_pid, args.owner_start_ticks)
    latch = FailureLatch()
    support_module: Any = None
    pins: SecurityPins | None = None
    telemetry: RosTelemetry | None = None
    listeners: list[NativeStateListener] = []
    server: socket.socket | None = None
    socket_created = False
    clean_shutdown = False
    stop_signal: int | None = None

    def handle_signal(number: int, _frame: Any) -> None:
        nonlocal stop_signal
        stop_signal = number

    previous_handlers = {
        number: signal.signal(number, handle_signal)
        for number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        support_module, source_pin = _load_bound_support_module(
            args.support_orchestrator, args.support_orchestrator_sha256)
        manifest = support_module.load_sealed_json(
            args.manifest, args.manifest_sha256)
        certificate = support_module.load_sealed_json(
            args.certificate, args.certificate_sha256)
        contract = support_module.validate_manifest_and_certificate(
            manifest, certificate)
        runtime = support_module.GazeboRuntime(
            contract, args.workspace, args.timeout)
        ownership_deadline = time.monotonic() + args.startup_timeout
        while True:
            try:
                runtime._require_ownership()
                break
            except (OSError, RuntimeError) as error:
                if time.monotonic() >= ownership_deadline:
                    raise DaemonError(
                        f"OWNERSHIP:STARTUP_TIMEOUT:{error}") from error
                time.sleep(0.05)
        pins = SecurityPins(
            support_module, runtime, source_pin,
            args.manifest, args.manifest_sha256,
            args.certificate, args.certificate_sha256)

        runtime._require_resident_plugin(
            support_module.DOOR_STATE_TOPIC,
            support_module.DOOR_ATTACH_TOPIC,
            support_module.DOOR_DETACH_TOPIC, True)
        runtime._require_resident_plugin(
            support_module.CUBE_STATE_TOPIC,
            support_module.CUBE_ATTACH_TOPIC,
            support_module.CUBE_DETACH_TOPIC, True)
        door = NativeStateListener(support_module.DOOR_STATE_TOPIC, latch)
        cube = NativeStateListener(support_module.CUBE_STATE_TOPIC, latch)
        listeners = [door, cube]
        _require(door.wait_initial(args.startup_timeout) == "detached" and
                 cube.wait_initial(args.startup_timeout) == "detached",
                 "INITIAL_SUPPORT_STATE")
        door.enforce("detached")
        cube.enforce("detached")
        runtime._require_resident_plugin(
            support_module.DOOR_STATE_TOPIC,
            support_module.DOOR_ATTACH_TOPIC,
            support_module.DOOR_DETACH_TOPIC, True)
        runtime._require_resident_plugin(
            support_module.CUBE_STATE_TOPIC,
            support_module.CUBE_ATTACH_TOPIC,
            support_module.CUBE_DETACH_TOPIC, True)

        telemetry = RosTelemetry(latch, args.telemetry_max_age)
        telemetry.start(args.startup_timeout)
        engine = SupportEngine(
            support_module, runtime, pins, telemetry, door, cube, latch,
            args.timeout, roundtrip=args.roundtrip,
            require_ready_claim=args.require_ready_claim)

        _socket_parent_gate(args.socket)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(args.socket))
        socket_created = True
        # Publish mode 0600 only after listen is active.  The flow treats that
        # exact mode as the readiness edge, so it can never observe a bound
        # but not-yet-listening socket.
        server.listen(1)
        server.settimeout(0.25)
        os.chmod(args.socket, 0o600)
        metadata = args.socket.lstat()
        _require(stat.S_ISSOCK(metadata.st_mode) and
                 metadata.st_uid == os.geteuid() and
                 stat.S_IMODE(metadata.st_mode) == 0o600,
                 "SOCKET_METADATA")
        ready_mode = " mode=roundtrip cycles=2" if args.roundtrip else ""
        ready_claim = (
            f" claim=required run_id={args.authority_run_id}"
            if args.require_ready_claim else "")
        print(
            "H1_FAST_SUPPORT_DAEMON_831=READY door=detached cube=detached" +
            ready_mode + ready_claim + f" socket={args.socket}", flush=True)

        while stop_signal is None and not clean_shutdown:
            latch.require_clear()
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            except OSError as error:
                raise DaemonError(f"SOCKET_ACCEPT:{error}") from error
            with connection:
                response = ""
                command = ""
                try:
                    _require(_peer_uid(connection) == os.geteuid(),
                             "SOCKET_PEER_UID")
                    raw_request = _recv_request(
                        connection, args.request_timeout)
                    if raw_request.startswith(b"claim-ready "):
                        command = "claim-ready"
                        _require(args.require_ready_claim and args.roundtrip,
                                 "READY_CLAIM_NOT_CONFIGURED")
                        nonce = _parse_ready_claim(
                            raw_request, args.authority_run_id)
                        ownership_sha256 = engine.claim_ready()
                        socket_metadata = _verified_socket_path_metadata(
                            args.socket, metadata)
                        daemon_pid = os.getpid()
                        daemon_start_ticks = _process_start_ticks(
                            daemon_pid, "READY_CLAIM_DAEMON")
                        response = (
                            "H1_FAST_SUPPORT_DAEMON_831_CLAIM=PASS "
                            f"run_id={args.authority_run_id} nonce={nonce} "
                            f"daemon_pid={daemon_pid} "
                            f"daemon_start_ticks={daemon_start_ticks} "
                            f"socket_dev={socket_metadata.st_dev} "
                            f"socket_ino={socket_metadata.st_ino} "
                            f"ownership_sha256={ownership_sha256} "
                            "door=detached cube=detached latch=locked "
                            "robot=launch cube_pose=origin publishers=0")
                    else:
                        command = _parse_request(
                            raw_request, roundtrip=args.roundtrip)
                        if command != "shutdown":
                            latch.require_clear()
                        response = engine.dispatch(command)
                        clean_shutdown = command == "shutdown"
                except BaseException as error:
                    latch.fail(str(error))
                    response = (
                        "H1_SUPPORT_ORCHESTRATOR_828=FAIL:" +
                        str(error).replace("\n", "_").replace("\r", "_")[:3000])
                encoded = (response + "\n").encode("utf-8")
                _require(len(encoded) <= MAX_RESPONSE_BYTES,
                         "PROTOCOL:RESPONSE_SIZE")
                connection.sendall(encoded)
                markers = (ROUNDTRIP_PASS_MARKERS if args.roundtrip else
                           PASS_MARKERS)
                if (response == markers.get(command) or
                        (command == "claim-ready" and response.startswith(
                            "H1_FAST_SUPPORT_DAEMON_831_CLAIM=PASS "))):
                    print(
                        "H1_FAST_SUPPORT_DAEMON_831_REQUEST=PASS "
                        f"command={command}", flush=True)
                else:
                    print(
                        "H1_FAST_SUPPORT_DAEMON_831_REQUEST=FAIL "
                        f"command={command or 'unparsed'} "
                        f"reason_sha256={_digest_bytes(response.encode('utf-8'))}",
                        flush=True)
        if stop_signal is not None:
            raise DaemonError(f"SIGNAL:{stop_signal}")
        return 0
    except (DaemonError, OSError, ValueError, RuntimeError) as error:
        print(f"H1_FAST_SUPPORT_DAEMON_831=FAIL:{error}",
              file=sys.stderr, flush=True)
        return 1
    finally:
        if server is not None:
            server.close()
        if telemetry is not None:
            telemetry.close()
        for listener in reversed(listeners):
            listener.close()
        if pins is not None:
            pins.close()
        # Socket disappearance is the flow's shutdown-complete edge.  Keep it
        # present until every telemetry/listener/security resource is closed,
        # so MOTION_WINDOW_END includes the daemon cleanup tail.
        if socket_created:
            try:
                args.socket.unlink()
            except FileNotFoundError:
                pass
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)


def _request(args: argparse.Namespace) -> int:
    commands = ROUNDTRIP_COMMANDS if args.roundtrip else COMMANDS
    _require(args.command in commands,
             f"PROTOCOL:COMMAND:{args.command}")
    _require(args.socket.is_absolute(), "SOCKET_PATH_NOT_ABSOLUTE")
    try:
        metadata = args.socket.lstat()
    except OSError as error:
        raise DaemonError(f"SOCKET_LSTAT:{error}") from error
    _require(stat.S_ISSOCK(metadata.st_mode) and not args.socket.is_symlink() and
             metadata.st_uid == os.geteuid() and
             stat.S_IMODE(metadata.st_mode) == 0o600,
             "SOCKET_METADATA")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(args.timeout)
        client.connect(str(args.socket))
        client.sendall((args.command + "\n").encode("ascii"))
        response = bytearray()
        while b"\n" not in response:
            block = client.recv(MAX_RESPONSE_BYTES + 1 - len(response))
            _require(bool(block), "PROTOCOL:RESPONSE_EOF")
            response.extend(block)
            _require(len(response) <= MAX_RESPONSE_BYTES,
                     "PROTOCOL:RESPONSE_SIZE")
    _require(response.endswith(b"\n") and response.count(b"\n") == 1,
             "PROTOCOL:RESPONSE_FRAME")
    try:
        line = bytes(response[:-1]).decode("utf-8")
    except UnicodeDecodeError as error:
        raise DaemonError(f"PROTOCOL:RESPONSE_UTF8:{error}") from error
    print(line, flush=True)
    markers = ROUNDTRIP_PASS_MARKERS if args.roundtrip else PASS_MARKERS
    return 0 if line == markers[args.command] else 1


def _self_test() -> int:
    expected = [
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-attach "
        "door=attached cube=detached boundary=E0_ENTRY",
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-attach "
        "door=attached cube=attached boundary=CUBE_CONTACT",
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-detach "
        "door=attached cube=detached boundary=CUBE_RELEASE",
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-detach "
        "door=detached cube=detached boundary=E0_RETURN",
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize door=q0 "
        "handle=q0 latch=locked supports=detached robot=launch publishers=0",
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=shutdown execution=none",
    ]
    _require([PASS_MARKERS[item] for item in COMMANDS] == expected,
             "SELF_TEST:MARKERS")
    state = OperationState()
    for command in (*TRANSITION_COMMANDS, "finalize"):
        state.admit(command)
        state.commit(command)
    try:
        state.admit("finalize")
    except DaemonError as error:
        _require(str(error) == "PROTOCOL:FINALIZE_ALREADY_DONE",
                 "SELF_TEST:FINALIZE_STATUS")
    else:
        raise DaemonError("SELF_TEST:FINALIZE_REPEATED")
    state.admit("shutdown")
    state.commit("shutdown")
    wrong = OperationState()
    try:
        wrong.admit("cube-attach")
    except DaemonError as error:
        _require(str(error).startswith("PROTOCOL:ORDER"),
                 "SELF_TEST:ORDER_STATUS")
    else:
        raise DaemonError("SELF_TEST:ORDER_ACCEPTED")
    roundtrip = OperationState(roundtrip=True)
    for command in (*ROUNDTRIP_PROTOCOL, "finalize", "shutdown"):
        roundtrip.admit(command)
        _require(command in ROUNDTRIP_PASS_MARKERS,
                 f"SELF_TEST:ROUNDTRIP_MARKER:{command}")
        roundtrip.commit(command)
    _require(roundtrip.finalized, "SELF_TEST:ROUNDTRIP_FINALIZE")
    premature_return = OperationState(roundtrip=True)
    try:
        premature_return.admit("door-attach-return")
    except DaemonError as error:
        _require(str(error).startswith("PROTOCOL:ORDER"),
                 "SELF_TEST:ROUNDTRIP_ORDER_STATUS")
    else:
        raise DaemonError("SELF_TEST:ROUNDTRIP_ORDER_ACCEPTED")
    for command in COMMANDS:
        _require(_parse_request((command + "\n").encode("ascii")) == command,
                 f"SELF_TEST:REQUEST:{command}")
    for malformed in (b"", b"door-attach", b"door-attach extra\n",
                      b"door-attach\ntrailing"):
        try:
            _parse_request(malformed)
        except DaemonError:
            pass
        else:
            raise DaemonError("SELF_TEST:MALFORMED_ACCEPTED")
    for command in ROUNDTRIP_COMMANDS:
        _require(
            _parse_request(
                (command + "\n").encode("ascii"), roundtrip=True) ==
            command,
            f"SELF_TEST:ROUNDTRIP_REQUEST:{command}")
    _require(NativeStateListener._parse_line('data: "attached"') == "attached",
             "SELF_TEST:STATE_ATTACHED")
    _require(NativeStateListener._parse_line("data: detached") == "detached",
             "SELF_TEST:STATE_DETACHED")
    print(
        "H1_FAST_SUPPORT_DAEMON_831_SELF_TEST=PASS "
        "commands=6 ros_publishers=0 failure_latch=enabled", flush=True)
    return 0


def _pdeath_native_state_listener(argv: Sequence[str]) -> int:
    """Arm Linux parent-death SIGKILL, then replace this helper with gz."""
    _require(len(argv) == 2, "INTERNAL_LISTENER:ARGUMENTS")
    try:
        expected_parent = int(argv[0])
    except ValueError as error:
        raise DaemonError("INTERNAL_LISTENER:PARENT_PID") from error
    topic = argv[1]
    _require(expected_parent > 1, "INTERNAL_LISTENER:PARENT_PID")
    _require(topic in (
        "/h1_2/right_door_support/state",
        "/h1_2/left_pose_grasp/state",
    ), "INTERNAL_LISTENER:TOPIC")
    try:
        import ctypes

        libc = ctypes.CDLL(None, use_errno=True)
        result = libc.prctl(1, int(signal.SIGKILL), 0, 0, 0)
        if result != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number))
    except (ImportError, OSError) as error:
        raise DaemonError(f"INTERNAL_LISTENER:PDEATHSIG:{error}") from error
    # Close the race in which the daemon died before PR_SET_PDEATHSIG.
    _require(os.getppid() == expected_parent,
             "INTERNAL_LISTENER:PARENT_REPLACED")
    os.execvp("gz", ["gz", "topic", "-e", "-t", topic])
    raise DaemonError("INTERNAL_LISTENER:EXEC_RETURNED")


def _pdeath_gz_support_mutation(argv: Sequence[str]) -> int:
    """Arm exact-parent death before the one permitted support mutation."""
    _require(len(argv) == 3, "INTERNAL_MUTATION:ARGUMENTS")
    try:
        expected_parent = int(argv[0])
        expected_start_ticks = int(argv[1])
    except ValueError as error:
        raise DaemonError("INTERNAL_MUTATION:PARENT_IDENTITY") from error
    topic = argv[2]
    _require(expected_parent > 1 and expected_start_ticks > 0,
             "INTERNAL_MUTATION:PARENT_IDENTITY")
    _require(topic in (
        "/h1_2/right_door_support/attach",
        "/h1_2/right_door_support/detach",
        "/h1_2/left_pose_grasp/attach",
        "/h1_2/left_pose_grasp/detach",
    ), "INTERNAL_MUTATION:TOPIC")
    _require(os.getppid() == expected_parent,
             "INTERNAL_MUTATION:PARENT_REPLACED_BEFORE_ARM")
    _require(
        _process_start_ticks(
            expected_parent, "INTERNAL_MUTATION:PARENT_BEFORE") ==
        expected_start_ticks,
        "INTERNAL_MUTATION:PARENT_START_BEFORE")
    try:
        import ctypes

        libc = ctypes.CDLL(None, use_errno=True)
        result = libc.prctl(1, int(signal.SIGKILL), 0, 0, 0)
        if result != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number))
    except (ImportError, OSError) as error:
        raise DaemonError(f"INTERNAL_MUTATION:PDEATHSIG:{error}") from error
    _require(os.getppid() == expected_parent,
             "INTERNAL_MUTATION:PARENT_REPLACED_AFTER_ARM")
    _require(
        _process_start_ticks(
            expected_parent, "INTERNAL_MUTATION:PARENT_AFTER") ==
        expected_start_ticks,
        "INTERNAL_MUTATION:PARENT_START_AFTER")
    os.execvp("gz", [
        "gz", "topic", "-t", topic,
        "-m", "gz.msgs.Empty", "-p", "",
    ])
    raise DaemonError("INTERNAL_MUTATION:EXEC_RETURNED")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    serve = subparsers.add_parser("serve")
    serve.add_argument("--socket", type=Path, required=True)
    serve.add_argument("--owner-pid", type=int, required=True)
    serve.add_argument("--owner-start-ticks", type=int, required=True)
    serve.add_argument("--support-orchestrator", type=Path,
                       default=DEFAULT_SUPPORT_SOURCE)
    serve.add_argument("--support-orchestrator-sha256", required=True)
    serve.add_argument("--manifest", type=Path, required=True)
    serve.add_argument("--manifest-sha256", required=True)
    serve.add_argument("--certificate", type=Path, required=True)
    serve.add_argument("--certificate-sha256", required=True)
    serve.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    serve.add_argument("--timeout", type=float, default=8.0)
    serve.add_argument("--startup-timeout", type=float, default=30.0)
    serve.add_argument("--request-timeout", type=float, default=5.0)
    serve.add_argument("--telemetry-max-age", type=float, default=1.0)
    serve.add_argument("--roundtrip", action="store_true")
    serve.add_argument("--authority-run-id", default="")
    serve.add_argument("--require-ready-claim", action="store_true")

    request = subparsers.add_parser("request")
    request.add_argument("--socket", type=Path, required=True)
    request.add_argument("command", choices=ALL_COMMANDS)
    request.add_argument("--timeout", type=float, default=30.0)
    request.add_argument("--roundtrip", action="store_true")
    subparsers.add_parser("self-test")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    actual_argv = list(sys.argv[1:] if argv is None else argv)
    if (actual_argv and
            actual_argv[0] == INTERNAL_NATIVE_LISTENER_MODE):
        try:
            return _pdeath_native_state_listener(actual_argv[1:])
        except (DaemonError, OSError, ValueError, RuntimeError) as error:
            print(f"H1_FAST_SUPPORT_DAEMON_831_INTERNAL=FAIL:{error}",
                  file=sys.stderr, flush=True)
            return 1
    if actual_argv and actual_argv[0] == INTERNAL_PDEATH_GZ_MODE:
        try:
            return _pdeath_gz_support_mutation(actual_argv[1:])
        except (DaemonError, OSError, ValueError, RuntimeError) as error:
            print(f"H1_FAST_SUPPORT_DAEMON_831_INTERNAL=FAIL:{error}",
                  file=sys.stderr, flush=True)
            return 1
    args = _parser().parse_args(actual_argv)
    try:
        if args.mode == "self-test":
            return _self_test()
        if args.mode == "request":
            _require(math.isfinite(args.timeout) and 1.0 <= args.timeout <= 120.0,
                     "TIMEOUT")
            return _request(args)
        _require(args.socket.is_absolute() and
                 args.support_orchestrator.is_absolute() and
                 args.manifest.is_absolute() and
                 args.certificate.is_absolute() and
                 args.workspace.is_absolute(), "PATH_NOT_ABSOLUTE")
        _require(
            (args.owner_pid == 0 and args.owner_start_ticks == 0) or
            (args.owner_pid > 1 and args.owner_start_ticks > 0),
            "OWNER_ARGUMENT")
        _require(_is_digest(args.support_orchestrator_sha256) and
                 _is_digest(args.manifest_sha256) and
                 _is_digest(args.certificate_sha256), "SHA256_ARGUMENT")
        _require(math.isfinite(args.timeout) and 3.0 <= args.timeout <= 30.0,
                 "TIMEOUT")
        _require(math.isfinite(args.startup_timeout) and
                 5.0 <= args.startup_timeout <= 120.0, "STARTUP_TIMEOUT")
        _require(math.isfinite(args.request_timeout) and
                 1.0 <= args.request_timeout <= 30.0, "REQUEST_TIMEOUT")
        _require(math.isfinite(args.telemetry_max_age) and
                 0.05 <= args.telemetry_max_age <= 2.0,
                 "TELEMETRY_MAX_AGE")
        if args.require_ready_claim:
            _require(args.roundtrip and _is_digest(args.authority_run_id),
                     "READY_CLAIM_ARGUMENTS")
        else:
            _require(args.authority_run_id == "",
                     "READY_CLAIM_RUN_ID_WITHOUT_REQUIREMENT")
        return _serve(args)
    except (DaemonError, OSError, ValueError, RuntimeError) as error:
        prefix = ("H1_SUPPORT_ORCHESTRATOR_828" if
                  getattr(args, "mode", None) == "request" else
                  "H1_FAST_SUPPORT_DAEMON_831")
        print(f"{prefix}=FAIL:{error}",
              file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
