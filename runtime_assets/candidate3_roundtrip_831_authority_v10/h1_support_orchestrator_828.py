#!/usr/bin/python3
"""Fail-closed Gazebo support orchestration for the certified 828 flow.

The program owns only the two compliant support plugins.  It never publishes a
robot trajectory, commands an incubator joint, or changes a model pose.  Every
operation is admitted by a mode-0400 support manifest and the exact middle
certificate.  ``preload`` must run before any support-state bridge/listener is
started; all later operations require its same-Gazebo ownership record.
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
import selectors
import stat
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence


MANIFEST_SCHEMA = "h1_support_orchestration_828/v1"
CONTRACT_SCHEMA = "h1_support_contract_828/v1"
EXPECTED_PHASES = (
    "door_pre_support",
    "door_open_supported",
    "door60_seam_to_cube",
    "left_cube_ready",
    "cube_contact",
    "cube_carry_place",
    "cube_release",
    "cube_retreat",
    "left_cube_park",
    "door60_seam_to_hinge",
    "door_close_supported",
)

RIGHT_JOINTS = (
    "torso_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "right_dex1_finger_1_joint",
    "right_dex1_finger_2_joint",
)
LEFT_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "left_dex1_finger_1_joint",
    "left_dex1_finger_2_joint",
)
ALL_ROBOT_JOINTS = (
    "torso_joint",
    *LEFT_JOINTS[:7],
    *RIGHT_JOINTS[1:8],
    *LEFT_JOINTS[7:],
    *RIGHT_JOINTS[8:],
)
TRAJECTORY_TOPICS = (
    "/torso_controller/joint_trajectory",
    "/right_arm_controller/joint_trajectory",
    "/right_gripper_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
)
TRAJECTORY_TOPIC_DISCOVERY_ATTEMPTS = 4
TRAJECTORY_TOPIC_DISCOVERY_SPIN_SECONDS = 2.0
TRAJECTORY_TOPIC_DISCOVERY_RETRY_DELAY_SECONDS = 0.25

CANONICAL_E0 = {
    "torso_joint": -0.04773321113053429,
    "right_shoulder_pitch_joint": -0.41373146194762,
    "right_shoulder_roll_joint": 0.23755434038212944,
    "right_shoulder_yaw_joint": 1.1008080707922079,
    "right_elbow_joint": 0.8747783439736734,
    "right_wrist_roll_joint": -0.3046852067271319,
    "right_wrist_pitch_joint": 0.4094159191441438,
    "right_wrist_yaw_joint": -0.9999635680199944,
    "right_dex1_finger_1_joint": 0.0108,
    "right_dex1_finger_2_joint": 0.007,
}
CANONICAL_FINAL_ROBOT = {
    "torso_joint": 0.0,
    "left_shoulder_pitch_joint": -0.000012933885986937658,
    "left_shoulder_roll_joint": 0.24998646984876535,
    "left_shoulder_yaw_joint": 0.000023774721497885907,
    "left_elbow_joint": 0.6000753185207717,
    "left_wrist_roll_joint": 0.000023572560034304773,
    "left_wrist_pitch_joint": -0.00008730492997967213,
    "left_wrist_yaw_joint": -0.00001654021997178485,
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": -0.25,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.60,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
    "left_dex1_finger_1_joint": -0.019,
    "left_dex1_finger_2_joint": -0.019,
    "right_dex1_finger_1_joint": -0.019,
    "right_dex1_finger_2_joint": -0.019,
}
CANONICAL_MIDDLE_SEAM = {
    **CANONICAL_E0,
    "left_shoulder_pitch_joint": -0.000012933885986937658,
    "left_shoulder_roll_joint": 0.316,
    "left_shoulder_yaw_joint": 0.000023774721497885907,
    "left_elbow_joint": 0.6000753185207717,
    "left_wrist_roll_joint": 0.000023572560034304773,
    "left_wrist_pitch_joint": -0.00008730492997967213,
    "left_wrist_yaw_joint": -0.00001654021997178485,
    "left_dex1_finger_1_joint": -0.019,
    "left_dex1_finger_2_joint": -0.019,
}

WORLD_NAME = "h1_world"
POSE_TOPIC = f"/world/{WORLD_NAME}/pose/info"
ADD_SERVICE = f"/world/{WORLD_NAME}/entity/system/add"
LATCH_TOPIC = "/incubator/latch/state"
DOOR_STATE_TOPIC = "/h1_2/right_door_support/state"
DOOR_ATTACH_TOPIC = "/h1_2/right_door_support/attach"
DOOR_DETACH_TOPIC = "/h1_2/right_door_support/detach"
CUBE_STATE_TOPIC = "/h1_2/left_pose_grasp/state"
CUBE_ATTACH_TOPIC = "/h1_2/left_pose_grasp/attach"
CUBE_DETACH_TOPIC = "/h1_2/left_pose_grasp/detach"

DOOR_CLOSED_TOLERANCE = 0.003490658503989
DOOR_OPEN_TOLERANCE = 0.034906585039887
HANDLE_TOLERANCE = 0.052359877559830
MECHANISM_VELOCITY_TOLERANCE = 0.002
ROBOT_ENDPOINT_TOLERANCE = 0.003
RIGHT_FINGER_ENDPOINT_TOLERANCE = 0.0015
CUBE_POSITION_TOLERANCE = 0.003
CUBE_ORIENTATION_TOLERANCE = 0.034906585039887
MAX_BOUND_SNAPSHOT_BYTES = 16 * 1024 * 1024
MAX_COMMAND_OUTPUT_BYTES = 16 * 1024 * 1024
MAX_LISTENER_BUFFER_BYTES = 1024 * 1024
SNAPSHOT_FRESH_TIMEOUT_SECONDS = 10
SNAPSHOT_PROCESS_TIMEOUT_SECONDS = 12.0

DEFAULT_WORKSPACE = Path(
    "/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")


class SupportError(RuntimeError):
    """Raised when an operation cannot be admitted or proved."""


def _kill_process_group(process: subprocess.Popen) -> None:
    # The session leader may have exited while descendants still hold inherited
    # pipe descriptors.  Always address its process group before deciding
    # whether the leader itself still needs to be reaped.
    try:
        os.killpg(process.pid, 9)
    except ProcessLookupError:
        pass
    if process.poll() is None:
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            pass


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise SupportError(status)


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8")


def _is_digest(value: Any) -> bool:
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _stat_identity(item: os.stat_result) -> tuple[int, int, int, int, int]:
    return (item.st_dev, item.st_ino, item.st_size, item.st_mtime_ns, item.st_ctime_ns)


def _no_duplicate_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise SupportError(f"JSON_DUPLICATE_KEY:{key}")
        result[key] = value
    return result


_ANSI_ESCAPE = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def _is_diagnostic_line(line: str) -> bool:
    """Accept only bounded, recognizable informational output."""
    clean = _ANSI_ESCAPE.sub("", line).strip()
    return (not clean or
            re.match(
                (r"^(?:\[(?:INFO|WARN|WARNING)\]|INFO:|"
                 r"WARN(?:ING)?(?::|\s|\[))"),
                clean, re.IGNORECASE) is not None)


def _state_from_line(
    line: str, allowed: Sequence[str], status: str,
) -> str | None:
    match = re.fullmatch(
        r"\s*data:\s*(?:\"([A-Za-z0-9_+-]+)\"|"
        r"'([A-Za-z0-9_+-]+)'|([A-Za-z0-9_+-]+))\s*",
        line)
    if match is not None:
        value = next(item for item in match.groups() if item is not None)
        _require(value in allowed, f"{status}:UNKNOWN:{value}")
        return value
    _require(_is_diagnostic_line(line), f"{status}:MALFORMED_LINE")
    return None


def _last_state_message(
    output: str, allowed: Sequence[str], status: str,
) -> str:
    observed = []
    for line in output.splitlines():
        value = _state_from_line(line, allowed, status)
        if value is not None:
            observed.append(value)
    _require(bool(observed), f"{status}:NO_FRAME")
    return observed[-1]


def _read_listener_bytes(stream: Any, buffer: bytearray, status: str) -> bool:
    """Drain a nonblocking pipe; return true only after observing EOF."""
    while True:
        try:
            block = os.read(stream.fileno(), 65536)
        except BlockingIOError:
            return False
        except OSError as error:
            raise SupportError(f"{status}:READ:{error}") from error
        if not block:
            return True
        buffer.extend(block)
        _require(len(buffer) <= MAX_LISTENER_BUFFER_BYTES,
                 f"{status}:BUFFER_LIMIT")


def _pop_listener_line(
    buffer: bytearray, *, eof: bool, status: str,
) -> str | None:
    newline = buffer.find(b"\n")
    if newline < 0:
        if not eof or not buffer:
            return None
        raw = bytes(buffer)
        buffer.clear()
    else:
        raw = bytes(buffer[:newline])
        del buffer[:newline + 1]
    try:
        return raw.rstrip(b"\r").decode("utf-8")
    except UnicodeDecodeError as error:
        raise SupportError(f"{status}:UTF8:{error}") from error


def _wait_listener_state(
    process: subprocess.Popen, selector: selectors.BaseSelector,
    buffer: bytearray, expected: str, allowed: Sequence[str],
    deadline: float, status: str,
) -> None:
    """Consume buffered frames before waiting for more pipe readiness."""
    eof = False
    latest_state = None
    while True:
        while True:
            line = _pop_listener_line(buffer, eof=eof, status=status)
            if line is None:
                break
            value = _state_from_line(line, allowed, status)
            if value is not None:
                latest_state = value
        # Decide only after draining every complete frame from the current
        # kernel read.  This makes the newest state authoritative and never
        # treats a stale pre-command frame as a post-command acknowledgement.
        if not buffer and latest_state == expected:
            return
        if eof:
            raise SupportError(f"{status}:EOF")
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise SupportError(f"{status}:TIMEOUT:{expected}")
        events = selector.select(remaining)
        if events:
            eof = _read_listener_bytes(process.stdout, buffer, status)
            continue
        if process.poll() is not None:
            eof = _read_listener_bytes(process.stdout, buffer, status)
            if not eof:
                eof = True


def _bounded_process_output(
    process: subprocess.Popen, timeout_seconds: float, command_name: str,
) -> tuple[str, str]:
    """Drain subprocess pipes concurrently with a hard per-stream limit."""
    stdout = process.stdout
    _require(stdout is not None, f"COMMAND_STDOUT:{command_name}")
    streams = [("stdout", stdout, bytearray())]
    if process.stderr is not None:
        streams.append(("stderr", process.stderr, bytearray()))
    selector = selectors.DefaultSelector()
    deadline = time.monotonic() + timeout_seconds
    try:
        for label, stream, buffer in streams:
            os.set_blocking(stream.fileno(), False)
            selector.register(
                stream, selectors.EVENT_READ, (label, buffer))
        while selector.get_map():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise subprocess.TimeoutExpired(command_name, timeout_seconds)
            events = selector.select(remaining)
            if not events:
                raise subprocess.TimeoutExpired(command_name, timeout_seconds)
            for key, _mask in events:
                label, buffer = key.data
                while True:
                    try:
                        block = os.read(key.fileobj.fileno(), 65536)
                    except BlockingIOError:
                        break
                    if not block:
                        selector.unregister(key.fileobj)
                        break
                    if len(buffer) + len(block) > MAX_COMMAND_OUTPUT_BYTES:
                        _kill_process_group(process)
                        raise SupportError(
                            f"COMMAND_OUTPUT_LIMIT:{command_name}:{label}")
                    buffer.extend(block)
        remaining = deadline - time.monotonic()
        if process.poll() is None:
            if remaining <= 0.0:
                raise subprocess.TimeoutExpired(command_name, timeout_seconds)
            process.wait(timeout=remaining)
    finally:
        selector.close()
        for _label, stream, _buffer in streams:
            stream.close()

    decoded = []
    for label, _stream, buffer in streams:
        try:
            decoded.append(bytes(buffer).decode("utf-8"))
        except UnicodeDecodeError as error:
            raise SupportError(
                f"COMMAND_UTF8:{command_name}:{label}:{error}") from error
    if len(decoded) == 1:
        decoded.append("")
    return decoded[0], decoded[1]


@dataclass(frozen=True)
class SealedJson:
    path: Path
    sha256: str
    value: Mapping[str, Any]


def load_sealed_json(path: Path, expected_sha256: str) -> SealedJson:
    """Read one owned mode-0400 JSON file without following a symlink."""
    _require(path.is_absolute(), "SEALED_PATH_NOT_ABSOLUTE")
    _require(_is_digest(expected_sha256), "SEALED_EXPECTED_SHA256")
    try:
        metadata = path.lstat()
    except OSError as error:
        raise SupportError(f"SEALED_LSTAT:{error}") from error
    _require(stat.S_ISREG(metadata.st_mode) and not path.is_symlink(),
             "SEALED_NOT_REGULAR")
    _require(metadata.st_uid == os.geteuid(), "SEALED_OWNER")
    _require(stat.S_IMODE(metadata.st_mode) == 0o400, "SEALED_MODE")
    _require(metadata.st_nlink == 1 and metadata.st_size > 0, "SEALED_IDENTITY")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise SupportError(f"SEALED_OPEN:{error}") from error
    try:
        before = os.fstat(descriptor)
        _require(_stat_identity(before) == _stat_identity(metadata), "SEALED_RACE_BEFORE")
        remaining = before.st_size
        chunks: list[bytes] = []
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            _require(bool(block), "SEALED_SHORT_READ")
            chunks.append(block)
            remaining -= len(block)
        _require(not os.read(descriptor, 1), "SEALED_GREW")
        _require(_stat_identity(before) == _stat_identity(os.fstat(descriptor)),
                 "SEALED_RACE_AFTER")
    finally:
        os.close(descriptor)
    raw = b"".join(chunks)
    actual_sha256 = _sha256_bytes(raw)
    _require(actual_sha256 == expected_sha256, "SEALED_SHA256")
    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_no_duplicate_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                SupportError(f"JSON_NONFINITE:{token}")))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SupportError(f"SEALED_JSON:{error}") from error
    _require(isinstance(value, Mapping), "SEALED_JSON_ROOT")
    return SealedJson(path=path, sha256=actual_sha256, value=value)


@dataclass(frozen=True)
class SupportContract:
    manifest_sha256: str
    certificate_sha256: str
    plan_sha256: str
    exact_result_sha256: str
    support_contract_sha256: str
    plugin_sha256: str
    snapshot_relative_path: str
    snapshot_sha256: str
    value: Mapping[str, Any]

    @property
    def boundaries(self) -> Mapping[str, Any]:
        return self.value["boundaries"]


def _validate_pose(pose: Any, label: str) -> None:
    _require(isinstance(pose, Mapping), f"{label}_TYPE")
    position = pose.get("position")
    orientation = pose.get("orientation")
    _require(isinstance(position, list) and len(position) == 3 and
             all(_finite(item) for item in position), f"{label}_POSITION")
    _require(isinstance(orientation, list) and len(orientation) == 4 and
             all(_finite(item) for item in orientation), f"{label}_ORIENTATION")
    norm = math.sqrt(sum(float(item) ** 2 for item in orientation))
    _require(abs(norm - 1.0) <= 1e-6, f"{label}_QUATERNION_NORM")


def _validate_positions(boundary: Mapping[str, Any], names: tuple[str, ...], label: str) -> None:
    positions = boundary.get("robot_positions")
    _require(isinstance(positions, Mapping) and set(positions) == set(names),
             f"{label}_JOINT_SET")
    _require(all(_finite(positions[name]) for name in names), f"{label}_JOINT_VALUE")


def _workspace_relative_path(value: Any, label: str) -> str:
    """Admit one canonical POSIX path below the selected workspace."""
    _require(isinstance(value, str) and 1 <= len(value) <= 4096,
             f"{label}_TYPE")
    _require(re.fullmatch(r"[A-Za-z0-9._+/-]+", value) is not None,
             f"{label}_CHARACTERS")
    relative = Path(value)
    _require(not relative.is_absolute(), f"{label}_ABSOLUTE")
    _require(relative.as_posix() == value and
             all(part not in ("", ".", "..") for part in relative.parts),
             f"{label}_ALIAS")
    return value


def validate_manifest_and_certificate(
        manifest: SealedJson, certificate: SealedJson) -> SupportContract:
    """Validate the immutable support topology and its exact-audit binding."""
    value = manifest.value
    _require("execution_ready" not in value, "MANIFEST_PREMATURE_EXECUTION_READY")
    _require(set(value) == {
        "schema", "controller_execution_authorized", "live_execution_passed",
        "bindings", "support_contract",
    }, "MANIFEST_KEYS")
    _require(value.get("schema") == MANIFEST_SCHEMA, "MANIFEST_SCHEMA")
    _require(value.get("controller_execution_authorized") is True,
             "MANIFEST_CONTROLLER_NOT_AUTHORIZED")
    _require(value.get("live_execution_passed") is False,
             "MANIFEST_LIVE_STATUS_NOT_PRERUN")
    bindings = value.get("bindings")
    contract = value.get("support_contract")
    _require(isinstance(bindings, Mapping), "MANIFEST_BINDINGS")
    _require(isinstance(contract, Mapping), "MANIFEST_SUPPORT_CONTRACT")
    _require(set(bindings) == {
        "plan_sha256", "plan_schema", "certificate_sha256",
        "certificate_schema", "exact_result_sha256", "exact_result_schema",
        "auditor_version", "support_contract_sha256",
    }, "MANIFEST_BINDING_KEYS")
    for name in ("plan_sha256", "certificate_sha256", "exact_result_sha256"):
        _require(_is_digest(bindings.get(name)), f"BINDING_{name.upper()}")
    _require(_is_digest(bindings.get("support_contract_sha256")),
             "BINDING_SUPPORT_CONTRACT_SHA256")
    for name in (
            "plan_schema", "certificate_schema", "exact_result_schema",
            "auditor_version"):
        field = bindings.get(name)
        _require(isinstance(field, str) and 1 <= len(field) <= 255 and
                 re.fullmatch(r"[A-Za-z0-9_./:+-]+", field) is not None,
                 f"BINDING_{name.upper()}")
    _require(bindings["certificate_sha256"] == certificate.sha256,
             "CERTIFICATE_FILE_BINDING")

    _require(set(contract) == {
        "schema", "phases", "world", "hooks", "boundaries", "plugin", "tools",
    }, "CONTRACT_KEYS")
    _require(contract.get("schema") == CONTRACT_SCHEMA, "CONTRACT_SCHEMA")
    _require(tuple(contract.get("phases", ())) == EXPECTED_PHASES, "CONTRACT_PHASES")
    world = contract.get("world")
    _require(isinstance(world, Mapping), "CONTRACT_WORLD")
    expected_world = {
        "world_name": WORLD_NAME,
        "table_x": 0.600,
        "incubator_x": 0.590,
        "incubator_y": 0.0,
        "incubator_z": 0.815,
        "cube_spawn_x": 0.334,
        "cube_spawn_y": 0.377,
        "cube_spawn_z": 0.830,
        "cube_spawn_yaw": math.pi / 2.0,
    }
    _require(set(world) == set(expected_world), "CONTRACT_WORLD_KEYS")
    for name, expected in expected_world.items():
        actual = world[name]
        if isinstance(expected, str):
            _require(actual == expected, f"CONTRACT_WORLD_{name.upper()}")
        else:
            _require(_finite(actual) and abs(float(actual) - expected) <= 1e-12,
                     f"CONTRACT_WORLD_{name.upper()}")

    hooks = contract.get("hooks")
    expected_hooks = [
        {"operation": "door-attach", "boundary": "E0_ENTRY",
         "phase_index": 0, "edge": "entry", "door_before": "detached",
         "door_after": "attached", "cube_before": "detached", "cube_after": "detached"},
        {"operation": "cube-attach", "boundary": "CUBE_CONTACT",
         "phase_index": 4, "edge": "exit", "door_before": "attached",
         "door_after": "attached", "cube_before": "detached", "cube_after": "attached"},
        {"operation": "cube-detach", "boundary": "CUBE_RELEASE",
         "phase_index": 5, "edge": "exit", "door_before": "attached",
         "door_after": "attached", "cube_before": "attached", "cube_after": "detached"},
        {"operation": "door-detach", "boundary": "E0_RETURN",
         "phase_index": 10, "edge": "exit", "door_before": "attached",
         "door_after": "detached", "cube_before": "detached", "cube_after": "detached"},
    ]
    _require(hooks == expected_hooks, "CONTRACT_HOOK_TOPOLOGY")

    boundaries = contract.get("boundaries")
    _require(isinstance(boundaries, Mapping) and set(boundaries) == {
        "E0_ENTRY", "E0_RETURN", "CUBE_CONTACT", "CUBE_RELEASE", "FINAL_ROBOT"},
             "CONTRACT_BOUNDARIES")
    for name in ("E0_ENTRY", "E0_RETURN"):
        boundary = boundaries[name]
        _require(isinstance(boundary, Mapping), f"{name}_TYPE")
        _validate_positions(boundary, ALL_ROBOT_JOINTS, name)
        for joint, expected in CANONICAL_MIDDLE_SEAM.items():
            _require(abs(float(boundary["robot_positions"][joint]) - expected) <= 1e-12,
                     f"{name}_CANONICAL_E0_{joint}")
    for name in ("CUBE_CONTACT", "CUBE_RELEASE"):
        boundary = boundaries[name]
        _require(isinstance(boundary, Mapping), f"{name}_TYPE")
        _validate_positions(boundary, LEFT_JOINTS, name)
        _validate_pose(boundary.get("cube_pose"), f"{name}_CUBE_POSE")
    final_boundary = boundaries["FINAL_ROBOT"]
    _require(isinstance(final_boundary, Mapping), "FINAL_ROBOT_TYPE")
    _validate_positions(final_boundary, ALL_ROBOT_JOINTS, "FINAL_ROBOT")
    for joint, expected in CANONICAL_FINAL_ROBOT.items():
        _require(abs(float(final_boundary["robot_positions"][joint]) - expected) <= 1e-12,
                 f"FINAL_ROBOT_CANONICAL_{joint}")

    plugin = contract.get("plugin")
    _require(isinstance(plugin, Mapping), "CONTRACT_PLUGIN")
    _require(set(plugin) == {"binary_sha256", "door", "cube"},
             "CONTRACT_PLUGIN_KEYS")
    _require(_is_digest(plugin.get("binary_sha256")), "CONTRACT_PLUGIN_SHA256")
    _require(plugin.get("door") == {
        "parent_scope": "h1_2::right_wrist_yaw_link",
        "child_scope": "incubator::door_link",
        "state_topic": DOOR_STATE_TOPIC,
        "attach_topic": DOOR_ATTACH_TOPIC,
        "detach_topic": DOOR_DETACH_TOPIC,
        "max_force": 120.0,
        "max_torque": 8.0,
        "gravity_compensation": False,
    }, "CONTRACT_DOOR_PLUGIN")
    _require(plugin.get("cube") == {
        "parent_scope": "h1_2::left_wrist_yaw_link",
        "child_model": "green_cube",
        "state_topic": CUBE_STATE_TOPIC,
        "attach_topic": CUBE_ATTACH_TOPIC,
        "detach_topic": CUBE_DETACH_TOPIC,
        "initially_attached": False,
    }, "CONTRACT_CUBE_PLUGIN")

    tools = contract.get("tools")
    _require(isinstance(tools, Mapping) and set(tools) == {"ros_snapshot"},
             "CONTRACT_TOOLS")
    snapshot = tools["ros_snapshot"]
    _require(isinstance(snapshot, Mapping) and
             set(snapshot) == {"workspace_relative_path", "sha256"},
             "CONTRACT_SNAPSHOT_KEYS")
    snapshot_relative_path = _workspace_relative_path(
        snapshot.get("workspace_relative_path"), "CONTRACT_SNAPSHOT_PATH")
    _require(_is_digest(snapshot.get("sha256")), "CONTRACT_SNAPSHOT_SHA256")

    certificate_value = certificate.value
    _require(certificate_value.get("schema") == bindings["certificate_schema"],
             "CERTIFICATE_SCHEMA_BINDING")
    _require(certificate_value.get("auditor_version") == bindings["auditor_version"],
             "CERTIFICATE_AUDITOR_BINDING")
    _require(certificate_value.get("valid") is True and
             certificate_value.get("status") == "PASS", "CERTIFICATE_NOT_PASS")
    contract_sha256 = _sha256_bytes(_canonical_bytes(contract))
    _require(bindings["support_contract_sha256"] == contract_sha256,
             "BINDING_SUPPORT_CONTRACT")
    _require(certificate_value.get("support_contract_sha256") == contract_sha256,
             "CERTIFICATE_SUPPORT_CONTRACT")
    _require(certificate_value.get("plan_schema") == bindings["plan_schema"],
             "CERTIFICATE_PLAN_SCHEMA_BINDING")
    _require(certificate_value.get("plan_sha256") == bindings["plan_sha256"],
             "CERTIFICATE_PLAN_BINDING")
    _require(certificate_value.get("exact_result_sha256") ==
             bindings["exact_result_sha256"], "CERTIFICATE_RESULT_BINDING")
    _require(certificate_value.get("exact_result_schema") ==
             bindings["exact_result_schema"], "CERTIFICATE_RESULT_SCHEMA_BINDING")
    return SupportContract(
        manifest_sha256=manifest.sha256,
        certificate_sha256=certificate.sha256,
        plan_sha256=bindings["plan_sha256"],
        exact_result_sha256=bindings["exact_result_sha256"],
        support_contract_sha256=contract_sha256,
        plugin_sha256=plugin["binary_sha256"],
        snapshot_relative_path=snapshot_relative_path,
        snapshot_sha256=snapshot["sha256"],
        value=contract,
    )


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def load_bound_snapshot_executable(
        workspace: Path, relative_path: str, expected_sha256: str) -> Path:
    """Hash one contract-bound mode-0500 executable without path aliases."""
    _require(workspace.is_absolute(), "WORKSPACE_NOT_ABSOLUTE")
    _require(_is_digest(expected_sha256), "SNAPSHOT_EXPECTED_SHA256")
    canonical_relative = _workspace_relative_path(
        relative_path, "SNAPSHOT_RELATIVE_PATH")
    try:
        workspace_metadata = workspace.lstat()
        resolved_workspace = workspace.resolve(strict=True)
    except OSError as error:
        raise SupportError(f"WORKSPACE_LSTAT:{error}") from error
    _require(stat.S_ISDIR(workspace_metadata.st_mode) and
             not workspace.is_symlink(), "WORKSPACE_NOT_DIRECTORY")
    _require(resolved_workspace == workspace, "WORKSPACE_PATH_ALIAS")

    candidate = workspace / canonical_relative
    try:
        resolved = candidate.resolve(strict=True)
        metadata = candidate.lstat()
    except OSError as error:
        raise SupportError(f"SNAPSHOT_LSTAT:{error}") from error
    _require(resolved == candidate, "SNAPSHOT_PATH_ALIAS")
    _require(stat.S_ISREG(metadata.st_mode) and not candidate.is_symlink(),
             "SNAPSHOT_NOT_REGULAR")
    _require(metadata.st_uid == os.geteuid(), "SNAPSHOT_OWNER")
    _require(stat.S_IMODE(metadata.st_mode) == 0o500, "SNAPSHOT_MODE")
    _require(metadata.st_nlink == 1 and metadata.st_size > 0,
             "SNAPSHOT_IDENTITY")
    _require(metadata.st_size <= MAX_BOUND_SNAPSHOT_BYTES, "SNAPSHOT_SIZE")

    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(candidate, flags)
    except OSError as error:
        raise SupportError(f"SNAPSHOT_OPEN:{error}") from error
    try:
        before = os.fstat(descriptor)
        _require(_stat_identity(before) == _stat_identity(metadata),
                 "SNAPSHOT_RACE_BEFORE")
        remaining = before.st_size
        digest = hashlib.sha256()
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            _require(bool(block), "SNAPSHOT_SHORT_READ")
            digest.update(block)
            remaining -= len(block)
        _require(not os.read(descriptor, 1), "SNAPSHOT_GREW")
        _require(_stat_identity(before) == _stat_identity(os.fstat(descriptor)),
                 "SNAPSHOT_RACE_AFTER")
        try:
            after_path = candidate.lstat()
        except OSError as error:
            raise SupportError(f"SNAPSHOT_LSTAT_AFTER:{error}") from error
        _require(_stat_identity(metadata) == _stat_identity(after_path),
                 "SNAPSHOT_PATH_RACE_AFTER")
    finally:
        os.close(descriptor)
    _require(digest.hexdigest() == expected_sha256, "SNAPSHOT_SHA256")
    return candidate


class GazeboRuntime:
    """Bounded native-Gazebo/ROS telemetry and support mutations."""

    def __init__(self, contract: SupportContract, workspace: Path,
                 timeout_seconds: float):
        self.contract = contract
        self.workspace = workspace
        self.timeout_seconds = timeout_seconds
        self.plugin = (
            workspace / "install/h1_2_moveit_demo/lib/h1_2_moveit_demo/"
            "libh1_gazebo_compliant_grasp_system.so")
        _require(workspace.is_absolute() and workspace.is_dir(), "WORKSPACE")
        self.snapshot_tool = load_bound_snapshot_executable(
            workspace, contract.snapshot_relative_path, contract.snapshot_sha256)
        _require(self.plugin.is_file(), "PLUGIN_FILE")
        _require(_file_sha256(self.plugin.resolve()) == contract.plugin_sha256,
                 "PLUGIN_SHA256")

    def _bound_snapshot_tool(self) -> Path:
        return load_bound_snapshot_executable(
            self.workspace, self.contract.snapshot_relative_path,
            self.contract.snapshot_sha256)

    def _run(self, command: Sequence[str], timeout: float | None = None,
             *, separate_stderr: bool = False) -> str:
        stderr_target = (subprocess.PIPE if separate_stderr
                         else subprocess.STDOUT)
        try:
            process = subprocess.Popen(
                list(command), stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=stderr_target,
                text=False, bufsize=0,
                start_new_session=True)
        except OSError as error:
            raise SupportError(f"COMMAND:{command[0]}:{error}") from error
        try:
            output, diagnostic = _bounded_process_output(
                process, timeout or self.timeout_seconds, command[0])
        except subprocess.TimeoutExpired as error:
            _kill_process_group(process)
            raise SupportError(
                f"COMMAND_TIMEOUT:{command[0]}:{error}") from error
        except OSError as error:
            _kill_process_group(process)
            raise SupportError(f"COMMAND:{command[0]}:{error}") from error
        _require(process.returncode == 0,
                 f"COMMAND_EXIT:{command[0]}:{process.returncode}:"
                 f"{(output + diagnostic).strip()}")
        return output

    def _topic_info(self, topic: str) -> str:
        return self._run(("gz", "topic", "-i", "-t", topic))

    @staticmethod
    def _endpoint_rows(info: str) -> list[tuple[str, str]]:
        rows = []
        for line in info.splitlines():
            match = re.match(r"^\s+[A-Za-z][A-Za-z0-9+.-]*://[^,]+,\s*(\S+)\s*$", line)
            if match:
                rows.append((line.strip().split(",", 1)[0], match.group(1)))
        return rows

    @staticmethod
    def _directional_types(info: str, direction: str) -> list[str]:
        _require(direction in ("Publishers", "Subscribers"), "TRANSPORT_DIRECTION")
        active = False
        result: list[str] = []
        for line in info.splitlines():
            if line.startswith("Publishers ["):
                active = direction == "Publishers"
                continue
            if line.startswith("Subscribers ["):
                active = direction == "Subscribers"
                continue
            match = re.match(
                r"^\s+[A-Za-z][A-Za-z0-9+.-]*://[^,]+,\s*(\S+)\s*$", line)
            if active and match:
                result.append(match.group(1))
        return result

    def _require_empty_topics(self) -> None:
        for topic in (
                DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC,
                CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC):
            info = self._topic_info(topic)
            _require(f"No publishers on topic [{topic}]" in info and
                     f"No subscribers on topic [{topic}]" in info and
                     not self._endpoint_rows(info), f"PRELOAD_TOPIC_NOT_EMPTY:{topic}")

    def _require_trajectory_topics_quiescent(self, label: str) -> None:
        """Require controller subscribers with no external command publisher."""
        _require(label in ("PRELOAD", "CHECKPOINT", "FINAL"),
                 "TRAJECTORY_TOPIC_LABEL")
        for topic in TRAJECTORY_TOPICS:
            retry_reason = ""
            for attempt in range(TRAJECTORY_TOPIC_DISCOVERY_ATTEMPTS):
                try:
                    output = self._run((
                        "ros2", "topic", "info", topic, "--no-daemon",
                        "--spin-time",
                        str(TRAJECTORY_TOPIC_DISCOVERY_SPIN_SECONDS)))
                except SupportError as error:
                    if re.match(
                            r"^COMMAND_EXIT:ros2:[0-9]+:.*\bUnknown topic\b",
                            str(error), re.S) is None:
                        raise
                    retry_reason = "UNKNOWN_TOPIC"
                else:
                    publisher_counts = [int(value) for value in re.findall(
                        r"^Publisher count: ([0-9]+)\s*$", output, re.M)]
                    subscriber_counts = [int(value) for value in re.findall(
                        r"^Subscription count: ([0-9]+)\s*$", output, re.M)]

                    # A discovered unsafe count is authoritative even if the
                    # rest of this CLI response is incomplete or duplicated.
                    _require(not any(value != 0 for value in publisher_counts),
                             f"{label}_TRAJECTORY_TOPIC:{topic}")
                    _require(not any(value < 1 for value in subscriber_counts),
                             f"{label}_TRAJECTORY_TOPIC:{topic}")
                    if (len(publisher_counts) == 1 and
                            len(subscriber_counts) == 1):
                        break
                    retry_reason = "INCOMPLETE_COUNTS"

                if attempt + 1 < TRAJECTORY_TOPIC_DISCOVERY_ATTEMPTS:
                    time.sleep(TRAJECTORY_TOPIC_DISCOVERY_RETRY_DELAY_SECONDS)
            else:
                raise SupportError(
                    f"{label}_TRAJECTORY_TOPIC_DISCOVERY:{topic}:"
                    f"{retry_reason}")

    def _require_resident_plugin(self, state: str, attach: str, detach: str,
                                 allow_state_listeners: bool = False) -> None:
        state_info = self._topic_info(state)
        attach_info = self._topic_info(attach)
        detach_info = self._topic_info(detach)
        state_publishers = self._directional_types(state_info, "Publishers")
        state_subscribers = self._directional_types(state_info, "Subscribers")
        _require(state_publishers == ["gz.msgs.StringMsg"],
                 f"RESIDENT_STATE_PUBLISHER:{state}")
        if allow_state_listeners:
            _require(all(item in ("google.protobuf.Message", "gz.msgs.StringMsg")
                         for item in state_subscribers),
                     f"RESIDENT_STATE_LISTENER_TYPE:{state}")
        else:
            _require(not state_subscribers and
                     f"No subscribers on topic [{state}]" in state_info,
                     f"RESIDENT_STATE_SUBSCRIBER:{state}")
        for info, topic in ((attach_info, attach), (detach_info, detach)):
            _require(not self._directional_types(info, "Publishers") and
                     self._directional_types(info, "Subscribers") == ["gz.msgs.Empty"],
                     f"RESIDENT_COMMAND_TOPIC:{topic}")

    def _require_add_service(self) -> None:
        output = self._run(("gz", "service", "-i", "-s", ADD_SERVICE))
        rows = [line.strip().split(", ") for line in output.splitlines()
                if re.match(r"^\s+[A-Za-z][A-Za-z0-9+.-]*://", line)]
        _require(len(rows) == 1 and len(rows[0]) == 3 and
                 rows[0][1:] == ["gz.msgs.EntityPlugin_V", "gz.msgs.Boolean"],
                 "ADD_SERVICE_CONTRACT")

    def _entity(self, name: str) -> Mapping[str, Any]:
        output = self._run((
            "gz", "topic", "-e", "--json-output", "-t", POSE_TOPIC, "-n", "1"),
            separate_stderr=True)
        decoder = json.JSONDecoder(
            object_pairs_hook=_no_duplicate_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                SupportError(f"JSON_NONFINITE:{token}")))
        frames: list[Any] = []
        cursor = 0
        while cursor < len(output):
            separator_start = cursor
            while cursor < len(output) and output[cursor].isspace():
                cursor += 1
            if cursor == len(output):
                break
            if frames and cursor == separator_start:
                raise SupportError(
                    f"ENTITY_JSON:{name}:FRAME_SEPARATOR:{len(frames)}")
            try:
                frame, cursor = decoder.raw_decode(output, cursor)
            except (json.JSONDecodeError, SupportError) as error:
                raise SupportError(f"ENTITY_JSON:{name}:{error}") from error
            frames.append(frame)
        _require(bool(frames), f"ENTITY_JSON:{name}:EMPTY")

        latest: Mapping[str, Any] | None = None
        stable_entity_id: int | None = None
        for frame_index, frame in enumerate(frames):
            if type(frame) is not dict or type(frame.get("pose")) is not list:
                raise SupportError(
                    f"ENTITY_JSON:{name}:FRAME_SCHEMA:{frame_index}")
            poses = frame["pose"]
            if any(type(pose) is not dict for pose in poses):
                raise SupportError(
                    f"ENTITY_JSON:{name}:POSE_SCHEMA:{frame_index}")
            matches = []
            for pose in poses:
                if pose.get("name") != name:
                    continue
                raw_id = pose.get("id", 0)
                if type(raw_id) is int:
                    entity_id = raw_id
                elif (type(raw_id) is str and
                      re.fullmatch(r"[0-9]+", raw_id) is not None):
                    entity_id = int(raw_id)
                else:
                    raise SupportError(
                        f"ENTITY_JSON:{name}:ENTITY_ID:{frame_index}")
                if entity_id > 0:
                    matches.append(pose)
            _require(
                len(matches) == 1,
                f"ENTITY_UNIQUE:{name}:FRAME:{frame_index}")
            matched_id = int(matches[0]["id"])
            if stable_entity_id is None:
                stable_entity_id = matched_id
            _require(matched_id == stable_entity_id,
                     f"ENTITY_IDENTITY:{name}:FRAME:{frame_index}")
            self._pose_from_entity(matches[0])
            latest = matches[0]
        assert latest is not None
        return latest

    @staticmethod
    def _pose_from_entity(entity: Mapping[str, Any]) -> dict[str, list[float]]:
        position = entity.get("position", {})
        orientation = entity.get("orientation", {})
        _require(type(position) is dict and type(orientation) is dict,
                 "POSE_FIELDS")
        result = {}
        for label, value, keys in (
                ("position", position, ("x", "y", "z")),
                ("orientation", orientation, ("x", "y", "z", "w"))):
            fields = [value.get(key, 0.0) for key in keys]
            _require(all(_finite(item) for item in fields),
                     f"POSE_FINITE:{label}")
            result[label] = [float(item) for item in fields]
        return result

    @staticmethod
    def _pose_error(actual: Mapping[str, Any], expected: Mapping[str, Any]) -> tuple[float, float]:
        a = actual["position"]
        e = expected["position"]
        _require(len(a) == 3 and len(e) == 3 and
                 all(_finite(item) for item in (*a, *e)), "POSE_POSITION")
        distance = math.hypot(
            *(float(a[index]) - float(e[index]) for index in range(3)))
        _require(math.isfinite(distance), "POSE_POSITION")
        aq = actual["orientation"]
        eq = expected["orientation"]
        _require(len(aq) == 4 and len(eq) == 4 and
                 all(_finite(item) for item in (*aq, *eq)), "POSE_QUATERNION")
        an = math.hypot(*(float(item) for item in aq))
        en = math.hypot(*(float(item) for item in eq))
        _require(math.isfinite(an) and math.isfinite(en) and
                 an > 0.99 and en > 0.99, "POSE_QUATERNION")
        dot = abs(sum(float(aq[index]) * float(eq[index])
                      for index in range(4)) / (an * en))
        _require(math.isfinite(dot), "POSE_QUATERNION")
        dot = min(1.0, dot)
        return distance, 2.0 * math.atan2(math.sqrt(max(0.0, 1.0 - dot * dot)), dot)

    def _cube_pose_gate(self, expected: Mapping[str, Any], label: str) -> None:
        actual = self._pose_from_entity(self._entity("green_cube"))
        position_error, orientation_error = self._pose_error(actual, expected)
        _require(position_error <= CUBE_POSITION_TOLERANCE and
                 orientation_error <= CUBE_ORIENTATION_TOLERANCE,
                 f"{label}_CUBE_POSE:{position_error:.9g}:{orientation_error:.9g}")

    def _static_model_pose_gate(self, name: str, expected_position: Sequence[float],
                                label: str) -> None:
        actual = self._pose_from_entity(self._entity(name))
        expected = {
            "position": list(expected_position),
            "orientation": [0.0, 0.0, 0.0, 1.0],
        }
        position_error, orientation_error = self._pose_error(actual, expected)
        _require(position_error <= CUBE_POSITION_TOLERANCE and
                 orientation_error <= 1e-6,
                 f"{label}_{name.upper()}_POSE:{position_error:.9g}:{orientation_error:.9g}")

    def _cube_stability_gate(self, expected: Mapping[str, Any], label: str) -> None:
        self._cube_pose_gate(expected, label)
        reference = self._pose_from_entity(self._entity("green_cube"))
        for _ in range(3):
            time.sleep(0.25)
            current = self._pose_from_entity(self._entity("green_cube"))
            position_error, orientation_error = self._pose_error(current, reference)
            _require(position_error <= CUBE_POSITION_TOLERANCE and
                     orientation_error <= CUBE_ORIENTATION_TOLERANCE,
                     f"{label}_CUBE_STABILITY")

    def _snapshot(self, mode: str) -> dict[str, tuple[float, ...]]:
        snapshot_tool = self._bound_snapshot_tool()
        output = self._run(
            (str(snapshot_tool), mode, "--timeout",
             str(SNAPSHOT_FRESH_TIMEOUT_SECONDS)),
            timeout=SNAPSHOT_PROCESS_TIMEOUT_SECONDS,
            separate_stderr=True)
        result: dict[str, tuple[float, ...]] = {}
        width = 2 if mode == "robot" else 3
        for line in output.splitlines():
            fields = line.split()
            _require(len(fields) == width and fields[0] not in result,
                     f"SNAPSHOT_ROW:{mode}")
            try:
                values = tuple(float(item) for item in fields[1:])
            except ValueError as error:
                raise SupportError(f"SNAPSHOT_VALUE:{mode}") from error
            _require(all(math.isfinite(item) for item in values), f"SNAPSHOT_FINITE:{mode}")
            result[fields[0]] = values
        _require(bool(result), f"SNAPSHOT_EMPTY:{mode}")
        return result

    def _boundary_gate(self, name: str) -> None:
        boundary = self.contract.boundaries[name]
        rows = self._snapshot("robot")
        for joint, expected in boundary["robot_positions"].items():
            _require(joint in rows, f"{name}_JOINT_MISSING:{joint}")
            if ((name == "FINAL_ROBOT" and "dex1_finger" in joint) or
                    joint.startswith("left_dex1_finger_")):
                tolerance = 0.001
            elif joint.startswith("right_dex1_finger_"):
                tolerance = RIGHT_FINGER_ENDPOINT_TOLERANCE
            else:
                tolerance = ROBOT_ENDPOINT_TOLERANCE
            _require(abs(rows[joint][0] - float(expected)) <= tolerance,
                     f"{name}_JOINT_ENDPOINT:{joint}")

    def _left_contact_gate(self) -> None:
        rows = self._snapshot("robot")
        q1 = rows.get("left_dex1_finger_1_joint", (math.nan,))[0]
        q2 = rows.get("left_dex1_finger_2_joint", (math.nan,))[0]
        gap = 0.050061 - (q1 + q2)
        center = (q2 - q1) / 2.0
        _require(-0.0195 <= q1 <= -0.0100 and -0.0195 <= q2 <= -0.0100 and
                 0.077 <= gap <= 0.083 and -0.003 <= center <= 0.003,
                 "LEFT_CUBE_CONTACT")

    def _mechanism_gate(self, door_expected: float, handle_expected: float,
                        door_tolerance: float, latch_expected: str, label: str) -> None:
        rows = self._snapshot("incubator")
        _require("door_hinge_joint" in rows and "handle_latch_joint" in rows,
                 f"{label}_MECHANISM_ROWS")
        door_position, door_velocity = rows["door_hinge_joint"]
        handle_position, handle_velocity = rows["handle_latch_joint"]
        _require(abs(door_position - door_expected) <= door_tolerance and
                 abs(handle_position - handle_expected) <= HANDLE_TOLERANCE and
                 abs(door_velocity) <= MECHANISM_VELOCITY_TOLERANCE and
                 abs(handle_velocity) <= MECHANISM_VELOCITY_TOLERANCE,
                 f"{label}_MECHANISM")
        _require(self._latch_state() == latch_expected, f"{label}_LATCH")

    def _mechanism_triplet_gate(self, door_expected: float, handle_expected: float,
                                door_tolerance: float, latch_expected: str,
                                label: str) -> None:
        for sample in range(3):
            self._mechanism_gate(
                door_expected, handle_expected, door_tolerance,
                latch_expected, f"{label}_SAMPLE_{sample + 1}")
            if sample != 2:
                time.sleep(0.02)

    def _latch_state(self) -> str:
        output = self._run(("gz", "topic", "-e", "-t", LATCH_TOPIC, "-n", "1"))
        return _last_state_message(
            output, ("locked", "unlocked"), "LATCH_STATE")

    def _support_state(self, topic: str) -> str:
        output = self._run(("gz", "topic", "-e", "-t", topic, "-n", "1"))
        return _last_state_message(
            output, ("attached", "detached"), f"SUPPORT_STATE:{topic}")

    def _transition(self, state_topic: str, command_topic: str,
                    before: str, after: str) -> None:
        try:
            process = subprocess.Popen(
                ["gz", "topic", "-e", "-t", state_topic],
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, text=False, bufsize=0,
                start_new_session=True)
        except OSError as error:
            raise SupportError(f"STATE_LISTENER:{error}") from error
        selector = None
        try:
            assert process.stdout is not None
            os.set_blocking(process.stdout.fileno(), False)
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ)
            buffer = bytearray()

            def wait_for(expected: str) -> None:
                deadline = time.monotonic() + self.timeout_seconds
                assert selector is not None
                _wait_listener_state(
                    process, selector, buffer, expected,
                    (before, after), deadline, "STATE_LISTENER")

            wait_for(before)
            self._run(("gz", "topic", "-t", command_topic,
                       "-m", "gz.msgs.Empty", "-p", ""))
            wait_for(after)
        finally:
            if selector is not None:
                selector.close()
            _kill_process_group(process)

    def _server_identity(self) -> tuple[int, int]:
        output = self._run(("pgrep", "-f", "gz sim server"))
        pids = [int(line) for line in output.splitlines() if line.isdigit()]
        _require(len(pids) == 1, "GAZEBO_SERVER_UNIQUE")
        pid = pids[0]
        try:
            fields = (Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
                      .rsplit(") ", 1)[1].split())
            start_ticks = int(fields[19])
        except (OSError, IndexError, ValueError) as error:
            raise SupportError(f"GAZEBO_SERVER_IDENTITY:{error}") from error
        _require(start_ticks > 0, "GAZEBO_SERVER_START")
        return pid, start_ticks

    def _ownership_path_and_payload(self) -> tuple[Path, bytes]:
        pid, start_ticks = self._server_identity()
        incubator_id = int(self._entity("incubator")["id"])
        cube_id = int(self._entity("green_cube")["id"])
        directory = Path(f"/tmp/h1_support_orchestrator_828_owner_{os.geteuid()}")
        if not directory.exists():
            directory.mkdir(mode=0o700)
        metadata = directory.lstat()
        _require(stat.S_ISDIR(metadata.st_mode) and not directory.is_symlink() and
                 metadata.st_uid == os.geteuid() and
                 stat.S_IMODE(metadata.st_mode) == 0o700, "OWNERSHIP_DIRECTORY")
        path = directory / (
            f"{self.contract.manifest_sha256}_{self.contract.certificate_sha256}_"
            f"{pid}_{start_ticks}.owner")
        payload = _canonical_bytes({
            "schema": "h1_support_orchestrator_828_owner/v1",
            "manifest_sha256": self.contract.manifest_sha256,
            "certificate_sha256": self.contract.certificate_sha256,
            "plan_sha256": self.contract.plan_sha256,
            "exact_result_sha256": self.contract.exact_result_sha256,
            "support_contract_sha256": self.contract.support_contract_sha256,
            "plugin_sha256": self.contract.plugin_sha256,
            "snapshot_sha256": self.contract.snapshot_sha256,
            "gazebo_server_pid": pid,
            "gazebo_server_start_ticks": start_ticks,
            "incubator_entity_id": incubator_id,
            "cube_entity_id": cube_id,
            "door_initially_attached": False,
            "cube_initially_attached": False,
        }) + b"\n"
        return path, payload

    def _create_ownership(self) -> None:
        path, payload = self._ownership_path_and_payload()
        _require(not path.exists() and not path.is_symlink(), "OWNERSHIP_ALREADY_EXISTS")
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(path, flags, 0o400)
        try:
            written = 0
            while written < len(payload):
                count = os.write(descriptor, payload[written:])
                _require(count > 0, "OWNERSHIP_SHORT_WRITE")
                written += count
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        os.chmod(path, 0o400)

    def _require_ownership(self) -> None:
        path, expected = self._ownership_path_and_payload()
        try:
            metadata = path.lstat()
        except OSError as error:
            raise SupportError(f"OWNERSHIP_LSTAT:{error}") from error
        _require(stat.S_ISREG(metadata.st_mode) and not path.is_symlink() and
                 metadata.st_uid == os.geteuid() and
                 stat.S_IMODE(metadata.st_mode) == 0o400 and metadata.st_nlink == 1,
                 "OWNERSHIP_METADATA")
        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(path, flags)
        try:
            before = os.fstat(descriptor)
            _require(_stat_identity(before) == _stat_identity(metadata),
                     "OWNERSHIP_RACE_BEFORE")
            data = b""
            remaining = before.st_size
            while remaining:
                block = os.read(descriptor, remaining)
                _require(bool(block), "OWNERSHIP_SHORT_READ")
                data += block
                remaining -= len(block)
            _require(not os.read(descriptor, 1), "OWNERSHIP_GREW")
            _require(_stat_identity(before) == _stat_identity(os.fstat(descriptor)),
                     "OWNERSHIP_RACE_AFTER")
        finally:
            os.close(descriptor)
        _require(data == expected, "OWNERSHIP_CONTENT")

    def _add_plugin(self, entity_id: int, entity_name: str, innerxml: str) -> None:
        request = (
            f'entity {{id: {entity_id}, name: "{entity_name}", type: MODEL}} '
            f'plugins {{name: "h1_2::GazeboCompliantGraspSystem", '
            f'filename: "{self.plugin.resolve()}", innerxml: "{innerxml}"}}')
        output = self._run((
            "gz", "service", "-s", ADD_SERVICE,
            "--reqtype", "gz.msgs.EntityPlugin_V", "--reptype", "gz.msgs.Boolean",
            "--timeout", "10000", "--req", request), timeout=12.0)
        _require(_last_state_message(
            output, ("true", "false"), "PLUGIN_ADD_STATE") == "true",
                 f"PLUGIN_ADD_REJECTED:{entity_name}")

    def _preload_one(self, entity_id: int, entity_name: str,
                     state_topic: str, innerxml: str) -> None:
        try:
            process = subprocess.Popen(
                ["gz", "topic", "-e", "-t", state_topic],
                stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, text=False, bufsize=0,
                start_new_session=True)
        except OSError as error:
            raise SupportError(f"PRELOAD_LISTENER:{error}") from error
        selector = None
        try:
            self._add_plugin(entity_id, entity_name, innerxml)
            assert process.stdout is not None
            selector = selectors.DefaultSelector()
            os.set_blocking(process.stdout.fileno(), False)
            selector.register(process.stdout, selectors.EVENT_READ)
            deadline = time.monotonic() + self.timeout_seconds
            _wait_listener_state(
                process, selector, bytearray(), "detached",
                ("attached", "detached"), deadline,
                f"PRELOAD_DETACHED_ACK:{entity_name}")
        finally:
            if selector is not None:
                selector.close()
            _kill_process_group(process)

    def preload(self) -> None:
        self._require_trajectory_topics_quiescent("PRELOAD")
        self._require_empty_topics()
        self._require_add_service()
        self._static_model_pose_gate("table", (0.600, 0.0, 0.0), "PRELOAD")
        self._static_model_pose_gate("incubator", (0.590, 0.0, 0.815), "PRELOAD")
        self._mechanism_triplet_gate(
            0.0, 0.0, DOOR_CLOSED_TOLERANCE, "locked", "PRELOAD")
        spawn_pose = {
            "position": [0.334, 0.377, 0.830],
            "orientation": [0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)],
        }
        self._cube_stability_gate(spawn_pose, "PRELOAD")
        incubator = self._entity("incubator")
        cube = self._entity("green_cube")
        door_xml = (
            f"<parent_scope>h1_2::right_wrist_yaw_link</parent_scope>"
            f"<child_scope>incubator::door_link</child_scope>"
            f"<gravity_compensation>false</gravity_compensation>"
            f"<max_force>120.0</max_force><max_torque>8.0</max_torque>"
            f"<initially_attached>false</initially_attached>"
            f"<detach_topic>{DOOR_DETACH_TOPIC}</detach_topic>"
            f"<attach_topic>{DOOR_ATTACH_TOPIC}</attach_topic>"
            f"<output_topic>{DOOR_STATE_TOPIC}</output_topic>")
        cube_xml = (
            f"<parent_scope>h1_2::left_wrist_yaw_link</parent_scope>"
            f"<initially_attached>false</initially_attached>"
            f"<detach_topic>{CUBE_DETACH_TOPIC}</detach_topic>"
            f"<attach_topic>{CUBE_ATTACH_TOPIC}</attach_topic>"
            f"<output_topic>{CUBE_STATE_TOPIC}</output_topic>")
        self._preload_one(int(incubator["id"]), "incubator", DOOR_STATE_TOPIC, door_xml)
        self._preload_one(int(cube["id"]), "green_cube", CUBE_STATE_TOPIC, cube_xml)
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC)
        _require(self._support_state(DOOR_STATE_TOPIC) == "detached" and
                 self._support_state(CUBE_STATE_TOPIC) == "detached",
                 "PRELOAD_FINAL_STATE")
        self._create_ownership()

    def door_attach(self) -> None:
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "detached" and
                 self._support_state(CUBE_STATE_TOPIC) == "detached", "DOOR_ATTACH_ORDER")
        self._boundary_gate("E0_ENTRY")
        self._mechanism_triplet_gate(
            0.0, math.pi / 2.0, DOOR_CLOSED_TOLERANCE,
            "unlocked", "DOOR_ATTACH_PRE")
        self._transition(DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, "detached", "attached")
        self._boundary_gate("E0_ENTRY")
        self._mechanism_triplet_gate(
            0.0, math.pi / 2.0, DOOR_CLOSED_TOLERANCE,
            "unlocked", "DOOR_ATTACH_POST")

    def cube_attach(self) -> None:
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "attached" and
                 self._support_state(CUBE_STATE_TOPIC) == "detached", "CUBE_ATTACH_ORDER")
        self._boundary_gate("CUBE_CONTACT")
        self._left_contact_gate()
        self._cube_pose_gate(self.contract.boundaries["CUBE_CONTACT"]["cube_pose"],
                             "CUBE_ATTACH_PRE")
        self._mechanism_gate(math.pi / 3.0, math.pi / 2.0, DOOR_OPEN_TOLERANCE,
                             "unlocked", "CUBE_ATTACH_PRE")
        self._transition(CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, "detached", "attached")
        self._boundary_gate("CUBE_CONTACT")
        self._left_contact_gate()

    def cube_attach_return(self) -> None:
        """Attach the cube at the certified inside endpoint for cycle 2."""
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "attached" and
                 self._support_state(CUBE_STATE_TOPIC) == "detached",
                 "CUBE_ATTACH_RETURN_ORDER")
        self._boundary_gate("CUBE_RELEASE")
        self._left_contact_gate()
        self._cube_pose_gate(
            self.contract.boundaries["CUBE_RELEASE"]["cube_pose"],
            "CUBE_ATTACH_RETURN_PRE")
        self._mechanism_gate(
            math.pi / 3.0, math.pi / 2.0, DOOR_OPEN_TOLERANCE,
            "unlocked", "CUBE_ATTACH_RETURN_PRE")
        self._transition(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, "detached", "attached")
        self._boundary_gate("CUBE_RELEASE")
        self._left_contact_gate()

    def cube_detach(self) -> None:
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "attached" and
                 self._support_state(CUBE_STATE_TOPIC) == "attached", "CUBE_DETACH_ORDER")
        self._boundary_gate("CUBE_RELEASE")
        self._left_contact_gate()
        pose = self.contract.boundaries["CUBE_RELEASE"]["cube_pose"]
        self._cube_stability_gate(pose, "CUBE_DETACH_PRE")
        self._mechanism_gate(math.pi / 3.0, math.pi / 2.0, DOOR_OPEN_TOLERANCE,
                             "unlocked", "CUBE_DETACH_PRE")
        self._transition(CUBE_STATE_TOPIC, CUBE_DETACH_TOPIC, "attached", "detached")
        self._boundary_gate("CUBE_RELEASE")
        self._left_contact_gate()
        self._cube_stability_gate(pose, "CUBE_DETACH_POST")

    def cube_detach_return(self) -> None:
        """Detach the cube at its certified origin endpoint for cycle 2."""
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "attached" and
                 self._support_state(CUBE_STATE_TOPIC) == "attached",
                 "CUBE_DETACH_RETURN_ORDER")
        self._boundary_gate("CUBE_CONTACT")
        self._left_contact_gate()
        pose = self.contract.boundaries["CUBE_CONTACT"]["cube_pose"]
        self._cube_stability_gate(pose, "CUBE_DETACH_RETURN_PRE")
        self._mechanism_gate(
            math.pi / 3.0, math.pi / 2.0, DOOR_OPEN_TOLERANCE,
            "unlocked", "CUBE_DETACH_RETURN_PRE")
        self._transition(
            CUBE_STATE_TOPIC, CUBE_DETACH_TOPIC, "attached", "detached")
        self._boundary_gate("CUBE_CONTACT")
        self._left_contact_gate()
        self._cube_stability_gate(pose, "CUBE_DETACH_RETURN_POST")

    def door_detach(self) -> None:
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "attached" and
                 self._support_state(CUBE_STATE_TOPIC) == "detached", "DOOR_DETACH_ORDER")
        self._boundary_gate("E0_RETURN")
        self._mechanism_triplet_gate(
            0.0, math.pi / 2.0, DOOR_CLOSED_TOLERANCE,
            "unlocked", "DOOR_DETACH_PRE")
        self._transition(DOOR_STATE_TOPIC, DOOR_DETACH_TOPIC, "attached", "detached")
        self._boundary_gate("E0_RETURN")
        self._mechanism_triplet_gate(
            0.0, math.pi / 2.0, DOOR_CLOSED_TOLERANCE,
            "unlocked", "DOOR_DETACH_POST")

    def roundtrip_checkpoint(self) -> None:
        """Prove the locked, released cycle-1 seam before the second prefix."""
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "detached" and
                 self._support_state(CUBE_STATE_TOPIC) == "detached",
                 "CHECKPOINT_SUPPORT_STATE")
        self._mechanism_triplet_gate(
            0.0, 0.0, DOOR_CLOSED_TOLERANCE, "locked", "CHECKPOINT")
        self._boundary_gate("FINAL_ROBOT")
        self._cube_stability_gate(
            self.contract.boundaries["CUBE_RELEASE"]["cube_pose"],
            "CHECKPOINT")
        self._require_trajectory_topics_quiescent("CHECKPOINT")

    def finalize(self) -> None:
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "detached" and
                 self._support_state(CUBE_STATE_TOPIC) == "detached", "FINAL_SUPPORT_STATE")
        self._mechanism_triplet_gate(
            0.0, 0.0, DOOR_CLOSED_TOLERANCE, "locked", "FINAL")
        self._boundary_gate("FINAL_ROBOT")
        self._cube_stability_gate(
            self.contract.boundaries["CUBE_RELEASE"]["cube_pose"], "FINAL")
        self._require_trajectory_topics_quiescent("FINAL")

    def finalize_roundtrip(self) -> None:
        """Prove the final locked launch seam with the cube back at origin."""
        self._require_ownership()
        self._require_resident_plugin(
            DOOR_STATE_TOPIC, DOOR_ATTACH_TOPIC, DOOR_DETACH_TOPIC, True)
        self._require_resident_plugin(
            CUBE_STATE_TOPIC, CUBE_ATTACH_TOPIC, CUBE_DETACH_TOPIC, True)
        _require(self._support_state(DOOR_STATE_TOPIC) == "detached" and
                 self._support_state(CUBE_STATE_TOPIC) == "detached",
                 "FINAL_SUPPORT_STATE")
        self._mechanism_triplet_gate(
            0.0, 0.0, DOOR_CLOSED_TOLERANCE, "locked", "FINAL_RETURN")
        self._boundary_gate("FINAL_ROBOT")
        self._cube_stability_gate(
            self.contract.boundaries["CUBE_CONTACT"]["cube_pose"],
            "FINAL_RETURN")
        self._require_trajectory_topics_quiescent("FINAL")


def execute_operation(
    operation: str, runtime: Any, *, roundtrip: bool = False,
) -> str:
    """Execute one admitted state transition and return its exact marker."""
    if roundtrip:
        if operation == "preload":
            runtime.preload()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=preload "
                    "mode=roundtrip door=detached cube=detached "
                    "boundary=CUBE_CONTACT_ORIGIN")
        if operation == "door-attach":
            runtime.door_attach()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-attach "
                    "cycle=1 door=attached cube=detached boundary=E0_ENTRY")
        if operation == "cube-attach":
            runtime.cube_attach()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-attach "
                    "cycle=1 door=attached cube=attached "
                    "boundary=CUBE_CONTACT_ORIGIN")
        if operation == "cube-detach":
            runtime.cube_detach()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-detach "
                    "cycle=1 door=attached cube=detached "
                    "boundary=CUBE_RELEASE_INSIDE")
        if operation == "door-detach":
            runtime.door_detach()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-detach "
                    "cycle=1 door=detached cube=detached boundary=E0_RETURN")
        if operation == "cycle-checkpoint":
            runtime.roundtrip_checkpoint()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS "
                    "operation=cycle-checkpoint cycle=1 door=q0 handle=q0 "
                    "latch=locked supports=detached robot=launch cube=inside "
                    "boundary=CUBE_RELEASE_INSIDE publishers=0")
        if operation == "door-attach-return":
            runtime.door_attach()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS "
                    "operation=door-attach-return cycle=2 door=attached "
                    "cube=detached boundary=E0_ENTRY")
        if operation == "cube-attach-return":
            runtime.cube_attach_return()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS "
                    "operation=cube-attach-return cycle=2 door=attached "
                    "cube=attached boundary=CUBE_RELEASE_INSIDE")
        if operation == "cube-detach-return":
            runtime.cube_detach_return()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS "
                    "operation=cube-detach-return cycle=2 door=attached "
                    "cube=detached boundary=CUBE_CONTACT_ORIGIN")
        if operation == "door-detach-return":
            runtime.door_detach()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS "
                    "operation=door-detach-return cycle=2 door=detached "
                    "cube=detached boundary=E0_RETURN")
        if operation == "finalize":
            runtime.finalize_roundtrip()
            return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize "
                    "mode=roundtrip cycle=2 door=q0 handle=q0 latch=locked "
                    "supports=detached robot=launch cube=origin "
                    "boundary=CUBE_CONTACT_ORIGIN publishers=0")
        _require(operation == "validate", "ROUNDTRIP_OPERATION")
        return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=validate "
                "mode=roundtrip execution=none")
    if operation == "preload":
        runtime.preload()
        return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=preload "
                "door=detached cube=detached")
    if operation == "door-attach":
        runtime.door_attach()
        return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-attach "
                "door=attached cube=detached boundary=E0_ENTRY")
    if operation == "cube-attach":
        runtime.cube_attach()
        return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-attach "
                "door=attached cube=attached boundary=CUBE_CONTACT")
    if operation == "cube-detach":
        runtime.cube_detach()
        return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-detach "
                "door=attached cube=detached boundary=CUBE_RELEASE")
    if operation == "door-detach":
        runtime.door_detach()
        return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-detach "
                "door=detached cube=detached boundary=E0_RETURN")
    if operation == "finalize":
        runtime.finalize()
        return ("H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize door=q0 "
                "handle=q0 latch=locked supports=detached robot=launch publishers=0")
    _require(operation == "validate", "OPERATION")
    return "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=validate execution=none"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=(
        "validate", "preload", "door-attach", "cube-attach", "cube-detach",
        "door-detach", "cycle-checkpoint", "door-attach-return",
        "cube-attach-return", "cube-detach-return", "door-detach-return",
        "finalize"))
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--certificate", type=Path, required=True)
    parser.add_argument("--certificate-sha256", required=True)
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--roundtrip", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        manifest = load_sealed_json(args.manifest, args.manifest_sha256)
        certificate = load_sealed_json(args.certificate, args.certificate_sha256)
        contract = validate_manifest_and_certificate(manifest, certificate)
        load_bound_snapshot_executable(
            args.workspace, contract.snapshot_relative_path,
            contract.snapshot_sha256)
        if args.operation == "validate":
            print(execute_operation(
                "validate", None, roundtrip=args.roundtrip))
            return 0
        _require(math.isfinite(args.timeout) and 3.0 <= args.timeout <= 30.0,
                 "TIMEOUT")
        runtime = GazeboRuntime(contract, args.workspace, args.timeout)
        print(execute_operation(
            args.operation, runtime, roundtrip=args.roundtrip))
        return 0
    except (SupportError, OSError, ValueError) as error:
        print(f"H1_SUPPORT_ORCHESTRATOR_828=FAIL:{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
