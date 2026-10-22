#!/usr/bin/python3
"""Execute the sealed 826.3 middle with the fast-831 timing profile.

All files, phase semantics, controller topics, points, and support hooks are
validated before ROS is imported.  The right gripper is never commanded by
the middle: fresh feedback must show that it remains closed throughout.  Any
runtime fault publishes a measured-position hold on all five robot
controllers and exits nonzero.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
import re
import socket
import stat
import subprocess
import sys
import threading
import time
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple


PLAN_SCHEMA = "h1_candidate3_middle_plan/828-e0-v1"
CERTIFICATE_SCHEMA = "h1_candidate3_middle_certificate/828-e0-v1"
RESULT_SCHEMA = "h1_candidate3_middle_exact_result/828-e0-v1"
AUDITOR_VERSION = "h1_candidate3_middle_e0_exact_auditor/828-e0-v1"
ASSET_SCHEMA = "h1_candidate3_middle_trajectory_asset/828-e0-v1"
SUPPORT_MANIFEST_SCHEMA = "h1_support_orchestration_828/v1"
SUPPORT_CONTRACT_SCHEMA = "h1_support_contract_828/v1"
RETURN_PLAN_SCHEMA = "h1_candidate3_return_plan/831-c2-v1"
RETURN_AUTHORIZATION_SCHEMA = (
    "h1_candidate3_return_execution_authorization/831-v1")
RELEASE_OWNER = "suffix_after_q90_to_q0"
PHASE_NAMES = (
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
RETURN_PHASE_NAMES = (
    "c2_return_door_pre_support",
    "c2_return_door_open_supported",
    "c2_return_door_open_seam_to_workspace",
    "c2_return_left_ready",
    "c2_return_enter_inside",
    "c2_return_grasp_inside",
    "c2_return_carry_to_origin",
    "c2_return_release_at_origin",
    "c2_return_retreat_from_origin",
    "c2_return_left_park",
    "c2_return_door_seam_to_hinge",
    "c2_return_door_close_supported",
)
EXECUTION_DIRECTIONS = ("insert", "return")

LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
RIGHT_ARM_JOINTS = tuple(
    name.replace("left_", "right_", 1) for name in LEFT_ARM_JOINTS)
LEFT_GRIPPER_JOINTS = (
    "left_dex1_finger_1_joint", "left_dex1_finger_2_joint",
)
RIGHT_GRIPPER_JOINTS = (
    "right_dex1_finger_1_joint", "right_dex1_finger_2_joint",
)
CONTROLLER_JOINTS: Mapping[str, Tuple[str, ...]] = {
    "/torso_controller/joint_trajectory": ("torso_joint",),
    "/left_arm_controller/joint_trajectory": LEFT_ARM_JOINTS,
    "/right_arm_controller/joint_trajectory": RIGHT_ARM_JOINTS,
    "/left_gripper_controller/joint_trajectory": LEFT_GRIPPER_JOINTS,
    "/right_gripper_controller/joint_trajectory": RIGHT_GRIPPER_JOINTS,
}
CANONICAL_CONTROLLER_TOPICS = tuple(CONTROLLER_JOINTS)

PHASE_COMMAND_TOPICS = (
    (),
    ("/torso_controller/joint_trajectory",
     "/right_arm_controller/joint_trajectory"),
    ("/torso_controller/joint_trajectory",
     "/right_arm_controller/joint_trajectory"),
    ("/left_arm_controller/joint_trajectory",),
    ("/left_arm_controller/joint_trajectory",
     "/left_gripper_controller/joint_trajectory"),
    ("/left_arm_controller/joint_trajectory",
     "/left_arm_controller/joint_trajectory"),
    ("/left_gripper_controller/joint_trajectory",),
    ("/left_arm_controller/joint_trajectory",),
    ("/left_arm_controller/joint_trajectory",),
    ("/torso_controller/joint_trajectory",
     "/right_arm_controller/joint_trajectory"),
    ("/torso_controller/joint_trajectory",
     "/right_arm_controller/joint_trajectory"),
)
SYNCHRONIZED_PHASES = frozenset((1, 2, 9, 10))
RETURN_SYNCHRONIZED_PHASES = frozenset((1, 2, 10, 11))
SUPPORT_EXPECTATIONS = (
    (False, True, False, False),
    (True, True, False, False),
    (True, True, False, False),
    (True, True, False, False),
    (True, True, False, True),
    (True, True, True, False),
    (True, True, False, False),
    (True, True, False, False),
    (True, True, False, False),
    (True, True, False, False),
    (True, True, False, False),
)
HOOK_EXPECTATIONS = (
    ("before_phase", 0, "door_attach", "detached", "attached"),
    ("after_phase", 4, "cube_attach", "detached", "attached"),
    ("after_phase", 5, "cube_detach", "attached", "detached"),
    ("terminal_post", 10, "door_detach", "attached", "detached"),
)
SUPPORT_CLI_ACTION = {
    "door_attach": "door-attach",
    "cube_attach": "cube-attach",
    "cube_detach": "cube-detach",
    "door_detach": "door-detach",
}

E0_TORSO_RAD = math.radians(-2.7349115403864994)
E0_RIGHT_ARM_RAD = (
    -0.41373146194762, 0.23755434038212944, 1.1008080707922079,
    0.8747783439736734, -0.3046852067271319, 0.4094159191441438,
    -0.9999635680199944,
)
E0_RIGHT_GRIPPER = (0.0108, 0.007)
E0_LEFT_ARM_RAD = (
    -0.000012933885986937658, 0.316, 0.000023774721497885907,
    0.6000753185207717, 0.000023572560034304773,
    -0.00008730492997967213, -0.00001654021997178485,
)
E0_LEFT_GRIPPER = (-0.019, -0.019)

HARD_MAX_TORSO_RAD = math.radians(44.5)
HARD_START_TOLERANCE_RAD = 0.004
HARD_ENDPOINT_TOLERANCE_RAD = 0.004
RETURN_CARRY_ORIGIN_ENDPOINT_TOLERANCE_RAD = 0.003
HARD_GRIPPER_TOLERANCE_M = 0.001
HARD_ENDPOINT_SETTLE_SIM_NS = 500_000_000
RETURN_CARRY_ORIGIN_ENDPOINT_SETTLE_SIM_NS = 2_000_000_000
RETURN_CARRY_ORIGIN_TAIL_POINT_COUNT = 12
HARD_ENDPOINT_CONSECUTIVE_SAMPLES = 3
HARD_MIN_LEAD_TIME_S = 0.25
HARD_MAX_PHASE_DURATION_S = 120.0
HARD_MIN_SPEED_SCALE = 0.05
HARD_SIM_CLOCK_STALL_S = 1.0
HARD_RIGHT_GRASP_FEEDBACK_MAX_AGE_S = 1.0
HARD_MAX_DISCOVERY_TIMEOUT_S = 60.0
HARD_MAX_WALL_TIMEOUT_FACTOR = 100.0
HARD_MIN_SUPPORT_TIMEOUT_S = 3.0
HARD_MAX_SUPPORT_TIMEOUT_S = 30.0
FAST_TIME_DIVISOR = 7.0
HARD_FAST_MAX_REVOLUTE_VELOCITY_RAD_S = 8.0
HARD_FAST_MAX_REVOLUTE_ACCELERATION_RAD_S2 = 800.0
HARD_FAST_MAX_PRISMATIC_VELOCITY_M_S = 0.05
HARD_FAST_MAX_PRISMATIC_ACCELERATION_M_S2 = 10.0
REQUIRED_CONTROLLER_AND_RECORDER_SUBSCRIPTIONS = 2
# The support CLI timeout bounds each child command; a live hook contains a
# finite sequence of those commands and receives this separate outer bound.
SUPPORT_PROCESS_WALL_TIMEOUT_S = 300.0
MAX_PROTECTED_BYTES = 1024 * 1024 * 1024
SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
FORBIDDEN_TEXT = ("right_release", "right_retreat")

JOINT_LIMITS: Mapping[str, Tuple[float, float]] = {
    "torso_joint": (-HARD_MAX_TORSO_RAD, HARD_MAX_TORSO_RAD),
    "left_shoulder_pitch_joint": (-3.14, 1.57),
    "left_shoulder_roll_joint": (-0.38, 3.4),
    "left_shoulder_yaw_joint": (-2.66, 3.01),
    "left_elbow_joint": (-0.95, 3.18),
    "left_wrist_roll_joint": (-3.01, 2.75),
    "left_wrist_pitch_joint": (-0.4625, 0.4625),
    "left_wrist_yaw_joint": (-1.27, 1.27),
    "right_shoulder_pitch_joint": (-3.14, 1.57),
    "right_shoulder_roll_joint": (-3.4, 0.38),
    "right_shoulder_yaw_joint": (-3.01, 2.66),
    "right_elbow_joint": (-0.95, 3.18),
    "right_wrist_roll_joint": (-2.75, 3.01),
    "right_wrist_pitch_joint": (-0.4625, 0.4625),
    "right_wrist_yaw_joint": (-1.27, 1.27),
    "left_dex1_finger_1_joint": (-0.020, 0.0245),
    "left_dex1_finger_2_joint": (-0.020, 0.0245),
    "right_dex1_finger_1_joint": (-0.020, 0.0245),
    "right_dex1_finger_2_joint": (-0.020, 0.0245),
}


class MiddleExecutionError(RuntimeError):
    """A fail-closed admission, support, or runtime failure."""


@dataclass(frozen=True)
class SealedDocument:
    path: Path
    sha256: str
    value: Mapping[str, Any]


@dataclass(frozen=True)
class Point:
    positions: Tuple[float, ...]
    time_from_start_ns: int


@dataclass(frozen=True)
class Command:
    topic: str
    joint_names: Tuple[str, ...]
    points: Tuple[Point, ...]


@dataclass(frozen=True)
class Phase:
    index: int
    name: str
    commands: Tuple[Command, ...]


@dataclass(frozen=True)
class ValidatedMiddle:
    workspace: Path
    scene_path: Path
    scene_sha256: str
    plan: SealedDocument
    certificate: SealedDocument
    result: SealedDocument
    phases: Tuple[Phase, ...]
    support_orchestrator: Path
    support_manifest: Path
    support_manifest_sha256: str
    support_certificate: Path
    support_certificate_sha256: str


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise MiddleExecutionError(status)


def _exact_keys(value: Any, keys: Iterable[str], status: str) -> Mapping[str, Any]:
    _require(type(value) is dict, f"{status}_TYPE")
    _require(frozenset(value) == frozenset(keys), f"{status}_KEYS")
    return value


def _finite(value: Any, status: str) -> float:
    _require(type(value) in (int, float) and type(value) is not bool, status)
    result = float(value)
    _require(math.isfinite(result), status)
    return result


def _sha(value: Any, status: str) -> str:
    _require(type(value) is str and SHA256_RE.fullmatch(value) is not None,
             status)
    return value


def _reject_duplicate_keys(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    output: Dict[str, Any] = {}
    for key, value in pairs:
        if key in output:
            raise MiddleExecutionError(f"JSON_DUPLICATE_KEY:{key}")
        output[key] = value
    return output


def _reject_constant(value: str) -> None:
    raise MiddleExecutionError(f"JSON_NONFINITE:{value}")


def _identity(metadata: os.stat_result) -> Tuple[int, int, int, int, int]:
    return (metadata.st_dev, metadata.st_ino, metadata.st_size,
            metadata.st_mtime_ns, metadata.st_ctime_ns)


def _read_sealed_bytes(path: Path, expected_sha256: str, mode: int) -> bytes:
    _sha(expected_sha256, "EXPECTED_SHA256")
    path = Path(path)
    _require(path.is_absolute(), "SEALED_PATH_NOT_ABSOLUTE")
    try:
        metadata = path.lstat()
    except OSError as error:
        raise MiddleExecutionError(f"SEALED_LSTAT:{error}") from error
    _require(stat.S_ISREG(metadata.st_mode), "SEALED_NOT_REGULAR")
    _require(not path.is_symlink(), "SEALED_SYMLINK")
    _require(metadata.st_uid == os.geteuid(), "SEALED_OWNER")
    _require(stat.S_IMODE(metadata.st_mode) == mode, "SEALED_MODE")
    _require(metadata.st_nlink == 1, "SEALED_NLINK")
    _require(0 < metadata.st_size <= MAX_PROTECTED_BYTES, "SEALED_SIZE")
    flags = (os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) |
             getattr(os, "O_NOFOLLOW", 0))
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise MiddleExecutionError(f"SEALED_OPEN:{error}") from error
    try:
        before = os.fstat(descriptor)
        _require(_identity(before) == _identity(metadata), "SEALED_RACE_BEFORE")
        remaining = before.st_size
        digest = hashlib.sha256()
        chunks = []
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            _require(bool(block), "SEALED_SHORT_READ")
            digest.update(block)
            chunks.append(block)
            remaining -= len(block)
        _require(not os.read(descriptor, 1), "SEALED_GREW")
        _require(_identity(before) == _identity(os.fstat(descriptor)),
                 "SEALED_RACE_AFTER")
    finally:
        os.close(descriptor)
    _require(hmac.compare_digest(digest.hexdigest(), expected_sha256),
             "SEALED_SHA256")
    return b"".join(chunks)


def _exec_verified_module(
    module_name: str, package_name: str, source: Path, source_bytes: bytes,
) -> Any:
    """Execute only already verified source bytes, never a path loader or pyc."""
    try:
        code = compile(source_bytes, str(source), "exec", dont_inherit=True)
    except (SyntaxError, ValueError, TypeError) as error:
        raise MiddleExecutionError(
            f"VERIFIED_SOURCE_COMPILE:{module_name}:{error}") from error
    module = types.ModuleType(module_name)
    module.__file__ = str(source)
    module.__package__ = package_name
    module.__loader__ = None
    module.__spec__ = None
    previous = sys.modules.get(module_name)
    sys.modules[module_name] = module
    try:
        exec(code, module.__dict__)
    except BaseException as error:
        if previous is None:
            sys.modules.pop(module_name, None)
        else:
            sys.modules[module_name] = previous
        raise MiddleExecutionError(
            f"VERIFIED_SOURCE_EXEC:{module_name}:{error}") from error
    return module


def _load_semantic_authority(args: argparse.Namespace) -> Tuple[Any, Any]:
    """Load the exact contract then admission from their verified bytes."""
    contract_bytes = _read_sealed_bytes(
        args.middle_contract, args.middle_contract_sha256, 0o500)
    admission_bytes = _read_sealed_bytes(
        args.middle_admission, args.middle_admission_sha256, 0o500)
    _require(args.middle_contract.resolve() == args.middle_contract,
             "MIDDLE_CONTRACT_PATH_ALIAS")
    _require(args.middle_admission.resolve() == args.middle_admission,
             "MIDDLE_ADMISSION_PATH_ALIAS")
    package = types.ModuleType("h1_2_vision")
    package.__package__ = "h1_2_vision"
    package.__path__ = []
    package.__loader__ = None
    package.__spec__ = None
    sys.modules[package.__name__] = package
    contract = _exec_verified_module(
        "h1_2_vision.candidate3_middle_execution_contract_828",
        package.__name__, args.middle_contract, contract_bytes)
    setattr(package, "candidate3_middle_execution_contract_828", contract)
    admission = _exec_verified_module(
        "h1_2_vision.candidate3_middle_admission",
        package.__name__, args.middle_admission, admission_bytes)
    setattr(package, "candidate3_middle_admission", admission)
    return contract, admission


def _read_sealed_json(path: Path, expected_sha256: str) -> SealedDocument:
    raw = _read_sealed_bytes(path, expected_sha256, 0o400)
    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_constant)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise MiddleExecutionError(f"SEALED_JSON:{error}") from error
    _require(type(value) is dict, "SEALED_JSON_ROOT")
    return SealedDocument(Path(path), expected_sha256, value)


def _canonical_sha256(value: Any) -> str:
    try:
        encoded = json.dumps(
            value, sort_keys=True, separators=(",", ":"),
            ensure_ascii=True, allow_nan=False).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise MiddleExecutionError(f"CANONICAL_JSON:{error}") from error
    return hashlib.sha256(encoded).hexdigest()


def _bound_path(workspace: Path, relative: Any, status: str) -> Path:
    _require(type(relative) is str and relative and not relative.startswith("/"),
             status)
    candidate = workspace / relative
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise MiddleExecutionError(f"{status}:{error}") from error
    _require(resolved == candidate, f"{status}_ALIAS")
    _require(workspace in resolved.parents, f"{status}_ESCAPE")
    return resolved


def _all_strings(value: Any):
    if type(value) is str:
        yield value
    elif type(value) is list:
        for item in value:
            yield from _all_strings(item)
    elif type(value) is dict:
        for key, item in value.items():
            yield key
            yield from _all_strings(item)


def _validate_hooks(plan: Mapping[str, Any]) -> None:
    hooks = plan.get("hooks")
    _require(type(hooks) is list and len(hooks) == 4, "PLAN_HOOKS")
    identifiers = set()
    for index, (hook, expected) in enumerate(zip(hooks, HOOK_EXPECTATIONS)):
        item = _exact_keys(hook, (
            "id", "boundary", "action", "pre_state", "post_state",
            "require_verified",
        ), f"PLAN_HOOK:{index}")
        identifier = item["id"]
        _require(type(identifier) is str and identifier and
                 identifier not in identifiers, f"PLAN_HOOK_ID:{index}")
        identifiers.add(identifier)
        boundary = _exact_keys(item["boundary"], (
            "kind", "phase_index", "phase_name",
        ), f"PLAN_HOOK_BOUNDARY:{index}")
        kind, phase_index, action, pre_state, post_state = expected
        _require(boundary["kind"] == kind and
                 boundary["phase_index"] == phase_index and
                 boundary["phase_name"] == PHASE_NAMES[phase_index],
                 f"PLAN_HOOK_BOUNDARY:{index}")
        _require(item["action"] == action and
                 item["pre_state"] == pre_state and
                 item["post_state"] == post_state and
                 item["require_verified"] is True,
                 f"PLAN_HOOK_CONTRACT:{index}")


def _validate_asset(
    workspace: Path, value: Any, cache: Dict[Tuple[str, str], SealedDocument],
    status: str,
) -> Mapping[str, Any]:
    item = _exact_keys(value, (
        "path", "file_sha256", "payload_sha256", "schema",
    ), status)
    file_sha = _sha(item["file_sha256"], f"{status}_FILE_SHA")
    payload_sha = _sha(item["payload_sha256"], f"{status}_PAYLOAD_SHA")
    _require(type(item["schema"]) is str and item["schema"],
             f"{status}_SCHEMA")
    _require(type(item["path"]) is str and item["path"],
             f"{status}_PATH")
    key = (item["path"], file_sha)
    document = cache.get(key)
    if document is None:
        path = _bound_path(workspace, item["path"], f"{status}_PATH")
        document = _read_sealed_json(path, file_sha)
        cache[key] = document
    root = document.value
    _require(frozenset(root) == frozenset(("payload", "sha256")),
             f"{status}_WRAPPER")
    payload = root["payload"]
    _require(type(payload) is dict, f"{status}_PAYLOAD")
    _require(root["sha256"] == payload_sha == _canonical_sha256(payload),
             f"{status}_PAYLOAD_SHA")
    _require(payload.get("schema") == item["schema"],
             f"{status}_PAYLOAD_SCHEMA")
    _require(item["schema"] == ASSET_SCHEMA, f"{status}_SCHEMA_CONTRACT")
    payload = _exact_keys(payload, (
        "schema", "asset_id", "messages", "synchronization_groups",
    ), f"{status}_PAYLOAD")
    _require(type(payload["asset_id"]) is str and payload["asset_id"],
             f"{status}_ASSET_ID")
    messages = payload["messages"]
    _require(type(messages) is list, f"{status}_MESSAGES")
    for index, message in enumerate(messages):
        _require(type(message) is dict and
                 message.get("topic") in CONTROLLER_JOINTS,
                 f"{status}_MESSAGE_TOPIC:{index}")
        _validate_command(
            message, message["topic"], f"{status}_MESSAGE:{index}")
    groups = payload["synchronization_groups"]
    _require(type(groups) is list, f"{status}_SYNCHRONIZATION_GROUPS")
    grouped_indices = set()
    for group_index, group in enumerate(groups):
        group_value = _exact_keys(group, (
            "message_indices", "shared_time_from_start_ns",
        ), f"{status}_SYNC_GROUP:{group_index}")
        indices = group_value["message_indices"]
        _require(type(indices) is list and len(indices) >= 2 and
                 all(type(index) is int and 0 <= index < len(messages)
                     for index in indices) and
                 len(indices) == len(frozenset(indices)) and
                 not grouped_indices.intersection(indices),
                 f"{status}_SYNC_INDICES:{group_index}")
        grouped_indices.update(indices)
        shared = group_value["shared_time_from_start_ns"]
        _require(type(shared) is list and shared and
                 all(type(item) is int and item >= 0 for item in shared),
                 f"{status}_SYNC_TIME:{group_index}")
        for index in indices:
            actual = [point["time_from_start_ns"]
                      for point in messages[index]["points"]]
            _require(actual == shared, f"{status}_SYNC_GRID:{group_index}")
    return payload


def _validate_point(value: Any, names: Sequence[str], status: str) -> Point:
    item = _exact_keys(value, ("positions", "time_from_start_ns"), status)
    positions = item["positions"]
    _require(type(positions) is list and len(positions) == len(names),
             f"{status}_POSITIONS")
    vector = tuple(_finite(position, f"{status}_POSITION")
                   for position in positions)
    for name, position in zip(names, vector):
        lower, upper = JOINT_LIMITS[name]
        _require(lower - 1e-12 <= position <= upper + 1e-12,
                 f"{status}_JOINT_LIMIT:{name}")
    duration = item["time_from_start_ns"]
    _require(type(duration) is int and duration >= 0,
             f"{status}_TIME")
    return Point(vector, duration)


def _validate_command(value: Any, expected_topic: str, status: str) -> Command:
    item = _exact_keys(value, ("topic", "joint_names", "points"), status)
    _require(item["topic"] == expected_topic, f"{status}_TOPIC")
    names = item["joint_names"]
    _require(type(names) is list and tuple(names) ==
             CONTROLLER_JOINTS[expected_topic], f"{status}_JOINT_NAMES")
    points = item["points"]
    _require(type(points) is list and points, f"{status}_POINTS")
    output = tuple(_validate_point(point, names, f"{status}_POINT:{index}")
                   for index, point in enumerate(points))
    previous = -1
    for point in output:
        _require(point.time_from_start_ns > previous, f"{status}_TIME_ORDER")
        previous = point.time_from_start_ns
    _require(previous / 1e9 <= HARD_MAX_PHASE_DURATION_S,
             f"{status}_DURATION")
    return Command(expected_topic, tuple(names), output)


def _validate_phases(
    workspace: Path, plan: Mapping[str, Any],
) -> Tuple[Phase, ...]:
    phases = plan.get("phases")
    _require(type(phases) is list and len(phases) == len(PHASE_NAMES),
             "PLAN_PHASES")
    assets: Dict[Tuple[str, str], SealedDocument] = {}
    output = []
    for index, (value, name, expected_topics, support_values) in enumerate(zip(
            phases, PHASE_NAMES, PHASE_COMMAND_TOPICS, SUPPORT_EXPECTATIONS)):
        item = _exact_keys(value, (
            "name", "commanded", "asset", "message_range", "commands",
            "support", "entry_state_sha256", "exit_state_sha256",
        ), f"PLAN_PHASE:{index}")
        _require(item["name"] == name, f"PLAN_PHASE_NAME:{index}")
        _require(item["commanded"] is bool(expected_topics),
                 f"PLAN_PHASE_COMMANDED:{index}")
        asset_payload = _validate_asset(
            workspace, item["asset"], assets, f"PLAN_PHASE_ASSET:{index}")
        message_range = item["message_range"]
        _require(type(message_range) is list and len(message_range) == 2 and
                 all(type(number) is int and number >= 0
                     for number in message_range) and
                 message_range[0] <= message_range[1],
                 f"PLAN_PHASE_RANGE:{index}")
        _sha(item["entry_state_sha256"], f"PLAN_PHASE_ENTRY:{index}")
        _sha(item["exit_state_sha256"], f"PLAN_PHASE_EXIT:{index}")
        support = _exact_keys(item["support"], (
            "door_pre", "door_post", "cube_pre", "cube_post",
        ), f"PLAN_PHASE_SUPPORT:{index}")
        actual_support = tuple(support[key] for key in (
            "door_pre", "door_post", "cube_pre", "cube_post"))
        _require(actual_support == support_values,
                 f"PLAN_PHASE_SUPPORT:{index}")
        commands = item["commands"]
        _require(type(commands) is list and len(commands) == len(expected_topics),
                 f"PLAN_PHASE_COMMANDS:{index}")
        _require(message_range[1] - message_range[0] == len(commands),
                 f"PLAN_PHASE_RANGE_COMMAND_COUNT:{index}")
        _require(message_range[1] <= len(asset_payload["messages"]),
                 f"PLAN_PHASE_RANGE_ASSET:{index}")
        _require(commands == asset_payload["messages"][
            message_range[0]:message_range[1]],
            f"PLAN_PHASE_ASSET_COMMAND_IDENTITY:{index}")
        overlapping_groups = [
            group for group in asset_payload["synchronization_groups"]
            if set(group["message_indices"]).intersection(
                range(message_range[0], message_range[1]))
        ]
        if index in SYNCHRONIZED_PHASES:
            _require(len(overlapping_groups) == 1 and
                     set(overlapping_groups[0]["message_indices"]) ==
                     set(range(message_range[0], message_range[1])),
                     f"PLAN_PHASE_SYNCHRONIZATION:{index}")
        else:
            _require(not overlapping_groups,
                     f"PLAN_PHASE_UNEXPECTED_SYNCHRONIZATION:{index}")
        parsed = tuple(_validate_command(
            command, topic, f"PLAN_PHASE_COMMAND:{index}:{command_index}")
            for command_index, (command, topic) in enumerate(
                zip(commands, expected_topics)))
        output.append(Phase(index, name, parsed))
    return tuple(output)


def _reverse_command(command: Command) -> Command:
    """Return the exact pointwise reverse with a fresh zero-based clock."""
    _require(bool(command.points),
             f"RETURN_REVERSE_POINTS:{command.topic}")
    start_ns = command.points[0].time_from_start_ns
    duration_ns = command.points[-1].time_from_start_ns
    points = tuple(
        Point(
            sealed.positions,
            start_ns + duration_ns - sealed.time_from_start_ns,
        )
        for sealed in reversed(command.points)
    )
    _require(points[0].time_from_start_ns == start_ns and
             points[-1].time_from_start_ns == duration_ns and
             all(right.time_from_start_ns > left.time_from_start_ns
                 for left, right in zip(points, points[1:])),
             f"RETURN_REVERSE_TIME_ORDER:{command.topic}")
    _require(
        tuple(point.positions for point in points) ==
        tuple(point.positions for point in reversed(command.points)),
        f"RETURN_REVERSE_POSITION_IDENTITY:{command.topic}")
    return Command(command.topic, command.joint_names, points)


def build_return_phases(bundle: ValidatedMiddle) -> Tuple[Phase, ...]:
    """Build the second-cycle rack-to-origin middle from audited reversals.

    The door remains a right-hand operation.  The left hand first follows the
    released retreat backwards to the cube in the rack, closes, follows the
    two loaded carry messages backwards to the spawn pose, releases there,
    and withdraws along the original contact path before parking.
    """
    source = bundle.phases
    _require(len(source) == len(PHASE_NAMES), "RETURN_SOURCE_PHASE_COUNT")
    commands = (
        (),
        source[1].commands,
        source[2].commands,
        source[3].commands,
        (_reverse_command(source[7].commands[0]),),
        (_reverse_command(source[6].commands[0]),),
        (_reverse_command(source[5].commands[1]),
         _reverse_command(source[5].commands[0])),
        source[6].commands,
        (_reverse_command(source[4].commands[0]),),
        source[8].commands,
        source[9].commands,
        source[10].commands,
    )
    phases = tuple(
        Phase(index, RETURN_PHASE_NAMES[index], tuple(phase_commands))
        for index, phase_commands in enumerate(commands)
    )
    _require(
        tuple(tuple(command.topic for command in phase.commands)
              for phase in phases) == (
            (),
            PHASE_COMMAND_TOPICS[1],
            PHASE_COMMAND_TOPICS[2],
            PHASE_COMMAND_TOPICS[3],
            ("/left_arm_controller/joint_trajectory",),
            ("/left_gripper_controller/joint_trajectory",),
            PHASE_COMMAND_TOPICS[5],
            PHASE_COMMAND_TOPICS[6],
            PHASE_COMMAND_TOPICS[7],
            PHASE_COMMAND_TOPICS[8],
            PHASE_COMMAND_TOPICS[9],
            PHASE_COMMAND_TOPICS[10],
        ),
        "RETURN_PHASE_COMMAND_TOPICS")
    _require(sum(len(phase.commands) for phase in phases) == 16,
             "RETURN_CONTROLLER_MESSAGE_COUNT")
    _require(not any(
        command.topic == "/right_gripper_controller/joint_trajectory"
        for phase in phases for command in phase.commands),
        "RETURN_RIGHT_GRIPPER_COMMAND_FORBIDDEN")

    # Re-check every per-controller seam.  The only intentional discontinuity
    # is the historical 0.222 mm compliant-grasp offset at the left fingers;
    # it remains within the unchanged 1 mm gripper admission gate.
    last: Dict[str, Tuple[float, ...]] = {}
    for phase in phases:
        for command in phase.commands:
            start = command.points[0].positions
            previous = last.get(command.topic)
            if previous is not None:
                tolerance = (HARD_GRIPPER_TOLERANCE_M
                             if "gripper" in command.topic else 1e-9)
                _require(
                    max(abs(actual - expected)
                        for actual, expected in zip(start, previous)) <=
                    tolerance + 1e-15,
                    f"RETURN_CONTROLLER_SEAM:{phase.name}:{command.topic}")
            last[command.topic] = command.points[-1].positions
    return phases


def _command_json(command: Command) -> Mapping[str, Any]:
    return {
        "topic": command.topic,
        "joint_names": list(command.joint_names),
        "points": [
            {
                "positions": list(point.positions),
                "time_from_start_ns": point.time_from_start_ns,
            }
            for point in command.points
        ],
    }


def _validate_return_plan(
    args: argparse.Namespace, bundle: ValidatedMiddle,
    phases: Sequence[Phase],
) -> SealedDocument:
    _require(args.return_plan is not None and
             args.return_plan_sha256 is not None,
             "RETURN_PLAN_ARGUMENTS")
    document = _read_sealed_json(
        args.return_plan, args.return_plan_sha256)
    value = _exact_keys(document.value, (
        "schema", "builder_version", "stage_namespace", "cycle_id",
        "operation", "topology", "authorization", "offline_only",
        "controller_commands_issued", "source_binding", "scene_binding",
        "cube_pose_contract", "path_contract", "hooks", "phase_names",
        "initial_controller_state", "initial_controller_state_sha256",
        "phases", "terminal_contract", "timing", "message_contract",
    ), "RETURN_PLAN")
    _require(value["schema"] == RETURN_PLAN_SCHEMA and
             value["stage_namespace"] == "C2/return" and
             value["cycle_id"] == "C2" and value["operation"] == "return" and
             value["offline_only"] is True and
             value["controller_commands_issued"] == 0,
             "RETURN_PLAN_IDENTITY")
    authorization = _exact_keys(value["authorization"], (
        "execution_authorized", "requires_fresh_runtime_audit", "status",
    ), "RETURN_PLAN_AUTHORIZATION")
    _require(authorization == {
        "execution_authorized": False,
        "requires_fresh_runtime_audit": True,
        "status": "PLAN_ONLY",
    }, "RETURN_PLAN_PLAN_ONLY_SOURCE")
    source_binding = _exact_keys(value["source_binding"], (
        "middle_plan", "scene_model_provenance",
    ), "RETURN_PLAN_SOURCE_BINDING")
    middle_binding = _exact_keys(source_binding["middle_plan"], (
        "path", "file_sha256", "schema",
    ), "RETURN_PLAN_SOURCE_MIDDLE")
    _require(middle_binding["file_sha256"] == bundle.plan.sha256 and
             middle_binding["schema"] == PLAN_SCHEMA,
             "RETURN_PLAN_SOURCE_MIDDLE_BINDING")
    _require(value["phase_names"] == list(RETURN_PHASE_NAMES),
             "RETURN_PLAN_PHASE_NAMES")
    phase_values = value["phases"]
    _require(type(phase_values) is list and
             len(phase_values) == len(phases),
             "RETURN_PLAN_PHASES")
    for index, (phase_value, phase) in enumerate(zip(phase_values, phases)):
        _require(type(phase_value) is dict and
                 phase_value.get("name") == phase.name and
                 phase_value.get("commands") == [
                     _command_json(command) for command in phase.commands],
                 f"RETURN_PLAN_PHASE_COMMAND_IDENTITY:{index}")
    _require(value["message_contract"] == {
        "right_handle_per_cycle": 14,
        "return_middle": 16,
        "return_cycle_total": 30,
        "two_cycle_total": 60,
    }, "RETURN_PLAN_MESSAGE_CONTRACT")
    path_contract = value["path_contract"]
    _require(type(path_contract) is dict and
             path_contract.get("candidate3_only") is True and
             path_contract.get("all_segments_required") is True and
             path_contract.get("contains_partial") is False and
             path_contract.get("partial_execution_allowed") is False and
             path_contract.get("inverse_transform") ==
             "reverse_points_mirror_time",
             "RETURN_PLAN_PATH_CONTRACT")
    cube = value["cube_pose_contract"]
    _require(type(cube) is dict and cube.get("final_pose") == "origin" and
             cube.get("attach_event") ==
             "C2_RETURN_CUBE_ATTACH_INSIDE" and
             cube.get("detach_event") ==
             "C2_RETURN_CUBE_DETACH_ORIGIN" and
             cube.get("inside", {}).get("pose", {}).get("position") ==
             [0.611, -0.03, 0.995] and
             cube.get("origin", {}).get("pose", {}).get("position") ==
             [0.334, 0.377, 0.83],
             "RETURN_PLAN_CUBE_CONTRACT")
    hooks = value["hooks"]
    _require(type(hooks) is list and len(hooks) == 4 and
             [hook.get("semantic_event") for hook in hooks] == [
                 "C2_RETURN_DOOR_ATTACH",
                 "C2_RETURN_CUBE_ATTACH_INSIDE",
                 "C2_RETURN_CUBE_DETACH_ORIGIN",
                 "C2_RETURN_DOOR_DETACH",
             ] and
             [hook.get("boundary", {}).get("phase_index")
              for hook in hooks] == [0, 5, 6, 11],
             "RETURN_PLAN_HOOKS")
    return document


def _validate_return_authorization(
    args: argparse.Namespace, bundle: ValidatedMiddle,
    return_plan: SealedDocument,
) -> SealedDocument:
    """Bind the PLAN_ONLY inverse to a separate sealed execution decision."""
    _require(args.return_authorization is not None and
             args.return_authorization_sha256 is not None,
             "RETURN_AUTHORIZATION_ARGUMENTS")
    document = _read_sealed_json(
        args.return_authorization, args.return_authorization_sha256)
    value = _exact_keys(document.value, (
        "schema", "run_id", "authorization_scope", "bindings",
        "execution_contract", "controller_execution_authorized",
        "live_execution_passed", "controller_commands_issued", "valid",
        "status",
    ), "RETURN_AUTHORIZATION")
    _require(
        value["schema"] == RETURN_AUTHORIZATION_SCHEMA and
        type(value["run_id"]) is str and len(value["run_id"]) == 64 and
        all(character in "0123456789abcdef"
            for character in value["run_id"]) and
        value["authorization_scope"] ==
        "candidate3_roundtrip_831_cycle2_return" and
        value["controller_execution_authorized"] is True and
        value["live_execution_passed"] is False and
        value["controller_commands_issued"] == 0 and
        value["valid"] is True and value["status"] == "READY",
        "RETURN_AUTHORIZATION_IDENTITY")

    bindings = _exact_keys(value["bindings"], (
        "return_plan", "middle_plan", "middle_certificate",
        "middle_exact_result",
    ), "RETURN_AUTHORIZATION_BINDINGS")
    expected_bindings = {
        "return_plan": (
            return_plan.path, return_plan.sha256, RETURN_PLAN_SCHEMA),
        "middle_plan": (
            bundle.plan.path, bundle.plan.sha256, PLAN_SCHEMA),
        "middle_certificate": (
            bundle.certificate.path, bundle.certificate.sha256,
            CERTIFICATE_SCHEMA),
        "middle_exact_result": (
            bundle.result.path, bundle.result.sha256, RESULT_SCHEMA),
    }
    for name, (expected_path, expected_sha256, expected_schema) in (
            expected_bindings.items()):
        binding = _exact_keys(bindings[name], (
            "path", "sha256", "schema",
        ), f"RETURN_AUTHORIZATION_BINDING:{name}")
        try:
            bound_path = Path(binding["path"]).resolve(strict=True)
            canonical_expected_path = expected_path.resolve(strict=True)
        except (OSError, TypeError) as error:
            raise MiddleExecutionError(
                f"RETURN_AUTHORIZATION_BINDING_PATH:{name}:{error}") from error
        _require(
            Path(binding["path"]).is_absolute() and
            bound_path == canonical_expected_path and
            binding["sha256"] == expected_sha256 and
            binding["schema"] == expected_schema,
            f"RETURN_AUTHORIZATION_BINDING_IDENTITY:{name}")

    contract = _exact_keys(value["execution_contract"], (
        "exact_inverse", "inverse_transform", "all_segments_required",
        "partial_execution_allowed", "requires_live_final_validation",
        "controller_message_count", "left_hand_handle_contact_forbidden",
        "hard_maximum_abs_torso_deg", "terminal_cube_pose",
        "terminal_door_state",
    ), "RETURN_AUTHORIZATION_EXECUTION_CONTRACT")
    _require(contract == {
        "exact_inverse": True,
        "inverse_transform": "reverse_points_mirror_time",
        "all_segments_required": True,
        "partial_execution_allowed": False,
        "requires_live_final_validation": True,
        "controller_message_count": 60,
        "left_hand_handle_contact_forbidden": True,
        "hard_maximum_abs_torso_deg": 44.5,
        "terminal_cube_pose": "origin",
        "terminal_door_state": "locked",
    }, "RETURN_AUTHORIZATION_EXECUTION_CONTRACT_VALUE")
    _require(abs(math.degrees(HARD_MAX_TORSO_RAD) -
                 contract["hard_maximum_abs_torso_deg"]) <= 1e-12,
             "RETURN_AUTHORIZATION_TORSO_RUNTIME_BINDING")
    return document


def _validate_support_contract_offline(args: argparse.Namespace) -> None:
    command = [
        str(args.support_orchestrator), "validate",
        "--manifest", str(args.support_manifest),
        "--manifest-sha256", args.support_manifest_sha256,
        "--certificate", str(args.support_certificate),
        "--certificate-sha256", args.support_certificate_sha256,
        "--workspace", str(args.workspace),
        "--timeout", str(args.support_timeout),
    ]
    try:
        process = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=sys.stderr, text=True)
        try:
            stdout, _ = process.communicate(timeout=args.support_timeout)
        except subprocess.TimeoutExpired as error:
            process.kill()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
            raise MiddleExecutionError(
                f"SUPPORT_VALIDATE_TIMEOUT:{error}") from error
    except OSError as error:
        raise MiddleExecutionError(
            f"SUPPORT_VALIDATE_PROCESS:{error}") from error
    _require(process.returncode == 0, "SUPPORT_VALIDATE_EXIT")
    _require(stdout.strip() ==
             "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=validate "
             "execution=none", "SUPPORT_VALIDATE_MARKER")


def _kill_process_group(process: subprocess.Popen) -> None:
    if process.poll() is None:
        try:
            os.killpg(process.pid, 9)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass


def _validate_support_dag(
    *,
    manifest: SealedDocument,
    support_certificate: SealedDocument,
    plan: SealedDocument,
    certificate: SealedDocument,
    result: SealedDocument,
) -> None:
    """Cross-bind the final support manifest to this exact invocation.

    This validation is intentionally local and precedes the support adapter
    subprocess.  An individually valid manifest/certificate from another
    bundle therefore cannot be swapped into this execution.
    """
    value = _exact_keys(manifest.value, (
        "schema", "controller_execution_authorized", "live_execution_passed",
        "bindings", "support_contract",
    ), "SUPPORT_MANIFEST")
    _require(value["schema"] == SUPPORT_MANIFEST_SCHEMA,
             "SUPPORT_MANIFEST_SCHEMA")
    _require(value["controller_execution_authorized"] is True,
             "SUPPORT_MANIFEST_NOT_AUTHORIZED")
    _require(value["live_execution_passed"] is False,
             "SUPPORT_MANIFEST_NOT_PRERUN")
    bindings = _exact_keys(value["bindings"], (
        "plan_sha256", "plan_schema", "certificate_sha256",
        "certificate_schema", "exact_result_sha256", "exact_result_schema",
        "auditor_version", "support_contract_sha256",
    ), "SUPPORT_BINDINGS")
    expected = {
        "plan_sha256": plan.sha256,
        "plan_schema": PLAN_SCHEMA,
        "certificate_sha256": certificate.sha256,
        "certificate_schema": CERTIFICATE_SCHEMA,
        "exact_result_sha256": result.sha256,
        "exact_result_schema": RESULT_SCHEMA,
        "auditor_version": AUDITOR_VERSION,
    }
    for name, expected_value in expected.items():
        _require(bindings[name] == expected_value,
                 f"SUPPORT_BINDING_{name.upper()}")
    contract = value["support_contract"]
    _require(type(contract) is dict and
             contract.get("schema") == SUPPORT_CONTRACT_SCHEMA,
             "SUPPORT_CONTRACT_SCHEMA")
    contract_sha256 = _canonical_sha256(contract)
    _require(bindings["support_contract_sha256"] == contract_sha256,
             "SUPPORT_BINDING_SUPPORT_CONTRACT_SHA256")
    _require(plan.value.get("support_contract_sha256") == contract_sha256,
             "PLAN_SUPPORT_CONTRACT_SHA256")
    _require(result.value.get("support_contract_sha256") == contract_sha256,
             "RESULT_SUPPORT_CONTRACT_SHA256")
    _require(certificate.value.get("support_contract_sha256") ==
             contract_sha256, "CERTIFICATE_SUPPORT_CONTRACT_SHA256")
    _require(support_certificate.path.resolve(strict=True) ==
             certificate.path.resolve(strict=True),
             "SUPPORT_CERTIFICATE_PATH_IDENTITY")
    _require(support_certificate.sha256 == certificate.sha256,
             "SUPPORT_CERTIFICATE_SHA256_IDENTITY")
    _require(support_certificate.value == certificate.value,
             "SUPPORT_CERTIFICATE_CONTENT_IDENTITY")


def validate_bundle(args: argparse.Namespace) -> ValidatedMiddle:
    """Validate every immutable input without importing ROS."""
    workspace = Path(args.workspace)
    _require(workspace.is_absolute(), "WORKSPACE_NOT_ABSOLUTE")
    try:
        workspace = workspace.resolve(strict=True)
    except OSError as error:
        raise MiddleExecutionError(f"WORKSPACE:{error}") from error
    _require(workspace.is_dir(), "WORKSPACE_NOT_DIRECTORY")

    plan = _read_sealed_json(args.plan, args.plan_sha256)
    certificate = _read_sealed_json(
        args.certificate, args.certificate_sha256)
    result = _read_sealed_json(args.result, args.result_sha256)
    _read_sealed_bytes(args.scene, args.scene_sha256, 0o400)
    _read_sealed_bytes(
        args.support_orchestrator, args.support_orchestrator_sha256, 0o500)
    support_manifest = _read_sealed_json(
        args.support_manifest, args.support_manifest_sha256)
    support_certificate = _read_sealed_json(
        args.support_certificate, args.support_certificate_sha256)

    # This is the sole external semantic authority.  It is deliberately
    # called only after all explicit file hashes and identities pass.
    contract, admission = _load_semantic_authority(args)
    try:
        admitted_plan = admission.admit_result(
            args.result, args.result_sha256)
    except Exception as error:
        raise MiddleExecutionError(f"RESULT_ADMISSION:{error}") from error
    _require(plan.value == admitted_plan, "PLAN_RESULT_IDENTITY")
    _require(result.value.get("schema") == RESULT_SCHEMA, "RESULT_SCHEMA")
    _require(result.value.get("plan") == plan.value, "RESULT_PLAN")
    _require("certificate" not in result.value, "RESULT_CERTIFICATE_FORBIDDEN")
    _require(result.value.get("plan_file_sha256") == plan.sha256,
             "RESULT_PLAN_FILE_SHA256")
    _require(plan.value.get("schema") == PLAN_SCHEMA, "PLAN_SCHEMA")
    scene_binding = plan.value.get("scene_binding")
    _require(type(scene_binding) is dict, "PLAN_SCENE_BINDING")
    _require(scene_binding.get("scene_cdr_sha256") == args.scene_sha256,
             "PLAN_SCENE_SHA")
    support_contract_sha256 = plan.value.get("support_contract_sha256")
    try:
        contract.validate_certificate(
            certificate.value,
            plan_file_sha256=plan.sha256,
            exact_result_file_sha256=result.sha256,
            result=result.value,
            support_contract_sha256=support_contract_sha256,
        )
    except Exception as error:
        raise MiddleExecutionError(f"CERTIFICATE_CONTRACT:{error}") from error
    _validate_support_dag(
        manifest=support_manifest,
        support_certificate=support_certificate,
        plan=plan,
        certificate=certificate,
        result=result,
    )
    _validate_support_contract_offline(args)
    _require(tuple(plan.value.get("phase_names", ())) == PHASE_NAMES,
             "PLAN_PHASE_NAMES")
    _require(plan.value.get("release_owner") == RELEASE_OWNER,
             "PLAN_RELEASE_OWNER")
    for key, item in plan.value.items():
        if key == "terminal_seam":
            continue
        for text in (key, *_all_strings(item)):
            _require(not any(forbidden in text for forbidden in FORBIDDEN_TEXT),
                     "PLAN_FORBIDDEN_RIGHT_RELEASE_RETREAT")
    _validate_hooks(plan.value)
    phases = _validate_phases(workspace, plan.value)
    return ValidatedMiddle(
        workspace, args.scene, args.scene_sha256, plan, certificate, result,
        phases, args.support_orchestrator, args.support_manifest,
        args.support_manifest_sha256, args.support_certificate,
        args.support_certificate_sha256)


class _JointFeedback:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._positions: Dict[str, float] = {}
        self._receipt_ns: Dict[str, int] = {}
        self._torso_violation: float | None = None
        self._right_grasp_armed = False
        self._right_grasp_violation: Tuple[str, float] | None = None

    def update(self, message: Any) -> None:
        if len(message.name) != len(message.position):
            return
        values = {}
        for name, position in zip(message.name, message.position):
            if not math.isfinite(position):
                return
            values[name] = float(position)
        with self._condition:
            receipt_ns = time.monotonic_ns()
            self._positions.update(values)
            for name in values:
                self._receipt_ns[name] = receipt_ns
            torso = values.get("torso_joint")
            if (torso is not None and
                    abs(torso) > HARD_MAX_TORSO_RAD + 1e-12):
                self._torso_violation = torso
            if self._right_grasp_armed:
                for name, expected in zip(
                        RIGHT_GRIPPER_JOINTS, E0_RIGHT_GRIPPER):
                    actual = values.get(name)
                    if (actual is not None and
                            abs(actual - expected) >
                            HARD_GRIPPER_TOLERANCE_M):
                        self._right_grasp_violation = (name, actual)
            self._condition.notify_all()

    def arm_right_grasp_watchdog(self) -> None:
        with self._condition:
            self._right_grasp_armed = True

    def require_torso_safe(self) -> None:
        with self._condition:
            if self._torso_violation is not None:
                raise MiddleExecutionError(
                    f"RUNTIME_TORSO_LIMIT:{self._torso_violation}")
            if self._right_grasp_violation is not None:
                name, actual = self._right_grasp_violation
                raise MiddleExecutionError(
                    f"RUNTIME_RIGHT_GRASP_VIOLATION:{name}:{actual}")
            if self._right_grasp_armed:
                now_ns = time.monotonic_ns()
                maximum_age_ns = int(
                    HARD_RIGHT_GRASP_FEEDBACK_MAX_AGE_S * 1_000_000_000)
                for name in RIGHT_GRIPPER_JOINTS:
                    receipt_ns = self._receipt_ns.get(name)
                    if receipt_ns is None:
                        raise MiddleExecutionError(
                            f"RUNTIME_RIGHT_GRASP_FEEDBACK_MISSING:{name}")
                    age_ns = now_ns - receipt_ns
                    if age_ns > maximum_age_ns:
                        raise MiddleExecutionError(
                            f"RUNTIME_RIGHT_GRASP_FEEDBACK_STALE:{name}:"
                            f"{age_ns / 1_000_000_000:.9g}")

    def snapshot(
        self, names: Sequence[str], newer_than_ns: int,
    ) -> Dict[str, float] | None:
        sample = self.snapshot_with_receipt(names, newer_than_ns)
        return None if sample is None else sample[0]

    def snapshot_with_receipt(
        self, names: Sequence[str], newer_than_ns: int,
    ) -> Tuple[Dict[str, float], int] | None:
        with self._condition:
            if any(name not in self._positions or
                   self._receipt_ns.get(name, 0) <= newer_than_ns
                   for name in names):
                return None
            # Advancing the next lower bound to the newest component receipt
            # forces every requested joint to be refreshed for the next
            # complete sample; a partially refreshed aggregate cannot count
            # twice toward endpoint convergence.
            receipt_ns = max(self._receipt_ns[name] for name in names)
            return ({name: self._positions[name] for name in names},
                    receipt_ns)

    def latest(self, names: Sequence[str]) -> Dict[str, float] | None:
        with self._condition:
            if any(name not in self._positions for name in names):
                return None
            return {name: self._positions[name] for name in names}


def _wait_feedback(
    executor: Any, feedback: _JointFeedback, names: Sequence[str],
    newer_than_ns: int, timeout_s: float, *, allow_unsafe_torso: bool = False,
) -> Dict[str, float]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not allow_unsafe_torso:
            feedback.require_torso_safe()
        values = feedback.snapshot(names, newer_than_ns)
        if values is not None:
            return values
        executor.spin_once(timeout_sec=0.02)
    raise MiddleExecutionError("JOINT_FEEDBACK_TIMEOUT")


def _feedback_tolerance(name: str, *, start: bool = False) -> float:
    return (HARD_GRIPPER_TOLERANCE_M if "finger" in name else
            (HARD_START_TOLERANCE_RAD if start
             else HARD_ENDPOINT_TOLERANCE_RAD))


def _endpoint_tolerance(
    name: str, phase_name: str, command_indices: Mapping[str, int],
) -> float:
    """Match the loaded return endpoint to the downstream support gate."""
    if (
        "finger" not in name and
        phase_name == "c2_return_carry_to_origin" and
        frozenset(command_indices.values()) == frozenset((1,))
    ):
        return RETURN_CARRY_ORIGIN_ENDPOINT_TOLERANCE_RAD
    return _feedback_tolerance(name)


def _close_feedback(
    measured: Mapping[str, float], expected: Mapping[str, float],
    names: Sequence[str], status: str, *, start: bool = False,
) -> None:
    for name in names:
        tolerance = _feedback_tolerance(name, start=start)
        _require(abs(measured[name] - expected[name]) <= tolerance,
                 f"{status}:{name}")


def _worst_endpoint_error(
    measured: Mapping[str, float], expected: Mapping[str, float],
    names: Sequence[str], command_indices: Mapping[str, int],
    phase_name: str,
) -> Tuple[int, str, float, float, float, float]:
    ranked = []
    for name in names:
        tolerance = _endpoint_tolerance(
            name, phase_name, command_indices)
        actual = measured[name]
        target = expected[name]
        error = abs(actual - target)
        ranked.append((
            error / tolerance, error, name, command_indices[name],
            actual, target, tolerance))
    _, error, name, command_index, actual, target, tolerance = max(ranked)
    return command_index, name, actual, target, error, tolerance


def _endpoint_failure(
    *, phase_name: str, reason: str, measured: Mapping[str, float],
    expected: Mapping[str, float], names: Sequence[str],
    command_indices: Mapping[str, int], consecutive: int,
    fresh_samples: int,
) -> MiddleExecutionError:
    (command_index, name, actual, target, error,
    tolerance) = _worst_endpoint_error(
         measured, expected, names, command_indices, phase_name)
    return MiddleExecutionError(
        f"PHASE_ENDPOINT:{phase_name}:reason={reason}:"
        f"command_index={command_index}:worst_joint={name}:"
        f"measured={actual:.17g}:expected={target:.17g}:"
        f"abs_error={error:.17g}:tolerance={tolerance:.17g}:"
        f"consecutive={consecutive}:"
        f"required={HARD_ENDPOINT_CONSECUTIVE_SAMPLES}:"
        f"fresh_samples={fresh_samples}")


def _endpoint_settle_sim_ns(
    phase_name: str, command_indices: Mapping[str, int],
) -> int:
    """Give the fast loaded return endpoint time to physically converge."""
    if (
        phase_name == "c2_return_carry_to_origin" and
        frozenset(command_indices.values()) == frozenset((1,))
    ):
        # Keep this one loaded endpoint under observation long enough to meet
        # the downstream 3 mrad support gate.  No trajectory point, speed, or
        # sample count is changed.
        return RETURN_CARRY_ORIGIN_ENDPOINT_SETTLE_SIM_NS
    return HARD_ENDPOINT_SETTLE_SIM_NS


def _wait_endpoint_convergence(
    executor: Any, node: Any, feedback: _JointFeedback,
    names: Sequence[str], expected: Mapping[str, float],
    command_indices: Mapping[str, int], phase_name: str,
    sim_target_ns: int, wall_deadline: float, last_clock_ns: int,
    last_clock_wall: float,
) -> None:
    settle_start_sim_ns = node.get_clock().now().nanoseconds
    _require(settle_start_sim_ns >= sim_target_ns,
             f"PHASE_ENDPOINT:{phase_name}:SIM_TARGET_NOT_REACHED")
    settle_start_wall = time.monotonic()
    if settle_start_sim_ns > last_clock_ns:
        last_clock_ns = settle_start_sim_ns
        last_clock_wall = settle_start_wall

    # This lower bound is deliberately captured only after the simulation
    # target has been observed.  Feedback received during trajectory travel
    # can never satisfy even the first endpoint sample.
    newer_than_ns = time.monotonic_ns()
    settle_deadline_sim_ns = sim_target_ns + _endpoint_settle_sim_ns(
        phase_name, command_indices)
    latest = feedback.latest(names)
    _require(latest is not None,
             f"PHASE_ENDPOINT:{phase_name}:FEEDBACK_MISSING")
    consecutive = 0
    fresh_samples = 0

    while True:
        wall_now = time.monotonic()
        if wall_now >= wall_deadline:
            raise _endpoint_failure(
                phase_name=phase_name, reason="WALL_TIMEOUT",
                measured=latest, expected=expected, names=names,
                command_indices=command_indices, consecutive=consecutive,
                fresh_samples=fresh_samples)
        feedback.require_torso_safe()
        _require_unique_controller_publishers(
            node, f"PHASE_ENDPOINT_SETTLE:{phase_name}")
        clock_now = node.get_clock().now().nanoseconds
        if clock_now > last_clock_ns:
            last_clock_ns = clock_now
            last_clock_wall = wall_now
        elif wall_now - last_clock_wall > HARD_SIM_CLOCK_STALL_S:
            raise MiddleExecutionError(
                f"PHASE_SIM_CLOCK_STALL:{phase_name}:ENDPOINT_SETTLE")
        if clock_now > settle_deadline_sim_ns:
            raise _endpoint_failure(
                phase_name=phase_name, reason="SIM_SETTLE_TIMEOUT",
                measured=latest, expected=expected, names=names,
                command_indices=command_indices, consecutive=consecutive,
                fresh_samples=fresh_samples)

        sample = feedback.snapshot_with_receipt(names, newer_than_ns)
        if sample is not None:
            latest, newer_than_ns = sample
            fresh_samples += 1
            in_tolerance = all(
                abs(latest[name] - expected[name]) <=
                _endpoint_tolerance(name, phase_name, command_indices)
                for name in names)
            consecutive = consecutive + 1 if in_tolerance else 0
            if consecutive >= HARD_ENDPOINT_CONSECUTIVE_SAMPLES:
                return
        executor.spin_once(timeout_sec=0.02)


def _e0_expected() -> Dict[str, float]:
    output = {"torso_joint": E0_TORSO_RAD}
    output.update(zip(LEFT_ARM_JOINTS, E0_LEFT_ARM_RAD))
    output.update(zip(RIGHT_ARM_JOINTS, E0_RIGHT_ARM_RAD))
    output.update(zip(LEFT_GRIPPER_JOINTS, E0_LEFT_GRIPPER))
    output.update(zip(RIGHT_GRIPPER_JOINTS, E0_RIGHT_GRIPPER))
    return output


def _require_fresh_e0(
    executor: Any, feedback: _JointFeedback, timeout_s: float, status: str,
) -> None:
    expected = _e0_expected()
    names = tuple(expected)
    requested = time.monotonic_ns()
    measured = _wait_feedback(
        executor, feedback, names, requested, timeout_s)
    _close_feedback(measured, expected, names, status, start=True)


def _require_fresh_right_grasp(
    executor: Any, feedback: _JointFeedback, timeout_s: float, status: str,
) -> None:
    expected = dict(zip(RIGHT_GRIPPER_JOINTS, E0_RIGHT_GRIPPER))
    requested = time.monotonic_ns()
    measured = _wait_feedback(
        executor, feedback, RIGHT_GRIPPER_JOINTS, requested, timeout_s)
    _close_feedback(
        measured, expected, RIGHT_GRIPPER_JOINTS, status, start=True)


def _duration_parts(nanoseconds: int) -> Tuple[int, int]:
    return nanoseconds // 1_000_000_000, nanoseconds % 1_000_000_000


def _fast_divisor(
    command: Command, phase_name: str, command_index: int,
) -> float:
    if command.topic == "/left_gripper_controller/joint_trajectory":
        return 1.0
    # Keep the fast profile everywhere except the final loaded placement
    # micro-segment.  r11 showed that compressing this sealed 0.987 s segment
    # to 0.141 s made the shoulder cross its target before the elbow arrived,
    # so the unchanged 0.004 rad all-joint / three-fresh-sample endpoint gate
    # had no simultaneous acceptance window.  Restoring this one segment's
    # sealed timing changes no position, threshold, phase, or command count.
    if ((phase_name == "cube_carry_place" and command_index == 1) or
            (phase_name == "c2_return_carry_to_origin" and
             command_index == 0)) and command.topic == (
                 "/left_arm_controller/joint_trajectory"):
        return 1.0
    return FAST_TIME_DIVISOR


def _fast_times_ns(
    command: Command, phase_name: str, command_index: int,
) -> Tuple[int, ...]:
    divisor = _fast_divisor(command, phase_name, command_index)
    result = [int(round(point.time_from_start_ns / divisor))
              for point in command.points]
    if (
        phase_name == "c2_return_carry_to_origin" and
        command_index == 1 and
        command.topic == "/left_arm_controller/joint_trajectory"
    ):
        # Keep the long return at the seven-times profile, but restore the
        # final 12 sealed point intervals.  The live bags showed that the
        # compressed 90 ms tail demanded an abrupt 2.856 rad/s elbow stop
        # while the cube was loaded at table contact.  This bounded retime
        # adds about 0.542 s and changes neither positions nor message count.
        tail_start = len(command.points) - RETURN_CARRY_ORIGIN_TAIL_POINT_COUNT
        _require(tail_start > 0, "FAST_RETURN_TAIL_POINT_COUNT")
        fast_anchor_ns = result[tail_start]
        sealed_anchor_ns = command.points[tail_start].time_from_start_ns
        for point_index in range(tail_start + 1, len(command.points)):
            result[point_index] = (
                fast_anchor_ns +
                command.points[point_index].time_from_start_ns -
                sealed_anchor_ns)
    _require(all(right > left for left, right in zip(result, result[1:])),
             f"FAST_TIME_ORDER:{command.topic}")
    return tuple(result)


def _pchip_command_velocities(
    command: Command, times_ns: Sequence[int],
) -> Tuple[Tuple[float, ...], ...]:
    """Shape-preserving C1 velocities for the unchanged sealed positions."""
    count = len(command.points)
    _require(count == len(times_ns) and count >= 1,
             f"FAST_PROFILE_INPUT:{command.topic}")
    output = [[0.0] * len(command.joint_names) for _ in range(count)]
    if count < 3:
        return tuple(tuple(row) for row in output)
    intervals = [
        (times_ns[index + 1] - times_ns[index]) / 1e9
        for index in range(count - 1)
    ]
    _require(all(math.isfinite(value) and value > 0.0 for value in intervals),
             f"FAST_PROFILE_INTERVAL:{command.topic}")
    for axis in range(len(command.joint_names)):
        slopes = [
            (command.points[index + 1].positions[axis] -
             command.points[index].positions[axis]) / intervals[index]
            for index in range(count - 1)
        ]
        for index in range(1, count - 1):
            before, after = slopes[index - 1], slopes[index]
            if before * after <= 0.0:
                continue
            before_dt, after_dt = intervals[index - 1], intervals[index]
            weight_before = 2.0 * after_dt + before_dt
            weight_after = after_dt + 2.0 * before_dt
            output[index][axis] = (
                (weight_before + weight_after) /
                (weight_before / before + weight_after / after))
    return tuple(tuple(row) for row in output)


def _validate_fast_cubic_profile(
    command: Command, times_ns: Sequence[int],
    velocities: Sequence[Sequence[float]], phase_name: str,
) -> None:
    """Analytically enforce the sealed fast-831 cubic motion bounds."""
    for segment in range(len(command.points) - 1):
        dt_s = (times_ns[segment + 1] - times_ns[segment]) / 1e9
        for axis, name in enumerate(command.joint_names):
            q0 = command.points[segment].positions[axis]
            q1 = command.points[segment + 1].positions[axis]
            v0 = velocities[segment][axis]
            v1 = velocities[segment + 1][axis]
            a = 2.0 * (q0 - q1) + dt_s * (v0 + v1)
            b = 3.0 * (q1 - q0) - dt_s * (2.0 * v0 + v1)
            c = dt_s * v0
            samples = [0.0, 1.0]
            if abs(a) > 1e-30:
                stationary = -b / (3.0 * a)
                if 0.0 < stationary < 1.0:
                    samples.append(stationary)
            maximum_velocity = max(abs(
                (3.0 * a * sample * sample + 2.0 * b * sample + c) /
                dt_s) for sample in samples)
            maximum_acceleration = max(abs(
                (6.0 * a * sample + 2.0 * b) / (dt_s * dt_s))
                for sample in (0.0, 1.0))
            if "finger" in name:
                velocity_limit = HARD_FAST_MAX_PRISMATIC_VELOCITY_M_S
                acceleration_limit = HARD_FAST_MAX_PRISMATIC_ACCELERATION_M_S2
            else:
                velocity_limit = HARD_FAST_MAX_REVOLUTE_VELOCITY_RAD_S
                acceleration_limit = HARD_FAST_MAX_REVOLUTE_ACCELERATION_RAD_S2
            _require(maximum_velocity <= velocity_limit + 1e-12,
                     f"FAST_VELOCITY:{phase_name}:{segment}:{name}")
            _require(maximum_acceleration <= acceleration_limit + 1e-9,
                     f"FAST_ACCELERATION:{phase_name}:{segment}:{name}")


def _require_unique_controller_publishers(node: Any, status: str) -> None:
    for topic in CANONICAL_CONTROLLER_TOPICS:
        publishers = node.get_publishers_info_by_topic(topic)
        _require(len(publishers) == 1,
                 f"{status}:PUBLISHER_COUNT:{topic}:{len(publishers)}")
        _require(publishers[0].node_name == node.get_name(),
                 f"{status}:PUBLISHER_OWNER:{topic}")


def _controller_and_recorder_ready(
    publishers: Mapping[str, Any],
) -> bool:
    """Require both the trajectory controller and sealed rosbag recorder."""
    return all(
        publisher.get_subscription_count() >=
        REQUIRED_CONTROLLER_AND_RECORDER_SUBSCRIPTIONS
        for publisher in publishers.values())


def _run_support_hook(
    node: Any, executor: Any, feedback: _JointFeedback,
    bundle: ValidatedMiddle,
    action: str, timeout_s: float, daemon_socket: Path,
    direction: str, roundtrip: bool,
) -> None:
    _require_unique_controller_publishers(
        node, f"SUPPORT_PRE:{action}")
    _require(direction in EXECUTION_DIRECTIONS,
             f"SUPPORT_DIRECTION:{direction}")
    _require(type(roundtrip) is bool, "SUPPORT_ROUNDTRIP_TYPE")
    _require(direction != "return" or roundtrip,
             "RETURN_SUPPORT_REQUIRES_ROUNDTRIP")
    cli_action = SUPPORT_CLI_ACTION[action]
    if direction == "return":
        cli_action += "-return"
    try:
        metadata = daemon_socket.lstat()
    except OSError as error:
        raise MiddleExecutionError(
            f"SUPPORT_DAEMON_SOCKET:{action}:{error}") from error
    _require(
        stat.S_ISSOCK(metadata.st_mode) and not daemon_socket.is_symlink() and
        metadata.st_uid == os.geteuid() and
        stat.S_IMODE(metadata.st_mode) == 0o600,
        f"SUPPORT_DAEMON_SOCKET_IDENTITY:{action}")
    response = bytearray()
    deadline = time.monotonic() + min(
        SUPPORT_PROCESS_WALL_TIMEOUT_S, max(10.0, timeout_s * 2.0))
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(min(2.0, timeout_s))
            client.connect(str(daemon_socket))
            client.sendall((cli_action + "\n").encode("ascii"))
            client.setblocking(False)
            while b"\n" not in response:
                if time.monotonic() >= deadline:
                    raise MiddleExecutionError(
                        f"SUPPORT_DAEMON_TIMEOUT:{action}")
                try:
                    block = client.recv(4097 - len(response))
                except BlockingIOError:
                    block = None
                if block == b"":
                    raise MiddleExecutionError(
                        f"SUPPORT_DAEMON_EOF:{action}")
                if block:
                    response.extend(block)
                    _require(len(response) <= 4096,
                             f"SUPPORT_DAEMON_RESPONSE_BUDGET:{action}")
                    continue
                executor.spin_once(timeout_sec=0.02)
                feedback.require_torso_safe()
                _require_unique_controller_publishers(
                    node, f"SUPPORT_ACTIVE:{action}")
    except OSError as error:
        raise MiddleExecutionError(
            f"SUPPORT_DAEMON_IO:{action}:{error}") from error
    _require(response.endswith(b"\n") and response.count(b"\n") == 1,
             f"SUPPORT_DAEMON_RESPONSE_FRAME:{action}")
    try:
        marker = bytes(response[:-1]).decode("utf-8")
    except UnicodeDecodeError as error:
        raise MiddleExecutionError(
            f"SUPPORT_DAEMON_RESPONSE_UTF8:{action}:{error}") from error
    if direction == "insert" and not roundtrip:
        expected = {
            "door_attach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=door-attach door=attached "
                           "cube=detached boundary=E0_ENTRY",
            "cube_attach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=cube-attach door=attached "
                           "cube=attached boundary=CUBE_CONTACT",
            "cube_detach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=cube-detach door=attached "
                           "cube=detached boundary=CUBE_RELEASE",
            "door_detach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=door-detach door=detached "
                           "cube=detached boundary=E0_RETURN",
        }[action]
    elif direction == "insert":
        expected = {
            "door_attach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=door-attach cycle=1 door=attached "
                           "cube=detached boundary=E0_ENTRY",
            "cube_attach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=cube-attach cycle=1 door=attached "
                           "cube=attached boundary=CUBE_CONTACT_ORIGIN",
            "cube_detach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=cube-detach cycle=1 door=attached "
                           "cube=detached boundary=CUBE_RELEASE_INSIDE",
            "door_detach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=door-detach cycle=1 door=detached "
                           "cube=detached boundary=E0_RETURN",
        }[action]
    else:
        expected = {
            "door_attach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=door-attach-return cycle=2 "
                           "door=attached cube=detached boundary=E0_ENTRY",
            "cube_attach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=cube-attach-return cycle=2 "
                           "door=attached cube=attached "
                           "boundary=CUBE_RELEASE_INSIDE",
            "cube_detach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=cube-detach-return cycle=2 "
                           "door=attached cube=detached "
                           "boundary=CUBE_CONTACT_ORIGIN",
            "door_detach": "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                           "operation=door-detach-return cycle=2 "
                           "door=detached cube=detached boundary=E0_RETURN",
        }[action]
    _require(marker == expected, f"SUPPORT_MARKER:{action}:{marker}")
    _require_unique_controller_publishers(
        node, f"SUPPORT_POST:{action}")
    print(expected, flush=True)


def _execute_ros(
    bundle: ValidatedMiddle, speed_scale: float, discovery_timeout: float,
    wall_timeout_factor: float, support_timeout: float,
    support_daemon_socket: Path, direction: str, roundtrip: bool,
) -> None:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    node = Node(
        f"h1_candidate3_middle_executor_828_{os.getpid()}",
        parameter_overrides=[Parameter(
            "use_sim_time", Parameter.Type.BOOL, True)],
        automatically_declare_parameters_from_overrides=True,
        context=context)
    executor.add_node(node)
    qos = QoSProfile(
        depth=10, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE)
    feedback = _JointFeedback()
    subscription = node.create_subscription(
        JointState, "/joint_states", feedback.update, 10)
    publishers = {
        topic: node.create_publisher(JointTrajectory, topic, qos)
        for topic in CANONICAL_CONTROLLER_TOPICS
    }
    _require(direction in EXECUTION_DIRECTIONS,
             f"EXECUTION_DIRECTION:{direction}")
    phases = (bundle.phases if direction == "insert" else
              build_return_phases(bundle))
    synchronized_phases = (
        SYNCHRONIZED_PHASES if direction == "insert" else
        RETURN_SYNCHRONIZED_PHASES)
    cube_attach_phase = 4 if direction == "insert" else 5
    cube_detach_phase = 5 if direction == "insert" else 6
    door_detach_phase = 10 if direction == "insert" else 11
    last_commanded = _e0_expected()

    def hold_current() -> None:
        names = tuple(
            name for group in CONTROLLER_JOINTS.values() for name in group)
        requested = time.monotonic_ns()
        try:
            measured = _wait_feedback(
                executor, feedback, names, requested, 0.5,
                allow_unsafe_torso=True)
        except MiddleExecutionError:
            measured = feedback.latest(names)
            if measured is None:
                measured = {name: last_commanded[name] for name in names}
        for topic, topic_names in CONTROLLER_JOINTS.items():
            message = JointTrajectory()
            message.joint_names = list(topic_names)
            point = JointTrajectoryPoint()
            point.positions = [measured[name] for name in topic_names]
            point.velocities = [0.0] * len(topic_names)
            point.accelerations = [0.0] * len(topic_names)
            point.effort = []
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 100_000_000
            message.points = [point]
            publishers[topic].publish(message)
        until = time.monotonic() + 0.3
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=0.02)
        print("CANDIDATE3_MIDDLE_828_FAILURE_HOLD=PASS", flush=True)

    def execute_group(
        phase: Phase,
        indexed_commands: Sequence[Tuple[int, Command]],
    ) -> None:
        nonlocal last_commanded
        commands = tuple(command for _, command in indexed_commands)
        names = tuple(
            name for command in commands for name in command.joint_names)
        _require(len(names) == len(frozenset(names)),
                 f"RUNTIME_SYNC_JOINT_DUPLICATE:{phase.name}")
        command_indices = {
            name: command_index
            for command_index, command in indexed_commands
            for name in command.joint_names
        }
        expected_start = {
            name: value for command in commands
            for name, value in zip(command.joint_names,
                                   command.points[0].positions)
        }
        requested = time.monotonic_ns()
        measured = _wait_feedback(
            executor, feedback, names, requested, discovery_timeout)
        _close_feedback(
            measured, expected_start, names,
            f"PHASE_START:{phase.name}", start=True)
        _require_fresh_right_grasp(
            executor, feedback, discovery_timeout,
            f"RIGHT_GRASP_PRE:{phase.name}")
        outgoing = []
        maximum_ns = 0
        for command_index, command in indexed_commands:
            fast_times_ns = _fast_times_ns(
                command, phase.name, command_index)
            command_velocities = _pchip_command_velocities(
                command, fast_times_ns)
            _validate_fast_cubic_profile(
                command, fast_times_ns, command_velocities, phase.name)
            message = JointTrajectory()
            message.joint_names = list(command.joint_names)
            for point_index, sealed_point in enumerate(command.points):
                point = JointTrajectoryPoint()
                point.positions = list(sealed_point.positions)
                point.velocities = list(command_velocities[point_index])
                point.accelerations = []
                point.effort = []
                scaled_ns = int(round(
                    fast_times_ns[point_index] / speed_scale))
                scheduled_ns = int(HARD_MIN_LEAD_TIME_S * 1e9) + scaled_ns
                point.time_from_start.sec, point.time_from_start.nanosec = (
                    _duration_parts(scheduled_ns))
                message.points.append(point)
                maximum_ns = max(maximum_ns, scheduled_ns)
            outgoing.append((command.topic, message))
        sim_start = node.get_clock().now().nanoseconds
        for topic, message in outgoing:
            publishers[topic].publish(message)
        for command in commands:
            last_commanded.update(zip(
                command.joint_names, command.points[-1].positions))
        sim_target = sim_start + maximum_ns + 50_000_000
        endpoint_settle_sim_ns = _endpoint_settle_sim_ns(
            phase.name, command_indices)
        wall_deadline = time.monotonic() + max(
            discovery_timeout,
            (maximum_ns + 50_000_000 + endpoint_settle_sim_ns) /
            1e9 * wall_timeout_factor)
        last_clock = sim_start
        last_clock_wall = time.monotonic()
        while time.monotonic() < wall_deadline:
            executor.spin_once(timeout_sec=0.02)
            feedback.require_torso_safe()
            _require_unique_controller_publishers(
                node, f"PHASE_ACTIVE:{phase.name}")
            clock_now = node.get_clock().now().nanoseconds
            if clock_now > last_clock:
                last_clock = clock_now
                last_clock_wall = time.monotonic()
            elif time.monotonic() - last_clock_wall > HARD_SIM_CLOCK_STALL_S:
                raise MiddleExecutionError(
                    f"PHASE_SIM_CLOCK_STALL:{phase.name}")
            if clock_now >= sim_target:
                break
        else:
            raise MiddleExecutionError(f"PHASE_SIM_TIMEOUT:{phase.name}")
        expected_end = {
            name: value for command in commands
            for name, value in zip(command.joint_names,
                                   command.points[-1].positions)
        }
        _wait_endpoint_convergence(
            executor, node, feedback, names, expected_end,
            command_indices, phase.name, sim_target, wall_deadline,
            last_clock, last_clock_wall)
        _require_fresh_right_grasp(
            executor, feedback, discovery_timeout,
            f"RIGHT_GRASP_POST:{phase.name}")

    try:
        discovery_deadline = time.monotonic() + discovery_timeout
        while time.monotonic() < discovery_deadline:
            executor.spin_once(timeout_sec=0.02)
            feedback.require_torso_safe()
            if _controller_and_recorder_ready(publishers):
                break
        else:
            raise MiddleExecutionError(
                "CONTROLLER_AND_RECORDER_DISCOVERY")
        _require_unique_controller_publishers(node, "EXECUTOR_READY")
        clock_deadline = time.monotonic() + discovery_timeout
        first_clock = 0
        while time.monotonic() < clock_deadline:
            executor.spin_once(timeout_sec=0.02)
            feedback.require_torso_safe()
            now = node.get_clock().now().nanoseconds
            if (bool(getattr(node.get_clock(), "ros_time_is_active", False))
                    and now > 0):
                if first_clock == 0:
                    first_clock = now
                elif now > first_clock:
                    break
        else:
            raise MiddleExecutionError("SIM_CLOCK_NOT_ADVANCING")

        _require_fresh_e0(
            executor, feedback, discovery_timeout, "MIDDLE_ENTRY_E0")
        feedback.arm_right_grasp_watchdog()
        for phase in phases:
            _require_unique_controller_publishers(
                node, f"PHASE_PRE:{phase.name}")
            if phase.index == 0:
                _run_support_hook(
                    node, executor, feedback, bundle,
                    "door_attach", support_timeout, support_daemon_socket,
                    direction, roundtrip)
            if phase.commands:
                if phase.index in synchronized_phases:
                    execute_group(phase, tuple(enumerate(phase.commands)))
                else:
                    for command_index, command in enumerate(phase.commands):
                        execute_group(
                            phase, ((command_index, command),))
            else:
                _require_fresh_right_grasp(
                    executor, feedback, discovery_timeout,
                    f"RIGHT_GRASP_EMPTY_PHASE:{phase.name}")
            if phase.index == cube_attach_phase:
                _run_support_hook(
                    node, executor, feedback, bundle,
                    "cube_attach", support_timeout, support_daemon_socket,
                    direction, roundtrip)
            elif phase.index == cube_detach_phase:
                _run_support_hook(
                    node, executor, feedback, bundle,
                    "cube_detach", support_timeout, support_daemon_socket,
                    direction, roundtrip)
            if phase.index == door_detach_phase:
                _require_fresh_e0(
                    executor, feedback, discovery_timeout,
                    "MIDDLE_TERMINAL_E0_BEFORE_DOOR_DETACH")
                _run_support_hook(
                    node, executor, feedback, bundle,
                    "door_detach", support_timeout, support_daemon_socket,
                    direction, roundtrip)
            print(
                f"CANDIDATE3_MIDDLE_828_PHASE=PASS index={phase.index} "
                f"name={phase.name} commands={len(phase.commands)}",
                flush=True)
        _require_fresh_e0(
            executor, feedback, discovery_timeout, "MIDDLE_TERMINAL_E0")
        marker = ("CANDIDATE3_MIDDLE_828" if direction == "insert" else
                  "CANDIDATE3_MIDDLE_RETURN_831")
        print(
            f"{marker}=PASS phases={len(phases)} right_gripper_commands=0 "
            "terminal_e0_held=true",
            flush=True)
    except BaseException:
        try:
            hold_current()
        except BaseException as hold_error:
            print(
                f"CANDIDATE3_MIDDLE_828_FAILURE_HOLD=FAIL:{hold_error}",
                file=sys.stderr, flush=True)
        raise
    finally:
        node.destroy_subscription(subscription)
        for publisher in publishers.values():
            node.destroy_publisher(publisher)
        executor.remove_node(node)
        try:
            executor.shutdown()
        finally:
            try:
                node.destroy_node()
            finally:
                rclpy.shutdown(context=context)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--scene-sha256", required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--certificate", type=Path, required=True)
    parser.add_argument("--certificate-sha256", required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--result-sha256", required=True)
    parser.add_argument("--middle-admission", type=Path, required=True)
    parser.add_argument("--middle-admission-sha256", required=True)
    parser.add_argument("--middle-contract", type=Path, required=True)
    parser.add_argument("--middle-contract-sha256", required=True)
    parser.add_argument("--support-orchestrator", type=Path, required=True)
    parser.add_argument("--support-orchestrator-sha256", required=True)
    parser.add_argument("--support-manifest", type=Path, required=True)
    parser.add_argument("--support-manifest-sha256", required=True)
    parser.add_argument("--support-certificate", type=Path, required=True)
    parser.add_argument("--support-certificate-sha256", required=True)
    parser.add_argument("--speed-scale", type=float, default=1.0)
    parser.add_argument("--discovery-timeout", type=float, default=10.0)
    parser.add_argument("--wall-timeout-factor", type=float, default=20.0)
    parser.add_argument("--support-timeout", type=float, default=30.0)
    parser.add_argument("--support-daemon-socket", type=Path)
    parser.add_argument(
        "--direction", choices=EXECUTION_DIRECTIONS, default="insert")
    parser.add_argument("--roundtrip", action="store_true")
    parser.add_argument("--return-plan", type=Path)
    parser.add_argument("--return-plan-sha256")
    parser.add_argument("--return-authorization", type=Path)
    parser.add_argument("--return-authorization-sha256")
    parser.add_argument("--execute", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        for name in (
            "workspace", "scene", "plan", "certificate", "result",
            "middle_admission", "middle_contract",
            "support_orchestrator", "support_manifest",
            "support_certificate",
        ):
            _require(getattr(args, name).is_absolute(),
                     f"{name.upper()}_NOT_ABSOLUTE")
        _require(type(args.speed_scale) is float and
                 math.isfinite(args.speed_scale) and
                 abs(args.speed_scale - 1.0) <= 1e-15,
                 "FAST_PROFILE_REQUIRES_SPEED_SCALE_ONE")
        _require(math.isfinite(args.discovery_timeout) and
                 0.0 < args.discovery_timeout <=
                 HARD_MAX_DISCOVERY_TIMEOUT_S,
                 "DISCOVERY_TIMEOUT_INVALID")
        _require(math.isfinite(args.wall_timeout_factor) and
                 1.0 <= args.wall_timeout_factor <=
                 HARD_MAX_WALL_TIMEOUT_FACTOR,
                 "WALL_TIMEOUT_FACTOR")
        _require(
            math.isfinite(args.support_timeout) and
            HARD_MIN_SUPPORT_TIMEOUT_S <= args.support_timeout <=
            HARD_MAX_SUPPORT_TIMEOUT_S,
            "SUPPORT_TIMEOUT_INVALID")
        if args.execute:
            _require(args.support_daemon_socket is not None and
                     args.support_daemon_socket.is_absolute(),
                     "SUPPORT_DAEMON_SOCKET_NOT_ABSOLUTE")
        bundle = validate_bundle(args)
        execution_phases = (
            bundle.phases if args.direction == "insert" else
            build_return_phases(bundle))
        if args.direction == "return":
            _require(args.roundtrip, "RETURN_REQUIRES_ROUNDTRIP")
            _require(args.return_plan is not None and
                     args.return_plan.is_absolute(),
                     "RETURN_PLAN_NOT_ABSOLUTE")
            return_plan = _validate_return_plan(
                args, bundle, execution_phases)
            _require(args.return_authorization is not None and
                     args.return_authorization.is_absolute(),
                     "RETURN_AUTHORIZATION_NOT_ABSOLUTE")
            _validate_return_authorization(args, bundle, return_plan)
        else:
            _require(args.return_plan is None and
                     args.return_plan_sha256 is None and
                     args.return_authorization is None and
                     args.return_authorization_sha256 is None,
                     "INSERT_RETURN_PLAN_ARGUMENT_FORBIDDEN")
        command_count = sum(
            len(phase.commands) for phase in execution_phases)
        point_count = sum(
            len(command.points) for phase in execution_phases
            for command in phase.commands)
        for phase in execution_phases:
            for command_index, command in enumerate(phase.commands):
                fast_times_ns = _fast_times_ns(
                    command, phase.name, command_index)
                velocities = _pchip_command_velocities(
                    command, fast_times_ns)
                _validate_fast_cubic_profile(
                    command, fast_times_ns, velocities, phase.name)
        if not args.execute:
            marker = ("CANDIDATE3_MIDDLE_828_VALIDATE"
                      if args.direction == "insert" else
                      "CANDIDATE3_MIDDLE_RETURN_831_VALIDATE")
            print(
                f"{marker}=PASS phases={len(execution_phases)} "
                f"commands={command_count} points={point_count} "
                "right_gripper_commands=0 ros_imports=0 support_calls=0 "
                "fast_time_divisor=7.0 "
                "loaded_place_microsegment_divisor=1.0")
            return 0
        _execute_ros(
            bundle, args.speed_scale, args.discovery_timeout,
            args.wall_timeout_factor, args.support_timeout,
            args.support_daemon_socket, args.direction, args.roundtrip)
        return 0
    except MiddleExecutionError as error:
        print(f"CANDIDATE3_MIDDLE_828=FAIL:{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
