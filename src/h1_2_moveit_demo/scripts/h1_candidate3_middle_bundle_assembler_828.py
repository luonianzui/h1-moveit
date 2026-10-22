#!/usr/bin/python3
"""Assemble, self-prove, and optionally publish the sealed 828 middle DAG.

The exact C++ audit is intentionally raw evidence, not an executable result.
This ROS-free program promotes only the frozen raw schema and verified input
bytes into four ordered nodes::

    plan -> exact result -> external certificate -> support manifest

Every node is canonical compact JSON followed by one newline.  All four are
created in a private staging directory, validated through the existing
contract, admission, and support authorities, and only then published with
no-replace hard links.  Publication is opt-in; the default is a dry run.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
import re
import shutil
import stat
import sys
import tempfile
import types
from typing import Any, Dict, Iterable, Mapping, MutableMapping, Sequence, Tuple


RAW_SCHEMA = "h1_candidate3_middle_e0_raw_combined_exact/828-e0-v1"
RAW_AUDITOR_VERSION = (
    "h1_candidate3_middle_e0_exact_auditor/828-e0-raw-v1")
RAW_AUDITOR_SOURCE_SHA256 = (
    "f3589a08b1f2ab59ee10bcd8830c39ba7a2779ff58bc4c2183e4118262eb7203")
EXPECTED_RAW_FILE_SHA256 = (
    "b1996cfe2a375e220814fbd81fe027ebd9b3f3067f2959d18a8ed7157d55c9b4")
RAW_STATUS = "RAW_EXACT_PASS_NOT_SEALED_RESULT"
RAW_AUDIT_METHOD = "FCL_UNPADDED_DYNAMIC_EXACT_STL_DENSE_INTERPOLATION"
RAW_BINDINGS_SCHEMA = (
    "h1_candidate3_middle_raw_asset_bindings/828-e0-v1")
PROVENANCE_SCHEMA = (
    "h1_candidate3_middle_scene_model_provenance/828-e0-v1")
AUDITOR_RUN_PLAN_SCHEMA = (
    "h1_candidate3_middle_auditor_run_plan/828-e0-v1")
AUDITOR_RUN_RESULT_SCHEMA = (
    "h1_candidate3_middle_auditor_run_result/828-e0-v1")
TIME_WRAPPER_PATH = Path("/usr/bin/time")
TIME_WRAPPER_SHA256 = (
    "3b11dec50514a8473e9f6efa7a34d584d0657538c09988f61b72d38ad4991a10")
TIME_WRAPPER_FORMAT = (
    "elapsed_seconds=%e\nuser_seconds=%U\nsystem_seconds=%S\nmax_rss_kb=%M")
WORKER_VERSION = "h1_candidate3_middle_bundle_assembler/828-e0-v1"
SUPPORT_MANIFEST_SCHEMA = "h1_support_orchestration_828/v1"
SUPPORT_CONTRACT_SCHEMA = "h1_support_contract_828/v1"
ASSET_SCHEMA = "h1_candidate3_middle_trajectory_asset/828-e0-v1"
ENDPOINT_SCHEMA = "h1_candidate3_middle_robot_endpoint/828-e0-v1"

RAW_RETREAT_KEYS = (
    "phase_name", "message_index", "start_point_index", "start_subdivision",
    "window_sample_count", "gate_point_index", "gate_subdivision",
    "gate_ordinal", "last_below_gate_ordinal", "gate_reached",
    "minimum_exact_distance_m", "maximum_exact_distance_m",
    "final_exact_distance_m", "maximum_exact_penetration_m",
    "minimum_time_step_ns", "minimum_path_parameter_step",
    "minimum_tcp_separation_projection_step_m", "final_tcp_projection_m",
    "post_gate_minimum_exact_distance_m",
    "post_gate_minimum_guard_physical_lower_bound_m", "penetration_geometry",
    "guard_geometry", "observed_pair_semantics", "observed_exact_pairs",
    "observed_guard_pairs", "observed_exact_links", "observed_guard_links",
    "gate_never_regressed",
)
RAW_RETREAT_CONTACT_KEYS = (
    "authorized_sample_count", "maximum_observed_penetration_m",
    "penetration_geometry", "guard_observation_geometry",
    "observed_pair_semantics", "observed_pairs", "guard_nearest_pairs",
)
RAW_RETREAT_ALLOWED_LINKS = frozenset((
    "left_dex1_finger_1_slider_link", "left_dex1_finger_1_link",
    "left_dex1_finger_1_pad_link", "left_dex1_finger_2_slider_link",
    "left_dex1_finger_2_link", "left_dex1_finger_2_pad_link",
))
RAW_RETREAT_PAIR_SEMANTICS = (
    "nearest_signed_distance_observation_not_collision_event")
RAW_RETREAT_EXACT_GEOMETRY = "green_cube_dynamic_exact_physical_only"
RAW_RETREAT_GUARD_GEOMETRY = (
    "green_cube_10mm_guard_conservative_nonphysical")
RAW_REQUIRED_CLEARANCE_M = 0.010001
RAW_ROOT_KEYS = (
    "schema", "auditor_version", "audit_method",
    "raw_asset_manifest_sha256", "raw_scene_hash", "physical_scene_hash",
    "raw_scene_cdr_sha256", "entry_projection", "robot_model_snapshot",
    "controller_commands_issued", "offline_only", "sample_count",
    "subdivision_count", "collision_free", "cube_bump", "final_target",
    "minimum_clearances_m", "cube_guard_exact_refinement_samples",
    "minimum_cube_exact_noncontact_clearance_m",
    "minimum_environment_clearance_is_lower_bound", "park_roll_rad",
    "environment_clearance_derivation", "park_static_clearance_required_m",
    "initial_park_exact_door_clearance", "final_park_exact_door_clearance",
    "right_handle_nonpad_clearance_required_m",
    "left_handle_clearance_required_m", "dynamic_handle_exact",
    "right_handle_allowed_contact_pairs", "right_door_allowed_contact_pairs",
    "left_cube_allowed_contact_links", "left_cube_object_ids",
    "contact_window_measurements", "torso_joint", "left_handle",
    "mask_projection", "mask_conflict_recheck", "cube_release_retreat",
    "per_phase_metrics", "raw_audit_complete", "geometry_valid", "valid",
    "status",
)

BASELINE_BACKUP_PATH = Path("/home/liu/h1_moveit/backups/826.3.tar.zst")
BASELINE_BACKUP_SHA256 = (
    "0c921cd91246a48007c9d82ea4bee27c682f07d17a2fda08e607e350719f3083")
PLUGIN_RELATIVE_PATH = (
    "install/h1_2_moveit_demo/lib/h1_2_moveit_demo/"
    "libh1_gazebo_compliant_grasp_system.so")

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
PHASE_ASSET_IDS = (
    "door_open", "door_open", "door60_seam_to_cube", "cube", "cube",
    "cube", "cube", "cube", "cube", "door60_seam_to_hinge", "door_close",
)
PHASE_RANGES = (
    (0, 0), (0, 2), (0, 2), (0, 1), (1, 3), (3, 5), (5, 6),
    (6, 7), (7, 8), (0, 2), (0, 2),
)
ASSET_IDS = (
    "door_open", "door60_seam_to_cube", "cube",
    "door60_seam_to_hinge", "door_close",
)
ASSET_TOPICS = {
    "door_open": (
        "/torso_controller/joint_trajectory",
        "/right_arm_controller/joint_trajectory",
    ),
    "door60_seam_to_cube": (
        "/torso_controller/joint_trajectory",
        "/right_arm_controller/joint_trajectory",
    ),
    "cube": (
        "/left_arm_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
        "/left_gripper_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
        "/left_gripper_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
    ),
    "door60_seam_to_hinge": (
        "/torso_controller/joint_trajectory",
        "/right_arm_controller/joint_trajectory",
    ),
    "door_close": (
        "/torso_controller/joint_trajectory",
        "/right_arm_controller/joint_trajectory",
    ),
}

TORSO_JOINTS = ("torso_joint",)
RIGHT_ARM_JOINTS = (
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
RIGHT_GRIPPER_JOINTS = (
    "right_dex1_finger_1_joint", "right_dex1_finger_2_joint")
LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
LEFT_GRIPPER_JOINTS = (
    "left_dex1_finger_1_joint", "left_dex1_finger_2_joint")
ALL_ROBOT_JOINTS = (
    *TORSO_JOINTS, *LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS,
    *LEFT_GRIPPER_JOINTS, *RIGHT_GRIPPER_JOINTS,
)
CONTROLLER_JOINTS = {
    "/torso_controller/joint_trajectory": TORSO_JOINTS,
    "/right_arm_controller/joint_trajectory": RIGHT_ARM_JOINTS,
    "/right_gripper_controller/joint_trajectory": RIGHT_GRIPPER_JOINTS,
    "/left_arm_controller/joint_trajectory": LEFT_ARM_JOINTS,
    "/left_gripper_controller/joint_trajectory": LEFT_GRIPPER_JOINTS,
}

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
AUTHORITY_IDS = (
    "contract", "admission", "right_executor", "middle_executor", "flow",
    "support",
)
AUDITOR_RUN_INPUT_IDS = (
    "asset_manifest", "cube", "cube_reference", "door_close", "door_open",
    "handle_stl", "robot_model_inventory", "scene", "seam_to_cube",
    "seam_to_hinge", "srdf", "urdf",
)
AUDITOR_RUN_OUTPUT_IDS = ("raw", "stderr", "stdout", "time")
AUDITOR_RUN_ARGUMENT_FLAGS = (
    "--scene", "--output", "--plan", "--park-roll", "--urdf", "--srdf",
    "--door-open", "--seam-to-cube", "--cube", "--cube-reference",
    "--seam-to-hinge", "--door-close", "--handle-stl",
    "--robot-model-inventory",
)

SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
MAX_JSON_BYTES = 1024 * 1024 * 1024
MAX_SOURCE_BYTES = 32 * 1024 * 1024
MAX_ARCHIVE_BYTES = 2 * 1024 * 1024 * 1024
ENDPOINT_TOLERANCE = 0.004


class MiddleBundleAssemblerError(RuntimeError):
    """A fail-closed input, semantic, staging, or publication failure."""


@dataclass(frozen=True)
class SealedDocument:
    path: Path
    sha256: str
    raw: bytes
    value: Mapping[str, Any]


@dataclass(frozen=True)
class Asset:
    asset_id: str
    binding: Mapping[str, Any]
    payload: Mapping[str, Any]
    path: Path


@dataclass(frozen=True)
class Bundle:
    documents: Mapping[str, Mapping[str, Any]]
    raw_bytes: Mapping[str, bytes]
    sha256: Mapping[str, str]
    staged_paths: Mapping[str, Path]
    published_paths: Mapping[str, Path]


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise MiddleBundleAssemblerError(status)


def _exact(value: Any, keys: Iterable[str], status: str) -> Mapping[str, Any]:
    expected = frozenset(keys)
    _require(type(value) is dict, f"{status}_TYPE")
    _require(frozenset(value) == expected, f"{status}_KEYS")
    return value


def _sha(value: Any, status: str) -> str:
    _require(type(value) is str and SHA256_RE.fullmatch(value) is not None,
             status)
    return value


def _finite(value: Any, status: str) -> float:
    _require(type(value) in (int, float) and type(value) is not bool, status)
    number = float(value)
    _require(math.isfinite(number), status)
    return number


def _integer(value: Any, status: str, minimum: int = 0) -> int:
    _require(type(value) is int and value >= minimum, status)
    return value


def _canonical_bytes(value: Any) -> bytes:
    try:
        return (json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
            allow_nan=False).encode("utf-8") + b"\n")
    except (TypeError, ValueError) as error:
        raise MiddleBundleAssemblerError(f"CANONICAL_JSON:{error}") from error


def _canonical_hash(value: Any) -> str:
    return hashlib.sha256(_canonical_bytes(value)[:-1]).hexdigest()


def _reject_duplicate_keys(
        pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    output: Dict[str, Any] = {}
    for key, value in pairs:
        if key in output:
            raise MiddleBundleAssemblerError(f"JSON_DUPLICATE_KEY:{key}")
        output[key] = value
    return output


def _reject_constant(token: str) -> None:
    raise MiddleBundleAssemblerError(f"JSON_NONFINITE:{token}")


def _identity(metadata: os.stat_result) -> Tuple[int, int, int, int, int]:
    return (
        metadata.st_dev, metadata.st_ino, metadata.st_size,
        metadata.st_mtime_ns, metadata.st_ctime_ns,
    )


def _read_exact_file(
    path: Path,
    expected_sha256: str,
    *,
    modes: Sequence[int],
    maximum_bytes: int,
    status: str,
    expected_uid: int | None = None,
) -> bytes:
    """Read one immutable regular file through one descriptor."""
    _sha(expected_sha256, f"{status}_EXPECTED_SHA256")
    path = Path(path)
    _require(path.is_absolute(), f"{status}_PATH_NOT_ABSOLUTE")
    try:
        metadata = path.lstat()
    except OSError as error:
        raise MiddleBundleAssemblerError(f"{status}_LSTAT:{error}") from error
    _require(stat.S_ISREG(metadata.st_mode), f"{status}_NOT_REGULAR")
    _require(not path.is_symlink(), f"{status}_SYMLINK")
    owner = os.geteuid() if expected_uid is None else expected_uid
    _require(metadata.st_uid == owner, f"{status}_OWNER")
    _require(stat.S_IMODE(metadata.st_mode) in tuple(modes),
             f"{status}_MODE")
    _require(metadata.st_nlink == 1, f"{status}_NLINK")
    _require(0 < metadata.st_size <= maximum_bytes, f"{status}_SIZE")
    flags = (os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) |
             getattr(os, "O_NOFOLLOW", 0))
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise MiddleBundleAssemblerError(f"{status}_OPEN:{error}") from error
    try:
        before = os.fstat(descriptor)
        _require(_identity(before) == _identity(metadata),
                 f"{status}_RACE_BEFORE")
        chunks = []
        digest = hashlib.sha256()
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            _require(bool(block), f"{status}_SHORT_READ")
            chunks.append(block)
            digest.update(block)
            remaining -= len(block)
        _require(not os.read(descriptor, 1), f"{status}_GREW")
        _require(_identity(before) == _identity(os.fstat(descriptor)),
                 f"{status}_RACE_AFTER")
    finally:
        os.close(descriptor)
    _require(hmac.compare_digest(digest.hexdigest(), expected_sha256),
             f"{status}_SHA256")
    return b"".join(chunks)


def _load_json(
    path: Path,
    expected_sha256: str,
    *,
    modes: Sequence[int] = (0o400,),
    status: str,
) -> SealedDocument:
    raw = _read_exact_file(
        path, expected_sha256, modes=modes, maximum_bytes=MAX_JSON_BYTES,
        status=status)
    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_constant)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise MiddleBundleAssemblerError(f"{status}_JSON:{error}") from error
    _require(type(value) is dict, f"{status}_JSON_ROOT")
    return SealedDocument(Path(path), expected_sha256, raw, value)


def _require_canonical_document(document: SealedDocument, status: str) -> None:
    _require(document.raw == _canonical_bytes(document.value),
             f"{status}_NOT_CANONICAL")


def _workspace(workspace: Path) -> Path:
    workspace = Path(workspace)
    _require(workspace.is_absolute(), "WORKSPACE_NOT_ABSOLUTE")
    try:
        metadata = workspace.lstat()
        resolved = workspace.resolve(strict=True)
    except OSError as error:
        raise MiddleBundleAssemblerError(f"WORKSPACE_LSTAT:{error}") from error
    _require(stat.S_ISDIR(metadata.st_mode) and not workspace.is_symlink(),
             "WORKSPACE_NOT_DIRECTORY")
    _require(resolved == workspace, "WORKSPACE_PATH_ALIAS")
    return workspace


def _workspace_path(
        workspace: Path, relative: Any, status: str,
        *, allow_final_symlink: bool = False) -> Path:
    _require(type(relative) is str and relative and
             not relative.startswith("/"), status)
    rel = Path(relative)
    _require(rel.as_posix() == relative and
             all(part not in ("", ".", "..") for part in rel.parts),
             f"{status}_ALIAS")
    candidate = workspace / rel
    try:
        resolved = candidate.resolve(strict=True)
        metadata = candidate.lstat()
    except OSError as error:
        raise MiddleBundleAssemblerError(f"{status}_RESOLVE:{error}") from error
    _require(resolved == workspace or workspace in resolved.parents,
             f"{status}_ESCAPE")
    if not allow_final_symlink:
        _require(resolved == candidate and not stat.S_ISLNK(metadata.st_mode),
                 f"{status}_SYMLINK_OR_ALIAS")
    return candidate


def _relative_to_workspace(workspace: Path, path: Path, status: str) -> str:
    try:
        relative = path.relative_to(workspace).as_posix()
    except ValueError as error:
        raise MiddleBundleAssemblerError(f"{status}_OUTSIDE_WORKSPACE") from error
    _require(relative and not relative.startswith("../"), status)
    return relative


def _exec_verified_module(
    module_name: str,
    package_name: str,
    source: Path,
    source_bytes: bytes,
) -> Any:
    try:
        code = compile(source_bytes, str(source), "exec", dont_inherit=True)
    except (SyntaxError, ValueError, TypeError) as error:
        raise MiddleBundleAssemblerError(
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
        raise MiddleBundleAssemblerError(
            f"VERIFIED_SOURCE_EXEC:{module_name}:{error}") from error
    return module


def _load_authorities(
    workspace: Path, provenance: Mapping[str, Any],
) -> Tuple[Any, Any, Any, Mapping[str, Mapping[str, Any]]]:
    authorities = _exact(
        provenance["authorities"], AUTHORITY_IDS, "PROVENANCE_AUTHORITIES")
    verified: Dict[str, Mapping[str, Any]] = {}
    source_bytes: Dict[str, bytes] = {}
    for identifier in AUTHORITY_IDS:
        binding = _exact(
            authorities[identifier], ("path", "sha256"),
            f"AUTHORITY:{identifier}")
        digest = _sha(binding["sha256"], f"AUTHORITY:{identifier}:SHA256")
        path = _workspace_path(
            workspace, binding["path"], f"AUTHORITY:{identifier}:PATH")
        raw = _read_exact_file(
            path, digest, modes=(0o500,), maximum_bytes=MAX_SOURCE_BYTES,
            status=f"AUTHORITY:{identifier}")
        source_bytes[identifier] = raw
        verified[identifier] = dict(binding)

    package_name = "_h1_middle_assembler_verified_vision"
    package = types.ModuleType(package_name)
    package.__package__ = package_name
    package.__path__ = []
    package.__loader__ = None
    package.__spec__ = None
    sys.modules[package_name] = package
    contract_name = package_name + ".candidate3_middle_execution_contract_828"
    contract = _exec_verified_module(
        contract_name, package_name,
        _workspace_path(workspace, verified["contract"]["path"],
                        "CONTRACT_SOURCE"),
        source_bytes["contract"])
    setattr(package, "candidate3_middle_execution_contract_828", contract)

    # Admission uses a relative import with the public source module name.
    admission_name = package_name + ".candidate3_middle_admission"
    admission = _exec_verified_module(
        admission_name, package_name,
        _workspace_path(workspace, verified["admission"]["path"],
                        "ADMISSION_SOURCE"),
        source_bytes["admission"])
    support = _exec_verified_module(
        "_h1_middle_assembler_verified_support", "",
        _workspace_path(workspace, verified["support"]["path"],
                        "SUPPORT_SOURCE"),
        source_bytes["support"])
    return contract, admission, support, verified


def _load_asset_wrapper(
    workspace: Path,
    manifest_directory: Path,
    asset_id: str,
    raw_binding: Any,
) -> Asset:
    binding = _exact(
        raw_binding, ("path", "file_sha256", "payload_sha256", "schema"),
        f"ASSET_BINDING:{asset_id}")
    _require(binding["schema"] == ASSET_SCHEMA,
             f"ASSET_SCHEMA:{asset_id}")
    file_sha = _sha(binding["file_sha256"], f"ASSET_FILE_SHA:{asset_id}")
    payload_sha = _sha(
        binding["payload_sha256"], f"ASSET_PAYLOAD_SHA:{asset_id}")
    raw_relative = binding["path"]
    _require(type(raw_relative) is str and raw_relative and
             not raw_relative.startswith("/"), f"ASSET_PATH:{asset_id}")
    candidate = manifest_directory / raw_relative
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise MiddleBundleAssemblerError(
            f"ASSET_PATH:{asset_id}:{error}") from error
    _require(resolved == candidate and workspace in resolved.parents,
             f"ASSET_PATH_ALIAS:{asset_id}")
    document = _load_json(
        resolved, file_sha, status=f"ASSET:{asset_id}")
    wrapper = _exact(document.value, ("payload", "sha256"),
                     f"ASSET_WRAPPER:{asset_id}")
    payload = _exact(
        wrapper["payload"],
        ("schema", "asset_id", "messages", "synchronization_groups"),
        f"ASSET_PAYLOAD:{asset_id}")
    _require(wrapper["sha256"] == payload_sha == _canonical_hash(payload),
             f"ASSET_PAYLOAD_HASH:{asset_id}")
    _require(payload["schema"] == ASSET_SCHEMA and
             payload["asset_id"] == asset_id,
             f"ASSET_PAYLOAD_IDENTITY:{asset_id}")
    _require(document.raw == _canonical_bytes(document.value),
             f"ASSET_WRAPPER_NOT_CANONICAL:{asset_id}")
    messages = payload["messages"]
    _require(type(messages) is list and
             len(messages) == len(ASSET_TOPICS[asset_id]),
             f"ASSET_MESSAGES:{asset_id}")
    for index, (message, expected_topic) in enumerate(zip(
            messages, ASSET_TOPICS[asset_id])):
        _validate_command(
            message, expected_topic, f"ASSET_MESSAGE:{asset_id}:{index}")
    _validate_synchronization_groups(
        messages, payload["synchronization_groups"], asset_id)
    plan_binding = {
        "path": _relative_to_workspace(workspace, resolved,
                                       f"ASSET_RELATIVE:{asset_id}"),
        "file_sha256": file_sha,
        "payload_sha256": payload_sha,
        "schema": ASSET_SCHEMA,
    }
    return Asset(asset_id, plan_binding, payload, resolved)


def _validate_command(value: Any, expected_topic: str, status: str) -> None:
    command = _exact(value, ("topic", "joint_names", "points"), status)
    _require(command["topic"] == expected_topic, f"{status}_TOPIC")
    expected_joints = CONTROLLER_JOINTS.get(expected_topic)
    _require(tuple(command["joint_names"]) == expected_joints,
             f"{status}_JOINTS")
    points = command["points"]
    _require(type(points) is list and points, f"{status}_POINTS")
    previous = -1
    for index, point_value in enumerate(points):
        point = _exact(
            point_value, ("positions", "time_from_start_ns"),
            f"{status}_POINT:{index}")
        positions = point["positions"]
        _require(type(positions) is list and
                 len(positions) == len(expected_joints),
                 f"{status}_POSITIONS:{index}")
        for position_index, position in enumerate(positions):
            _finite(position, f"{status}_POSITION:{index}:{position_index}")
        duration = point["time_from_start_ns"]
        _require(type(duration) is int and duration > previous,
                 f"{status}_TIME:{index}")
        previous = duration


def _validate_synchronization_groups(
    messages: Sequence[Mapping[str, Any]], value: Any, asset_id: str,
) -> None:
    _require(type(value) is list, f"ASSET_SYNC_GROUPS:{asset_id}")
    occupied = set()
    for group_index, group_value in enumerate(value):
        group = _exact(
            group_value, ("message_indices", "shared_time_from_start_ns"),
            f"ASSET_SYNC_GROUP:{asset_id}:{group_index}")
        indices = group["message_indices"]
        _require(type(indices) is list and len(indices) >= 2 and
                 all(type(index) is int and 0 <= index < len(messages)
                     for index in indices) and
                 len(indices) == len(set(indices)) and
                 not occupied.intersection(indices),
                 f"ASSET_SYNC_INDICES:{asset_id}:{group_index}")
        occupied.update(indices)
        grid = group["shared_time_from_start_ns"]
        _require(type(grid) is list and grid,
                 f"ASSET_SYNC_GRID:{asset_id}:{group_index}")
        for message_index in indices:
            actual = [point["time_from_start_ns"]
                      for point in messages[message_index]["points"]]
            _require(actual == grid,
                     f"ASSET_SYNC_GRID:{asset_id}:{group_index}")
    if asset_id != "cube":
        _require(len(value) == 1 and value[0]["message_indices"] == [0, 1],
                 f"ASSET_SYNC_TOPOLOGY:{asset_id}")
    else:
        _require(value == [], "ASSET_SYNC_TOPOLOGY:cube")


def _load_assets(
    workspace: Path, manifest: SealedDocument,
) -> Tuple[Mapping[str, Asset], Mapping[str, Any]]:
    root = _exact(
        manifest.value,
        ("schema", "controller_commands_issued", "offline_only", "assets",
         "phase_message_ranges"),
        "RAW_ASSET_MANIFEST")
    _require(root["schema"] == RAW_BINDINGS_SCHEMA,
             "RAW_ASSET_MANIFEST_SCHEMA")
    _require(root["controller_commands_issued"] == 0 and
             root["offline_only"] is True,
             "RAW_ASSET_MANIFEST_OFFLINE")
    bindings = _exact(root["assets"], ASSET_IDS, "RAW_ASSET_BINDINGS")
    ranges = _exact(root["phase_message_ranges"], PHASE_NAMES,
                    "RAW_PHASE_RANGES")
    expected_ranges = dict(zip(PHASE_NAMES, PHASE_RANGES))
    for phase_name, expected in expected_ranges.items():
        _require(ranges[phase_name] == list(expected),
                 f"RAW_PHASE_RANGE:{phase_name}")
    assets = {
        asset_id: _load_asset_wrapper(
            workspace, manifest.path.parent, asset_id, bindings[asset_id])
        for asset_id in ASSET_IDS
    }
    return assets, {key: dict(asset.binding) for key, asset in assets.items()}


def _initial_state(contract: Any) -> Dict[str, float]:
    state: Dict[str, float] = {
        "torso_joint": float(contract.E0_TORSO_RAD),
    }
    state.update(dict(zip(RIGHT_ARM_JOINTS, contract.E0_RIGHT_ARM_RAD)))
    state.update({key: float(value)
                  for key, value in contract.E0_RIGHT_FINGERS.items()})
    state.update(dict(zip(LEFT_ARM_JOINTS, contract.PARK_LEFT_ARM_RAD)))
    state.update({key: float(value)
                  for key, value in contract.OPEN_LEFT_FINGERS.items()})
    _require(frozenset(state) == frozenset(ALL_ROBOT_JOINTS),
             "INITIAL_STATE_JOINTS")
    return state


def _endpoint_value(state: Mapping[str, float]) -> Mapping[str, Any]:
    return {
        "schema": ENDPOINT_SCHEMA,
        "robot_positions": {name: state[name] for name in ALL_ROBOT_JOINTS},
    }


def _endpoint_hash(state: Mapping[str, float]) -> str:
    return _canonical_hash(_endpoint_value(state))


def _accumulate_phases(
    contract: Any, assets: Mapping[str, Asset],
) -> Tuple[list[Mapping[str, Any]], list[Mapping[str, float]], Mapping[str, Any]]:
    state = _initial_state(contract)
    phase_states: list[Mapping[str, float]] = []
    phases: list[Mapping[str, Any]] = []
    continuity_rows = []
    phase_durations = []
    for index, (name, asset_id, message_range, support_values) in enumerate(zip(
            PHASE_NAMES, PHASE_ASSET_IDS, PHASE_RANGES,
            SUPPORT_EXPECTATIONS)):
        asset = assets[asset_id]
        start, end = message_range
        messages = asset.payload["messages"]
        _require(0 <= start <= end <= len(messages),
                 f"PHASE_RANGE:{index}")
        commands = messages[start:end]
        entry_hash = _endpoint_hash(state)
        maximum_first_point_error = 0.0
        duration = 0
        for message_index, command in enumerate(commands, start=start):
            first = command["points"][0]["positions"]
            errors = [abs(float(value) - state[joint])
                      for joint, value in zip(command["joint_names"], first)]
            command_error = max(errors, default=0.0)
            maximum_first_point_error = max(
                maximum_first_point_error, command_error)
            _require(command_error <= ENDPOINT_TOLERANCE + 1e-12,
                     f"PHASE_FIRST_POINT_CONTINUITY:{index}:{message_index}")
            final = command["points"][-1]["positions"]
            for joint, value in zip(command["joint_names"], final):
                state[joint] = float(value)
            duration = max(duration,
                           int(command["points"][-1]["time_from_start_ns"]))
        exit_hash = _endpoint_hash(state)
        phase_states.append(dict(state))
        continuity_rows.append({
            "phase": name,
            "maximum_first_point_error": maximum_first_point_error,
            "limit": ENDPOINT_TOLERANCE,
        })
        phase_durations.append({"phase": name, "duration_ns": duration})
        support = dict(zip(
            ("door_pre", "door_post", "cube_pre", "cube_post"),
            support_values))
        phases.append({
            "name": name,
            "commanded": bool(commands),
            "asset": dict(asset.binding),
            "message_range": [start, end],
            "commands": commands,
            "support": support,
            "entry_state_sha256": entry_hash,
            "exit_state_sha256": exit_hash,
        })
    timing = {
        "phase_durations_ns": phase_durations,
        "sequential_total_duration_ns": sum(
            row["duration_ns"] for row in phase_durations),
        "first_point_continuity": continuity_rows,
        "first_point_continuity_limit": ENDPOINT_TOLERANCE,
    }
    return phases, phase_states, timing


def _hooks() -> list[Mapping[str, Any]]:
    result = []
    for index, (kind, phase_index, action, pre, post) in enumerate(
            HOOK_EXPECTATIONS):
        result.append({
            "id": f"middle-828-hook-{index}",
            "boundary": {
                "kind": kind,
                "phase_index": phase_index,
                "phase_name": PHASE_NAMES[phase_index],
            },
            "action": action,
            "pre_state": pre,
            "post_state": post,
            "require_verified": True,
        })
    return result


def _pose(value: Any, status: str) -> Mapping[str, Any]:
    pose = _exact(value, ("position", "orientation"), status)
    position = pose["position"]
    orientation = pose["orientation"]
    _require(type(position) is list and len(position) == 3,
             f"{status}_POSITION")
    _require(type(orientation) is list and len(orientation) == 4,
             f"{status}_ORIENTATION")
    positions = [_finite(item, f"{status}_POSITION") for item in position]
    orientations = [_finite(item, f"{status}_ORIENTATION")
                    for item in orientation]
    norm = math.sqrt(sum(item * item for item in orientations))
    _require(abs(norm - 1.0) <= 1e-6, f"{status}_ORIENTATION_NORM")
    return {"position": positions, "orientation": orientations}


def _verify_following_workspace_path(
    workspace: Path, relative: str, expected_sha256: str, status: str,
) -> Path:
    path = _workspace_path(
        workspace, relative, status, allow_final_symlink=True)
    try:
        resolved = path.resolve(strict=True)
        metadata = resolved.lstat()
    except OSError as error:
        raise MiddleBundleAssemblerError(f"{status}:{error}") from error
    _require(stat.S_ISREG(metadata.st_mode) and metadata.st_uid == os.geteuid(),
             f"{status}_REGULAR_OWNER")
    _read_exact_file(
        resolved, expected_sha256, modes=(0o500, 0o555, 0o755, 0o775),
        maximum_bytes=MAX_JSON_BYTES, status=status)
    return path


def _validate_robot_model(
    workspace: Path,
    provenance_model: Any,
    raw_model: Any,
) -> Mapping[str, Any]:
    model = _exact(
        provenance_model,
        ("inventory_path", "inventory_sha256", "urdf_path", "urdf_sha256",
         "srdf_path", "srdf_sha256", "mesh_count"),
        "PROVENANCE_ROBOT_MODEL")
    raw_value = _exact(
        raw_model, ("inventory_sha256", "urdf_sha256", "srdf_sha256",
                    "mesh_count"), "RAW_ROBOT_MODEL")
    for key in ("inventory_sha256", "urdf_sha256", "srdf_sha256"):
        _sha(model[key], f"PROVENANCE_ROBOT_MODEL:{key}")
        _require(model[key] == raw_value[key], f"ROBOT_MODEL_RAW_BINDING:{key}")
    mesh_count = _integer(model["mesh_count"], "ROBOT_MODEL_MESH_COUNT", 1)
    _require(raw_value["mesh_count"] == mesh_count,
             "ROBOT_MODEL_RAW_MESH_COUNT")
    inventory_path = _workspace_path(
        workspace, model["inventory_path"], "ROBOT_MODEL_INVENTORY_PATH")
    inventory = _load_json(
        inventory_path, model["inventory_sha256"],
        status="ROBOT_MODEL_INVENTORY")
    inventory_value = _exact(
        inventory.value,
        ("schema", "source_urdf_sha256", "source_srdf_sha256",
         "derived_urdf_path", "derived_urdf_sha256", "srdf_path",
         "srdf_sha256", "mesh_files"),
        "ROBOT_MODEL_INVENTORY_ROOT")
    _require(inventory_value["schema"] ==
             "h1_candidate3_middle_robot_model_snapshot/828-e0-v1",
             "ROBOT_MODEL_INVENTORY_SCHEMA")
    _require(inventory_value["derived_urdf_path"] == model["urdf_path"] and
             inventory_value["derived_urdf_sha256"] == model["urdf_sha256"] and
             inventory_value["srdf_path"] == model["srdf_path"] and
             inventory_value["srdf_sha256"] == model["srdf_sha256"],
             "ROBOT_MODEL_INVENTORY_BINDING")
    for label, path_key, sha_key in (
        ("ROBOT_MODEL_URDF", "urdf_path", "urdf_sha256"),
        ("ROBOT_MODEL_SRDF", "srdf_path", "srdf_sha256"),
    ):
        path = _workspace_path(workspace, model[path_key], label + "_PATH")
        _read_exact_file(path, model[sha_key], modes=(0o400,),
                         maximum_bytes=MAX_JSON_BYTES, status=label)
    meshes = inventory_value["mesh_files"]
    _require(type(meshes) is list and len(meshes) == mesh_count,
             "ROBOT_MODEL_MESH_LIST")
    seen_paths = set()
    for index, mesh_value in enumerate(meshes):
        mesh = _exact(
            mesh_value,
            ("source_uri", "snapshot_path", "resolved_file_uri",
             "file_sha256", "size_bytes", "roles"),
            f"ROBOT_MODEL_MESH:{index}")
        mesh_path = _workspace_path(
            workspace, mesh["snapshot_path"], f"ROBOT_MODEL_MESH_PATH:{index}")
        _require(mesh["snapshot_path"] not in seen_paths,
                 f"ROBOT_MODEL_MESH_DUPLICATE:{index}")
        seen_paths.add(mesh["snapshot_path"])
        _require(mesh["resolved_file_uri"] == "file://" + str(mesh_path),
                 f"ROBOT_MODEL_MESH_URI:{index}")
        raw = _read_exact_file(
            mesh_path, _sha(mesh["file_sha256"],
                            f"ROBOT_MODEL_MESH_SHA:{index}"),
            modes=(0o400,), maximum_bytes=MAX_JSON_BYTES,
            status=f"ROBOT_MODEL_MESH:{index}")
        _require(mesh["size_bytes"] == len(raw),
                 f"ROBOT_MODEL_MESH_SIZE:{index}")
        _require(type(mesh["roles"]) is list and mesh["roles"],
                 f"ROBOT_MODEL_MESH_ROLES:{index}")
    return dict(model)


def _validate_auditor_run(
    workspace: Path,
    binding_value: Any,
    auditor: Mapping[str, Any],
    robot_model: Mapping[str, Any],
    cube_reference: Mapping[str, Any],
    scene: Mapping[str, Any],
    raw_document: SealedDocument,
    manifest: SealedDocument,
) -> Mapping[str, str]:
    """Bind the frozen raw JSON to the sealed process run that produced it."""
    binding = _exact(
        binding_value,
        ("plan_path", "plan_sha256", "result_path", "result_sha256"),
        "AUDITOR_RUN_BINDING")
    plan_path = _workspace_path(
        workspace, binding["plan_path"], "AUDITOR_RUN_PLAN_PATH")
    result_path = _workspace_path(
        workspace, binding["result_path"], "AUDITOR_RUN_RESULT_PATH")
    _require(plan_path != result_path, "AUDITOR_RUN_DOCUMENT_ALIAS")
    _require(plan_path.name == "auditor_run_plan_828.json" and
             result_path.name == "auditor_run_result_828.json" and
             plan_path.parent == result_path.parent,
             "AUDITOR_RUN_DOCUMENT_LAYOUT")
    plan_document = _load_json(
        plan_path, _sha(binding["plan_sha256"], "AUDITOR_RUN_PLAN_SHA256"),
        status="AUDITOR_RUN_PLAN")
    result_document = _load_json(
        result_path,
        _sha(binding["result_sha256"], "AUDITOR_RUN_RESULT_SHA256"),
        status="AUDITOR_RUN_RESULT")
    _require_canonical_document(plan_document, "AUDITOR_RUN_PLAN")
    _require_canonical_document(result_document, "AUDITOR_RUN_RESULT")

    plan = _exact(
        plan_document.value,
        ("schema", "working_directory", "offline_only",
         "controller_commands_issued", "auditor", "wrapper", "inputs",
         "outputs", "arguments", "expected_raw"),
        "AUDITOR_RUN_PLAN")
    _require(plan["schema"] == AUDITOR_RUN_PLAN_SCHEMA,
             "AUDITOR_RUN_PLAN_SCHEMA")
    _require(plan["working_directory"] == str(workspace),
             "AUDITOR_RUN_WORKING_DIRECTORY")
    _require(plan["offline_only"] is True and
             type(plan["controller_commands_issued"]) is int and
             plan["controller_commands_issued"] == 0,
             "AUDITOR_RUN_PLAN_OFFLINE")
    plan_auditor = _exact(
        plan["auditor"],
        ("source_path", "source_sha256", "binary_path", "binary_sha256"),
        "AUDITOR_RUN_PLAN_AUDITOR")
    _require(plan_auditor == auditor, "AUDITOR_RUN_AUDITOR_BINDING")

    wrapper = _exact(
        plan["wrapper"], ("path", "sha256", "time_format"),
        "AUDITOR_RUN_WRAPPER")
    _require(wrapper["path"] == str(TIME_WRAPPER_PATH) and
             wrapper["sha256"] == TIME_WRAPPER_SHA256 and
             wrapper["time_format"] == TIME_WRAPPER_FORMAT,
             "AUDITOR_RUN_WRAPPER_BINDING")
    _read_exact_file(
        TIME_WRAPPER_PATH, TIME_WRAPPER_SHA256, modes=(0o755,),
        maximum_bytes=MAX_SOURCE_BYTES, status="AUDITOR_RUN_WRAPPER",
        expected_uid=0)

    inputs = _exact(
        plan["inputs"], AUDITOR_RUN_INPUT_IDS, "AUDITOR_RUN_INPUTS")
    input_bindings: Dict[str, Mapping[str, str]] = {}
    input_paths: Dict[str, Path] = {}
    for identifier in AUDITOR_RUN_INPUT_IDS:
        value = _exact(
            inputs[identifier], ("path", "sha256"),
            f"AUDITOR_RUN_INPUT:{identifier}")
        path = _workspace_path(
            workspace, value["path"], f"AUDITOR_RUN_INPUT_PATH:{identifier}")
        digest = _sha(
            value["sha256"], f"AUDITOR_RUN_INPUT_SHA:{identifier}")
        _read_exact_file(
            path, digest, modes=(0o400,), maximum_bytes=MAX_JSON_BYTES,
            status=f"AUDITOR_RUN_INPUT:{identifier}")
        input_bindings[identifier] = dict(value)
        input_paths[identifier] = path
    _require(len(set(input_paths.values())) == len(AUDITOR_RUN_INPUT_IDS),
             "AUDITOR_RUN_INPUT_PATH_ALIAS")

    expected_inputs: Dict[str, Mapping[str, str]] = {
        "asset_manifest": {
            "path": _relative_to_workspace(
                workspace, manifest.path, "AUDITOR_RUN_MANIFEST_PATH"),
            "sha256": manifest.sha256,
        },
        "cube_reference": {
            "path": cube_reference["path"],
            "sha256": cube_reference["file_sha256"],
        },
        "robot_model_inventory": {
            "path": robot_model["inventory_path"],
            "sha256": robot_model["inventory_sha256"],
        },
        "scene": {
            "path": scene["path"],
            "sha256": raw_document.value["raw_scene_cdr_sha256"],
        },
        "srdf": {
            "path": robot_model["srdf_path"],
            "sha256": robot_model["srdf_sha256"],
        },
        "urdf": {
            "path": robot_model["urdf_path"],
            "sha256": robot_model["urdf_sha256"],
        },
    }
    run_asset_ids = {
        "cube": "cube", "door_close": "door_close",
        "door_open": "door_open",
        "seam_to_cube": "door60_seam_to_cube",
        "seam_to_hinge": "door60_seam_to_hinge",
    }
    manifest_assets = manifest.value["assets"]
    for run_id, asset_id in run_asset_ids.items():
        asset_binding = manifest_assets[asset_id]
        asset_path = (manifest.path.parent / asset_binding["path"])
        expected_inputs[run_id] = {
            "path": _relative_to_workspace(
                workspace, asset_path, f"AUDITOR_RUN_ASSET_PATH:{run_id}"),
            "sha256": asset_binding["file_sha256"],
        }
    expected_inputs["handle_stl"] = {
        "path": input_bindings["handle_stl"]["path"],
        "sha256": raw_document.value["dynamic_handle_exact"]
        ["geometry_sha256"],
    }
    _require(input_bindings == expected_inputs,
             "AUDITOR_RUN_INPUT_BINDING")

    outputs = _exact(
        plan["outputs"], AUDITOR_RUN_OUTPUT_IDS, "AUDITOR_RUN_OUTPUTS")
    output_paths = {
        identifier: _workspace_path(
            workspace, outputs[identifier],
            f"AUDITOR_RUN_OUTPUT_PATH:{identifier}")
        for identifier in AUDITOR_RUN_OUTPUT_IDS
    }
    _require(len(set(output_paths.values())) == len(AUDITOR_RUN_OUTPUT_IDS),
             "AUDITOR_RUN_OUTPUT_PATH_ALIAS")
    expected_output_names = {
        "raw": "raw_combined_exact.json",
        "stderr": "raw_combined_exact.stderr",
        "stdout": "raw_combined_exact.stdout",
        "time": "raw_combined_exact.time",
    }
    _require(all(path.parent == plan_path.parent and
                 path.name == expected_output_names[identifier]
                 for identifier, path in output_paths.items()),
             "AUDITOR_RUN_OUTPUT_LAYOUT")
    raw_relative = _relative_to_workspace(
        workspace, raw_document.path, "AUDITOR_RUN_RAW_CLI_PATH")
    _require(outputs["raw"] == raw_relative,
             "AUDITOR_RUN_RAW_CLI_BINDING")

    expected_raw = _exact(
        plan["expected_raw"],
        ("schema", "auditor_version", "file_sha256", "status"),
        "AUDITOR_RUN_EXPECTED_RAW")
    _require(expected_raw == {
        "schema": RAW_SCHEMA,
        "auditor_version": RAW_AUDITOR_VERSION,
        "file_sha256": raw_document.sha256,
        "status": RAW_STATUS,
    }, "AUDITOR_RUN_EXPECTED_RAW_BINDING")

    arguments = plan["arguments"]
    _require(type(arguments) is list and len(arguments) == 28 and
             all(type(item) is str and item for item in arguments),
             "AUDITOR_RUN_ARGUMENTS")
    expected_argument_values = (
        input_bindings["scene"]["path"], outputs["raw"],
        input_bindings["asset_manifest"]["path"], "0.316",
        input_bindings["urdf"]["path"], input_bindings["srdf"]["path"],
        input_bindings["door_open"]["path"],
        input_bindings["seam_to_cube"]["path"],
        input_bindings["cube"]["path"],
        input_bindings["cube_reference"]["path"],
        input_bindings["seam_to_hinge"]["path"],
        input_bindings["door_close"]["path"],
        input_bindings["handle_stl"]["path"],
        input_bindings["robot_model_inventory"]["path"],
    )
    expected_arguments = [
        item for pair in zip(
            AUDITOR_RUN_ARGUMENT_FLAGS, expected_argument_values)
        for item in pair
    ]
    _require(arguments == expected_arguments,
             "AUDITOR_RUN_ARGUMENT_BINDING")

    result = _exact(
        result_document.value,
        ("schema", "offline_only", "controller_commands_issued", "plan",
         "execution", "outputs", "raw_validation", "timing", "valid",
         "status"),
        "AUDITOR_RUN_RESULT")
    _require(result["schema"] == AUDITOR_RUN_RESULT_SCHEMA,
             "AUDITOR_RUN_RESULT_SCHEMA")
    _require(result["offline_only"] is True and
             type(result["controller_commands_issued"]) is int and
             result["controller_commands_issued"] == 0,
             "AUDITOR_RUN_RESULT_OFFLINE")
    result_plan = _exact(
        result["plan"], ("path", "sha256"), "AUDITOR_RUN_RESULT_PLAN")
    _require(result_plan == {
        "path": binding["plan_path"], "sha256": binding["plan_sha256"]},
        "AUDITOR_RUN_RESULT_PLAN_BINDING")

    execution = _exact(
        result["execution"],
        ("argc", "executable_dev", "executable_file_sha256",
         "executable_inode", "executable_mode", "executable_nlink",
         "exit_code", "proc_cmdline_file_sha256",
         "proc_cmdline_normalized_sha256", "proc_cmdline_path"),
        "AUDITOR_RUN_EXECUTION")
    _require(_integer(execution["argc"], "AUDITOR_RUN_ARGC", 1) ==
             len(arguments) + 1 and
             _integer(execution["exit_code"], "AUDITOR_RUN_EXIT_CODE") == 0 and
             execution["executable_file_sha256"] ==
             auditor["binary_sha256"] and
             execution["executable_mode"] == "0500" and
             _integer(execution["executable_nlink"],
                      "AUDITOR_RUN_EXECUTABLE_NLINK", 1) == 1,
             "AUDITOR_RUN_EXECUTION_BINDING")
    _integer(execution["executable_dev"], "AUDITOR_RUN_EXECUTABLE_DEV", 1)
    _integer(
        execution["executable_inode"], "AUDITOR_RUN_EXECUTABLE_INODE", 1)
    proc_path = _workspace_path(
        workspace, execution["proc_cmdline_path"],
        "AUDITOR_RUN_PROC_CMDLINE_PATH")
    _require(proc_path == plan_path.parent / "proc_cmdline.bin",
             "AUDITOR_RUN_PROC_CMDLINE_LAYOUT")
    proc_raw = _read_exact_file(
        proc_path,
        _sha(execution["proc_cmdline_file_sha256"],
             "AUDITOR_RUN_PROC_CMDLINE_SHA"),
        modes=(0o400,), maximum_bytes=MAX_SOURCE_BYTES,
        status="AUDITOR_RUN_PROC_CMDLINE")
    _require(proc_raw.endswith(b"\0") and b"\0\0" not in proc_raw,
             "AUDITOR_RUN_PROC_CMDLINE_FRAMING")
    try:
        argv = [part.decode("utf-8") for part in proc_raw[:-1].split(b"\0")]
    except UnicodeDecodeError as error:
        raise MiddleBundleAssemblerError(
            f"AUDITOR_RUN_PROC_CMDLINE_UTF8:{error}") from error
    _require(argv == [auditor["binary_path"], *arguments],
             "AUDITOR_RUN_PROC_CMDLINE_ARGV")
    normalized = ("\n".join(argv) + "\n").encode("utf-8")
    _require(hashlib.sha256(normalized).hexdigest() ==
             _sha(execution["proc_cmdline_normalized_sha256"],
                  "AUDITOR_RUN_PROC_NORMALIZED_SHA"),
             "AUDITOR_RUN_PROC_NORMALIZED_BINDING")

    result_outputs = _exact(
        result["outputs"], AUDITOR_RUN_OUTPUT_IDS,
        "AUDITOR_RUN_RESULT_OUTPUTS")
    output_raw: Dict[str, bytes] = {}
    for identifier in AUDITOR_RUN_OUTPUT_IDS:
        output = _exact(
            result_outputs[identifier], ("path", "sha256", "size_bytes"),
            f"AUDITOR_RUN_RESULT_OUTPUT:{identifier}")
        _require(output["path"] == outputs[identifier],
                 f"AUDITOR_RUN_RESULT_OUTPUT_PATH:{identifier}")
        size = _integer(
            output["size_bytes"],
            f"AUDITOR_RUN_RESULT_OUTPUT_SIZE:{identifier}", 1)
        maximum = MAX_JSON_BYTES if identifier == "raw" else MAX_SOURCE_BYTES
        raw_bytes = _read_exact_file(
            output_paths[identifier],
            _sha(output["sha256"],
                 f"AUDITOR_RUN_RESULT_OUTPUT_SHA:{identifier}"),
            modes=(0o400,), maximum_bytes=maximum,
            status=f"AUDITOR_RUN_RESULT_OUTPUT:{identifier}")
        _require(len(raw_bytes) == size,
                 f"AUDITOR_RUN_RESULT_OUTPUT_SIZE:{identifier}")
        output_raw[identifier] = raw_bytes
    _require(output_raw["raw"] == raw_document.raw and
             result_outputs["raw"]["sha256"] == raw_document.sha256,
             "AUDITOR_RUN_RAW_DOCUMENT_BINDING")

    try:
        stdout = output_raw["stdout"].decode("utf-8")
        stderr = output_raw["stderr"].decode("utf-8")
        time_text = output_raw["time"].decode("ascii")
    except UnicodeDecodeError as error:
        raise MiddleBundleAssemblerError(
            f"AUDITOR_RUN_OUTPUT_ENCODING:{error}") from error
    stdout_match = re.fullmatch(
        r"CANDIDATE3_MIDDLE_EXACT_AUDIT=PASS samples=([0-9]+) "
        r"minimum_environment_clearance_m=([-+0-9.eE]+) "
        r"minimum_left_handle_clearance_m=([-+0-9.eE]+)\n",
        stdout)
    _require(stdout_match is not None, "AUDITOR_RUN_STDOUT_PASS")
    _require(int(stdout_match.group(1)) == raw_document.value["sample_count"] and
             _finite(float(stdout_match.group(2)),
                     "AUDITOR_RUN_STDOUT_ENVIRONMENT") ==
             raw_document.value["minimum_clearances_m"][
                 "unauthorized_environment"] and
             _finite(float(stdout_match.group(3)),
                     "AUDITOR_RUN_STDOUT_LEFT_HANDLE") ==
             raw_document.value["left_handle"]["minimum_clearance_m"],
             "AUDITOR_RUN_STDOUT_BINDING")
    upper_stderr = stderr.upper()
    _require(all(token not in upper_stderr
                 for token in ("ERROR", "FATAL", "EXCEPTION")),
             "AUDITOR_RUN_STDERR_FAILURE_TOKEN")

    time_match = re.fullmatch(
        r"elapsed_seconds=([-+0-9.eE]+)\n"
        r"user_seconds=([-+0-9.eE]+)\n"
        r"system_seconds=([-+0-9.eE]+)\n"
        r"max_rss_kb=([0-9]+)\n", time_text)
    _require(time_match is not None, "AUDITOR_RUN_TIME_FORMAT")
    parsed_timing = {
        "elapsed_seconds": _finite(
            float(time_match.group(1)), "AUDITOR_RUN_TIME_ELAPSED"),
        "user_seconds": _finite(
            float(time_match.group(2)), "AUDITOR_RUN_TIME_USER"),
        "system_seconds": _finite(
            float(time_match.group(3)), "AUDITOR_RUN_TIME_SYSTEM"),
        "maximum_resident_kb": int(time_match.group(4)),
    }
    _require(all(parsed_timing[key] >= 0.0 for key in (
        "elapsed_seconds", "user_seconds", "system_seconds")) and
        parsed_timing["maximum_resident_kb"] > 0,
        "AUDITOR_RUN_TIME_VALUES")
    timing = _exact(
        result["timing"],
        ("elapsed_seconds", "user_seconds", "system_seconds",
         "maximum_resident_kb"),
        "AUDITOR_RUN_RESULT_TIMING")
    _require(timing == parsed_timing, "AUDITOR_RUN_TIME_BINDING")

    validation = _exact(
        result["raw_validation"],
        ("schema", "auditor_version", "file_sha256",
         "byte_equal_to_original_reviewed_raw", "strict_json_finite",
         "sample_count", "subdivision_count", "collision_free",
         "geometry_valid", "valid", "status"),
        "AUDITOR_RUN_RAW_VALIDATION")
    _require(validation == {
        "schema": raw_document.value["schema"],
        "auditor_version": raw_document.value["auditor_version"],
        "file_sha256": raw_document.sha256,
        "byte_equal_to_original_reviewed_raw": True,
        "strict_json_finite": True,
        "sample_count": raw_document.value["sample_count"],
        "subdivision_count": raw_document.value["subdivision_count"],
        "collision_free": raw_document.value["collision_free"],
        "geometry_valid": raw_document.value["geometry_valid"],
        "valid": raw_document.value["valid"],
        "status": raw_document.value["status"],
    }, "AUDITOR_RUN_RAW_VALIDATION_BINDING")
    for key in (
        "byte_equal_to_original_reviewed_raw", "strict_json_finite",
        "collision_free", "geometry_valid", "valid",
    ):
        _require(type(validation[key]) is bool,
                 f"AUDITOR_RUN_RAW_VALIDATION_TYPE:{key}")
    _integer(validation["sample_count"],
             "AUDITOR_RUN_RAW_VALIDATION_SAMPLE_COUNT", 1)
    _integer(validation["subdivision_count"],
             "AUDITOR_RUN_RAW_VALIDATION_SUBDIVISION_COUNT", 1)
    _require(result["valid"] is True and result["status"] == "PASS",
             "AUDITOR_RUN_RESULT_STATUS")

    plan_mtime = plan_path.lstat().st_mtime_ns
    result_mtime = result_path.lstat().st_mtime_ns
    proc_mtime = proc_path.lstat().st_mtime_ns
    output_mtimes = {
        key: path.lstat().st_mtime_ns for key, path in output_paths.items()}
    _require(plan_mtime <= proc_mtime <= result_mtime,
             "AUDITOR_RUN_MTIME_PLAN_PROC_RESULT")
    _require(all(plan_mtime <= value <= result_mtime
                 for value in output_mtimes.values()),
             "AUDITOR_RUN_MTIME_PLAN_OUTPUT_RESULT")
    # stderr is opened before the process snapshot and may receive only early
    # loader warnings.  Completion-bearing raw/stdout/time must postdate it.
    _require(all(proc_mtime <= output_mtimes[key]
                 for key in ("raw", "stdout", "time")),
             "AUDITOR_RUN_MTIME_PROC_COMPLETION_OUTPUT")
    return dict(binding)


def _validate_provenance(
    workspace: Path,
    document: SealedDocument,
    raw_document: SealedDocument,
    manifest: SealedDocument,
) -> Tuple[Mapping[str, Any], Any, Any, Any, Mapping[str, Mapping[str, Any]]]:
    raw = raw_document.value
    root = _exact(
        document.value,
        ("schema", "baseline_backup", "scene", "robot_model",
         "cube_reference", "auditor", "auditor_run", "authorities",
         "support", "cube_poses"),
        "PROVENANCE")
    _require(root["schema"] == PROVENANCE_SCHEMA, "PROVENANCE_SCHEMA")
    baseline = _exact(root["baseline_backup"], ("path", "sha256"),
                      "PROVENANCE_BASELINE")
    _require(baseline["path"] == str(BASELINE_BACKUP_PATH) and
             baseline["sha256"] == BASELINE_BACKUP_SHA256,
             "PROVENANCE_BASELINE_BINDING")
    _read_exact_file(
        BASELINE_BACKUP_PATH, BASELINE_BACKUP_SHA256, modes=(0o444,),
        maximum_bytes=MAX_ARCHIVE_BYTES, status="BASELINE_BACKUP")

    scene = _exact(
        root["scene"],
        ("path", "scene_name", "planning_frame", "required_object_ids"),
        "PROVENANCE_SCENE")
    scene_path = _workspace_path(workspace, scene["path"], "SCENE_PATH")
    _read_exact_file(
        scene_path, _sha(raw["raw_scene_cdr_sha256"], "RAW_SCENE_CDR_SHA"),
        modes=(0o400,), maximum_bytes=MAX_JSON_BYTES, status="RAW_SCENE")
    _require(type(scene["scene_name"]) is str and scene["scene_name"] and
             scene["planning_frame"] == "world",
             "PROVENANCE_SCENE_IDENTITY")
    required_ids = scene["required_object_ids"]
    _require(type(required_ids) is list and required_ids and
             all(type(item) is str and item for item in required_ids) and
             len(required_ids) == len(set(required_ids)),
             "PROVENANCE_SCENE_OBJECTS")

    model = _validate_robot_model(
        workspace, root["robot_model"], raw["robot_model_snapshot"])

    cube_reference = _exact(
        root["cube_reference"], ("path", "file_sha256", "payload_sha256"),
        "PROVENANCE_CUBE_REFERENCE")
    reference_path = _workspace_path(
        workspace, cube_reference["path"], "CUBE_REFERENCE_PATH")
    reference = _load_json(
        reference_path,
        _sha(cube_reference["file_sha256"], "CUBE_REFERENCE_FILE_SHA"),
        status="CUBE_REFERENCE")
    wrapper = _exact(reference.value, ("payload", "sha256"),
                     "CUBE_REFERENCE_WRAPPER")
    _require(wrapper["sha256"] ==
             _sha(cube_reference["payload_sha256"],
                  "CUBE_REFERENCE_PAYLOAD_SHA") ==
             _canonical_hash(wrapper["payload"]),
             "CUBE_REFERENCE_PAYLOAD_BINDING")
    bump = _exact(
        raw["cube_bump"],
        ("reference_file_sha256", "reference_payload_sha256",
         "only_modified_joint", "comparison", "maximum_tcp_translation_m",
         "maximum_tcp_rotation_rad", "translation_limit_m",
         "rotation_limit_rad"),
        "RAW_CUBE_BUMP")
    _require(bump["reference_file_sha256"] == cube_reference["file_sha256"] and
             bump["reference_payload_sha256"] ==
             cube_reference["payload_sha256"],
             "RAW_CUBE_REFERENCE_BINDING")

    auditor = _exact(
        root["auditor"],
        ("source_path", "source_sha256", "binary_path", "binary_sha256"),
        "PROVENANCE_AUDITOR")
    source_path = _workspace_path(
        workspace, auditor["source_path"], "AUDITOR_SOURCE_PATH")
    _require(auditor["source_sha256"] == RAW_AUDITOR_SOURCE_SHA256,
             "AUDITOR_FROZEN_SOURCE_SHA")
    _read_exact_file(
        source_path, _sha(auditor["source_sha256"], "AUDITOR_SOURCE_SHA"),
        modes=(0o400,), maximum_bytes=MAX_SOURCE_BYTES,
        status="AUDITOR_SOURCE")
    binary_path = _workspace_path(
        workspace, auditor["binary_path"], "AUDITOR_BINARY_PATH")
    _read_exact_file(
        binary_path, _sha(auditor["binary_sha256"], "AUDITOR_BINARY_SHA"),
        modes=(0o500,), maximum_bytes=MAX_JSON_BYTES,
        status="AUDITOR_BINARY")

    support_binding = _exact(
        root["support"],
        ("plugin_path", "plugin_sha256", "ros_snapshot_path",
         "ros_snapshot_sha256"),
        "PROVENANCE_SUPPORT")
    _require(support_binding["plugin_path"] == PLUGIN_RELATIVE_PATH,
             "PROVENANCE_PLUGIN_PATH")
    plugin_sha = _sha(support_binding["plugin_sha256"],
                      "PROVENANCE_PLUGIN_SHA")
    _verify_following_workspace_path(
        workspace, support_binding["plugin_path"], plugin_sha,
        "SUPPORT_PLUGIN")
    snapshot_path = _workspace_path(
        workspace, support_binding["ros_snapshot_path"],
        "SUPPORT_SNAPSHOT_PATH")
    _read_exact_file(
        snapshot_path,
        _sha(support_binding["ros_snapshot_sha256"],
             "SUPPORT_SNAPSHOT_SHA"),
        modes=(0o500,), maximum_bytes=MAX_SOURCE_BYTES,
        status="SUPPORT_SNAPSHOT")

    _validate_auditor_run(
        workspace, root["auditor_run"], auditor, model, cube_reference,
        scene, raw_document, manifest)

    cube_poses = _exact(
        root["cube_poses"], ("contact", "release"),
        "PROVENANCE_CUBE_POSES")
    contact_pose = _pose(cube_poses["contact"], "CUBE_CONTACT_POSE")
    release_pose = _pose(cube_poses["release"], "CUBE_RELEASE_POSE")
    expected_contact = [0.334, 0.377, 0.830]
    expected_release = [0.611, -0.030, 0.995]
    _require(all(abs(actual - expected) <= 1e-12
                 for actual, expected in zip(
                     contact_pose["position"], expected_contact)),
             "CUBE_CONTACT_POSE_FIXED")
    _require(all(abs(actual - expected) <= 1e-12
                 for actual, expected in zip(
                     release_pose["position"], expected_release)),
             "CUBE_RELEASE_POSE_FIXED")
    expected_orientation = [0.0, 0.0, math.sin(math.pi / 4.0),
                            math.cos(math.pi / 4.0)]
    for label, pose in (("CONTACT", contact_pose), ("RELEASE", release_pose)):
        _require(all(abs(actual - expected) <= 1e-12
                     for actual, expected in zip(
                         pose["orientation"], expected_orientation)),
                 f"CUBE_{label}_POSE_FIXED_ORIENTATION")

    contract, admission, support, authorities = _load_authorities(
        workspace, root)
    return root, contract, admission, support, authorities


def _raw_retreat_pairs(
    value: Any,
    *,
    object_id: str,
    status: str,
) -> list[list[str]]:
    _require(type(value) is list and value, f"{status}_EMPTY")
    output = []
    normalized = set()
    for index, pair in enumerate(value):
        _require(type(pair) is list and len(pair) == 2 and
                 all(type(item) is str and item for item in pair),
                 f"{status}:{index}")
        _require(pair[0] == object_id and
                 pair[1] in RAW_RETREAT_ALLOWED_LINKS,
                 f"{status}_AUTH:{index}")
        normalized_pair = tuple(pair)
        _require(normalized_pair not in normalized,
                 f"{status}_DUPLICATE:{index}")
        normalized.add(normalized_pair)
        output.append(list(pair))
    return output


def _raw_retreat_links(
    value: Any,
    pairs: Sequence[Sequence[str]],
    *,
    status: str,
) -> None:
    _require(type(value) is list and value and
             all(type(link) is str and link for link in value) and
             len(value) == len(frozenset(value)), status)
    _require(frozenset(value) == frozenset(pair[1] for pair in pairs),
             f"{status}_PAIR_BINDING")


def _validate_raw_release_retreat(raw: Mapping[str, Any]) -> None:
    retreat_value = raw["cube_release_retreat"]
    if type(retreat_value) is dict:
        for key in RAW_RETREAT_KEYS:
            _require(key in retreat_value, f"RAW_RETREAT_REQUIRED:{key}")
    retreat = _exact(retreat_value, RAW_RETREAT_KEYS, "RAW_RETREAT")
    _require(retreat["phase_name"] == "cube_retreat", "RAW_RETREAT_PHASE")
    _require(_integer(
        retreat["message_index"], "RAW_RETREAT_MESSAGE_INDEX") == 6,
        "RAW_RETREAT_MESSAGE_INDEX")
    _require(_integer(
        retreat["start_point_index"], "RAW_RETREAT_START_POINT") == 0 and
        _integer(retreat["start_subdivision"],
                 "RAW_RETREAT_START_SUBDIVISION") == 0,
        "RAW_RETREAT_START")

    window_samples = _integer(
        retreat["window_sample_count"], "RAW_RETREAT_WINDOW_SAMPLES", 1)
    gate_point = _integer(
        retreat["gate_point_index"], "RAW_RETREAT_GATE_POINT")
    gate_subdivision = _integer(
        retreat["gate_subdivision"], "RAW_RETREAT_GATE_SUBDIVISION", 1)
    gate_ordinal = _integer(
        retreat["gate_ordinal"], "RAW_RETREAT_GATE_ORDINAL", 1)
    last_below = _integer(
        retreat["last_below_gate_ordinal"],
        "RAW_RETREAT_LAST_BELOW_GATE_ORDINAL")
    _require(gate_point >= retreat["start_point_index"] and
             gate_subdivision > retreat["start_subdivision"],
             "RAW_RETREAT_GATE_COORDINATE")
    _require(retreat["gate_reached"] is True and
             retreat["gate_never_regressed"] is True,
             "RAW_RETREAT_GATE_FLAGS")
    _require(gate_ordinal == last_below + 1,
             "RAW_RETREAT_SHORTEST_PREFIX")
    _require(window_samples == gate_ordinal,
             "RAW_RETREAT_WINDOW_GATE_BINDING")

    minimum_exact = _finite(
        retreat["minimum_exact_distance_m"], "RAW_RETREAT_MINIMUM_EXACT")
    maximum_exact = _finite(
        retreat["maximum_exact_distance_m"], "RAW_RETREAT_MAXIMUM_EXACT")
    final_exact = _finite(
        retreat["final_exact_distance_m"], "RAW_RETREAT_FINAL_EXACT")
    _require(0.0 <= minimum_exact <= final_exact <= maximum_exact,
             "RAW_RETREAT_DISTANCE_ORDER")
    penetration = _finite(
        retreat["maximum_exact_penetration_m"],
        "RAW_RETREAT_EXACT_PENETRATION")
    _require(penetration == 0.0, "RAW_RETREAT_EXACT_PENETRATION")
    _integer(retreat["minimum_time_step_ns"],
             "RAW_RETREAT_TIME_STEP", 1)
    path_step = _finite(
        retreat["minimum_path_parameter_step"], "RAW_RETREAT_PATH_STEP")
    _require(0.0 < path_step <= 1.0, "RAW_RETREAT_PATH_STEP")
    _finite(
        retreat["minimum_tcp_separation_projection_step_m"],
        "RAW_RETREAT_TCP_PROJECTION_STEP")
    _require(_finite(
        retreat["final_tcp_projection_m"],
        "RAW_RETREAT_FINAL_TCP_PROJECTION") > 0.0,
        "RAW_RETREAT_FINAL_TCP_PROJECTION")
    post_gate_exact = _finite(
        retreat["post_gate_minimum_exact_distance_m"],
        "RAW_RETREAT_POST_GATE_EXACT_CLEARANCE")
    post_gate_guard = _finite(
        retreat["post_gate_minimum_guard_physical_lower_bound_m"],
        "RAW_RETREAT_POST_GATE_GUARD_CLEARANCE")
    _require(post_gate_exact >= RAW_REQUIRED_CLEARANCE_M and
             minimum_exact <= post_gate_exact <= maximum_exact,
             "RAW_RETREAT_POST_GATE_EXACT_CLEARANCE")
    _require(post_gate_guard >= RAW_REQUIRED_CLEARANCE_M,
             "RAW_RETREAT_POST_GATE_GUARD_CLEARANCE")

    _require(retreat["penetration_geometry"] == RAW_RETREAT_EXACT_GEOMETRY,
             "RAW_RETREAT_PENETRATION_GEOMETRY")
    _require(retreat["guard_geometry"] == RAW_RETREAT_GUARD_GEOMETRY,
             "RAW_RETREAT_GUARD_GEOMETRY")
    _require(retreat["observed_pair_semantics"] ==
             RAW_RETREAT_PAIR_SEMANTICS,
             "RAW_RETREAT_PAIR_SEMANTICS")
    left_links = raw["left_cube_allowed_contact_links"]
    _require(type(left_links) is list and
             len(left_links) == len(frozenset(left_links)) and
             frozenset(left_links) == RAW_RETREAT_ALLOWED_LINKS,
             "RAW_RETREAT_ALLOWED_LINKS")
    exact_pairs = _raw_retreat_pairs(
        retreat["observed_exact_pairs"],
        object_id="green_cube_dynamic_exact", status="RAW_RETREAT_EXACT_PAIRS")
    guard_pairs = _raw_retreat_pairs(
        retreat["observed_guard_pairs"],
        object_id="green_cube_10mm_guard", status="RAW_RETREAT_GUARD_PAIRS")
    _raw_retreat_links(
        retreat["observed_exact_links"], exact_pairs,
        status="RAW_RETREAT_EXACT_LINKS")
    _raw_retreat_links(
        retreat["observed_guard_links"], guard_pairs,
        status="RAW_RETREAT_GUARD_LINKS")

    measurements = _exact(
        raw["contact_window_measurements"],
        ("right_handle_hold", "cube_grasp_carry_release",
         "cube_release_retreat"),
        "RAW_CONTACT_WINDOWS")
    contact = _exact(
        measurements["cube_release_retreat"], RAW_RETREAT_CONTACT_KEYS,
        "RAW_RETREAT_CONTACT_WINDOW")
    _require(_integer(
        contact["authorized_sample_count"],
        "RAW_RETREAT_CONTACT_SAMPLES", 1) == window_samples,
        "RAW_RETREAT_CONTACT_SAMPLE_BINDING")
    _require(_finite(
        contact["maximum_observed_penetration_m"],
        "RAW_RETREAT_CONTACT_PENETRATION") == penetration,
        "RAW_RETREAT_CONTACT_PENETRATION_BINDING")
    _require(contact["penetration_geometry"] == RAW_RETREAT_EXACT_GEOMETRY and
             contact["guard_observation_geometry"] ==
             RAW_RETREAT_GUARD_GEOMETRY and
             contact["observed_pair_semantics"] ==
             RAW_RETREAT_PAIR_SEMANTICS,
             "RAW_RETREAT_CONTACT_SEMANTICS")
    contact_exact_pairs = _raw_retreat_pairs(
        contact["observed_pairs"], object_id="green_cube_dynamic_exact",
        status="RAW_RETREAT_CONTACT_EXACT_PAIRS")
    contact_guard_pairs = _raw_retreat_pairs(
        contact["guard_nearest_pairs"], object_id="green_cube_10mm_guard",
        status="RAW_RETREAT_CONTACT_GUARD_PAIRS")
    _require(contact_exact_pairs == exact_pairs and
             contact_guard_pairs == guard_pairs,
             "RAW_RETREAT_CONTACT_PAIR_BINDING")


def _validate_raw_root_contract(raw: Mapping[str, Any]) -> None:
    _require(raw["minimum_environment_clearance_is_lower_bound"] is True,
             "RAW_ENVIRONMENT_CLEARANCE_LOWER_BOUND")
    _require(_finite(
        raw["park_roll_rad"], "RAW_PARK_ROLL") == 0.316,
        "RAW_PARK_ROLL")
    park_required = _finite(
        raw["park_static_clearance_required_m"],
        "RAW_PARK_STATIC_CLEARANCE_REQUIRED")
    _require(park_required == 0.0105,
             "RAW_PARK_STATIC_CLEARANCE_REQUIRED")
    _require(_finite(
        raw["right_handle_nonpad_clearance_required_m"],
        "RAW_RIGHT_HANDLE_NONPAD_CLEARANCE_REQUIRED") == 0.0006,
        "RAW_RIGHT_HANDLE_NONPAD_CLEARANCE_REQUIRED")
    _require(_finite(
        raw["left_handle_clearance_required_m"],
        "RAW_LEFT_HANDLE_CLEARANCE_REQUIRED") == 0.05,
        "RAW_LEFT_HANDLE_CLEARANCE_REQUIRED")
    derivation = _exact(
        raw["environment_clearance_derivation"],
        ("exact_world_objects", "environment_table_10mm_margin",
         "green_cube_10mm_guard", "self_pairs_excluded"),
        "RAW_ENVIRONMENT_CLEARANCE_DERIVATION")
    _require(derivation == {
        "exact_world_objects": "direct_unpadded_FCL_distance",
        "environment_table_10mm_margin": "0.010_plus_guard_distance",
        "green_cube_10mm_guard": "0.010_plus_guard_distance",
        "self_pairs_excluded": True,
    }, "RAW_ENVIRONMENT_CLEARANCE_DERIVATION")
    for label in ("initial", "final"):
        park = _exact(
            raw[f"{label}_park_exact_door_clearance"],
            ("distance_m", "link"), f"RAW_{label.upper()}_PARK_CLEARANCE")
        _require(_finite(
            park["distance_m"],
            f"RAW_{label.upper()}_PARK_CLEARANCE") >= park_required and
            type(park["link"]) is str and park["link"],
            f"RAW_{label.upper()}_PARK_CLEARANCE")
    refinement_samples = _integer(
        raw["cube_guard_exact_refinement_samples"],
        "RAW_CUBE_GUARD_REFINEMENT_SAMPLES")
    refinement_minimum = raw["minimum_cube_exact_noncontact_clearance_m"]
    if refinement_samples == 0:
        _require(refinement_minimum is None,
                 "RAW_CUBE_GUARD_REFINEMENT_NULLABILITY")
    else:
        _require(_finite(
            refinement_minimum,
            "RAW_CUBE_GUARD_REFINEMENT_CLEARANCE") >= 0.0,
            "RAW_CUBE_GUARD_REFINEMENT_CLEARANCE")


def _validate_raw(raw: Mapping[str, Any], manifest: SealedDocument) -> None:
    raw = _exact(raw, RAW_ROOT_KEYS, "RAW")
    _require(raw.get("schema") == RAW_SCHEMA, "RAW_SCHEMA")
    _require(raw.get("auditor_version") == RAW_AUDITOR_VERSION,
             "RAW_AUDITOR_VERSION")
    _require(raw.get("audit_method") == RAW_AUDIT_METHOD,
             "RAW_AUDIT_METHOD")
    _require(raw.get("raw_asset_manifest_sha256") == manifest.sha256,
             "RAW_ASSET_MANIFEST_BINDING")
    _require(raw.get("controller_commands_issued") == 0 and
             raw.get("offline_only") is True,
             "RAW_OFFLINE_ONLY")
    _require(raw.get("collision_free") is True and
             raw.get("geometry_valid") is True and
             raw.get("raw_audit_complete") is True,
             "RAW_GEOMETRY_NOT_PASS")
    # Raw evidence must never claim to be the final result/certificate.
    _require(raw.get("valid") is False and raw.get("status") == RAW_STATUS,
             "RAW_PROMOTION_STATUS")
    for key in (
        "raw_scene_hash", "physical_scene_hash", "raw_scene_cdr_sha256"):
        _sha(raw.get(key), f"RAW:{key}")
    _require(raw["physical_scene_hash"] == raw["raw_scene_cdr_sha256"],
             "RAW_PHYSICAL_SCENE_CDR_BINDING")
    required = (
        "robot_model_snapshot", "cube_bump", "final_target",
        "minimum_clearances_m", "contact_window_measurements", "torso_joint",
        "left_handle", "mask_projection", "cube_release_retreat",
        "per_phase_metrics", "dynamic_handle_exact",
        "right_handle_allowed_contact_pairs", "right_door_allowed_contact_pairs",
        "left_cube_allowed_contact_links", "left_cube_object_ids",
        "entry_projection",
    )
    for key in required:
        _require(key in raw, f"RAW_REQUIRED_FIELD:{key}")
    samples = _integer(raw.get("sample_count"), "RAW_SAMPLE_COUNT", 1)
    subdivisions = _integer(
        raw.get("subdivision_count"), "RAW_SUBDIVISION_COUNT", 1)
    _require(samples == subdivisions, "RAW_SAMPLE_SUBDIVISION_BINDING")
    _validate_raw_root_contract(raw)
    _validate_raw_release_retreat(raw)


def _scene_and_projection(
    contract: Any,
    raw: Mapping[str, Any],
    provenance: Mapping[str, Any],
    projected_endpoint_error: float,
) -> Tuple[Mapping[str, Any], Mapping[str, Any]]:
    mask = _exact(
        raw["mask_projection"],
        ("raw_scene_hash", "physical_scene_hash", "removed_visual_mask_ids",
         "unknown_visual_mask_ids", "classification",
         "handle_exact_retained"),
        "RAW_MASK_PROJECTION")
    _require(mask["raw_scene_hash"] == raw["raw_scene_hash"] and
             mask["physical_scene_hash"] == raw["physical_scene_hash"] and
             mask["handle_exact_retained"] is True,
             "RAW_MASK_PROJECTION_BINDING")
    scene_source = provenance["scene"]
    scene_binding = {
        "scene_path": scene_source["path"],
        "scene_cdr_sha256": raw["raw_scene_cdr_sha256"],
        "scene_name": scene_source["scene_name"],
        "raw_scene_hash": raw["raw_scene_hash"],
        "planning_frame": scene_source["planning_frame"],
        "world": dict(contract.WORLD_VALUE),
        "required_object_ids": scene_source["required_object_ids"],
        "removed_visual_mask_ids": mask["removed_visual_mask_ids"],
        "unknown_visual_mask_ids": mask["unknown_visual_mask_ids"],
    }
    _require(raw["entry_projection"] ==
             "prefix_certified_endpoint_projection_to_canonical_e0_q90_held",
             "RAW_ENTRY_PROJECTION_METHOD")
    raw_robot_state = {
        "label": "raw_scene_q0",
        "torso_yaw_rad": 0.0,
        "right_arm_rad": list(contract.Q0_RIGHT_ARM_RAD),
        "right_fingers_m": dict(contract.OPEN_RIGHT_FINGERS),
        "left_arm_rad": list(contract.LAUNCH_LEFT_ARM_RAD),
        "left_fingers_m": dict(contract.OPEN_LEFT_FINGERS),
    }
    projected_e0 = {
        "label": "canonical_e0_q90_held",
        "torso_yaw_rad": contract.E0_TORSO_RAD,
        "right_arm_rad": list(contract.E0_RIGHT_ARM_RAD),
        "right_fingers_m": dict(contract.E0_RIGHT_FINGERS),
        "left_arm_rad": list(contract.PARK_LEFT_ARM_RAD),
        "left_fingers_m": dict(contract.OPEN_LEFT_FINGERS),
    }
    handle = raw["dynamic_handle_exact"]
    entry_projection = {
        "schema": contract.ENTRY_PROJECTION_SCHEMA,
        "method": "prefix_certified_endpoint_projection",
        "source_scene_sha256": raw["raw_scene_cdr_sha256"],
        "raw_robot_state": raw_robot_state,
        "source_prefix": dict(contract.RIGHT_PREFIX_BINDING),
        "projected_e0": projected_e0,
        "projected_door_angle_rad": handle["door_angle_min_rad"],
        "projected_handle_angle_rad": handle["angle_rad"],
        # Derived from the first points of the verified door-open wrapper.
        "max_abs_joint_error_rad": projected_endpoint_error,
        "valid": True,
    }
    return scene_binding, entry_projection


def _maximum_error(
        actual: Mapping[str, float], expected: Mapping[str, float]) -> float:
    return max(abs(float(actual[name]) - float(expected[name]))
               for name in expected)


def _terminal_seam(
    contract: Any,
    raw: Mapping[str, Any],
    phase_states: Sequence[Mapping[str, float]],
) -> Mapping[str, Any]:
    entry_state = _initial_state(contract)
    final_state = phase_states[-1]
    expected_e0 = {
        "torso_joint": contract.E0_TORSO_RAD,
        **dict(zip(RIGHT_ARM_JOINTS, contract.E0_RIGHT_ARM_RAD)),
        **dict(contract.E0_RIGHT_FINGERS),
    }
    expected_left = {
        **dict(zip(LEFT_ARM_JOINTS, contract.PARK_LEFT_ARM_RAD)),
        **dict(contract.OPEN_LEFT_FINGERS),
    }
    computed_entry = _maximum_error(entry_state, expected_e0)
    computed_return = _maximum_error(final_state, expected_e0)
    computed_left = _maximum_error(final_state, expected_left)
    _require(computed_entry <= 1e-12 and computed_return <= 1e-12 and
             computed_left <= 1e-12, "TERMINAL_SEAM_ENDPOINT_ERROR")
    handle = _exact(
        raw["dynamic_handle_exact"],
        ("id", "geometry_type", "mesh_count", "primitive_count",
         "geometry_sha256", "retained_every_phase", "audited_phases",
         "angle_rad", "door_angle_min_rad", "door_angle_max_rad"),
        "RAW_DYNAMIC_HANDLE")
    _require(handle["id"] == "incubator_handle_exact" and
             handle["retained_every_phase"] is True and
             handle["audited_phases"] == list(PHASE_NAMES),
             "RAW_DYNAMIC_HANDLE_BINDING")
    return {
        "entry_e0": {
            "torso_yaw_rad": entry_state["torso_joint"],
            "right_arm_rad": [entry_state[name] for name in RIGHT_ARM_JOINTS],
            "right_fingers_m": {
                name: entry_state[name] for name in RIGHT_GRIPPER_JOINTS},
            "max_abs_error": computed_entry,
        },
        "return_e0": {
            "torso_yaw_rad": final_state["torso_joint"],
            "right_arm_rad": [final_state[name] for name in RIGHT_ARM_JOINTS],
            "right_fingers_m": {
                name: final_state[name] for name in RIGHT_GRIPPER_JOINTS},
            "max_abs_error": computed_return,
        },
        "left_park_open": {
            "left_arm_rad": [final_state[name] for name in LEFT_ARM_JOINTS],
            "left_fingers_m": {
                name: final_state[name] for name in LEFT_GRIPPER_JOINTS},
            "max_abs_error": computed_left,
        },
        "door_state": {
            "entry_angle_rad": handle["door_angle_min_rad"],
            "return_angle_rad": handle["door_angle_min_rad"],
            "handle_angle_rad": handle["angle_rad"],
        },
        "support_state": {
            "before_entry": "detached",
            "after_entry": "attached",
            "before_return": "attached",
            "after_return": "detached",
        },
        "release_owner": contract.RELEASE_OWNER,
        "forbidden_middle_phases": ["right_release", "right_retreat"],
        "valid": True,
    }


def _support_contract(
    support: Any,
    provenance: Mapping[str, Any],
    phase_states: Sequence[Mapping[str, float]],
) -> Mapping[str, Any]:
    contact_state = phase_states[4]
    release_state = phase_states[5]
    left_contact = {name: contact_state[name] for name in support.LEFT_JOINTS}
    left_release = {name: release_state[name] for name in support.LEFT_JOINTS}
    cube_poses = provenance["cube_poses"]
    support_binding = provenance["support"]
    return {
        "schema": SUPPORT_CONTRACT_SCHEMA,
        "phases": list(PHASE_NAMES),
        "world": {
            "world_name": "h1_world",
            "table_x": 0.600,
            "incubator_x": 0.590,
            "incubator_y": 0.0,
            "incubator_z": 0.815,
            "cube_spawn_x": 0.334,
            "cube_spawn_y": 0.377,
            "cube_spawn_z": 0.830,
            "cube_spawn_yaw": math.pi / 2.0,
        },
        "hooks": [
            {"operation": "door-attach", "boundary": "E0_ENTRY",
             "phase_index": 0, "edge": "entry", "door_before": "detached",
             "door_after": "attached", "cube_before": "detached",
             "cube_after": "detached"},
            {"operation": "cube-attach", "boundary": "CUBE_CONTACT",
             "phase_index": 4, "edge": "exit", "door_before": "attached",
             "door_after": "attached", "cube_before": "detached",
             "cube_after": "attached"},
            {"operation": "cube-detach", "boundary": "CUBE_RELEASE",
             "phase_index": 5, "edge": "exit", "door_before": "attached",
             "door_after": "attached", "cube_before": "attached",
             "cube_after": "detached"},
            {"operation": "door-detach", "boundary": "E0_RETURN",
             "phase_index": 10, "edge": "exit", "door_before": "attached",
             "door_after": "detached", "cube_before": "detached",
             "cube_after": "detached"},
        ],
        "boundaries": {
            "E0_ENTRY": {
                "robot_positions": dict(support.CANONICAL_MIDDLE_SEAM)},
            "E0_RETURN": {
                "robot_positions": dict(support.CANONICAL_MIDDLE_SEAM)},
            "CUBE_CONTACT": {
                "robot_positions": left_contact,
                "cube_pose": cube_poses["contact"],
            },
            "CUBE_RELEASE": {
                "robot_positions": left_release,
                "cube_pose": cube_poses["release"],
            },
            "FINAL_ROBOT": {
                "robot_positions": dict(support.CANONICAL_FINAL_ROBOT)},
        },
        "plugin": {
            "binary_sha256": support_binding["plugin_sha256"],
            "door": {
                "parent_scope": "h1_2::right_wrist_yaw_link",
                "child_scope": "incubator::door_link",
                "state_topic": support.DOOR_STATE_TOPIC,
                "attach_topic": support.DOOR_ATTACH_TOPIC,
                "detach_topic": support.DOOR_DETACH_TOPIC,
                "max_force": 120.0,
                "max_torque": 8.0,
                "gravity_compensation": False,
            },
            "cube": {
                "parent_scope": "h1_2::left_wrist_yaw_link",
                "child_model": "green_cube",
                "state_topic": support.CUBE_STATE_TOPIC,
                "attach_topic": support.CUBE_ATTACH_TOPIC,
                "detach_topic": support.CUBE_DETACH_TOPIC,
                "initially_attached": False,
            },
        },
        "tools": {
            "ros_snapshot": {
                "workspace_relative_path": support_binding["ros_snapshot_path"],
                "sha256": support_binding["ros_snapshot_sha256"],
            },
        },
    }


def _clearances_from_phase(metric: Mapping[str, Any], index: int) -> Mapping[str, float]:
    mapping = {
        "unauthorized_environment": "minimum_environment_clearance_m",
        "table": "minimum_table_clearance_m",
        "door": "minimum_door_clearance_m",
        "handle": "minimum_handle_clearance_m",
        "cube": "minimum_cube_clearance_m",
        "self": "minimum_self_clearance_m",
        "right_handle_nonpad": "minimum_right_handle_nonpad_clearance_m",
        "left_handle": "minimum_left_handle_clearance_m",
    }
    return {name: _finite(metric[source],
                          f"RAW_PHASE_CLEARANCE:{index}:{source}")
            for name, source in mapping.items()}


def _phase_results(
    raw: Mapping[str, Any], phases: Sequence[Mapping[str, Any]],
) -> list[Mapping[str, Any]]:
    raw_metrics = raw["per_phase_metrics"]
    _require(type(raw_metrics) is list and len(raw_metrics) == len(PHASE_NAMES),
             "RAW_PHASE_METRICS")
    result = []
    for index, (name, asset_id, expected_range, metric, phase) in enumerate(zip(
            PHASE_NAMES, PHASE_ASSET_IDS, PHASE_RANGES, raw_metrics, phases)):
        _require(type(metric) is dict, f"RAW_PHASE_METRIC:{index}")
        _require(metric.get("phase") == name and
                 metric.get("asset_id") == asset_id and
                 metric.get("message_range") == list(expected_range),
                 f"RAW_PHASE_METRIC_BINDING:{index}")
        samples = _integer(metric.get("samples"),
                           f"RAW_PHASE_SAMPLES:{index}", 1)
        subdivisions = _integer(metric.get("subdivisions"),
                                f"RAW_PHASE_SUBDIVISIONS:{index}", 1)
        _require(metric.get("right_handle_authorized_samples") == samples and
                 metric.get("minimum_is_lower_bound") is True,
                 f"RAW_PHASE_AUTHORIZATION:{index}")
        minimum_torso = _finite(metric.get("minimum_torso_rad"),
                                f"RAW_PHASE_TORSO_MIN:{index}")
        maximum_torso = _finite(metric.get("maximum_torso_rad"),
                                f"RAW_PHASE_TORSO_MAX:{index}")
        window_ids = ["right_handle_pad_contact"]
        if index in (4, 5, 6):
            window_ids.append("left_cube_grasp_carry")
        if index == 7:
            window_ids.append("left_cube_release_retreat")
        result.append({
            "index": index,
            "name": name,
            "asset_id": asset_id,
            "message_range": {
                "start_message_index": expected_range[0],
                "end_message_index_exclusive": expected_range[1],
            },
            "topics": [command["topic"] for command in phase["commands"]],
            "samples": samples,
            "subdivisions": subdivisions,
            "minimum_clearances_m": _clearances_from_phase(metric, index),
            "maximum_abs_torso_yaw_rad": max(
                abs(minimum_torso), abs(maximum_torso)),
            "contact_window_ids": window_ids,
            "endpoint_sha256": phase["exit_state_sha256"],
            "valid": True,
            "status": "PASS",
        })
    _require(sum(item["samples"] for item in result) == raw["sample_count"],
             "RAW_PHASE_SAMPLE_SUM")
    _require(sum(item["subdivisions"] for item in result) ==
             raw["subdivision_count"], "RAW_PHASE_SUBDIVISION_SUM")
    return result


def _pairs(value: Any, status: str) -> list[list[str]]:
    _require(type(value) is list, status)
    output = []
    for index, pair in enumerate(value):
        _require(type(pair) is list and len(pair) == 2 and
                 all(type(item) is str and item for item in pair),
                 f"{status}:{index}")
        output.append(list(pair))
    return output


def _contact_authorizations(
    contract: Any,
    raw: Mapping[str, Any],
    phase_results: Sequence[Mapping[str, Any]],
) -> Mapping[str, Any]:
    measurements = _exact(
        raw["contact_window_measurements"],
        ("right_handle_hold", "cube_grasp_carry_release",
         "cube_release_retreat"),
        "RAW_CONTACT_WINDOWS")
    expected_right_pairs = {
        ("incubator_handle_exact", link)
        for link in contract.RIGHT_HANDLE_PADS}
    actual_right_pairs = {
        tuple(pair) for pair in _pairs(
            raw["right_handle_allowed_contact_pairs"],
            "RAW_RIGHT_HANDLE_ALLOWED_PAIRS")}
    _require(actual_right_pairs == expected_right_pairs,
             "RAW_RIGHT_HANDLE_ALLOWED_PAIR_SET")
    _require(raw.get("right_door_allowed_contact_pairs") == [],
             "RAW_RIGHT_DOOR_ALLOWED_PAIR_SET")
    _require(frozenset(raw["left_cube_allowed_contact_links"]) ==
             contract.LEFT_CUBE_TOUCH_LINKS,
             "RAW_LEFT_CUBE_ALLOWED_LINK_SET")
    _require(raw["left_cube_object_ids"] == list(contract.CUBE_OBJECT_IDS),
             "RAW_LEFT_CUBE_OBJECT_IDS")

    def binding(index: int) -> Mapping[str, Any]:
        phase = phase_results[index]
        return {
            "phase_name": phase["name"],
            "asset_id": phase["asset_id"],
            "message_range": dict(phase["message_range"]),
        }

    right = measurements["right_handle_hold"]
    cube = measurements["cube_grasp_carry_release"]
    retreat = measurements["cube_release_retreat"]
    for label, value in (("RIGHT", right), ("CUBE", cube),
                         ("RETREAT", retreat)):
        _require(type(value) is dict, f"RAW_CONTACT_{label}")
        _integer(value.get("authorized_sample_count"),
                 f"RAW_CONTACT_{label}_SAMPLES", 1)
        _finite(value.get("maximum_observed_penetration_m"),
                f"RAW_CONTACT_{label}_PENETRATION")
        _pairs(value.get("observed_pairs"), f"RAW_CONTACT_{label}_PAIRS")
    _require(right["authorized_sample_count"] == raw["sample_count"],
             "RAW_CONTACT_RIGHT_SAMPLE_BINDING")
    _require(cube["authorized_sample_count"] == sum(
        phase_results[index]["samples"] for index in (4, 5, 6)),
        "RAW_CONTACT_CUBE_SAMPLE_BINDING")
    release_metric = raw["cube_release_retreat"]
    _require(retreat["authorized_sample_count"] ==
             release_metric["window_sample_count"],
             "RAW_CONTACT_RETREAT_SAMPLE_BINDING")
    _require(retreat["maximum_observed_penetration_m"] ==
             release_metric["maximum_exact_penetration_m"] and
             retreat["observed_pairs"] == release_metric["observed_exact_pairs"],
             "RAW_CONTACT_RETREAT_METRIC_BINDING")

    left_handle = _exact(
        raw["left_handle"],
        ("allowed_contact_pairs", "contact_count", "minimum_clearance_m"),
        "RAW_LEFT_HANDLE")
    _require(left_handle["allowed_contact_pairs"] == [] and
             left_handle["contact_count"] == 0,
             "RAW_LEFT_HANDLE_CONTACT")
    # The raw auditor only reaches collision_free/raw_audit_complete after
    # rejecting every pair outside its explicit three windows.  Therefore the
    # final zero is a logical projection of that gate, not a sampled metric.
    unauthorized = {"count": 0, "pairs": []}
    return {
        "schema": contract.CONTACT_AUTHORIZATION_SCHEMA,
        "windows": [
            {
                "id": "right_handle_pad_contact",
                "phase_bindings": [binding(index) for index in range(11)],
                "object_ids": ["incubator_handle_exact"],
                "allowed_links": sorted(contract.RIGHT_HANDLE_PADS),
                "penetration_policy": "AUTHORIZED_PAD_CONTACT",
                "maximum_observed_penetration_m":
                    right["maximum_observed_penetration_m"],
                "observed_pairs": right["observed_pairs"],
                "authorized_sample_count": right["authorized_sample_count"],
                "valid": True,
            },
            {
                "id": "left_cube_grasp_carry",
                "phase_bindings": [binding(index) for index in (4, 5, 6)],
                "object_ids": list(contract.CUBE_OBJECT_IDS),
                "allowed_links": sorted(contract.LEFT_CUBE_TOUCH_LINKS),
                "penetration_policy": "AUTHORIZED_GRASP_CONTACT",
                "maximum_observed_penetration_m":
                    cube["maximum_observed_penetration_m"],
                "observed_pairs": cube["observed_pairs"],
                "authorized_sample_count": cube["authorized_sample_count"],
                "valid": True,
            },
            {
                "id": "left_cube_release_retreat",
                "phase_bindings": [binding(7)],
                "object_ids": list(contract.CUBE_OBJECT_IDS),
                "allowed_links": sorted(contract.LEFT_CUBE_TOUCH_LINKS),
                "penetration_policy": "NONPENETRATING",
                "maximum_observed_penetration_m":
                    retreat["maximum_observed_penetration_m"],
                "observed_pairs": retreat["observed_pairs"],
                "authorized_sample_count": retreat["authorized_sample_count"],
                "valid": True,
            },
        ],
        "forbidden_left_handle": {
            "object_id": "incubator_handle_exact",
            "link_prefix": "left_",
            "contact_count": left_handle["contact_count"],
            "minimum_clearance_m": left_handle["minimum_clearance_m"],
            "required_minimum_clearance_m": 0.05,
            "valid": True,
        },
        "unauthorized_contacts": dict(unauthorized),
        "valid": True,
    }


def _metrics(raw: Mapping[str, Any]) -> Mapping[str, Any]:
    clearances = _exact(
        raw["minimum_clearances_m"],
        ("unauthorized_environment", "table", "door", "handle", "cube",
         "self", "right_handle_nonpad", "left_handle"),
        "RAW_GLOBAL_CLEARANCES")
    for name, value in clearances.items():
        _finite(value, f"RAW_GLOBAL_CLEARANCE:{name}")
    torso = _exact(
        raw["torso_joint"],
        ("commanded", "minimum_rad", "maximum_rad", "maximum_absolute_rad"),
        "RAW_TORSO")
    _require(torso["commanded"] is True, "RAW_TORSO_COMMANDED")
    bump = raw["cube_bump"]
    target = _exact(
        raw["final_target"],
        ("translation_error_m", "rotation_error_rad", "translation_limit_m",
         "rotation_limit_rad"),
        "RAW_FINAL_TARGET")
    retreat_raw = raw["cube_release_retreat"]
    retreat_keys = (
        "phase_name", "message_index", "start_point_index",
        "start_subdivision", "window_sample_count", "gate_point_index",
        "gate_subdivision", "minimum_exact_distance_m",
        "maximum_exact_distance_m", "final_exact_distance_m",
        "maximum_exact_penetration_m", "minimum_time_step_ns",
        "minimum_path_parameter_step",
        "minimum_tcp_separation_projection_step_m",
        "post_gate_minimum_exact_distance_m",
        "post_gate_minimum_guard_physical_lower_bound_m",
        "observed_exact_pairs", "observed_guard_pairs", "gate_never_regressed",
    )
    for key in retreat_keys:
        _require(key in retreat_raw, f"RAW_RETREAT_REQUIRED:{key}")
    retreat = {key: retreat_raw[key] for key in retreat_keys}
    return {
        "sample_count": raw["sample_count"],
        "subdivision_count": raw["subdivision_count"],
        "minimum_clearances_m": dict(clearances),
        "maximum_abs_torso_yaw_rad": torso["maximum_absolute_rad"],
        "cube_bump": {
            "maximum_tcp_translation_m": bump["maximum_tcp_translation_m"],
            "maximum_tcp_rotation_rad": bump["maximum_tcp_rotation_rad"],
            "translation_limit_m": bump["translation_limit_m"],
            "rotation_limit_rad": bump["rotation_limit_rad"],
        },
        "cube_release_retreat": retreat,
        "final_target": dict(target),
        "all_thresholds_passed": True,
    }


def _plan_provenance(
    workspace: Path,
    raw_document: SealedDocument,
    manifest: SealedDocument,
    provenance_document: SealedDocument,
    provenance: Mapping[str, Any],
    authorities: Mapping[str, Mapping[str, Any]],
) -> Mapping[str, Any]:
    return {
        "baseline_backup": dict(provenance["baseline_backup"]),
        "raw_combined_exact": {
            "path": _relative_to_workspace(
                workspace, raw_document.path, "RAW_RELATIVE_PATH"),
            "file_sha256": raw_document.sha256,
            "schema": RAW_SCHEMA,
            "auditor_version": RAW_AUDITOR_VERSION,
        },
        "raw_asset_manifest": {
            "path": _relative_to_workspace(
                workspace, manifest.path, "RAW_MANIFEST_RELATIVE_PATH"),
            "file_sha256": manifest.sha256,
            "schema": RAW_BINDINGS_SCHEMA,
        },
        "scene_model_provenance": {
            "path": _relative_to_workspace(
                workspace, provenance_document.path,
                "PROVENANCE_RELATIVE_PATH"),
            "file_sha256": provenance_document.sha256,
            "schema": PROVENANCE_SCHEMA,
        },
        "robot_model": dict(provenance["robot_model"]),
        "cube_reference": dict(provenance["cube_reference"]),
        "raw_auditor": dict(provenance["auditor"]),
        "raw_auditor_run": dict(provenance["auditor_run"]),
        "execution_authorities": {
            key: dict(authorities[key]) for key in AUTHORITY_IDS},
        "support_binary_and_snapshot": dict(provenance["support"]),
        "endpoint_derivation": {
            "schema": ENDPOINT_SCHEMA,
            "method": "verified_wrapper_phase_slice_state_accumulator",
            "phase_ranges": {
                name: list(message_range)
                for name, message_range in zip(PHASE_NAMES, PHASE_RANGES)},
        },
    }


def _write_staged(path: Path, raw: bytes) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
    descriptor = os.open(path, flags, 0o600)
    try:
        offset = 0
        while offset < len(raw):
            written = os.write(descriptor, raw[offset:])
            _require(written > 0, "STAGE_SHORT_WRITE")
            offset += written
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o400)
    finally:
        os.close(descriptor)


def _publish_no_replace(
    staged: Mapping[str, Path], output_directory: Path,
) -> Mapping[str, Path]:
    names = {
        "plan": "candidate3_middle_plan_828.json",
        "result": "candidate3_middle_exact_result_828.json",
        "certificate": "candidate3_middle_certificate_828.json",
        "support_manifest": "candidate3_middle_support_manifest_828.json",
    }
    targets = {key: output_directory / name for key, name in names.items()}
    _require(all(not path.exists() and not path.is_symlink()
                 for path in targets.values()), "PUBLISH_TARGET_EXISTS")
    created = []
    def rollback() -> None:
        for target in reversed(created):
            try:
                target.unlink()
            except FileNotFoundError:
                pass

    try:
        for key in ("plan", "result", "certificate", "support_manifest"):
            os.link(staged[key], targets[key], follow_symlinks=False)
            created.append(targets[key])
        for key in ("plan", "result", "certificate", "support_manifest"):
            staged[key].unlink()
        directory_descriptor = os.open(
            output_directory,
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
            getattr(os, "O_CLOEXEC", 0))
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
        for target in targets.values():
            metadata = target.lstat()
            _require(stat.S_ISREG(metadata.st_mode) and
                     stat.S_IMODE(metadata.st_mode) == 0o400 and
                     metadata.st_nlink == 1 and
                     metadata.st_uid == os.geteuid(),
                     "PUBLISHED_IDENTITY")
    except OSError as error:
        rollback()
        raise MiddleBundleAssemblerError(
            f"PUBLISH_NO_REPLACE:{error}") from error
    except BaseException:
        rollback()
        raise
    return targets


def assemble(args: argparse.Namespace) -> Bundle:
    """Build and self-prove one bundle; publish only with ``args.publish``."""
    workspace = _workspace(args.workspace)
    raw_document = _load_json(
        args.raw, args.raw_sha256, status="RAW_COMBINED")
    _require(hmac.compare_digest(
        raw_document.sha256, EXPECTED_RAW_FILE_SHA256),
        "RAW_COMBINED_FROZEN_SHA256")
    manifest = _load_json(
        args.asset_bindings, args.asset_bindings_sha256,
        status="RAW_ASSET_MANIFEST")
    provenance_document = _load_json(
        args.provenance, args.provenance_sha256, status="PROVENANCE")
    _validate_raw(raw_document.value, manifest)
    provenance, contract, admission, support, authorities = (
        _validate_provenance(
            workspace, provenance_document, raw_document, manifest))
    _require(tuple(contract.PHASE_NAMES) == PHASE_NAMES,
             "CONTRACT_PHASE_TOPOLOGY")
    _require(tuple(support.EXPECTED_PHASES) == PHASE_NAMES,
             "SUPPORT_PHASE_TOPOLOGY")
    assets, plan_bindings = _load_assets(workspace, manifest)
    phases, phase_states, timing = _accumulate_phases(contract, assets)
    support_contract = _support_contract(support, provenance, phase_states)
    support_contract_sha = _canonical_hash(support_contract)
    scene_binding, entry_projection = _scene_and_projection(
        contract, raw_document.value, provenance,
        timing["first_point_continuity"][1]["maximum_first_point_error"])
    terminal_seam = _terminal_seam(
        contract, raw_document.value, phase_states)
    plan = {
        "schema": contract.PLAN_SCHEMA,
        "worker_version": WORKER_VERSION,
        "topology": "right_door_open_left_cube_right_door_close_828_e0",
        "offline_only": True,
        "controller_commands_issued": 0,
        "scene_binding": scene_binding,
        "entry_projection": entry_projection,
        "assets": plan_bindings,
        "phase_names": list(PHASE_NAMES),
        "phases": phases,
        "hooks": _hooks(),
        "support_contract_sha256": support_contract_sha,
        "release_owner": contract.RELEASE_OWNER,
        "terminal_seam": terminal_seam,
        "final_state": {
            "endpoint": _endpoint_value(phase_states[-1]),
            "door": "closed_supported",
            "cube": "released_at_fixed_target",
            "release_owner": contract.RELEASE_OWNER,
        },
        "required_thresholds": {
            "minimum_environment_clearance_m": 0.010001,
            "minimum_right_handle_nonpad_clearance_m": 0.0006,
            "minimum_left_handle_clearance_m": 0.05,
            "maximum_abs_torso_yaw_rad": math.pi / 4.0,
            "cube_bump_translation_limit_m": 0.006,
            "cube_bump_rotation_limit_rad": 0.013962634015954637,
            "final_target_translation_limit_m": 0.003,
            "final_target_rotation_limit_rad": 0.03490658503988659,
        },
        "timing": timing,
        "provenance": _plan_provenance(
            workspace, raw_document, manifest, provenance_document,
            provenance, authorities),
    }
    try:
        contract.validate_plan(plan)
    except Exception as error:
        raise MiddleBundleAssemblerError(
            f"SELF_VALIDATE_PLAN:{error}") from error

    output_directory = Path(args.output_directory)
    _require(output_directory.is_absolute(), "OUTPUT_DIRECTORY_NOT_ABSOLUTE")
    try:
        output_metadata = output_directory.lstat()
        output_resolved = output_directory.resolve(strict=True)
    except OSError as error:
        raise MiddleBundleAssemblerError(
            f"OUTPUT_DIRECTORY:{error}") from error
    _require(stat.S_ISDIR(output_metadata.st_mode) and
             not output_directory.is_symlink() and
             output_metadata.st_uid == os.geteuid() and
             output_resolved == output_directory,
             "OUTPUT_DIRECTORY_IDENTITY")
    staging_directory = Path(tempfile.mkdtemp(
        prefix=".candidate3_middle_828_stage_", dir=output_directory))
    staged_paths = {
        "plan": staging_directory / "01_plan.json",
        "result": staging_directory / "02_result.json",
        "certificate": staging_directory / "03_certificate.json",
        "support_manifest": staging_directory / "04_support_manifest.json",
    }
    published_paths: Mapping[str, Path] = {}
    try:
        plan_raw = _canonical_bytes(plan)
        _write_staged(staged_paths["plan"], plan_raw)
        plan_sha = hashlib.sha256(plan_raw).hexdigest()

        phase_results = _phase_results(raw_document.value, phases)
        metrics = _metrics(raw_document.value)
        contact_authorizations = _contact_authorizations(
            contract, raw_document.value, phase_results)
        result = {
            "schema": contract.RESULT_SCHEMA,
            "auditor_version": contract.AUDITOR_VERSION,
            "audit_method": raw_document.value["audit_method"],
            "offline_only": True,
            "controller_commands_issued": 0,
            "scene_binding": scene_binding,
            "entry_projection": entry_projection,
            "plan": plan,
            "plan_file_sha256": plan_sha,
            "support_contract_sha256": support_contract_sha,
            "assets": plan_bindings,
            "phase_results": phase_results,
            "contact_authorizations": contact_authorizations,
            "metrics": metrics,
            "terminal_seam": terminal_seam,
            "valid": True,
            "status": "PASS",
        }
        try:
            contract.validate_result(result)
        except Exception as error:
            raise MiddleBundleAssemblerError(
                f"SELF_VALIDATE_RESULT:{error}") from error
        result_raw = _canonical_bytes(result)
        _write_staged(staged_paths["result"], result_raw)
        result_sha = hashlib.sha256(result_raw).hexdigest()

        certificate = {
            "schema": contract.CERTIFICATE_SCHEMA,
            "auditor_version": contract.AUDITOR_VERSION,
            "offline_only": True,
            "controller_commands_issued": 0,
            "plan_schema": contract.PLAN_SCHEMA,
            "plan_sha256": plan_sha,
            "exact_result_schema": contract.RESULT_SCHEMA,
            "exact_result_sha256": result_sha,
            "support_contract_sha256": support_contract_sha,
            "scene_binding_sha256": _canonical_hash(scene_binding),
            "entry_projection_sha256": _canonical_hash(entry_projection),
            "asset_bindings_sha256": _canonical_hash(plan_bindings),
            "terminal_seam_sha256": _canonical_hash(terminal_seam),
            "metrics": metrics,
            "valid": True,
            "status": "PASS",
        }
        try:
            contract.validate_certificate(
                certificate, plan_file_sha256=plan_sha,
                exact_result_file_sha256=result_sha, result=result,
                support_contract_sha256=support_contract_sha)
        except Exception as error:
            raise MiddleBundleAssemblerError(
                f"SELF_VALIDATE_CERTIFICATE:{error}") from error
        certificate_raw = _canonical_bytes(certificate)
        _write_staged(staged_paths["certificate"], certificate_raw)
        certificate_sha = hashlib.sha256(certificate_raw).hexdigest()

        support_manifest = {
            "schema": SUPPORT_MANIFEST_SCHEMA,
            "controller_execution_authorized": True,
            "live_execution_passed": False,
            "bindings": {
                "plan_sha256": plan_sha,
                "plan_schema": contract.PLAN_SCHEMA,
                "certificate_sha256": certificate_sha,
                "certificate_schema": contract.CERTIFICATE_SCHEMA,
                "exact_result_sha256": result_sha,
                "exact_result_schema": contract.RESULT_SCHEMA,
                "auditor_version": contract.AUDITOR_VERSION,
                "support_contract_sha256": support_contract_sha,
            },
            "support_contract": support_contract,
        }
        support_manifest_raw = _canonical_bytes(support_manifest)
        _write_staged(
            staged_paths["support_manifest"], support_manifest_raw)
        support_manifest_sha = hashlib.sha256(
            support_manifest_raw).hexdigest()

        # Independent self-proofs over the staged, mode-0400 nodes.
        try:
            admitted_plan = admission.admit_result(
                staged_paths["result"], result_sha)
            _require(admitted_plan == plan, "ADMISSION_PLAN_MISMATCH")
        except MiddleBundleAssemblerError:
            raise
        except Exception as error:
            raise MiddleBundleAssemblerError(
                f"SELF_VALIDATE_ADMISSION:{error}") from error
        try:
            sealed_manifest = support.load_sealed_json(
                staged_paths["support_manifest"], support_manifest_sha)
            sealed_certificate = support.load_sealed_json(
                staged_paths["certificate"], certificate_sha)
            validated_support = support.validate_manifest_and_certificate(
                sealed_manifest, sealed_certificate)
            _require(validated_support.support_contract_sha256 ==
                     support_contract_sha, "SUPPORT_CONTRACT_HASH_MISMATCH")
            support.load_bound_snapshot_executable(
                workspace, validated_support.snapshot_relative_path,
                validated_support.snapshot_sha256)
        except MiddleBundleAssemblerError:
            raise
        except Exception as error:
            raise MiddleBundleAssemblerError(
                f"SELF_VALIDATE_SUPPORT:{error}") from error

        documents = {
            "plan": plan,
            "result": result,
            "certificate": certificate,
            "support_manifest": support_manifest,
        }
        raw_bytes = {
            "plan": plan_raw,
            "result": result_raw,
            "certificate": certificate_raw,
            "support_manifest": support_manifest_raw,
        }
        digests = {
            "plan": plan_sha,
            "result": result_sha,
            "certificate": certificate_sha,
            "support_manifest": support_manifest_sha,
        }
        if args.publish:
            published_paths = _publish_no_replace(
                staged_paths, output_directory)
        return Bundle(
            documents, raw_bytes, digests, dict(staged_paths),
            published_paths)
    finally:
        shutil.rmtree(staging_directory, ignore_errors=True)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--raw-sha256", required=True)
    parser.add_argument("--asset-bindings", type=Path, required=True)
    parser.add_argument("--asset-bindings-sha256", required=True)
    parser.add_argument("--provenance", type=Path, required=True)
    parser.add_argument("--provenance-sha256", required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument(
        "--publish", action="store_true",
        help="publish four no-replace mode-0400 files after every self-proof")
    parser.add_argument("--quiet", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        bundle = assemble(args)
    except MiddleBundleAssemblerError as error:
        print(f"CANDIDATE3_MIDDLE_BUNDLE_828=FAIL:{error}", file=sys.stderr)
        return 1
    if not args.quiet:
        state = "PUBLISHED" if args.publish else "DRY_RUN"
        print(
            f"CANDIDATE3_MIDDLE_BUNDLE_828=PASS state={state} "
            f"plan_sha256={bundle.sha256['plan']} "
            f"result_sha256={bundle.sha256['result']} "
            f"certificate_sha256={bundle.sha256['certificate']} "
            f"support_manifest_sha256={bundle.sha256['support_manifest']} "
            "controller_commands=0 ros_imports=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
