#!/usr/bin/python3
"""Certificate-bound fast-831 executor for the sealed 826.3 right-hand path.

Validation is deliberately completed before importing ROS.  The executor
reconstructs exactly the stage topology audited by
``right_handle_path_exact_auditor``; a manifest cannot inject extra waypoints
or controller topics.  It publishes only the five canonical robot controller
topics and never talks to an incubator, support, or simulator command API.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
import re
import stat
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple


MANIFEST_SCHEMA = "h1_right_handle_controller_execution_authorization/v1"
# The final manifest additionally binds the exact v3 producer version.  The
# distinct schema gate makes every fixed-torso v2 asset inadmissible.
PATH_SCHEMA = "h1_moveit/right_handle_variable_torso_path/v4"
AUDIT_SCHEMA = "h1_right_handle_path_exact_audit/v2"
INGRESS_SCHEMA = "h1_right_handle_splice_search/v1"
EXECUTION_TOPOLOGY = "right_q0_q90_hold_middle_hold_q90_q0_release/v1"
EXPECTED_AUDITOR = "h1_right_handle_path_exact_auditor/829-a1-torso-margin"
EXPECTED_PATH_SHA256 = (
    "92d0f131324db1755ecdf2a1e03b102b450fbd783a647625e9baf9dfb0a0fb98"
)
EXPECTED_AUDIT_SHA256 = (
    "0e0ec724187063f92578c9903df820c3aa66a87563dd7a5201e62be728272c17"
)
EXPECTED_INGRESS_SHA256 = (
    "2e61bc02c8d5ac3baf0e373b5668cfd7cd66e88fc4197a92d1e8cce9ac1942ff"
)
# The sealed ingress was searched against this geometry-identical v2 source.
# Its own SHA and the v3 audit's q0_ingress_asset_sha256 bind it into the v3
# bundle; this lineage field must not be confused with the v3 document SHA.
EXPECTED_INGRESS_SOURCE_PATH_SHA256 = (
    "afcec0b5c675019157e4339dd41d15728233fdce6344501db6a50d20a0d58088"
)
EXPECTED_HANDLE_STL_SHA256 = (
    "ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e"
)
SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
MAX_PROTECTED_BYTES = 256 * 1024 * 1024
PLANNED_MAX_TORSO_RAD = math.radians(44.25)
HARD_MAX_TORSO_RAD = math.radians(44.5)
# The planned path leaves a physical 0.25-degree margin to the formal hard
# limit.  This epsilon is only for floating-point comparison, not tracking.
RUNTIME_TORSO_ABS_TOL_RAD = 1e-12
HARD_DENSE_ANGULAR_STEP_RAD = math.radians(0.1)
HARD_DENSE_PRISMATIC_STEP_M = 0.0001
HARD_START_TOLERANCE_RAD = 0.003
HARD_ENDPOINT_TOLERANCE_RAD = 0.003
HARD_GRIPPER_TOLERANCE_M = 0.001
HARD_MIN_LEAD_TIME_S = 0.25
HARD_MIN_POINT_DT_S = 0.004
HARD_MAX_DISCOVERY_TIMEOUT_S = 60.0
HARD_MAX_WALL_TIMEOUT_FACTOR = 100.0
HARD_MAX_TIME_SCALE = 10.0
HARD_MAX_STAGE_DURATION_S = 600.0
HARD_SIM_CLOCK_STALL_S = 1.0
HARD_ENDPOINT_SETTLE_SIM_S = 0.5
HARD_ENDPOINT_CONSECUTIVE_SAMPLES = 3
HARD_FAST_MAX_REVOLUTE_VELOCITY_RAD_S = 2.0
HARD_FAST_MAX_REVOLUTE_ACCELERATION_RAD_S2 = 750.0
HARD_FAST_MAX_PRISMATIC_VELOCITY_M_S = 0.10
HARD_FAST_MAX_PRISMATIC_ACCELERATION_M_S2 = 30.0
MAX_DENSE_POINTS_PER_STAGE = 100_000
REQUIRED_CONTROLLER_AND_RECORDER_SUBSCRIPTIONS = 2

LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
RIGHT_ARM_JOINTS = tuple(name.replace("left_", "right_", 1)
                         for name in LEFT_ARM_JOINTS)
RIGHT_GRIPPER_JOINTS = (
    "right_dex1_finger_1_joint", "right_dex1_finger_2_joint",
)
LEFT_GRIPPER_JOINTS = (
    "left_dex1_finger_1_joint", "left_dex1_finger_2_joint",
)
CONTROLLER_JOINTS: Mapping[str, Tuple[str, ...]] = {
    "/torso_controller/joint_trajectory": ("torso_joint",),
    "/left_arm_controller/joint_trajectory": LEFT_ARM_JOINTS,
    "/right_arm_controller/joint_trajectory": RIGHT_ARM_JOINTS,
    "/left_gripper_controller/joint_trajectory": LEFT_GRIPPER_JOINTS,
    "/right_gripper_controller/joint_trajectory": RIGHT_GRIPPER_JOINTS,
}
CANONICAL_CONTROLLER_TOPICS = tuple(CONTROLLER_JOINTS)

LAUNCH_LEFT_ARM = (
    -0.000012933885986937658, 0.24998646984876535,
    0.000023774721497885907, 0.6000753185207717,
    0.000023572560034304773, -0.00008730492997967213,
    -0.00001654021997178485,
)
PARK_LEFT_ARM = (
    LAUNCH_LEFT_ARM[0], 0.316, *LAUNCH_LEFT_ARM[2:],
)
NEUTRAL_RIGHT_ARM = (0.0, -0.25, 0.0, 0.60, 0.0, 0.0, 0.0)
OPEN_RIGHT_GRIPPER = (-0.019, -0.019)
Q90_MIDDLE_SEAM_TORSO_RAD = math.radians(-2.7349115403864994)
Q90_MIDDLE_SEAM_RIGHT_ARM = (
    -0.41373146194762, 0.23755434038212944, 1.1008080707922079,
    0.8747783439736734, -0.3046852067271319, 0.4094159191441438,
    -0.9999635680199944,
)
Q90_MIDDLE_SEAM_RIGHT_GRIPPER = (0.0108, 0.007)

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
    "right_dex1_finger_1_joint": (-0.020, 0.0245),
    "right_dex1_finger_2_joint": (-0.020, 0.0245),
    "left_dex1_finger_1_joint": (-0.020, 0.0245),
    "left_dex1_finger_2_joint": (-0.020, 0.0245),
}

PREFIX_STAGE_ORDER = (
    "prepare_left_park", "ingress_open_to_q0", "close_q0_right_hand",
    "contact_arc_q0_to_q90_hold",
)
SUFFIX_STAGE_ORDER = (
    "contact_arc_q90_hold_to_q0",
    "release_q0_right_hand", "egress_open_from_q0",
    "return_left_launch_seam",
)

AUDITED_PREFIX_STAGES = frozenset((
    "prepare_left_park", "ingress_open_to_q0", "close_q0",
    "contact_q0_to_q90_hold", "seam_start",
))
AUDITED_SUFFIX_STAGES = frozenset((
    "q90_continuous_grasp_seam", "contact_reverse_q90_to_q0",
    "release_q0", "egress_open_from_q0", "return_left",
))


class ExecutionError(RuntimeError):
    """Fail-closed validation or runtime error."""


@dataclass(frozen=True)
class SealedDocument:
    path: Path
    sha256: str
    value: Mapping[str, Any]


@dataclass(frozen=True)
class Limits:
    maximum_abs_torso_rad: float
    dense_angular_step_rad: float
    dense_prismatic_step_m: float
    start_tolerance_rad: float
    endpoint_tolerance_rad: float
    gripper_tolerance_m: float
    lead_time_s: float
    point_dt_s: float


@dataclass(frozen=True)
class ValidatedBundle:
    manifest: SealedDocument
    path: SealedDocument
    ingress: SealedDocument
    audit: SealedDocument
    scene_path: Path
    scene_sha256: str
    limits: Limits
    rows: Tuple[Mapping[str, Any], ...]
    ingress_waypoints: Tuple[Tuple[float, ...], ...]


@dataclass(frozen=True)
class Stage:
    name: str
    topics: Tuple[str, ...]
    states: Tuple[Mapping[str, float], ...]


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise ExecutionError(status)


def _exact_keys(value: Any, keys: Iterable[str], status: str) -> Mapping[str, Any]:
    _require(type(value) is dict, f"{status}_TYPE")
    expected = frozenset(keys)
    _require(frozenset(value) == expected, f"{status}_KEYS")
    return value


def _finite(value: Any, status: str) -> float:
    _require(type(value) in (int, float) and type(value) is not bool, status)
    result = float(value)
    _require(math.isfinite(result), status)
    return result


def _sha(value: Any, status: str) -> str:
    _require(type(value) is str and SHA256_RE.fullmatch(value) is not None, status)
    return value


def _reject_duplicate_keys(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ExecutionError(f"JSON_DUPLICATE_KEY:{key}")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ExecutionError(f"JSON_NONFINITE:{value}")


def _identity(metadata: os.stat_result) -> Tuple[int, int, int, int, int]:
    return (metadata.st_dev, metadata.st_ino, metadata.st_size,
            metadata.st_mtime_ns, metadata.st_ctime_ns)


def _read_sealed_bytes(path: Path, expected_sha256: str) -> bytes:
    _sha(expected_sha256, "EXPECTED_SHA256")
    path = Path(path)
    _require(path.is_absolute(), "SEALED_PATH_NOT_ABSOLUTE")
    try:
        path_metadata = path.lstat()
    except OSError as error:
        raise ExecutionError(f"SEALED_LSTAT:{error}") from error
    _require(stat.S_ISREG(path_metadata.st_mode), "SEALED_NOT_REGULAR")
    _require(not path.is_symlink(), "SEALED_SYMLINK")
    _require(path_metadata.st_uid == os.geteuid(), "SEALED_OWNER")
    _require(stat.S_IMODE(path_metadata.st_mode) == 0o400, "SEALED_MODE")
    _require(path_metadata.st_nlink == 1, "SEALED_NLINK")
    _require(0 < path_metadata.st_size <= MAX_PROTECTED_BYTES, "SEALED_SIZE")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise ExecutionError(f"SEALED_OPEN:{error}") from error
    try:
        before = os.fstat(descriptor)
        _require(_identity(before) == _identity(path_metadata), "SEALED_RACE_BEFORE")
        remaining = before.st_size
        digest = hashlib.sha256()
        chunks = []
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            _require(bool(block), "SEALED_SHORT_READ")
            chunks.append(block)
            digest.update(block)
            remaining -= len(block)
        _require(not os.read(descriptor, 1), "SEALED_GREW")
        after = os.fstat(descriptor)
        _require(_identity(before) == _identity(after), "SEALED_RACE_AFTER")
    finally:
        os.close(descriptor)
    actual = digest.hexdigest()
    _require(hmac.compare_digest(actual, expected_sha256), "SEALED_SHA256")
    return b"".join(chunks)


def _read_sealed_json(path: Path, expected_sha256: str) -> SealedDocument:
    raw = _read_sealed_bytes(path, expected_sha256)
    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ExecutionError(f"SEALED_JSON:{error}") from error
    _require(type(value) is dict, "SEALED_JSON_ROOT")
    return SealedDocument(Path(path), expected_sha256, value)


def _bound_path(workspace: Path, relative: Any, status: str) -> Path:
    _require(type(relative) is str and relative and not relative.startswith("/"), status)
    candidate = workspace / relative
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise ExecutionError(f"{status}:{error}") from error
    _require(resolved == candidate, f"{status}_ALIAS")
    _require(workspace == resolved or workspace in resolved.parents, f"{status}_ESCAPE")
    return resolved


def _vector(value: Any, width: int, status: str) -> Tuple[float, ...]:
    _require(type(value) is list and len(value) == width, status)
    return tuple(_finite(item, status) for item in value)


def _near(first: float, second: float, tolerance: float = 1e-9) -> bool:
    return abs(first - second) <= tolerance


def _validate_joint_vector(
    names: Sequence[str], positions: Sequence[float], status: str,
    torso_limit: float = HARD_MAX_TORSO_RAD,
) -> None:
    _require(len(names) == len(positions), f"{status}_WIDTH")
    for name, position in zip(names, positions):
        lower, upper = JOINT_LIMITS[name]
        if name == "torso_joint":
            lower, upper = -torso_limit, torso_limit
        _require(lower - 1e-12 <= position <= upper + 1e-12,
                 f"{status}_JOINT_LIMIT:{name}")


def _binding(value: Any, status: str) -> Tuple[str, str]:
    item = _exact_keys(value, ("path", "sha256"), status)
    return item["path"], _sha(item["sha256"], f"{status}_SHA")


def _validate_limits(value: Any) -> Limits:
    item = _exact_keys(value, (
        "maximum_abs_torso_rad", "dense_angular_step_rad",
        "dense_prismatic_step_m", "start_tolerance_rad",
        "endpoint_tolerance_rad", "gripper_tolerance_m", "lead_time_s",
        "point_dt_s",
    ), "LIMITS")
    torso = _finite(item["maximum_abs_torso_rad"], "LIMIT_TORSO")
    angular = _finite(item["dense_angular_step_rad"], "LIMIT_ANGULAR")
    prismatic = _finite(item["dense_prismatic_step_m"], "LIMIT_PRISMATIC")
    start = _finite(item["start_tolerance_rad"], "LIMIT_START")
    endpoint = _finite(item["endpoint_tolerance_rad"], "LIMIT_ENDPOINT")
    gripper = _finite(item["gripper_tolerance_m"], "LIMIT_GRIPPER")
    lead = _finite(item["lead_time_s"], "LIMIT_LEAD")
    point_dt = _finite(item["point_dt_s"], "LIMIT_POINT_DT")
    _require(0.0 < torso <= HARD_MAX_TORSO_RAD, "LIMIT_TORSO_HARD")
    _require(_near(torso, HARD_MAX_TORSO_RAD, 1e-15),
             "LIMIT_TORSO_CONTRACT")
    _require(0.0 < angular <= HARD_DENSE_ANGULAR_STEP_RAD + 1e-15,
             "LIMIT_ANGULAR_HARD")
    _require(0.0 < prismatic <= HARD_DENSE_PRISMATIC_STEP_M + 1e-15,
             "LIMIT_PRISMATIC_HARD")
    _require(0.0 < start <= HARD_START_TOLERANCE_RAD, "LIMIT_START_HARD")
    _require(0.0 < endpoint <= HARD_ENDPOINT_TOLERANCE_RAD,
             "LIMIT_ENDPOINT_HARD")
    _require(0.0 < gripper <= HARD_GRIPPER_TOLERANCE_M,
             "LIMIT_GRIPPER_HARD")
    _require(abs(lead - HARD_MIN_LEAD_TIME_S) <= 1e-15,
             "LIMIT_LEAD_HARD")
    _require(abs(point_dt - HARD_MIN_POINT_DT_S) <= 1e-15,
             "LIMIT_POINT_DT_HARD")
    return Limits(torso, angular, prismatic, start, endpoint, gripper,
                  lead, point_dt)


def _validate_path(
    document: SealedDocument, limits: Limits,
) -> Tuple[Mapping[str, Any], ...]:
    root = document.value
    _require(root.get("$schema") == PATH_SCHEMA, "PATH_SCHEMA")
    _require(document.sha256 == EXPECTED_PATH_SHA256, "PATH_REGISTRY_SHA")
    _require(root.get("schema_version") == 4, "PATH_VERSION")
    _require(root.get("execution_topology") == EXECUTION_TOPOLOGY,
             "PATH_TOPOLOGY")
    status = _exact_keys(root.get("status"), (
        "canonical_e0_q90_endpoint_present", "exact_q90_endpoint_present",
        "execution_ready", "fail_closed", "forward_dense_exact_complete",
        "forward_discrete_path_valid", "full_execution_topology_exact_complete",
        "gazebo_slow_contact_dynamics_complete", "ik_retarget_complete",
        "offline_kinematic_path_valid", "reason",
        "reverse_dense_exact_complete", "reverse_discrete_path_valid",
        "torso_variable",
    ), "PATH_STATUS")
    for key in (
        "exact_q90_endpoint_present", "canonical_e0_q90_endpoint_present",
        "torso_variable", "ik_retarget_complete", "fail_closed",
    ):
        _require(status.get(key) is True, f"PATH_STATUS:{key}")
    for key in (
        "execution_ready", "gazebo_slow_contact_dynamics_complete",
        "offline_kinematic_path_valid", "forward_discrete_path_valid",
        "reverse_discrete_path_valid", "forward_dense_exact_complete",
        "reverse_dense_exact_complete", "full_execution_topology_exact_complete",
    ):
        _require(status.get(key) is False, f"PATH_EXTERNAL_AUTHORITY:{key}")
    _require(type(status.get("reason")) is str and bool(status["reason"]),
             "PATH_STATUS_REASON")
    contract = _exact_keys(root.get("contract"), (
        "all_nonpad_handle_contact_forbidden", "authorized_load_contact",
        "handle_end_deg", "handle_start_deg", "left_hand_handle_contact_forbidden",
        "maximum_adjacent_joint_step_deg",
        "maximum_dense_contact_geometry_angular_step_deg",
        "maximum_dense_contact_geometry_linear_step_mm",
        "maximum_dense_slider_step_mm", "middle_initial", "middle_terminal",
        "minimum_arm_joint_limit_margin_deg",
        "opposite_right_pad_contact_permitted", "partial_path_forbidden",
        "prefix_terminal", "q90_regrasp_commands", "q90_release_commands",
        "suffix_initial", "suffix_release_only_after_q0", "torso_abs_limit_deg",
    ), "PATH_CONTRACT")
    _require(_near(_finite(contract["torso_abs_limit_deg"],
                           "PATH_CONTRACT_TORSO_DEG"), 44.5, 1e-12),
             "PATH_CONTRACT_TORSO_DEG")
    _require(contract["left_hand_handle_contact_forbidden"] is True and
             contract["all_nonpad_handle_contact_forbidden"] is True and
             contract["partial_path_forbidden"] is True,
             "PATH_CONTRACT_CONTACT_SCOPE")
    _require(contract["q90_release_commands"] == 0 and
             contract["q90_regrasp_commands"] == 0,
             "PATH_CONTRACT_Q90_CONTINUITY")
    metrics = _exact_keys(root.get("metrics"), (
        "maximum_abs_torso_deg", "maximum_adjacent_8dof_step_deg",
        "maximum_tcp_orientation_retarget_error_deg",
        "maximum_tcp_position_retarget_error_mm",
        "minimum_arm_margin_deg_discrete", "retargeted_row_count", "row_count",
    ), "PATH_METRICS")
    reported_peak_torso_rad = math.radians(_finite(
        metrics["maximum_abs_torso_deg"], "PATH_METRIC_TORSO_DEG"))
    _require(0.0 < reported_peak_torso_rad
             <= PLANNED_MAX_TORSO_RAD + 1e-12,
             "PATH_METRIC_TORSO_DEG")
    _require(type(metrics["row_count"]) is int and
             type(metrics["retargeted_row_count"]) is int and
             metrics["row_count"] >= 2 and
             0 <= metrics["retargeted_row_count"] <= metrics["row_count"],
             "PATH_METRIC_COUNTS")
    rows = root.get("rows")
    _require(type(rows) is list and len(rows) >= 2, "PATH_ROWS")
    _require(metrics["row_count"] == len(rows), "PATH_METRIC_ROW_COUNT")
    result = []
    previous_angle = -math.inf
    for index, row in enumerate(rows):
        _exact_keys(row, (
            "index", "handle_angle_deg", "contact_index", "action",
            "geometry_source", "phase_deg", "z_mm",
            "palm_down_tilt_deg", "side_tilt_deg", "outward_offset_mm",
            "local_x_offset_mm", "right_pad1_slider_mm",
            "right_pad2_slider_mm", "torso_yaw_deg", "right_arm_rad",
            "original_stl_gate",
        ), f"PATH_ROW:{index}")
        _require(row.get("index") == index, f"PATH_ROW_INDEX:{index}")
        _require(type(row.get("contact_index")) is int and
                 row["contact_index"] >= 0, f"PATH_ROW_CONTACT:{index}")
        _require(row.get("action") ==
                 "contact_arc_right_pad1_load_bearing",
                 f"PATH_ROW_ACTION:{index}")
        _require(type(row.get("geometry_source")) is str and
                 bool(row["geometry_source"]),
                 f"PATH_ROW_GEOMETRY_SOURCE:{index}")
        angle = _finite(row.get("handle_angle_deg"), f"PATH_ROW_ANGLE:{index}")
        _require(angle + 1e-12 >= previous_angle, f"PATH_ROW_MONOTONIC:{index}")
        torso = math.radians(
            _finite(row.get("torso_yaw_deg"), f"PATH_ROW_TORSO:{index}"))
        arm = _vector(row.get("right_arm_rad"), 7, f"PATH_ROW_ARM:{index}")
        for field in (
            "phase_deg", "z_mm", "palm_down_tilt_deg", "side_tilt_deg",
            "outward_offset_mm", "local_x_offset_mm",
        ):
            _finite(row.get(field), f"PATH_ROW_GEOMETRY:{index}:{field}")
        gate = _exact_keys(row.get("original_stl_gate"), (
            "pass_", "pad1_contact_authorized", "nonpad_contact_forbidden",
            "left_hand_handle_contact_forbidden",
        ), f"PATH_ROW_STL_GATE:{index}")
        _require(gate.get("pass_") is True and
                 gate.get("pad1_contact_authorized") is True,
                 f"PATH_ROW_STL_GATE:{index}")
        _require(gate.get("nonpad_contact_forbidden") is True and
                 gate.get("left_hand_handle_contact_forbidden") is True,
                 f"PATH_ROW_STL_SCOPE:{index}")
        fingers = (
            _finite(row.get("right_pad1_slider_mm"), f"PATH_ROW_Q1:{index}") / 1000.0,
            _finite(row.get("right_pad2_slider_mm"), f"PATH_ROW_Q2:{index}") / 1000.0,
        )
        _validate_joint_vector(("torso_joint",), (torso,),
                               f"PATH_ROW_TORSO:{index}", limits.maximum_abs_torso_rad)
        _validate_joint_vector(RIGHT_ARM_JOINTS, arm, f"PATH_ROW_ARM:{index}")
        _validate_joint_vector(RIGHT_GRIPPER_JOINTS, fingers,
                               f"PATH_ROW_GRIPPER:{index}")
        result.append({
            "handle_angle_deg": angle, "torso_joint": torso,
            "right_arm": arm, "right_gripper": fingers,
        })
        previous_angle = angle
    _require(_near(result[0]["handle_angle_deg"], 0.0, 1e-9), "PATH_Q0")
    _require(_near(result[-1]["handle_angle_deg"], 90.0, 1e-9), "PATH_Q90")
    q90 = result[-1]
    _require(_near(q90["torso_joint"], Q90_MIDDLE_SEAM_TORSO_RAD, 1e-12),
             "PATH_Q90_SEAM_TORSO")
    for index, (actual, expected) in enumerate(zip(
            q90["right_arm"], Q90_MIDDLE_SEAM_RIGHT_ARM)):
        _require(_near(actual, expected, 1e-12),
                 f"PATH_Q90_SEAM_ARM:{index}")
    for index, (actual, expected) in enumerate(zip(
            q90["right_gripper"], Q90_MIDDLE_SEAM_RIGHT_GRIPPER)):
        _require(_near(actual, expected, 1e-12),
                 f"PATH_Q90_SEAM_GRIPPER:{index}")
    torso_values = tuple(row["torso_joint"] for row in result)
    _require(max(abs(value) for value in torso_values)
             <= PLANNED_MAX_TORSO_RAD + 1e-12,
             "PATH_PLANNED_TORSO_MARGIN")
    _require(_near(max(abs(value) for value in torso_values),
                   reported_peak_torso_rad, 1e-12),
             "PATH_REPORTED_TORSO_PEAK")
    _require(max(torso_values) - min(torso_values) > 1e-6,
             "PATH_VARIABLE_TORSO_REQUIRED")
    return tuple(result)


def _validate_ingress(
    document: SealedDocument, path: SealedDocument, scene_sha256: str,
    rows: Sequence[Mapping[str, Any]], limits: Limits,
) -> Tuple[Tuple[float, ...], ...]:
    value = document.value
    _require(value.get("schema") == INGRESS_SCHEMA, "INGRESS_SCHEMA")
    _require(document.sha256 == EXPECTED_INGRESS_SHA256,
             "INGRESS_REGISTRY_SHA")
    _require(value.get("valid") is True and value.get("status") == "PASS",
             "INGRESS_NOT_PASS")
    _require(value.get("controller_commands_issued") == 0, "INGRESS_COMMANDS")
    _require(value.get("offline_only") is True, "INGRESS_OFFLINE")
    _require(value.get("path_sha256") ==
             EXPECTED_INGRESS_SOURCE_PATH_SHA256,
             "INGRESS_SOURCE_PATH_SHA")
    _require(value.get("scene_cdr_sha256") == scene_sha256,
             "INGRESS_SCENE_SHA")
    _require(_near(_finite(value.get("handle_angle_deg"), "INGRESS_HANDLE"),
                   0.0, 1e-12), "INGRESS_HANDLE")
    configuration = ("torso_joint", *RIGHT_ARM_JOINTS)
    _require(tuple(value.get("configuration_order", ())) == configuration,
             "INGRESS_CONFIGURATION_ORDER")
    fingers = _exact_keys(value.get("right_fingers_rad_or_m"),
                          RIGHT_GRIPPER_JOINTS,
                          "INGRESS_OPEN_FINGERS")
    for name in RIGHT_GRIPPER_JOINTS:
        _require(_near(_finite(fingers[name], "INGRESS_OPEN_FINGER"),
                       OPEN_RIGHT_GRIPPER[RIGHT_GRIPPER_JOINTS.index(name)],
                       1e-12), f"INGRESS_OPEN_FINGER:{name}")
    _require(_finite(value.get("strict_revalidation_step_rad"),
                     "INGRESS_REVALIDATION_STEP")
             <= limits.dense_angular_step_rad + 1e-15,
             "INGRESS_REVALIDATION_STEP")
    waypoints = value.get("waypoints")
    _require(type(waypoints) is list and len(waypoints) >= 2,
             "INGRESS_WAYPOINTS")
    _require(value.get("waypoint_count") == len(waypoints),
             "INGRESS_WAYPOINT_COUNT")
    output = []
    for index, waypoint in enumerate(waypoints):
        vector = _vector(waypoint, 8, f"INGRESS_WAYPOINT:{index}")
        _validate_joint_vector(configuration, vector,
                               f"INGRESS_WAYPOINT:{index}",
                               limits.maximum_abs_torso_rad)
        output.append(vector)
    neutral = (0.0, *NEUTRAL_RIGHT_ARM)
    for index, (actual, expected) in enumerate(zip(output[0], neutral)):
        _require(abs(actual - expected) <= limits.start_tolerance_rad,
                 f"INGRESS_LAUNCH_SEAM:{index}")
    q0 = rows[0]
    q0_vector = (q0["torso_joint"], *q0["right_arm"])
    for index, (actual, expected) in enumerate(zip(output[-1], q0_vector)):
        _require(_near(actual, expected, 1e-12),
                 f"INGRESS_Q0_SEAM:{index}")
    goal = _vector(value.get("goal_right_arm_rad"), 7, "INGRESS_GOAL_ARM")
    for index, (actual, expected) in enumerate(zip(goal, output[-1][1:])):
        _require(_near(actual, expected, 1e-12),
                 f"INGRESS_GOAL_ARM:{index}")
    return tuple(output)


def _validate_audit(
    audit: SealedDocument, path: SealedDocument, ingress: SealedDocument,
    scene_sha256: str, limits: Limits, expected_auditor: str,
) -> Mapping[str, Tuple[float, ...]]:
    value = audit.value
    _require(value.get("schema") == AUDIT_SCHEMA, "AUDIT_SCHEMA")
    _require(audit.sha256 == EXPECTED_AUDIT_SHA256, "AUDIT_REGISTRY_SHA")
    _require(expected_auditor == EXPECTED_AUDITOR,
             "MANIFEST_AUDITOR_REGISTRY")
    _require(value.get("auditor_version") == EXPECTED_AUDITOR,
             "AUDIT_VERSION")
    _require(value.get("execution_topology") == EXECUTION_TOPOLOGY,
             "AUDIT_TOPOLOGY")
    _require(value.get("valid") is True and value.get("status") == "PASS",
             "AUDIT_NOT_PASS")
    _require(value.get("controller_commands_issued") == 0, "AUDIT_COMMANDS")
    _require(value.get("offline_only") is True, "AUDIT_OFFLINE")
    _require(value.get("path_sha256") == path.sha256, "AUDIT_PATH_SHA")
    _require(value.get("q0_ingress_asset_sha256") == ingress.sha256,
             "AUDIT_INGRESS_SHA")
    _require(value.get("q0_ingress_waypoint_count") ==
             len(ingress.value.get("waypoints", ())),
             "AUDIT_INGRESS_WAYPOINT_COUNT")
    _require(value.get("raw_scene_cdr_sha256") == scene_sha256,
             "AUDIT_SCENE_SHA")
    for key in (
        "forward_passed", "reverse_passed", "prefix_passed",
        "suffix_passed", "execution_splices_included",
        "q90_continuous_grasp_seam_passed",
        "q0_ingress_forward_passed",
        "q0_ingress_exact_reverse_egress_passed", "variable_torso_path",
        "collision_free", "self_collision_checked",
        "full_world_and_table_guard_checked",
    ):
        _require(value.get(key) is True, f"AUDIT_GATE:{key}")
    _require(value.get("original_handle_stl_sha256") == EXPECTED_HANDLE_STL_SHA256,
             "AUDIT_HANDLE_STL")
    _require(_finite(value.get("minimum_nonpad_handle_clearance_m"),
                     "AUDIT_NONPAD") + 1e-12 >= 0.0006, "AUDIT_NONPAD")
    _require(_finite(value.get("minimum_left_handle_clearance_m"),
                     "AUDIT_LEFT_HANDLE") + 1e-12 >= 0.010,
             "AUDIT_LEFT_HANDLE")
    _require(_finite(value.get("maximum_abs_torso_rad"), "AUDIT_TORSO")
             <= limits.maximum_abs_torso_rad + 1e-12, "AUDIT_TORSO")
    _require(_finite(value.get("maximum_abs_torso_rad"), "AUDIT_TORSO")
             <= PLANNED_MAX_TORSO_RAD + 1e-12, "AUDIT_PLANNED_TORSO")
    _require(_near(_finite(value.get("torso_abs_contract_rad"),
                           "AUDIT_TORSO_CONTRACT"),
                   HARD_MAX_TORSO_RAD, 1e-12),
             "AUDIT_TORSO_CONTRACT")
    dense = _exact_keys(value.get("dense_limits"), ("angular_rad", "prismatic_m"),
                        "AUDIT_DENSE")
    _require(_finite(dense["angular_rad"], "AUDIT_DENSE_ANGULAR")
             <= limits.dense_angular_step_rad + 1e-15,
             "AUDIT_DENSE_ANGULAR")
    _require(_finite(dense["prismatic_m"], "AUDIT_DENSE_PRISMATIC")
             <= limits.dense_prismatic_step_m + 1e-15,
             "AUDIT_DENSE_PRISMATIC")
    pairs = value.get("left_handle_allowed_contact_pairs")
    _require(pairs == [], "AUDIT_LEFT_HANDLE_CONTACT")
    allowed = value.get("authorized_handle_contact_pairs")
    _require(allowed == [
        ["incubator_handle_exact", "right_dex1_finger_1_pad_link"],
        ["incubator_handle_exact", "right_dex1_finger_2_pad_link"],
    ], "AUDIT_RIGHT_HANDLE_CONTACT")
    _require(value.get("q90_release_commands") == 0, "AUDIT_Q90_RELEASE")
    _require(value.get("q90_regrasp_commands") == 0, "AUDIT_Q90_REGRASP")
    contract = _exact_keys(value.get("execution_contract"), (
        "prefix_stages", "suffix_stages", "q90_release_forbidden",
        "q90_regrasp_forbidden", "q0_ingress_fingers_open_m",
    ), "AUDIT_EXECUTION_CONTRACT")
    _require(tuple(contract["prefix_stages"]) == (
        "prepare_left_park", "ingress_open_to_q0", "close_q0",
        "contact_q0_to_q90_hold",
    ), "AUDIT_PREFIX_CONTRACT")
    _require(tuple(contract["suffix_stages"]) == (
        "q90_continuous_grasp_seam", "contact_reverse_q90_to_q0",
        "release_q0", "egress_open_from_q0", "return_left",
    ), "AUDIT_SUFFIX_CONTRACT")
    _require(contract["q90_release_forbidden"] is True and
             contract["q90_regrasp_forbidden"] is True,
             "AUDIT_Q90_FORBIDDEN_CONTRACT")
    _require(_near(_finite(contract["q0_ingress_fingers_open_m"],
                          "AUDIT_Q0_OPEN"), -0.019, 1e-12),
             "AUDIT_Q0_OPEN")
    directions = value.get("directions")
    _require(type(directions) is list and len(directions) == 2, "AUDIT_DIRECTIONS")
    by_name = {item.get("name"): item for item in directions if type(item) is dict}
    _require(frozenset(by_name) == frozenset(("prefix", "suffix")),
             "AUDIT_DIRECTIONS")
    for direction, required_stages in (
        ("prefix", AUDITED_PREFIX_STAGES),
        ("suffix", AUDITED_SUFFIX_STAGES),
    ):
        metrics = by_name[direction].get("stage_samples")
        _require(type(metrics) is dict, f"AUDIT_STAGES:{direction}")
        _require(required_stages.issubset(metrics),
                 f"AUDIT_STAGES:{direction}")
        for stage in required_stages:
            _require(type(metrics[stage]) is int and metrics[stage] > 0,
                     f"AUDIT_STAGE_EMPTY:{direction}:{stage}")
    launch_roll = _finite(
        value.get("launch_left_shoulder_roll_rad"), "AUDIT_LAUNCH_ROLL")
    _require(_near(launch_roll, LAUNCH_LEFT_ARM[1], 1e-12),
             "AUDIT_LAUNCH_ROLL")
    park_roll = _finite(value.get("park_left_shoulder_roll_rad"), "AUDIT_PARK_ROLL")
    _require(_near(park_roll, PARK_LEFT_ARM[1], 1e-12),
             "AUDIT_PARK_ROLL")
    _require(_near(
        park_roll,
        _finite(ingress.value.get("park_left_shoulder_roll_rad"),
                "INGRESS_PARK_ROLL"),
        1e-12), "AUDIT_INGRESS_PARK_ROLL")
    return {
        "launch_left_arm": LAUNCH_LEFT_ARM,
        "park_left_arm": PARK_LEFT_ARM,
    }


def validate_bundle(
    workspace: Path, manifest_path: Path, manifest_sha256: str,
) -> ValidatedBundle:
    """Validate all sealed inputs without importing ROS."""
    workspace = Path(workspace)
    _require(workspace.is_absolute(), "WORKSPACE_NOT_ABSOLUTE")
    try:
        workspace = workspace.resolve(strict=True)
    except OSError as error:
        raise ExecutionError(f"WORKSPACE:{error}") from error
    _require(workspace.is_dir(), "WORKSPACE_NOT_DIRECTORY")
    manifest = _read_sealed_json(Path(manifest_path), manifest_sha256)
    root = manifest.value
    # Controller authorization is intentionally distinct from a post-run
    # completion claim.  This gate permits the first slow Gazebo run without
    # circularly claiming that live execution already passed.
    _require(root.get("controller_execution_authorized") is True,
             "MANIFEST_NOT_CONTROLLER_AUTHORIZED")
    _exact_keys(root, (
        "schema", "controller_execution_authorized", "execution_ready",
        "live_execution_passed", "live_result",
        "controller_commands_issued", "offline_only", "bindings",
        "controller_topics", "stage_order", "limits", "auditor_version",
        "execution_topology",
    ), "MANIFEST")
    _require(root.get("schema") == MANIFEST_SCHEMA, "MANIFEST_SCHEMA")
    _require(root.get("execution_ready") is False and
             root.get("live_execution_passed") is False and
             root.get("live_result") is None,
             "MANIFEST_PRE_RUN_STATE")
    _require(root.get("controller_commands_issued") == 0, "MANIFEST_COMMANDS")
    _require(root.get("offline_only") is True, "MANIFEST_OFFLINE")
    auditor_version = root.get("auditor_version")
    _require(auditor_version == EXPECTED_AUDITOR,
             "MANIFEST_AUDITOR_VERSION")
    _require(root.get("execution_topology") == EXECUTION_TOPOLOGY,
             "MANIFEST_TOPOLOGY")
    _require(tuple(root.get("controller_topics", ())) == CANONICAL_CONTROLLER_TOPICS,
             "MANIFEST_CONTROLLER_TOPICS")
    stage_order = _exact_keys(root.get("stage_order"), ("prefix", "suffix"),
                              "MANIFEST_STAGE_ORDER")
    _require(tuple(stage_order["prefix"]) == PREFIX_STAGE_ORDER,
             "MANIFEST_PREFIX_ORDER")
    _require(tuple(stage_order["suffix"]) == SUFFIX_STAGE_ORDER,
             "MANIFEST_SUFFIX_ORDER")
    limits = _validate_limits(root.get("limits"))
    bindings = _exact_keys(root.get("bindings"), (
        "contact_path", "ingress_path", "exact_audit", "scene_snapshot",
    ), "MANIFEST_BINDINGS")
    loaded = {}
    for name in ("contact_path", "ingress_path", "exact_audit", "scene_snapshot"):
        relative, digest = _binding(bindings[name], f"BINDING:{name}")
        loaded[name] = (_bound_path(workspace, relative, f"BINDING_PATH:{name}"), digest)
    _require(loaded["contact_path"][1] == EXPECTED_PATH_SHA256,
             "BINDING_PATH_REGISTRY_SHA")
    _require(loaded["ingress_path"][1] == EXPECTED_INGRESS_SHA256,
             "BINDING_INGRESS_REGISTRY_SHA")
    _require(loaded["exact_audit"][1] == EXPECTED_AUDIT_SHA256,
             "BINDING_AUDIT_REGISTRY_SHA")
    path = _read_sealed_json(*loaded["contact_path"])
    ingress = _read_sealed_json(*loaded["ingress_path"])
    audit = _read_sealed_json(*loaded["exact_audit"])
    scene_path, scene_sha256 = loaded["scene_snapshot"]
    _read_sealed_bytes(scene_path, scene_sha256)
    rows = _validate_path(path, limits)
    ingress_waypoints = _validate_ingress(
        ingress, path, scene_sha256, rows, limits)
    _validate_audit(
        audit, path, ingress, scene_sha256, limits, auditor_version)
    return ValidatedBundle(manifest, path, ingress, audit, scene_path,
                           scene_sha256, limits, rows, ingress_waypoints)


def _base_state(park_left_arm: Sequence[float]) -> Dict[str, float]:
    state = {name: value for name, value in zip(LEFT_ARM_JOINTS, park_left_arm)}
    state.update({name: value for name, value in zip(RIGHT_ARM_JOINTS,
                                                     NEUTRAL_RIGHT_ARM)})
    state.update({name: value for name, value in zip(RIGHT_GRIPPER_JOINTS,
                                                     OPEN_RIGHT_GRIPPER)})
    state.update({name: -0.019 for name in LEFT_GRIPPER_JOINTS})
    state["torso_joint"] = 0.0
    return state


def _changed(
    state: Mapping[str, float], names: Sequence[str], values: Sequence[float],
) -> Dict[str, float]:
    result = dict(state)
    result.update(zip(names, values))
    return result


def _contact_state(base: Mapping[str, float], row: Mapping[str, Any],
                   open_gripper: bool = False) -> Dict[str, float]:
    result = _changed(base, ("torso_joint",), (row["torso_joint"],))
    result = _changed(result, RIGHT_ARM_JOINTS, row["right_arm"])
    fingers = OPEN_RIGHT_GRIPPER if open_gripper else row["right_gripper"]
    return _changed(result, RIGHT_GRIPPER_JOINTS, fingers)


def build_stages(bundle: ValidatedBundle, direction: str) -> Tuple[Stage, ...]:
    """Reconstruct the continuous-grasp topology without waypoint injection."""
    _require(direction in ("prefix", "suffix"), "DIRECTION")
    audit = bundle.audit.value
    _finite(audit["park_left_shoulder_roll_rad"], "PARK_ROLL")
    _finite(audit["launch_left_shoulder_roll_rad"], "LAUNCH_ROLL")
    park_left = PARK_LEFT_ARM
    park = _base_state(park_left)
    launch_left = LAUNCH_LEFT_ARM
    launch = _changed(park, LEFT_ARM_JOINTS, launch_left)
    q0_row = bundle.rows[0]
    ingress_states = []
    for waypoint in bundle.ingress_waypoints:
        state = _changed(park, ("torso_joint",), waypoint[:1])
        state = _changed(state, RIGHT_ARM_JOINTS, waypoint[1:])
        ingress_states.append(state)
    q0_closed = _contact_state(park, q0_row)
    if direction == "prefix":
        contact_states = tuple(_contact_state(park, row) for row in bundle.rows)
        stages = [
            Stage("prepare_left_park",
                  ("/left_arm_controller/joint_trajectory",), (launch, park)),
            Stage("ingress_open_to_q0", (
                "/torso_controller/joint_trajectory",
                "/right_arm_controller/joint_trajectory",
            ), tuple(ingress_states)),
            Stage("close_q0_right_hand",
                  ("/right_gripper_controller/joint_trajectory",),
                  (ingress_states[-1], q0_closed)),
            Stage("contact_arc_q0_to_q90_hold", (
                "/torso_controller/joint_trajectory",
                "/right_arm_controller/joint_trajectory",
                "/right_gripper_controller/joint_trajectory",
            ), contact_states),
        ]
    else:
        contact_states = tuple(
            _contact_state(park, row) for row in reversed(bundle.rows))
        q0_released = _contact_state(park, q0_row, open_gripper=True)
        stages = [
            Stage("contact_arc_q90_hold_to_q0", (
                "/torso_controller/joint_trajectory",
                "/right_arm_controller/joint_trajectory",
                "/right_gripper_controller/joint_trajectory",
            ), contact_states),
            Stage("release_q0_right_hand",
                  ("/right_gripper_controller/joint_trajectory",),
                  (contact_states[-1], q0_released)),
            Stage("egress_open_from_q0", (
                "/torso_controller/joint_trajectory",
                "/right_arm_controller/joint_trajectory",
            ), tuple(reversed(ingress_states))),
            Stage("return_left_launch_seam",
                  ("/left_arm_controller/joint_trajectory",), (park, launch)),
        ]
    expected = PREFIX_STAGE_ORDER if direction == "prefix" else SUFFIX_STAGE_ORDER
    _require(tuple(stage.name for stage in stages) == expected,
             "INTERNAL_STAGE_ORDER")
    return tuple(stages)


def _is_prismatic(name: str) -> bool:
    return "finger" in name


def densify_stage(stage: Stage, limits: Limits) -> Tuple[Mapping[str, float], ...]:
    controlled = tuple(name for topic in stage.topics for name in CONTROLLER_JOINTS[topic])
    _require(len(stage.states) >= 2, f"STAGE_TOO_SHORT:{stage.name}")
    output = [dict(stage.states[0])]
    for first, second in zip(stage.states, stage.states[1:]):
        subdivisions = 1
        for name in controlled:
            step = (limits.dense_prismatic_step_m if _is_prismatic(name)
                    else limits.dense_angular_step_rad)
            subdivisions = max(subdivisions,
                               int(math.ceil(abs(second[name] - first[name]) / step)))
        _require(len(output) + subdivisions <= MAX_DENSE_POINTS_PER_STAGE,
                 f"STAGE_DENSE_SIZE:{stage.name}")
        for index in range(1, subdivisions + 1):
            fraction = index / subdivisions
            sample = dict(first)
            for name in controlled:
                sample[name] = first[name] + (second[name] - first[name]) * fraction
            output.append(sample)
    for first, second in zip(output, output[1:]):
        for name in controlled:
            maximum = (limits.dense_prismatic_step_m if _is_prismatic(name)
                       else limits.dense_angular_step_rad)
            _require(abs(second[name] - first[name]) <= maximum + 1e-12,
                     f"STAGE_DENSE_STEP:{stage.name}:{name}")
    return tuple(output)


def _pchip_velocities(
    states: Sequence[Mapping[str, float]], names: Sequence[str], dt_s: float,
) -> Tuple[Mapping[str, float], ...]:
    """Return shape-preserving C1 velocities without changing a waypoint."""
    _require(len(states) >= 2 and math.isfinite(dt_s) and dt_s > 0.0,
             "FAST_PROFILE_INPUT")
    result = [{name: 0.0 for name in names} for _ in states]
    for name in names:
        slopes = [
            (states[index + 1][name] - states[index][name]) / dt_s
            for index in range(len(states) - 1)
        ]
        for index in range(1, len(states) - 1):
            before, after = slopes[index - 1], slopes[index]
            if before * after > 0.0:
                result[index][name] = 2.0 * before * after / (before + after)
    return tuple(result)


def _validate_fast_cubic_profile(
    states: Sequence[Mapping[str, float]],
    velocities: Sequence[Mapping[str, float]],
    names: Sequence[str], dt_s: float, stage_name: str,
) -> None:
    """Analytically bound every cubic Hermite segment before publication."""
    _require(len(states) == len(velocities),
             f"FAST_PROFILE_LENGTH:{stage_name}")
    for segment, (first, second, first_v, second_v) in enumerate(zip(
            states, states[1:], velocities, velocities[1:])):
        for name in names:
            q0, q1 = first[name], second[name]
            v0, v1 = first_v[name], second_v[name]
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
            if _is_prismatic(name):
                velocity_limit = HARD_FAST_MAX_PRISMATIC_VELOCITY_M_S
                acceleration_limit = HARD_FAST_MAX_PRISMATIC_ACCELERATION_M_S2
            else:
                velocity_limit = HARD_FAST_MAX_REVOLUTE_VELOCITY_RAD_S
                acceleration_limit = HARD_FAST_MAX_REVOLUTE_ACCELERATION_RAD_S2
            _require(maximum_velocity <= velocity_limit + 1e-12,
                     f"FAST_VELOCITY:{stage_name}:{segment}:{name}")
            _require(maximum_acceleration <= acceleration_limit + 1e-9,
                     f"FAST_ACCELERATION:{stage_name}:{segment}:{name}")


class _JointFeedback:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._positions: Dict[str, float] = {}
        self._receipt_ns: Dict[str, int] = {}
        self._torso_violation: float | None = None

    def update(self, message: Any) -> None:
        if len(message.name) != len(message.position):
            return
        values = {}
        for name, position in zip(message.name, message.position):
            if not math.isfinite(position):
                if name == "torso_joint":
                    with self._condition:
                        self._torso_violation = float(position)
                        self._condition.notify_all()
                return
            values[name] = float(position)
        with self._condition:
            receipt_ns = time.monotonic_ns()
            self._positions.update(values)
            for name in values:
                self._receipt_ns[name] = receipt_ns
            torso = values.get("torso_joint")
            if (torso is not None and
                    abs(torso) > (HARD_MAX_TORSO_RAD +
                                  RUNTIME_TORSO_ABS_TOL_RAD)):
                self._torso_violation = torso
            self._condition.notify_all()

    def require_torso_safe(self) -> None:
        with self._condition:
            if self._torso_violation is not None:
                raise ExecutionError(
                    f"RUNTIME_TORSO_LIMIT:{self._torso_violation}")

    def snapshot(self, names: Sequence[str], newer_than_ns: int) -> Dict[str, float] | None:
        with self._condition:
            if any(name not in self._positions or
                   self._receipt_ns.get(name, 0) <= newer_than_ns
                   for name in names):
                return None
            return {name: self._positions[name] for name in names}

    def latest(self, names: Sequence[str]) -> Dict[str, float] | None:
        with self._condition:
            if any(name not in self._positions for name in names):
                return None
            return {name: self._positions[name] for name in names}


def _wait_feedback(executor: Any, feedback: _JointFeedback, names: Sequence[str],
                   newer_than_ns: int, timeout_s: float, *,
                   allow_unsafe_torso: bool = False) -> Dict[str, float]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not allow_unsafe_torso:
            feedback.require_torso_safe()
        values = feedback.snapshot(names, newer_than_ns)
        if values is not None:
            return values
        executor.spin_once(timeout_sec=0.02)
    raise ExecutionError("JOINT_FEEDBACK_TIMEOUT")


def _feedback_tolerance(name: str, limits: Limits, *, start: bool = False) -> float:
    if _is_prismatic(name):
        return limits.gripper_tolerance_m
    return (limits.start_tolerance_rad if start
            else limits.endpoint_tolerance_rad)


def _feedback_is_close(
    measured: Mapping[str, float], expected: Mapping[str, float],
    names: Sequence[str], limits: Limits, *, start: bool = False,
) -> bool:
    return all(
        abs(measured[name] - expected[name]) <=
        _feedback_tolerance(name, limits, start=start)
        for name in names
    )


def _require_feedback_close(
    measured: Mapping[str, float], expected: Mapping[str, float],
    names: Sequence[str], limits: Limits, status: str, *, start: bool = False,
) -> None:
    for name in names:
        tolerance = _feedback_tolerance(name, limits, start=start)
        _require(abs(measured[name] - expected[name]) <= tolerance,
                 f"{status}:{name}")


def _endpoint_failure(
    stage_name: str, measured: Mapping[str, float] | None,
    expected: Mapping[str, float], names: Sequence[str], limits: Limits,
    consecutive: int, fresh_samples: int,
) -> ExecutionError:
    if fresh_samples == 0:
        reason = "NO_POST_TARGET_FRESH_FEEDBACK"
    elif (measured is not None and
          not _feedback_is_close(measured, expected, names, limits)):
        reason = "OUT_OF_TOLERANCE"
    else:
        reason = "INSUFFICIENT_CONSECUTIVE_SAMPLES"
    if measured is None:
        worst_name = names[0]
        measured_value = math.nan
        error = math.inf
    else:
        # Normalize by each joint's certified tolerance so angular and
        # prismatic joints are compared on the same fail-closed scale.
        worst_name = max(
            names,
            key=lambda name: (
                abs(measured[name] - expected[name]) /
                _feedback_tolerance(name, limits)),
        )
        measured_value = measured[worst_name]
        error = abs(measured_value - expected[worst_name])
    tolerance = _feedback_tolerance(worst_name, limits)
    return ExecutionError(
        f"STAGE_ENDPOINT:{stage_name}:reason={reason}:"
        f"fresh_samples={fresh_samples}:worst_joint={worst_name}:"
        f"measured={measured_value:.17g}:"
        f"expected={expected[worst_name]:.17g}:error={error:.17g}:"
        f"tolerance={tolerance:.17g}:consecutive={consecutive}/"
        f"{HARD_ENDPOINT_CONSECUTIVE_SAMPLES}")


def _duration_message(seconds: float) -> Tuple[int, int]:
    nanoseconds = int(round(seconds * 1_000_000_000))
    return nanoseconds // 1_000_000_000, nanoseconds % 1_000_000_000


def check_q90_seam(
    workspace: Path, manifest_path: Path, manifest_sha256: str,
    discovery_timeout: float = 10.0,
) -> None:
    """Read fresh sim-time feedback and prove the canonical E0 held seam."""
    bundle = validate_bundle(workspace, manifest_path, manifest_sha256)
    _require(math.isfinite(discovery_timeout) and
             0.0 < discovery_timeout <= HARD_MAX_DISCOVERY_TIMEOUT_S,
             "DISCOVERY_TIMEOUT")
    # As with execution, ROS is imported only after the complete sealed bundle
    # and canonical E0 endpoint have passed offline validation.
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    node = Node(
        f"h1_right_handle_e0_checker_828_{os.getpid()}",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        automatically_declare_parameters_from_overrides=True,
        context=context,
    )
    executor.add_node(node)
    feedback = _JointFeedback()
    subscription = node.create_subscription(
        JointState, "/joint_states", feedback.update, 10)
    try:
        clock_deadline = time.monotonic() + discovery_timeout
        first_clock = 0
        while time.monotonic() < clock_deadline:
            executor.spin_once(timeout_sec=0.02)
            feedback.require_torso_safe()
            now = node.get_clock().now().nanoseconds
            if bool(getattr(node.get_clock(), "ros_time_is_active", False)) and now > 0:
                if first_clock == 0:
                    first_clock = now
                elif now > first_clock:
                    break
        else:
            raise ExecutionError("SIM_CLOCK_NOT_ADVANCING")
        names = ("torso_joint", *RIGHT_ARM_JOINTS, *RIGHT_GRIPPER_JOINTS)
        request_ns = time.monotonic_ns()
        measured = _wait_feedback(
            executor, feedback, names, request_ns, discovery_timeout)
        expected = _contact_state(
            _base_state((0.0, 0.316, 0.0, 0.60, 0.0, 0.0, 0.0)),
            bundle.rows[-1])
        _require_feedback_close(
            measured, expected, names, bundle.limits,
            "Q90_CANONICAL_E0_FEEDBACK", start=True)
        print("RIGHT_HANDLE_EXECUTOR_828_Q90_E0=PASS fresh_feedback=true "
              "controller_commands=0", flush=True)
    finally:
        node.destroy_subscription(subscription)
        executor.remove_node(node)
        try:
            executor.shutdown()
        finally:
            try:
                node.destroy_node()
            finally:
                rclpy.shutdown(context=context)


def check_launch_seam(
    workspace: Path, manifest_path: Path, manifest_sha256: str,
    discovery_timeout: float = 10.0,
) -> None:
    """Prove the full suffix terminal state using fresh sim-time feedback."""
    bundle = validate_bundle(workspace, manifest_path, manifest_sha256)
    _require(math.isfinite(discovery_timeout) and
             0.0 < discovery_timeout <= HARD_MAX_DISCOVERY_TIMEOUT_S,
             "DISCOVERY_TIMEOUT")
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState

    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    node = Node(
        f"h1_right_handle_launch_checker_828_{os.getpid()}",
        parameter_overrides=[Parameter(
            "use_sim_time", Parameter.Type.BOOL, True)],
        automatically_declare_parameters_from_overrides=True,
        context=context,
    )
    executor.add_node(node)
    feedback = _JointFeedback()
    subscription = node.create_subscription(
        JointState, "/joint_states", feedback.update, 10)
    try:
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
            raise ExecutionError("SIM_CLOCK_NOT_ADVANCING")
        ingress_launch = bundle.ingress_waypoints[0]
        expected = {
            "torso_joint": ingress_launch[0],
            **dict(zip(LEFT_ARM_JOINTS, LAUNCH_LEFT_ARM)),
            **dict(zip(RIGHT_ARM_JOINTS, ingress_launch[1:])),
            **dict(zip(LEFT_GRIPPER_JOINTS, OPEN_RIGHT_GRIPPER)),
            **dict(zip(RIGHT_GRIPPER_JOINTS, OPEN_RIGHT_GRIPPER)),
        }
        names = tuple(expected)
        request_ns = time.monotonic_ns()
        measured = _wait_feedback(
            executor, feedback, names, request_ns, discovery_timeout)
        _require_feedback_close(
            measured, expected, names, bundle.limits,
            "SUFFIX_TERMINAL_LAUNCH_FEEDBACK", start=True)
        print(
            "RIGHT_HANDLE_EXECUTOR_828_LAUNCH_SEAM=PASS "
            "fresh_feedback=true q0_released=true reverse_ingress=true "
            "left_launch=true torso_launch=true controller_commands=0",
            flush=True)
    finally:
        node.destroy_subscription(subscription)
        executor.remove_node(node)
        try:
            executor.shutdown()
        finally:
            try:
                node.destroy_node()
            finally:
                rclpy.shutdown(context=context)


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


def _settle_stage_endpoint(
    executor: Any, feedback: _JointFeedback, node: Any,
    names: Sequence[str], expected: Mapping[str, float], limits: Limits,
    stage_name: str, settle_start_sim_ns: int, wall_deadline: float,
) -> None:
    # This gate is deliberately created only after sim_target was observed.
    # Advancing it after every accepted snapshot also makes the three samples
    # distinct; a cached JointState can never satisfy more than one sample.
    freshness_ns = time.monotonic_ns()
    settle_deadline_sim_ns = settle_start_sim_ns + int(
        HARD_ENDPOINT_SETTLE_SIM_S * 1e9)
    last_clock = settle_start_sim_ns
    last_clock_wall = time.monotonic()
    consecutive = 0
    fresh_samples = 0
    # Diagnostics must never represent a pre-target cache entry as a measured
    # endpoint.  Only snapshots admitted through freshness_ns populate this.
    last_measured = None
    while True:
        if time.monotonic() >= wall_deadline:
            raise ExecutionError(f"STAGE_SIM_TIMEOUT:{stage_name}")
        executor.spin_once(timeout_sec=0.02)
        feedback.require_torso_safe()
        _require_unique_controller_publishers(
            node, f"STAGE_SETTLE:{stage_name}")
        wall_now = time.monotonic()
        if wall_now >= wall_deadline:
            raise ExecutionError(f"STAGE_SIM_TIMEOUT:{stage_name}")
        clock_now = node.get_clock().now().nanoseconds
        if clock_now > last_clock:
            last_clock = clock_now
            last_clock_wall = wall_now
        elif wall_now - last_clock_wall > HARD_SIM_CLOCK_STALL_S:
            raise ExecutionError(f"STAGE_SIM_CLOCK_STALL:{stage_name}")
        if clock_now > settle_deadline_sim_ns:
            raise _endpoint_failure(
                stage_name, last_measured, expected, names, limits,
                consecutive, fresh_samples)
        measured = feedback.snapshot(names, freshness_ns)
        if measured is not None:
            fresh_samples += 1
            last_measured = measured
            if _feedback_is_close(measured, expected, names, limits):
                consecutive += 1
                if consecutive >= HARD_ENDPOINT_CONSECUTIVE_SAMPLES:
                    return
            else:
                consecutive = 0
            freshness_ns = time.monotonic_ns()
        if clock_now >= settle_deadline_sim_ns:
            raise _endpoint_failure(
                stage_name, last_measured, expected, names, limits,
                consecutive, fresh_samples)


def _execute_ros(bundle: ValidatedBundle, direction: str, time_scale: float,
                 discovery_timeout: float, wall_timeout_factor: float) -> None:
    # ROS imports are intentionally below complete sealed-bundle validation.
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
        f"h1_right_handle_executor_828_{os.getpid()}",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        automatically_declare_parameters_from_overrides=True,
        context=context,
    )
    executor.add_node(node)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    feedback = _JointFeedback()
    subscription = node.create_subscription(JointState, "/joint_states",
                                            feedback.update, 10)
    publishers = {
        topic: node.create_publisher(JointTrajectory, topic, qos)
        for topic in CANONICAL_CONTROLLER_TOPICS
    }
    stages = build_stages(bundle, direction)
    last_commanded = dict(stages[0].states[0])

    def hold_current() -> None:
        names = tuple(name for group in CONTROLLER_JOINTS.values() for name in group)
        request_ns = time.monotonic_ns()
        try:
            measured = _wait_feedback(
                executor, feedback, names, request_ns, 0.5,
                allow_unsafe_torso=True)
        except ExecutionError:
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
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        print("RIGHT_HANDLE_EXECUTOR_828_FAILURE_HOLD=PASS", flush=True)

    try:
        deadline = time.monotonic() + discovery_timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            feedback.require_torso_safe()
            if _controller_and_recorder_ready(publishers):
                break
        else:
            raise ExecutionError("CONTROLLER_AND_RECORDER_DISCOVERY")
        _require_unique_controller_publishers(node, "EXECUTOR_READY")
        clock_deadline = time.monotonic() + discovery_timeout
        first_clock = 0
        while time.monotonic() < clock_deadline:
            executor.spin_once(timeout_sec=0.02)
            feedback.require_torso_safe()
            now = node.get_clock().now().nanoseconds
            if bool(getattr(node.get_clock(), "ros_time_is_active", False)) and now > 0:
                if first_clock == 0:
                    first_clock = now
                elif now > first_clock:
                    break
        else:
            raise ExecutionError("SIM_CLOCK_NOT_ADVANCING")

        for stage in stages:
            _require_unique_controller_publishers(
                node, f"STAGE_PRE:{stage.name}")
            dense = densify_stage(stage, bundle.limits)
            sim_duration = (bundle.limits.lead_time_s +
                            (len(dense) - 1) *
                            bundle.limits.point_dt_s * time_scale)
            _require(sim_duration <= HARD_MAX_STAGE_DURATION_S,
                     f"STAGE_DURATION:{stage.name}")
            names = tuple(name for topic in stage.topics for name in CONTROLLER_JOINTS[topic])
            request_ns = time.monotonic_ns()
            measured = _wait_feedback(
                executor, feedback, names, request_ns, 3.0)
            _require_feedback_close(measured, dense[0], names, bundle.limits,
                                    f"STAGE_START:{stage.name}", start=True)
            outgoing = {}
            for topic in stage.topics:
                joint_names = CONTROLLER_JOINTS[topic]
                point_dt_s = bundle.limits.point_dt_s * time_scale
                velocities = _pchip_velocities(dense, joint_names, point_dt_s)
                _validate_fast_cubic_profile(
                    dense, velocities, joint_names, point_dt_s, stage.name)
                message = JointTrajectory()
                message.joint_names = list(joint_names)
                for index, state in enumerate(dense):
                    point = JointTrajectoryPoint()
                    point.positions = [state[name] for name in joint_names]
                    point.velocities = [
                        velocities[index][name] for name in joint_names]
                    point.accelerations = []
                    point.effort = []
                    seconds = (bundle.limits.lead_time_s +
                               index * bundle.limits.point_dt_s * time_scale)
                    point.time_from_start.sec, point.time_from_start.nanosec = (
                        _duration_message(seconds))
                    message.points.append(point)
                outgoing[topic] = message
            sim_start = node.get_clock().now().nanoseconds
            for topic in stage.topics:
                publishers[topic].publish(outgoing[topic])
            last_commanded.update(dense[-1])
            wall_deadline = time.monotonic() + max(
                discovery_timeout,
                (sim_duration + 0.05 + HARD_ENDPOINT_SETTLE_SIM_S) *
                wall_timeout_factor)
            sim_target = sim_start + int((sim_duration + 0.05) * 1e9)
            last_clock = sim_start
            last_clock_wall = time.monotonic()
            while time.monotonic() < wall_deadline:
                executor.spin_once(timeout_sec=0.02)
                feedback.require_torso_safe()
                _require_unique_controller_publishers(
                    node, f"STAGE_ACTIVE:{stage.name}")
                clock_now = node.get_clock().now().nanoseconds
                if clock_now > last_clock:
                    last_clock = clock_now
                    last_clock_wall = time.monotonic()
                elif time.monotonic() - last_clock_wall > HARD_SIM_CLOCK_STALL_S:
                    raise ExecutionError(
                        f"STAGE_SIM_CLOCK_STALL:{stage.name}")
                if clock_now >= sim_target:
                    break
            else:
                raise ExecutionError(f"STAGE_SIM_TIMEOUT:{stage.name}")
            _settle_stage_endpoint(
                executor, feedback, node, names, dense[-1], bundle.limits,
                stage.name, clock_now, wall_deadline)
            print(f"RIGHT_HANDLE_EXECUTOR_828_STAGE=PASS name={stage.name} "
                  f"dense_points={len(dense)}", flush=True)
        print(f"RIGHT_HANDLE_EXECUTOR_828=PASS direction={direction}", flush=True)
    except BaseException:
        try:
            hold_current()
        except BaseException as hold_error:
            print(f"RIGHT_HANDLE_EXECUTOR_828_FAILURE_HOLD=FAIL:{hold_error}",
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
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--manifest-sha256", required=True)
    parser.add_argument("--direction", choices=("prefix", "suffix"), required=True)
    parser.add_argument("--expected-direction", choices=("prefix", "suffix"), required=True)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--time-scale", type=float, default=1.0,
                        help="fast-831 is sealed at exactly 1.0")
    parser.add_argument("--discovery-timeout", type=float, default=10.0)
    parser.add_argument("--wall-timeout-factor", type=float, default=20.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        _require(args.direction == args.expected_direction, "DIRECTION_CONFIRMATION")
        _require(math.isfinite(args.time_scale) and
                 abs(args.time_scale - 1.0) <= 1e-15,
                 "FAST_PROFILE_REQUIRES_TIME_SCALE_ONE")
        _require(math.isfinite(args.discovery_timeout) and
                 0.0 < args.discovery_timeout <= HARD_MAX_DISCOVERY_TIMEOUT_S,
                 "DISCOVERY_TIMEOUT")
        _require(math.isfinite(args.wall_timeout_factor) and
                 1.0 <= args.wall_timeout_factor <=
                 HARD_MAX_WALL_TIMEOUT_FACTOR, "WALL_TIMEOUT_FACTOR")
        bundle = validate_bundle(args.workspace, args.manifest,
                                 args.manifest_sha256)
        stages = build_stages(bundle, args.direction)
        dense_count = 0
        for stage in stages:
            dense = densify_stage(stage, bundle.limits)
            dense_count += len(dense)
            for topic in stage.topics:
                names = CONTROLLER_JOINTS[topic]
                velocities = _pchip_velocities(
                    dense, names, bundle.limits.point_dt_s)
                _validate_fast_cubic_profile(
                    dense, velocities, names, bundle.limits.point_dt_s,
                    stage.name)
        if not args.execute:
            print(f"RIGHT_HANDLE_EXECUTOR_828_VALIDATE=PASS direction={args.direction} "
                  f"stages={len(stages)} dense_points={dense_count} no_ros_import=true "
                  "controller_commands=0 fast_point_dt_s=0.004")
            return 0
        _execute_ros(bundle, args.direction, args.time_scale,
                     args.discovery_timeout, args.wall_timeout_factor)
        return 0
    except ExecutionError as error:
        print(f"RIGHT_HANDLE_EXECUTOR_828=FAIL:{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
