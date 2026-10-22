#!/usr/bin/python3
"""Assemble or verify the fail-closed Candidate 3 fast-831 certificate.

This is deliberately a post-processor.  It never launches ROS nodes and it
never publishes controller commands.  Production CDR decoding is imported
lazily from ``rclpy``; tests can inject a decoder without importing ROS.

The pre-run plan is the root of trust.  It fixes the run id, evidence paths,
formal-authority digest, process argv digests, and the maximum telemetry gap.
The resulting certificate is canonical JSON and is published atomically with
no replacement of an existing path.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import stat
import tempfile
from typing import Any, Dict, Iterable, Mapping, Protocol, Sequence, Tuple


PLAN_SCHEMA = "h1_candidate3_live_run_plan/829-v1"
MARKER_SCHEMA = "h1_candidate3_live_marker/829-v1"
MARKER_PAYLOAD_SCHEMA = "h1_candidate3_live_marker_payload/829-v1"
PROC_SCHEMA = "h1_candidate3_live_proc_snapshot/829-v1"
VIDEO_SCHEMA = "h1_candidate3_live_video_audit/829-v1"
VIDEO_WINDOW_SCHEMA = "h1_candidate3_video_window_capture/829-v1"
GUI_CAMERA_SCHEMA = "h1_candidate3_gui_camera_pose/829-v1"
POSTFLIGHT_SCHEMA = "h1_candidate3_live_postflight/829-v1"
WORKCELL_POSE_SCHEMA = "h1_candidate3_live_workcell_pose_audit/829-v1"
FORMAL_SCHEMA = "h1_candidate3_formal_authority_bindings/831-v1"
RIGHT_EXECUTION_AUTHORIZATION_SCHEMA = (
    "h1_right_handle_controller_execution_authorization/v1")
EXACT_FCL_REPLAY_SCHEMA = "h1_candidate3_live_exact_fcl_replay/829-v1"
CERTIFICATE_SCHEMA = "h1_candidate3_full_flow_live_certificate/831-v1"
VERIFIER_VERSION = "h1_candidate3_live_certificate_831/1"

SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
RUN_ID_RE = re.compile(r"[0-9a-f]{64}\Z")
MAX_JSON_BYTES = 32 * 1024 * 1024
MAX_JSONL_BYTES = 64 * 1024 * 1024
MAX_DB_BYTES = 8 * 1024 * 1024 * 1024
MAX_VIDEO_BYTES = 64 * 1024 * 1024 * 1024
MAX_CDR_BYTES = 64 * 1024 * 1024
MAX_JOINT_GAP_NS = 50_000_000
MAX_FAST_MOTION_WINDOW_NS = 90_000_000_000
NATIVE_SIM_PERIOD_NS = 5_000_000
ROBOT_JOINT_PERIOD_NS = 10_000_000
MIDDLE_EXECUTION_LEAD_NS = 250_000_000
# Measured 5 ms DART hard-stop envelope from the complete fast-r2 capture:
# door -4.924e-5..+2.060e-4 rad and handle down to -5.612e-6 rad.  Keep one
# explicit, fail-closed 0.0144-degree envelope and retain the measured values
# unchanged in the exact FCL replay.
MECHANISM_PHYSICS_RANGE_TOLERANCE_RAD = 2.5e-4
MAX_ABS_TORSO_RAD = math.radians(44.5)
MAX_REPLAY_REVOLUTE_STEP_RAD = math.radians(0.1)
MAX_REPLAY_LINEAR_STEP_M = 0.0001
RIGHT_PAD_GRIP_PROXIMITY_M = 0.0003
BUFFERED_NONPAD_HANDLE_CLEARANCE_M = 0.0006
ADJACENT_FINGER_HANDLE_CLEARANCE_M = 0.0
LEFT_HANDLE_CLEARANCE_M = 0.05
WORKCELL_POSITION_TOLERANCE_M = 0.003
WORKCELL_STATIC_ORIENTATION_TOLERANCE_RAD = 1e-6
WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD = 0.03490658503988659
EMPTY_SHA256 = hashlib.sha256(b"").hexdigest()

TOPIC_TYPES = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/incubator/joint_states": "sensor_msgs/msg/JointState",
    "/torso_controller/joint_trajectory":
        "trajectory_msgs/msg/JointTrajectory",
    "/right_arm_controller/joint_trajectory":
        "trajectory_msgs/msg/JointTrajectory",
    "/left_arm_controller/joint_trajectory":
        "trajectory_msgs/msg/JointTrajectory",
    "/left_gripper_controller/joint_trajectory":
        "trajectory_msgs/msg/JointTrajectory",
    "/right_gripper_controller/joint_trajectory":
        "trajectory_msgs/msg/JointTrajectory",
    "/h1_candidate3/live_829/markers": "std_msgs/msg/String",
}
MARKER_TOPIC = "/h1_candidate3/live_829/markers"
RECORDER_INCLUDE_UNPUBLISHED_FLAG = "--include-unpublished-topics"
RECORDER_TOPICS_FLAG = "--topics"
RECORDER_QOS_FLAG = "--qos-profile-overrides-path"
RECORDER_CACHE_FLAG = "--max-cache-size"
RECORDER_CACHE_BYTES = 1_073_741_824
RECORDER_QOS_OVERRIDES_SHA256 = (
    "da086156e2c7599f05e8c3ccc9066af8e4c6acc8210451de4ecc8b20bc0a6f1c")
TELEMETRY_BRIDGE_CONFIG_SHA256 = (
    "615231d3fc7d810f38165cbc679e4044263d4000cace8b56cc0ed35705ec9b0d")
RECORDER_TOPICS = tuple(TOPIC_TYPES)
CONTROLLER_COUNTS = {
    "/torso_controller/joint_trajectory": 8,
    "/right_arm_controller/joint_trajectory": 8,
    "/left_arm_controller/joint_trajectory": 8,
    "/left_gripper_controller/joint_trajectory": 2,
    "/right_gripper_controller/joint_trajectory": 4,
}
CONTROLLER_JOINTS = {
    "/torso_controller/joint_trajectory": ("torso_joint",),
    "/right_arm_controller/joint_trajectory": (
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint", "right_elbow_joint",
        "right_wrist_roll_joint", "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ),
    "/left_arm_controller/joint_trajectory": (
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint", "left_elbow_joint",
        "left_wrist_roll_joint", "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ),
    "/left_gripper_controller/joint_trajectory": (
        "left_dex1_finger_1_joint", "left_dex1_finger_2_joint"),
    "/right_gripper_controller/joint_trajectory": (
        "right_dex1_finger_1_joint", "right_dex1_finger_2_joint"),
}
CONTROLLED_ROBOT_JOINTS = (
    *CONTROLLER_JOINTS["/torso_controller/joint_trajectory"],
    *CONTROLLER_JOINTS["/left_arm_controller/joint_trajectory"],
    *CONTROLLER_JOINTS["/right_arm_controller/joint_trajectory"],
    *CONTROLLER_JOINTS["/left_gripper_controller/joint_trajectory"],
    *CONTROLLER_JOINTS["/right_gripper_controller/joint_trajectory"],
)
LEG_JOINTS = (
    "left_hip_yaw_joint", "left_hip_pitch_joint",
    "left_hip_roll_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_pitch_joint",
    "right_hip_roll_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
)
ALL_ROBOT_JOINTS = (*CONTROLLED_ROBOT_JOINTS, *LEG_JOINTS)
INCUBATOR_JOINTS = ("door_hinge_joint", "handle_latch_joint")
PROCESS_ROLES = (
    "launch", "recorder", "video", "prefix", "middle_suffix",
    "postflight",
)
FORMAL_AUTHORITY_IDS = (
    "right_execution_authorization", "right_manifest", "middle_plan",
    "middle_exact_result", "middle_certificate", "middle_support_manifest",
    "right_executor_source", "middle_executor_source", "flow_source",
    "fast_support_daemon_source", "support_source", "middle_admission",
    "middle_execution_contract",
    "gazebo_launch_source", "gazebo_world_sdf", "incubator_model_sdf",
    "robot_gazebo_control_xacro", "ros2_controllers_config",
    "robot_joint_state_relay_source", "telemetry_bridge_config",
    "rosbag_qos_overrides",
)
INPUT_IDS = (
    "rosbag", "markers", "proc_snapshot", "video_audit", "postflight",
    "workcell_pose_audit", "gui_camera_audit", "video_window_audit",
    "formal_authorities", "exact_fcl_replay",
)
WORKCELL_SOURCE_ARGV = (
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz", "topic", "-e",
    "--json-output", "-t",
    "/world/h1_world/pose/info", "-n", "1",
)
GUI_CAMERA_REQUEST = (
    "pose { position { x: 0.06292473524808884 "
    "y: -0.46163156628608704 z: 1.8398312330245972 } "
    "orientation { x: -0.15106521546840668 y: 0.24716269969940186 "
    "z: 0.49844232201576233 w: 0.8170956969261169 } }"
)
GUI_CAMERA_ARGV = (
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz", "service", "-s",
    "/gui/move_to/pose",
    "--reqtype", "gz.msgs.GUICamera", "--reptype", "gz.msgs.Boolean",
    "--timeout", "5000", "--req", GUI_CAMERA_REQUEST,
)
EVIDENCE_TOOL_IDS = ("supervisor_source", "certificate_source")
ALLOWED_HANDLE_CONTACT_PAIRS = (
    ("incubator_handle_exact", "right_dex1_finger_1_pad_link"),
    ("incubator_handle_exact", "right_dex1_finger_2_pad_link"),
)
NATIVE_CONTACT_UNAVAILABLE_REASON = (
    "DART_HANDLE_MESH_CONTACT_TOPIC_NOT_ADVERTISED")
FCL_PREBOUND_FILE_IDS = (
    "auditor_source", "auditor_path_base_source",
    "auditor_middle_base_source", "auditor_binary", "contact_path",
    "original_handle_stl", "scene_cdr", "urdf", "srdf",
)
REPLAY_TSV_HEADER = (
    "id\tcontact_required\tdoor_angle_deg\thandle_angle_deg\ttorso_rad\t"
    "leg_rad\tleft_arm_rad\tright_arm_rad\tleft_gripper_m\t"
    "right_gripper_m")
FCL_ROW_KEYS = (
    "id", "status", "valid", "contact_required", "door_angle_deg",
    "handle_angle_deg", "torso_rad", "collision_free",
    "pad1_patch_valid", "pad2_patch_valid", "right_pad_grip_engaged",
    "pad1_contacts", "pad2_contacts", "pad1_handle_distance_m",
    "pad2_handle_distance_m", "nonpad_handle_clearance_m",
    "adjacent_finger_handle_clearance_m",
    "left_handle_clearance_m", "patch_span_m", "patch_station_m",
    "patch_radius_m", "geometric_only", "force_or_wrench_claimed",
)
FCL_SUMMARY_KEYS = (
    "status", "valid", "evaluated", "passed", "contact_required",
    "contact_required_passed", "noncontact", "noncontact_passed",
    "pad1_patch_valid", "pad2_patch_valid", "right_pad_grip_engaged",
    "max_torso_rad", "max_contact_minimum_right_pad_distance_m",
    "min_nonpad_handle_clearance_m",
    "min_adjacent_finger_handle_clearance_m",
    "min_left_handle_clearance_m",
    "min_patch_span_m", "min_patch_station_m", "min_patch_radius_m",
    "collision_free", "scene_sha256", "path_sha256", "telemetry_sha256",
    "urdf_sha256", "srdf_sha256", "handle_stl_sha256", "reason",
    "geometric_only", "force_or_wrench_claimed",
)


def _exact_fcl_scope() -> Mapping[str, Any]:
    return {
        "live_fcl_domain": "right_handle_and_non_cube_collision_geometry",
        "full_flow_telemetry_used": True,
        "full_robot_joint_count": 31,
        "leg_joint_order": list(LEG_JOINTS),
        "excluded_gripper_transition_windows": [
            "close_q0_right_gripper_transition",
            "release_q0_right_gripper_transition",
        ],
        "gripper_transition_authority_ids": [
            "right_execution_authorization", "right_manifest"],
        "dynamic_cube_objects_included": False,
        "dynamic_cube_object_ids": [
            "green_cube_dynamic_exact", "green_cube_10mm_guard"],
        "dynamic_cube_authority_ids": [
            "middle_exact_result", "middle_certificate"],
        "right_grip_evidence": {
            "kind": "minimum_authorized_right_pad_handle_signed_distance",
            "contact_required_maximum_distance_m":
                RIGHT_PAD_GRIP_PROXIMITY_M,
            "noncontact_pad_distances_strictly_greater_than_m":
                RIGHT_PAD_GRIP_PROXIMITY_M,
            "patch_metrics_diagnostic_only": True,
            "force_or_wrench_claimed": False,
        },
        "adjacent_right_finger_body_minimum_clearance_m":
            ADJACENT_FINGER_HANDLE_CLEARANCE_M,
        "buffered_nonpad_minimum_handle_clearance_m":
            BUFFERED_NONPAD_HANDLE_CLEARANCE_M,
        "left_hand_minimum_handle_clearance_m": LEFT_HANDLE_CLEARANCE_M,
        "allowed_handle_collision_pairs": [
            list(pair) for pair in ALLOWED_HANDLE_CONTACT_PAIRS],
        "force_or_wrench_claimed": False,
    }


@dataclass(frozen=True)
class MarkerSpec:
    name: str
    process_role: str
    pattern: re.Pattern[str]


def _literal_marker(name: str, role: str, line: str) -> MarkerSpec:
    return MarkerSpec(name, role, re.compile(re.escape(line) + r"\Z"))


def _marker_specs() -> Tuple[MarkerSpec, ...]:
    specs = [
        _literal_marker(
            "CAPTURE_START", "supervisor",
            "H1_CANDIDATE3_LIVE_829=START"),
        _literal_marker(
            "SUPPORT_PRELOAD", "prefix",
            "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=preload "
            "door=detached cube=detached"),
        MarkerSpec(
            "MOTION_WINDOW_START", "prefix", re.compile(
                r"CANDIDATE3_FAST_831_MOTION_WINDOW=START "
                r"monotonic_ns=[1-9][0-9]*\Z")),
    ]
    for marker_name, stage_name in (
        ("PREFIX_PREPARE_LEFT_PARK", "prepare_left_park"),
        ("PREFIX_INGRESS_OPEN_TO_Q0", "ingress_open_to_q0"),
        ("PREFIX_CLOSE_Q0_RIGHT_HAND", "close_q0_right_hand"),
        ("PREFIX_CONTACT_ARC_Q0_Q90", "contact_arc_q0_to_q90_hold"),
    ):
        specs.append(MarkerSpec(
            marker_name, "prefix", re.compile(
                r"RIGHT_HANDLE_EXECUTOR_828_STAGE=PASS name=" +
                re.escape(stage_name) + r" dense_points=[1-9][0-9]*\Z")))
    specs.extend((
        _literal_marker(
            "RIGHT_PREFIX_PASS", "prefix",
            "RIGHT_HANDLE_EXECUTOR_828=PASS direction=prefix"),
        _literal_marker(
            "FLOW_PREFIX_PASS", "prefix",
            "CANDIDATE3_FLOW_828_PREFIX=PASS"),
        _literal_marker(
            "Q90_BEFORE_MIDDLE", "middle_suffix",
            "RIGHT_HANDLE_EXECUTOR_828_Q90_E0=PASS fresh_feedback=true "
            "controller_commands=0"),
        _literal_marker(
            "SUPPORT_DOOR_ATTACH", "middle_suffix",
            "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-attach "
            "door=attached cube=detached boundary=E0_ENTRY"),
    ))
    phase_names = (
        "door_pre_support", "door_open_supported", "door60_seam_to_cube",
        "left_cube_ready", "cube_contact", "cube_carry_place",
        "cube_release", "cube_retreat", "left_cube_park",
        "door60_seam_to_hinge", "door_close_supported",
    )
    phase_commands = (0, 2, 2, 1, 2, 2, 1, 1, 1, 2, 2)
    support_after = {
        3: _literal_marker(
            "SUPPORT_CUBE_ATTACH", "middle_suffix",
            "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-attach "
            "door=attached cube=attached boundary=CUBE_CONTACT"),
        4: _literal_marker(
            "SUPPORT_CUBE_DETACH", "middle_suffix",
            "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-detach "
            "door=attached cube=detached boundary=CUBE_RELEASE"),
        9: _literal_marker(
            "SUPPORT_DOOR_DETACH", "middle_suffix",
            "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-detach "
            "door=detached cube=detached boundary=E0_RETURN"),
    }
    for index, (phase, commands) in enumerate(zip(
            phase_names, phase_commands)):
        specs.append(_literal_marker(
            f"MIDDLE_PHASE_{index}", "middle_suffix",
            f"CANDIDATE3_MIDDLE_828_PHASE=PASS index={index} "
            f"name={phase} commands={commands}"))
        if index in support_after:
            specs.append(support_after[index])
    specs.extend((
        _literal_marker(
            "MIDDLE_PASS", "middle_suffix",
            "CANDIDATE3_MIDDLE_828=PASS phases=11 "
            "right_gripper_commands=0 terminal_e0_held=true"),
        _literal_marker(
            "Q90_AFTER_MIDDLE", "middle_suffix",
            "RIGHT_HANDLE_EXECUTOR_828_Q90_E0=PASS fresh_feedback=true "
            "controller_commands=0"),
    ))
    for marker_name, stage_name in (
        ("SUFFIX_CONTACT_ARC_Q90_Q0", "contact_arc_q90_hold_to_q0"),
        ("SUFFIX_RELEASE_Q0_RIGHT_HAND", "release_q0_right_hand"),
        ("SUFFIX_EGRESS_OPEN_FROM_Q0", "egress_open_from_q0"),
        ("SUFFIX_RETURN_LEFT_LAUNCH", "return_left_launch_seam"),
    ):
        specs.append(MarkerSpec(
            marker_name, "middle_suffix", re.compile(
                r"RIGHT_HANDLE_EXECUTOR_828_STAGE=PASS name=" +
                re.escape(stage_name) + r" dense_points=[1-9][0-9]*\Z")))
    finalize_line = (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize door=q0 "
        "handle=q0 latch=locked supports=detached robot=launch publishers=0")
    specs.extend((
        _literal_marker(
            "RIGHT_SUFFIX_PASS", "middle_suffix",
            "RIGHT_HANDLE_EXECUTOR_828=PASS direction=suffix"),
        _literal_marker(
            "LAUNCH_SEAM_PASS", "middle_suffix",
            "RIGHT_HANDLE_EXECUTOR_828_LAUNCH_SEAM=PASS "
            "fresh_feedback=true q0_released=true reverse_ingress=true "
            "left_launch=true torso_launch=true controller_commands=0"),
        _literal_marker(
            "SUPPORT_FINALIZE", "middle_suffix", finalize_line),
        MarkerSpec(
            "MOTION_WINDOW_END", "middle_suffix", re.compile(
                r"CANDIDATE3_FAST_831_MOTION_WINDOW=END "
                r"monotonic_ns=[1-9][0-9]*\Z")),
        _literal_marker(
            "FLOW_MIDDLE_SUFFIX_PASS", "middle_suffix",
            "CANDIDATE3_FLOW_828_MIDDLE_SUFFIX=PASS"),
        _literal_marker(
            "POSTFLIGHT_FINALIZE", "postflight", finalize_line),
        _literal_marker(
            "CAPTURE_END", "supervisor",
            "H1_CANDIDATE3_LIVE_829=END"),
    ))
    return tuple(specs)


MARKER_SPECS = _marker_specs()
MARKER_NAMES = tuple(spec.name for spec in MARKER_SPECS)
SUPPORT_SEQUENCE = (
    {"operation": "preload", "door": "detached", "cube": "detached"},
    {"operation": "door-attach", "door": "attached", "cube": "detached"},
    {"operation": "cube-attach", "door": "attached", "cube": "attached"},
    {"operation": "cube-detach", "door": "attached", "cube": "detached"},
    {"operation": "door-detach", "door": "detached", "cube": "detached"},
    {"operation": "finalize", "door": "q0", "handle": "q0",
     "latch": "locked", "supports": "detached", "robot": "launch",
     "publishers": 0},
    {"operation": "postflight-finalize", "door": "q0", "handle": "q0",
     "latch": "locked", "supports": "detached", "robot": "launch",
     "publishers": 0},
)


class LiveCertificateError(RuntimeError):
    """A fail-closed evidence, semantic, or publication error."""


class CdrDeserializer(Protocol):
    def deserialize(self, type_name: str, payload: bytes) -> Mapping[str, Any]:
        """Return the normalized mapping required for ``type_name``."""


class RclpyCdrDeserializer:
    """Lazy production decoder; importing this module remains ROS-free."""

    def __init__(self) -> None:
        try:
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
        except ImportError as error:
            raise LiveCertificateError(f"RCLPY_CDR_IMPORT:{error}") from error
        self._deserialize_message = deserialize_message
        self._get_message = get_message
        self._classes: Dict[str, Any] = {}

    def deserialize(self, type_name: str, payload: bytes) -> Mapping[str, Any]:
        try:
            message_class = self._classes.setdefault(
                type_name, self._get_message(type_name))
            message = self._deserialize_message(payload, message_class)
        except Exception as error:
            raise LiveCertificateError(
                f"CDR_DESERIALIZE:{type_name}:{error}") from error
        if type_name == "sensor_msgs/msg/JointState":
            return {
                "header_stamp_ns": (
                    int(message.header.stamp.sec) * 1_000_000_000 +
                    int(message.header.stamp.nanosec)),
                "names": list(message.name),
                "positions": list(message.position),
            }
        if type_name == "trajectory_msgs/msg/JointTrajectory":
            return {
                "joint_names": list(message.joint_names),
                "points": [{
                    "positions": list(point.positions),
                    "time_from_start_ns": (
                        int(point.time_from_start.sec) * 1_000_000_000 +
                        int(point.time_from_start.nanosec)),
                } for point in message.points],
            }
        if type_name == "rosgraph_msgs/msg/Clock":
            return {
                "clock_ns": int(message.clock.sec) * 1_000_000_000 +
                int(message.clock.nanosec),
            }
        if type_name == "std_msgs/msg/String":
            return {"data": str(message.data)}
        raise LiveCertificateError(f"UNSUPPORTED_CDR_TYPE:{type_name}")


@dataclass(frozen=True)
class FileEvidence:
    path: Path
    raw: bytes
    sha256: str
    size_bytes: int
    mode: str


@dataclass(frozen=True)
class CertificateArtifact:
    value: Mapping[str, Any]
    raw: bytes
    sha256: str
    path: Path


@dataclass(frozen=True)
class ReplayTelemetry:
    raw: bytes
    rows: Tuple[Mapping[str, Any], ...]
    source_sample_count: int
    maximum_source_gap_ns: int
    maximum_revolute_step_rad: float
    maximum_linear_step_m: float
    close_command_timestamp_ns: int
    close_pass_timestamp_ns: int
    release_command_timestamp_ns: int
    release_pass_timestamp_ns: int
    excluded_close_transition_rows: int
    excluded_release_transition_rows: int


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise LiveCertificateError(status)


def _exact(value: Any, keys: Iterable[str], status: str) -> Mapping[str, Any]:
    _require(type(value) is dict, f"{status}_TYPE")
    _require(frozenset(value) == frozenset(keys), f"{status}_KEYS")
    return value


def _integer(value: Any, status: str, minimum: int = 0) -> int:
    _require(type(value) is int and value >= minimum, status)
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


def _run_id(value: Any, status: str) -> str:
    _require(type(value) is str and RUN_ID_RE.fullmatch(value) is not None,
             status)
    return value


def _reject_duplicate_keys(
        pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise LiveCertificateError(f"DUPLICATE_JSON_KEY:{key}")
        result[key] = value
    return result


def _reject_constant(token: str) -> None:
    raise LiveCertificateError(f"NONFINITE_JSON:{token}")


def _canonical_bytes(value: Any) -> bytes:
    try:
        return (json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
            allow_nan=False).encode("utf-8") + b"\n")
    except (TypeError, ValueError) as error:
        raise LiveCertificateError(f"CANONICAL_JSON:{error}") from error


def _canonical_hash(value: Any) -> str:
    return hashlib.sha256(_canonical_bytes(value)[:-1]).hexdigest()


def _parse_json(raw: bytes, status: str) -> Any:
    try:
        return json.loads(
            raw.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_constant)
    except LiveCertificateError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise LiveCertificateError(f"{status}_JSON:{error}") from error


def _ensure_finite_tree(value: Any, status: str) -> None:
    if type(value) is float:
        _require(math.isfinite(value), status)
    elif type(value) is list:
        for item in value:
            _ensure_finite_tree(item, status)
    elif type(value) is dict:
        for item in value.values():
            _ensure_finite_tree(item, status)


def _identity(metadata: os.stat_result) -> Tuple[int, ...]:
    return (
        metadata.st_dev, metadata.st_ino, metadata.st_mode,
        metadata.st_nlink, metadata.st_uid, metadata.st_size,
        metadata.st_mtime_ns,
    )


def _canonical_absolute_path(value: Any, status: str) -> Path:
    _require(type(value) is str and "\x00" not in value, status)
    path = Path(value)
    _require(path.is_absolute() and os.path.normpath(value) == value, status)
    return path


def _read_sealed_file(
    path: Path, *, expected_sha256: str | None = None,
    expected_mode: str = "0400", maximum_bytes: int = MAX_JSON_BYTES,
    status: str,
) -> FileEvidence:
    _require(path.is_absolute(), f"{status}_PATH_NOT_ABSOLUTE")
    try:
        before = path.lstat()
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise LiveCertificateError(f"{status}_LSTAT:{error}") from error
    _require(resolved == path and not path.is_symlink(), f"{status}_PATH_ALIAS")
    required_mode = int(expected_mode, 8)
    _require(
        stat.S_ISREG(before.st_mode) and before.st_nlink == 1 and
        before.st_uid == os.geteuid() and
        stat.S_IMODE(before.st_mode) == required_mode and
        0 <= before.st_size <= maximum_bytes,
        f"{status}_IDENTITY")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(
        os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise LiveCertificateError(f"{status}_OPEN:{error}") from error
    try:
        opened = os.fstat(descriptor)
        _require(_identity(opened) == _identity(before),
                 f"{status}_OPEN_RACE")
        chunks = []
        total = 0
        while True:
            chunk = os.read(descriptor, min(1024 * 1024,
                                            maximum_bytes + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            _require(total <= maximum_bytes, f"{status}_TOO_LARGE")
        after = os.fstat(descriptor)
        _require(_identity(after) == _identity(opened),
                 f"{status}_READ_RACE")
    finally:
        os.close(descriptor)
    raw = b"".join(chunks)
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None:
        _sha(expected_sha256, f"{status}_EXPECTED_SHA256")
        _require(hmac.compare_digest(digest, expected_sha256),
                 f"{status}_SHA256")
    return FileEvidence(path, raw, digest, len(raw), expected_mode)


def _load_canonical_json(
    path: Path, *, expected_sha256: str | None = None,
    status: str,
) -> Tuple[FileEvidence, Mapping[str, Any]]:
    evidence = _read_sealed_file(
        path, expected_sha256=expected_sha256, status=status)
    value = _parse_json(evidence.raw, status)
    _ensure_finite_tree(value, f"{status}_FINITE")
    _require(type(value) is dict, f"{status}_ROOT_TYPE")
    _require(evidence.raw == _canonical_bytes(value),
             f"{status}_NONCANONICAL")
    return evidence, value


def _binding(evidence: FileEvidence) -> Mapping[str, Any]:
    return {
        "path": str(evidence.path),
        "sha256": evidence.sha256,
        "size_bytes": evidence.size_bytes,
        "mode": evidence.mode,
    }


def _validate_file_binding(value: Any, status: str) -> FileEvidence:
    binding = _exact(
        value, ("path", "sha256", "size_bytes", "mode"), status)
    path = _canonical_absolute_path(binding["path"], f"{status}_PATH")
    digest = _sha(binding["sha256"], f"{status}_SHA256_FORMAT")
    size = _integer(binding["size_bytes"], f"{status}_SIZE")
    _require(binding["mode"] in ("0400", "0500"), f"{status}_MODE")
    maximum = MAX_VIDEO_BYTES if path.suffix.lower() == ".mp4" else MAX_DB_BYTES
    evidence = _read_sealed_file(
        path, expected_sha256=digest, expected_mode=binding["mode"],
        maximum_bytes=maximum, status=status)
    _require(evidence.size_bytes == size, f"{status}_SIZE_MISMATCH")
    return evidence


def _validate_plan(value: Mapping[str, Any]) -> Mapping[str, Any]:
    plan = _exact(value, (
        "schema", "run_id", "sealed_monotonic_ns", "input_paths",
        "formal_authorities_sha256", "expected_process_argv_sha256",
        "expected_exact_fcl_replay_argv_sha256",
        "expected_exact_fcl_file_sha256", "evidence_tools",
        "max_joint_gap_ns", "valid", "status",
    ), "PLAN")
    _require(plan["schema"] == PLAN_SCHEMA, "PLAN_SCHEMA")
    _run_id(plan["run_id"], "PLAN_RUN_ID")
    _integer(plan["sealed_monotonic_ns"], "PLAN_SEALED_MONOTONIC_NS", 1)
    paths = _exact(plan["input_paths"], INPUT_IDS, "PLAN_INPUT_PATHS")
    for identifier in INPUT_IDS:
        _canonical_absolute_path(paths[identifier], f"PLAN_PATH_{identifier}")
    _sha(plan["formal_authorities_sha256"], "PLAN_FORMAL_SHA256")
    argv_hashes = _exact(
        plan["expected_process_argv_sha256"], PROCESS_ROLES,
        "PLAN_PROCESS_ARGV")
    for role in PROCESS_ROLES:
        _sha(argv_hashes[role], f"PLAN_PROCESS_ARGV_{role}")
    _sha(plan["expected_exact_fcl_replay_argv_sha256"],
         "PLAN_EXACT_FCL_REPLAY_ARGV_SHA256")
    fcl_files = _exact(
        plan["expected_exact_fcl_file_sha256"], FCL_PREBOUND_FILE_IDS,
        "PLAN_EXACT_FCL_FILES")
    for identifier in FCL_PREBOUND_FILE_IDS:
        _sha(fcl_files[identifier], f"PLAN_EXACT_FCL_FILE:{identifier}")
    tool_bindings = _exact(
        plan["evidence_tools"], EVIDENCE_TOOL_IDS,
        "PLAN_EVIDENCE_TOOLS")
    tools = [
        _validate_file_binding(
            tool_bindings[identifier], f"PLAN_EVIDENCE_TOOL:{identifier}")
        for identifier in EVIDENCE_TOOL_IDS
    ]
    _require(all(tool.mode == "0500" for tool in tools) and
             len({tool.path for tool in tools}) == len(tools),
             "PLAN_EVIDENCE_TOOL_IDENTITY")
    try:
        running_source = Path(__file__).resolve(strict=True)
    except OSError as error:
        raise LiveCertificateError(
            f"PLAN_CERTIFICATE_SOURCE_SELF:LSTAT:{error}") from error
    _require(tools[EVIDENCE_TOOL_IDS.index("certificate_source")].path ==
             running_source,
             "PLAN_CERTIFICATE_SOURCE_SELF_PATH")
    _require(plan["max_joint_gap_ns"] == MAX_JOINT_GAP_NS,
             "PLAN_MAX_JOINT_GAP")
    _require(plan["valid"] is True and plan["status"] == "READY",
             "PLAN_NOT_READY")
    return plan


def _load_markers(
    path: Path, run_id: str,
) -> Tuple[FileEvidence, Tuple[Mapping[str, Any], ...]]:
    evidence = _read_sealed_file(
        path, maximum_bytes=MAX_JSONL_BYTES, status="MARKERS")
    _require(evidence.raw.endswith(b"\n"), "MARKERS_FINAL_NEWLINE")
    raw_lines = evidence.raw.splitlines(keepends=True)
    _require(len(raw_lines) == len(MARKER_SPECS), "MARKER_COUNT")
    markers = []
    previous_monotonic = -1
    previous_bag = -1
    for index, (raw_line, spec) in enumerate(zip(raw_lines, MARKER_SPECS)):
        value = _parse_json(raw_line, f"MARKER_{index}")
        _ensure_finite_tree(value, f"MARKER_{index}_FINITE")
        _require(raw_line == _canonical_bytes(value),
                 f"MARKER_{index}_NONCANONICAL")
        marker = _exact(value, (
            "schema", "run_id", "sequence", "monotonic_ns",
            "bag_timestamp_ns", "process_role", "name", "line",
        ), f"MARKER_{index}")
        _require(marker["schema"] == MARKER_SCHEMA,
                 f"MARKER_{index}_SCHEMA")
        _require(marker["run_id"] == run_id, f"MARKER_{index}_RUN_ID")
        _require(marker["sequence"] == index, f"MARKER_{index}_SEQUENCE")
        monotonic_ns = _integer(
            marker["monotonic_ns"], f"MARKER_{index}_MONOTONIC", 1)
        bag_timestamp_ns = _integer(
            marker["bag_timestamp_ns"], f"MARKER_{index}_BAG_TIME", 1)
        _require(monotonic_ns > previous_monotonic,
                 f"MARKER_{index}_MONOTONIC_ORDER")
        _require(bag_timestamp_ns > previous_bag,
                 f"MARKER_{index}_BAG_TIME_ORDER")
        previous_monotonic = monotonic_ns
        previous_bag = bag_timestamp_ns
        _require(marker["process_role"] == spec.process_role,
                 f"MARKER_{index}_PROCESS_ROLE")
        _require(marker["name"] == spec.name,
                 f"MARKER_{index}_ORDER")
        _require(type(marker["line"]) is str and
                 spec.pattern.fullmatch(marker["line"]) is not None,
                 f"MARKER_{index}_LINE")
        markers.append(marker)
    marker_by_name = _marker_map(markers)
    start = marker_by_name["MOTION_WINDOW_START"]
    end = marker_by_name["MOTION_WINDOW_END"]
    printed_start = int(start["line"].rsplit("=", 1)[1])
    printed_end = int(end["line"].rsplit("=", 1)[1])
    printed_duration = printed_end - printed_start
    observed_duration = end["monotonic_ns"] - start["monotonic_ns"]
    _require(
        0 < printed_duration <= MAX_FAST_MOTION_WINDOW_NS,
        "MOTION_WINDOW_PRINTED_DURATION_90S")
    _require(
        0 < observed_duration <= MAX_FAST_MOTION_WINDOW_NS,
        "MOTION_WINDOW_OBSERVED_DURATION_90S")
    return evidence, tuple(markers)


def _marker_map(
        markers: Sequence[Mapping[str, Any]]) -> Mapping[str, Mapping[str, Any]]:
    return {marker["name"]: marker for marker in markers}


def _argv_sha256(argv: Sequence[str]) -> str:
    return hashlib.sha256(
        b"".join(item.encode("utf-8") + b"\0" for item in argv)).hexdigest()


def _flag_value(argv: Sequence[str], flag: str, status: str) -> str:
    indices = [index for index, value in enumerate(argv) if value == flag]
    _require(len(indices) == 1 and indices[0] + 1 < len(argv), status)
    value = argv[indices[0] + 1]
    _require(value != "" and not value.startswith("--"), status)
    return value


def _flag_value_any(
    argv: Sequence[str], flags: Sequence[str], status: str,
) -> str:
    indices = [index for index, value in enumerate(argv) if value in flags]
    _require(len(indices) == 1 and indices[0] + 1 < len(argv), status)
    value = argv[indices[0] + 1]
    _require(value != "" and not value.startswith("--"), status)
    return value


def _validate_recorder_argv(
    argv: Sequence[str], expected_output: str,
    expected_topics: Sequence[str], qos_binding: Mapping[str, Any],
) -> None:
    _require("bag" in argv and "record" in argv and
             "-a" not in argv and "--all" not in argv,
             "PROC_RECORDER_COMMAND_CONTRACT")
    _require(
        argv.count(RECORDER_INCLUDE_UNPUBLISHED_FLAG) == 1 and
        argv.index(RECORDER_INCLUDE_UNPUBLISHED_FLAG) > argv.index("record"),
        "PROC_RECORDER_INCLUDE_UNPUBLISHED_TOPICS")
    _require(
        _flag_value_any(argv, ("-s", "--storage"),
                        "PROC_RECORDER_STORAGE") == "sqlite3" and
        _flag_value_any(argv, ("-o", "--output"),
                        "PROC_RECORDER_OUTPUT") == expected_output and
        all(argv.count(topic) == 1 for topic in expected_topics),
        "PROC_RECORDER_COMMAND_CONTRACT")
    _require(
        _flag_value(argv, RECORDER_QOS_FLAG, "PROC_RECORDER_QOS") ==
        qos_binding["path"] and
        _flag_value(argv, RECORDER_CACHE_FLAG, "PROC_RECORDER_CACHE") ==
        str(RECORDER_CACHE_BYTES) and qos_binding["mode"] == "0400" and
        qos_binding.get("sha256") == RECORDER_QOS_OVERRIDES_SHA256,
        "PROC_RECORDER_QOS_CACHE_CONTRACT")
    _require(
        argv.count(RECORDER_TOPICS_FLAG) == 1 and
        argv.index(RECORDER_TOPICS_FLAG) >
        argv.index(RECORDER_INCLUDE_UNPUBLISHED_FLAG),
        "PROC_RECORDER_TOPICS_FLAG")
    topics_index = argv.index(RECORDER_TOPICS_FLAG)
    _require(argv.index(RECORDER_QOS_FLAG) < topics_index and
             argv.index(RECORDER_CACHE_FLAG) < topics_index,
             "PROC_RECORDER_QOS_CACHE_ORDER")
    _require(tuple(argv[topics_index + 1:]) == tuple(expected_topics),
             "PROC_RECORDER_TOPICS_EXACT_ORDER")


def _validate_proc_snapshot(
    value: Mapping[str, Any], plan: Mapping[str, Any], plan_sha256: str,
    plan_path: Path, markers: Sequence[Mapping[str, Any]],
    formal_summary: Mapping[str, Any],
) -> Mapping[str, Any]:
    snapshot = _exact(value, (
        "schema", "run_id", "boot_id_sha256", "supervisor_pid",
        "supervisor_start_ticks", "sealed_plan_sha256",
        "preexisting_processes", "recorder_ready", "captured_monotonic_ns",
        "processes", "valid", "status",
    ), "PROC")
    _require(snapshot["schema"] == PROC_SCHEMA, "PROC_SCHEMA")
    _require(snapshot["run_id"] == plan["run_id"], "PROC_RUN_ID")
    _sha(snapshot["boot_id_sha256"], "PROC_BOOT_ID_SHA256")
    supervisor_pid = _integer(snapshot["supervisor_pid"], "PROC_SUPERVISOR_PID", 1)
    _integer(snapshot["supervisor_start_ticks"],
             "PROC_SUPERVISOR_START_TICKS", 1)
    _require(snapshot["sealed_plan_sha256"] == plan_sha256,
             "PROC_PLAN_SHA256")
    _require(snapshot["preexisting_processes"] == [],
             "PROC_PREEXISTING_PROCESSES")
    captured = _integer(
        snapshot["captured_monotonic_ns"], "PROC_CAPTURED_MONOTONIC", 1)
    _require(type(snapshot["processes"]) is list and
             len(snapshot["processes"]) == len(PROCESS_ROLES),
             "PROC_PROCESS_COUNT")
    processes: Dict[str, Mapping[str, Any]] = {}
    identities = set()
    for index, item in enumerate(snapshot["processes"]):
        process = _exact(item, (
            "role", "pid", "ppid", "start_ticks", "started_monotonic_ns",
            "ended_monotonic_ns", "exit_code", "run_id", "argv",
            "cmdline_sha256",
        ), f"PROC_PROCESS_{index}")
        role = process["role"]
        _require(role == PROCESS_ROLES[index], f"PROC_PROCESS_{index}_ROLE")
        pid = _integer(process["pid"], f"PROC_PROCESS_{role}_PID", 1)
        _require(process["ppid"] == supervisor_pid,
                 f"PROC_PROCESS_{role}_PARENT")
        start_ticks = _integer(
            process["start_ticks"], f"PROC_PROCESS_{role}_START_TICKS", 1)
        _require((pid, start_ticks) not in identities,
                 f"PROC_PROCESS_{role}_IDENTITY_REUSE")
        identities.add((pid, start_ticks))
        started = _integer(
            process["started_monotonic_ns"],
            f"PROC_PROCESS_{role}_STARTED", 1)
        ended = _integer(
            process["ended_monotonic_ns"],
            f"PROC_PROCESS_{role}_ENDED", 1)
        _require(plan["sealed_monotonic_ns"] <= started < ended <= captured,
                 f"PROC_PROCESS_{role}_INTERVAL")
        _require(process["exit_code"] == 0,
                 f"PROC_PROCESS_{role}_EXIT")
        _require(process["run_id"] == plan["run_id"],
                 f"PROC_PROCESS_{role}_RUN_ID")
        argv = process["argv"]
        _require(type(argv) is list and len(argv) > 0 and all(
            type(arg) is str and arg != "" and "\x00" not in arg
            for arg in argv), f"PROC_PROCESS_{role}_ARGV")
        actual_argv_sha = _argv_sha256(argv)
        _require(process["cmdline_sha256"] == actual_argv_sha,
                 f"PROC_PROCESS_{role}_CMDLINE_SHA256")
        _require(hmac.compare_digest(
            actual_argv_sha,
            plan["expected_process_argv_sha256"][role]),
            f"PROC_PROCESS_{role}_PLANNED_ARGV")
        processes[role] = process
    ready = _exact(snapshot["recorder_ready"], (
        "captured_monotonic_ns", "subscription_topics",
    ), "PROC_RECORDER_READY")
    ready_timestamp = _integer(
        ready["captured_monotonic_ns"], "PROC_RECORDER_READY_TIME", 1)
    expected_subscriptions = sorted(TOPIC_TYPES)
    _require(ready["subscription_topics"] == expected_subscriptions and
             processes["recorder"]["started_monotonic_ns"] <=
             ready_timestamp < markers[0]["monotonic_ns"],
             "PROC_RECORDER_READY_BINDING")
    _require(snapshot["valid"] is True and snapshot["status"] == "PASS",
             "PROC_NOT_PASS")

    marker_by_name = _marker_map(markers)
    first = markers[0]["monotonic_ns"]
    last = markers[-1]["monotonic_ns"]
    for role in ("launch", "recorder", "video"):
        _require(
            processes[role]["started_monotonic_ns"] <= first and
            processes[role]["ended_monotonic_ns"] >= last,
            f"PROC_PROCESS_{role}_COVERAGE")
    _require(
        processes["prefix"]["started_monotonic_ns"] <=
        marker_by_name["SUPPORT_PRELOAD"]["monotonic_ns"] and
        processes["prefix"]["ended_monotonic_ns"] >=
        marker_by_name["FLOW_PREFIX_PASS"]["monotonic_ns"],
        "PROC_PREFIX_COVERAGE")
    _require(
        processes["middle_suffix"]["started_monotonic_ns"] <=
        marker_by_name["Q90_BEFORE_MIDDLE"]["monotonic_ns"] and
        processes["middle_suffix"]["ended_monotonic_ns"] >=
        marker_by_name["FLOW_MIDDLE_SUFFIX_PASS"]["monotonic_ns"],
        "PROC_MIDDLE_SUFFIX_COVERAGE")
    _require(
        processes["postflight"]["started_monotonic_ns"] <=
        marker_by_name["POSTFLIGHT_FINALIZE"]["monotonic_ns"] and
        processes["postflight"]["ended_monotonic_ns"] >=
        marker_by_name["POSTFLIGHT_FINALIZE"]["monotonic_ns"],
        "PROC_POSTFLIGHT_COVERAGE")

    authority = {
        row["id"]: row for row in formal_summary["authority_bindings"]}

    def bind_flags(
        argv: Sequence[str], identifier: str, path_flag: str,
        sha_flag: str, status: str,
    ) -> None:
        binding = authority[identifier]
        _require(_flag_value(argv, path_flag, f"{status}_PATH") ==
                 binding["path"] and
                 _flag_value(argv, sha_flag, f"{status}_SHA") ==
                 binding["sha256"], status)

    launch_argv = processes["launch"]["argv"]
    prefix_argv = processes["prefix"]["argv"]
    middle_argv = processes["middle_suffix"]["argv"]
    postflight_argv = processes["postflight"]["argv"]
    video_argv = processes["video"]["argv"]
    recorder_argv = processes["recorder"]["argv"]
    supervisor_tool = plan["evidence_tools"]["supervisor_source"]
    for role, argv in (("launch", launch_argv), ("video", video_argv)):
        _require(len(argv) >= 3 and argv[0] == "/usr/bin/python3" and
                 argv[1] == supervisor_tool["path"] and
                 _flag_value(argv, "--self-sha256",
                             f"PROC_{role.upper()}_SELF_SHA") ==
                 supervisor_tool["sha256"] and
                 _flag_value(argv, "--plan",
                             f"PROC_{role.upper()}_PLAN") ==
                 str(plan_path),
                 f"PROC_{role.upper()}_HELPER_BINDING")
    _require(launch_argv[2] == "_launch-child",
             "PROC_LAUNCH_HELPER_ENTRY")
    _require(video_argv[2] == "_video-child" and
             _flag_value(video_argv, "--run-id", "PROC_VIDEO_RUN_ID") ==
             plan["run_id"] and
             _flag_value(video_argv, "--window-audit",
                         "PROC_VIDEO_WINDOW_AUDIT") ==
             plan["input_paths"]["video_window_audit"] and
             _flag_value(video_argv, "--crop", "PROC_VIDEO_CROP") ==
             "1160:1530:0:80" and
             _flag_value(video_argv, "--framerate",
                         "PROC_VIDEO_FRAMERATE") == "12" and
             _flag_value(video_argv, "--crf", "PROC_VIDEO_CRF") == "20",
             "PROC_VIDEO_HELPER_CONTRACT")
    for identifier, path_flag, sha_flag in (
        ("gazebo_launch_source", "--gazebo-launch-source",
         "--gazebo-launch-source-sha256"),
        ("gazebo_world_sdf", "--gazebo-world-sdf",
         "--gazebo-world-sdf-sha256"),
        ("incubator_model_sdf", "--incubator-model-sdf",
         "--incubator-model-sdf-sha256"),
        ("robot_gazebo_control_xacro", "--robot-gazebo-control-xacro",
         "--robot-gazebo-control-xacro-sha256"),
        ("ros2_controllers_config", "--ros2-controllers-config",
         "--ros2-controllers-config-sha256"),
        ("robot_joint_state_relay_source",
         "--robot-joint-state-relay-source",
         "--robot-joint-state-relay-source-sha256"),
        ("telemetry_bridge_config", "--telemetry-bridge-config",
         "--telemetry-bridge-config-sha256"),
    ):
        bind_flags(launch_argv, identifier, path_flag, sha_flag,
                   f"PROC_LAUNCH_AUTHORITY:{identifier}")
    _require(authority["telemetry_bridge_config"]["sha256"] ==
             TELEMETRY_BRIDGE_CONFIG_SHA256,
             "PROC_LAUNCH_TELEMETRY_BRIDGE_CONFIG_CONTRACT")
    assignments = {}
    for item in launch_argv:
        if ":=" in item:
            key, raw_value = item.split(":=", 1)
            _require(key not in assignments,
                     f"PROC_LAUNCH_DUPLICATE_ASSIGNMENT:{key}")
            assignments[key] = raw_value
    _require(set(assignments) == {
        "spawn_incubator", "table_x", "incubator_x", "incubator_y",
        "incubator_z", "cube_x", "bridge_support_states",
        "robot_joint_state_relay_source",
        "robot_joint_state_relay_sha256",
        "telemetry_bridge_config",
        "telemetry_bridge_config_sha256",
    } and assignments["spawn_incubator"] == "true" and
             assignments["bridge_support_states"] == "false" and
             assignments["robot_joint_state_relay_source"] ==
             authority["robot_joint_state_relay_source"]["path"] and
             assignments["robot_joint_state_relay_sha256"] ==
             authority["robot_joint_state_relay_source"]["sha256"] and
             assignments["telemetry_bridge_config"] ==
             authority["telemetry_bridge_config"]["path"] and
             assignments["telemetry_bridge_config_sha256"] ==
             authority["telemetry_bridge_config"]["sha256"],
             "PROC_LAUNCH_FIXED_ASSIGNMENTS")
    try:
        launch_geometry = {
            name: float(assignments[name]) for name in (
                "table_x", "incubator_x", "incubator_y", "incubator_z",
                "cube_x")}
    except ValueError as error:
        raise LiveCertificateError(
            "PROC_LAUNCH_FIXED_ASSIGNMENTS") from error
    _require(launch_geometry == {
        "table_x": 0.600, "incubator_x": 0.590,
        "incubator_y": 0.0, "incubator_z": 0.815, "cube_x": 0.334,
    }, "PROC_LAUNCH_FIXED_GEOMETRY")
    _validate_recorder_argv(
        recorder_argv, str(Path(plan["input_paths"]["rosbag"]).parent),
        RECORDER_TOPICS, authority["rosbag_qos_overrides"])
    for argv in (prefix_argv, middle_argv):
        _require(argv[1] == authority["flow_source"]["path"],
                 "PROC_FLOW_SOURCE_PATH")
        for identifier, path_flag, sha_flag in (
            ("right_executor_source", "--right-executor",
             "--right-executor-sha256"),
            ("right_execution_authorization", "--right-manifest",
             "--right-manifest-sha256"),
            ("support_source", "--support-orchestrator",
             "--support-orchestrator-sha256"),
            ("middle_support_manifest", "--support-manifest",
             "--support-manifest-sha256"),
        ):
            bind_flags(argv, identifier, path_flag, sha_flag,
                       f"PROC_FLOW_AUTHORITY:{identifier}")
    for identifier, path_flag, sha_flag in (
        ("middle_plan", "--plan", "--plan-sha256"),
        ("middle_exact_result", "--result", "--result-sha256"),
        ("middle_certificate", "--certificate", "--certificate-sha256"),
        ("middle_executor_source", "--middle-executor",
         "--middle-executor-sha256"),
        ("middle_admission", "--middle-admission",
         "--middle-admission-sha256"),
        ("middle_execution_contract", "--middle-contract",
         "--middle-contract-sha256"),
    ):
        bind_flags(middle_argv, identifier, path_flag, sha_flag,
                   f"PROC_MIDDLE_AUTHORITY:{identifier}")
    bind_flags(
        middle_argv, "fast_support_daemon_source",
        "--fast-support-daemon", "--fast-support-daemon-sha256",
        "PROC_MIDDLE_AUTHORITY:fast_support_daemon_source")
    _require(postflight_argv[1] == authority["support_source"]["path"],
             "PROC_POSTFLIGHT_SUPPORT_SOURCE")
    bind_flags(postflight_argv, "middle_support_manifest",
               "--manifest", "--manifest-sha256",
               "PROC_POSTFLIGHT_SUPPORT_MANIFEST")

    expected_authorization = authority["right_execution_authorization"]
    right_authorization_evidence = _read_sealed_file(
        Path(expected_authorization["path"]),
        expected_sha256=expected_authorization["sha256"],
        expected_mode=expected_authorization["mode"],
        status="PROC_RIGHT_EXECUTION_AUTHORIZATION")
    right_authorization_document = _exact(_parse_json(
        right_authorization_evidence.raw,
        "PROC_RIGHT_EXECUTION_AUTHORIZATION"), (
            "schema", "controller_execution_authorized",
            "execution_ready", "live_execution_passed", "live_result",
            "controller_commands_issued", "offline_only", "bindings",
            "controller_topics", "stage_order", "limits",
            "auditor_version", "execution_topology",
        ), "PROC_RIGHT_EXECUTION_AUTHORIZATION")
    _require(right_authorization_document["schema"] ==
             RIGHT_EXECUTION_AUTHORIZATION_SCHEMA and
             right_authorization_document[
                 "controller_execution_authorized"] is True,
             "PROC_RIGHT_EXECUTION_AUTHORIZATION_HEADER")
    _require(right_authorization_document["execution_ready"] is False and
             right_authorization_document["live_execution_passed"] is False
             and right_authorization_document["live_result"] is None,
             "PROC_RIGHT_EXECUTION_AUTHORIZATION_PRE_RUN_STATE")
    _require(right_authorization_document[
                 "controller_commands_issued"] == 0 and
             right_authorization_document["offline_only"] is True,
             "PROC_RIGHT_EXECUTION_AUTHORIZATION_OFFLINE_STATE")

    right_manifest_binding = authority["right_manifest"]
    _require(right_manifest_binding["path"] !=
             expected_authorization["path"],
             "PROC_RIGHT_MANIFEST_AUTHORIZATION_PATH_ALIAS")
    right_manifest_evidence = _read_sealed_file(
        Path(right_manifest_binding["path"]),
        expected_sha256=right_manifest_binding["sha256"],
        expected_mode=right_manifest_binding["mode"],
        status="PROC_RIGHT_MANIFEST")
    right_manifest = _parse_json(
        right_manifest_evidence.raw, "PROC_RIGHT_MANIFEST")
    _require(type(right_manifest) is dict, "PROC_RIGHT_MANIFEST_TYPE")
    sealed_bindings = right_manifest.get("sealed_bindings")
    _require(type(sealed_bindings) is dict and
             type(sealed_bindings.get(
                 "controller_execution_authorization")) is dict,
             "PROC_RIGHT_AUTHORIZATION_MANIFEST")
    right_authorization = sealed_bindings[
        "controller_execution_authorization"]
    raw_auth_path = right_authorization.get("path")
    auth_path_matches = (
        type(raw_auth_path) is str and
        ((Path(raw_auth_path).is_absolute() and
          expected_authorization["path"] == raw_auth_path) or
         (not Path(raw_auth_path).is_absolute() and
          expected_authorization["path"].endswith("/" + raw_auth_path))))
    _require(auth_path_matches and
             right_authorization.get("sha256") ==
             expected_authorization["sha256"] and
             right_authorization.get("mode_octal") == "0400" and
             right_authorization.get("nlink") == 1 and
             right_authorization.get("schema") ==
             RIGHT_EXECUTION_AUTHORIZATION_SCHEMA,
             "PROC_RIGHT_AUTHORIZATION_BINDING")

    _require(prefix_argv.count("prefix") == 1, "PROC_PREFIX_OPERATION")
    _require(middle_argv.count("middle-and-suffix") == 1,
             "PROC_MIDDLE_SUFFIX_OPERATION")
    try:
        prefix_right_scale = float(_flag_value(
            prefix_argv, "--right-time-scale", "PROC_PREFIX_RIGHT_SCALE"))
        middle_right_scale = float(_flag_value(
            middle_argv, "--right-time-scale", "PROC_MIDDLE_RIGHT_SCALE"))
        middle_speed_scale = float(_flag_value(
            middle_argv, "--middle-speed-scale", "PROC_MIDDLE_SPEED_SCALE"))
    except ValueError as error:
        raise LiveCertificateError(f"PROC_SPEED_PARSE:{error}") from error
    for value_, status in (
        (prefix_right_scale, "PROC_PREFIX_RIGHT_SCALE_RANGE"),
        (middle_right_scale, "PROC_MIDDLE_RIGHT_SCALE_RANGE"),
        (middle_speed_scale, "PROC_MIDDLE_SPEED_SCALE_RANGE"),
    ):
        _require(math.isfinite(value_), status)
    _require(prefix_right_scale == 1.0 and middle_right_scale == 1.0,
             "PROC_FAST_RIGHT_SCALE")
    _require(prefix_right_scale == middle_right_scale,
             "PROC_RIGHT_SCALE_MISMATCH")
    _require(middle_speed_scale == 1.0,
             "PROC_FAST_MIDDLE_SPEED_SCALE")
    return {
        "boot_id_sha256": snapshot["boot_id_sha256"],
        "fresh_launch": True,
        "preexisting_process_count": 0,
        "process_count": len(PROCESS_ROLES),
        "recorder_ready_monotonic_ns": ready_timestamp,
        "recorder_subscription_topics": expected_subscriptions,
        "process_intervals": {
            role: {
                "started_monotonic_ns":
                    processes[role]["started_monotonic_ns"],
                "ended_monotonic_ns":
                    processes[role]["ended_monotonic_ns"],
            } for role in PROCESS_ROLES
        },
        "process_argv": {
            role: list(processes[role]["argv"]) for role in PROCESS_ROLES},
        "process_identity_sha256": _canonical_hash([{
            "role": role,
            "pid": processes[role]["pid"],
            "start_ticks": processes[role]["start_ticks"],
            "cmdline_sha256": processes[role]["cmdline_sha256"],
        } for role in PROCESS_ROLES]),
        "right_time_scale": prefix_right_scale,
        "middle_speed_scale": middle_speed_scale,
    }


def _validate_formal_authorities(
    value: Mapping[str, Any], run_id: str,
) -> Mapping[str, Any]:
    formal = _exact(value, (
        "schema", "run_id", "authorities", "valid", "status",
    ), "FORMAL")
    _require(formal["schema"] == FORMAL_SCHEMA, "FORMAL_SCHEMA")
    _require(formal["run_id"] == run_id, "FORMAL_RUN_ID")
    _require(type(formal["authorities"]) is list and
             len(formal["authorities"]) == len(FORMAL_AUTHORITY_IDS),
             "FORMAL_AUTHORITY_COUNT")
    normalized = []
    for index, item in enumerate(formal["authorities"]):
        authority = _exact(item, (
            "id", "path", "sha256", "size_bytes", "mode",
        ), f"FORMAL_AUTHORITY_{index}")
        _require(authority["id"] == FORMAL_AUTHORITY_IDS[index],
                 f"FORMAL_AUTHORITY_{index}_ID")
        evidence = _validate_file_binding(
            {key: authority[key]
             for key in ("path", "sha256", "size_bytes", "mode")},
            f"FORMAL_AUTHORITY_{authority['id']}")
        normalized.append({
            "id": authority["id"],
            "path": str(evidence.path),
            "sha256": evidence.sha256,
            "size_bytes": evidence.size_bytes,
            "mode": evidence.mode,
        })
    _require(formal["valid"] is True and formal["status"] == "PASS",
             "FORMAL_NOT_PASS")
    return {
        "authority_count": len(normalized),
        "authority_ids": list(FORMAL_AUTHORITY_IDS),
        "authority_bindings": normalized,
        "authority_bindings_sha256": _canonical_hash(normalized),
    }


def _validate_dynamic_cube_delegation(
    formal_summary: Mapping[str, Any], plan: Mapping[str, Any],
    middle_speed_scale: float,
) -> Mapping[str, Any]:
    middle_speed_scale = _finite(
        middle_speed_scale, "DELEGATION_MIDDLE_SPEED_SCALE")
    _require(0.05 <= middle_speed_scale <= 1.0,
             "DELEGATION_MIDDLE_SPEED_SCALE_RANGE")
    bindings = {
        row["id"]: row for row in formal_summary["authority_bindings"]}

    def load(identifier: str) -> Mapping[str, Any]:
        binding = bindings[identifier]
        evidence = _read_sealed_file(
            Path(binding["path"]), expected_sha256=binding["sha256"],
            expected_mode=binding["mode"], status=f"DELEGATION_{identifier}")
        value = _parse_json(evidence.raw, f"DELEGATION_{identifier}")
        _ensure_finite_tree(value, f"DELEGATION_{identifier}_FINITE")
        _require(type(value) is dict, f"DELEGATION_{identifier}_TYPE")
        return value

    middle_plan = load("middle_plan")
    result = load("middle_exact_result")
    certificate = load("middle_certificate")
    right_manifest = load("right_manifest")
    support_manifest = load("middle_support_manifest")
    _require(middle_plan.get("schema") ==
             "h1_candidate3_middle_plan/828-e0-v1" and
             middle_plan.get("offline_only") is True and
             middle_plan.get("controller_commands_issued") == 0,
             "DELEGATION_MIDDLE_PLAN")
    _require(result.get("schema") ==
             "h1_candidate3_middle_exact_result/828-e0-v1" and
             result.get("status") == "PASS" and
             result.get("valid") is True and
             result.get("offline_only") is True and
             result.get("controller_commands_issued") == 0 and
             result.get("plan_file_sha256") ==
             bindings["middle_plan"]["sha256"],
             "DELEGATION_MIDDLE_RESULT")
    scene = result.get("scene_binding")
    required_objects = [
        "environment_table_10mm_margin", "green_cube_10mm_guard",
        "green_cube_dynamic_exact", "incubator_body_exact",
        "incubator_door_exact", "incubator_handle_exact",
    ]
    _require(type(scene) is dict and
             scene.get("required_object_ids") == required_objects and
             scene.get("scene_cdr_sha256") ==
             plan["expected_exact_fcl_file_sha256"]["scene_cdr"] and
             scene.get("planning_frame") == "world",
             "DELEGATION_MIDDLE_SCENE")
    authorizations = result.get("contact_authorizations")
    _require(type(authorizations) is dict and
             authorizations.get("schema") ==
             "h1_candidate3_middle_contact_authorizations/828-e0-v1" and
             authorizations.get("valid") is True and
             authorizations.get("unauthorized_contacts") == {
                 "count": 0, "pairs": []},
             "DELEGATION_CONTACT_AUTHORIZATIONS")
    windows = authorizations.get("windows")
    _require(type(windows) is list and
             [window.get("id") for window in windows] == [
                 "right_handle_pad_contact", "left_cube_grasp_carry",
                 "left_cube_release_retreat"],
             "DELEGATION_CONTACT_WINDOWS")
    cube_objects = ["green_cube_dynamic_exact", "green_cube_10mm_guard"]
    expected_cube_phases = {
        "left_cube_grasp_carry": [
            "cube_contact", "cube_carry_place", "cube_release"],
        "left_cube_release_retreat": ["cube_retreat"],
    }
    for window in windows:
        _require(window.get("valid") is True and
                 type(window.get("authorized_sample_count")) is int and
                 window["authorized_sample_count"] > 0,
                 f"DELEGATION_CONTACT_WINDOW:{window.get('id')}")
        if window["id"] in expected_cube_phases:
            _require(window.get("object_ids") == cube_objects and
                     [phase.get("phase_name")
                      for phase in window.get("phase_bindings", [])] ==
                     expected_cube_phases[window["id"]],
                     f"DELEGATION_CUBE_WINDOW:{window['id']}")
    _require(certificate.get("schema") ==
             "h1_candidate3_middle_certificate/828-e0-v1" and
             certificate.get("status") == "PASS" and
             certificate.get("valid") is True and
             certificate.get("offline_only") is True and
             certificate.get("controller_commands_issued") == 0 and
             certificate.get("exact_result_sha256") ==
             bindings["middle_exact_result"]["sha256"] and
             certificate.get("plan_sha256") ==
             bindings["middle_plan"]["sha256"],
             "DELEGATION_MIDDLE_CERTIFICATE")
    support_bindings = support_manifest.get("bindings")
    support_contract = support_manifest.get("support_contract")
    _require(type(support_contract) is dict and
             support_contract.get("schema") ==
             "h1_support_contract_828/v1",
             "DELEGATION_SUPPORT_CONTRACT_SCHEMA")
    support_contract_sha256 = _canonical_hash(support_contract)
    _require(support_manifest.get("schema") ==
             "h1_support_orchestration_828/v1" and
             support_manifest.get("controller_execution_authorized") is
             True and
             support_manifest.get("live_execution_passed") is False and
             type(support_bindings) is dict and
             support_bindings.get("plan_schema") ==
             "h1_candidate3_middle_plan/828-e0-v1" and
             support_bindings.get("plan_sha256") ==
             bindings["middle_plan"]["sha256"] and
             support_bindings.get("exact_result_schema") ==
             "h1_candidate3_middle_exact_result/828-e0-v1" and
             support_bindings.get("exact_result_sha256") ==
             bindings["middle_exact_result"]["sha256"] and
             support_bindings.get("certificate_schema") ==
             "h1_candidate3_middle_certificate/828-e0-v1" and
             support_bindings.get("certificate_sha256") ==
             bindings["middle_certificate"]["sha256"] and
             support_bindings.get("support_contract_sha256") ==
             support_contract_sha256 and
             middle_plan.get("support_contract_sha256") ==
             support_contract_sha256 and
             result.get("support_contract_sha256") ==
             support_contract_sha256 and
             certificate.get("support_contract_sha256") ==
             support_contract_sha256,
             "DELEGATION_SUPPORT_MANIFEST_BINDINGS")
    release_pose = support_contract.get("boundaries", {}).get(
            "CUBE_RELEASE", {}).get("cube_pose")
    _require(type(release_pose) is dict and
             release_pose.get("position") == [0.611, -0.030, 0.995] and
             release_pose.get("orientation") == [
                 0.0, 0.0, 0.7071067811865475,
                 0.7071067811865476],
             "DELEGATION_SUPPORT_CUBE_RELEASE_TARGET")
    source_prefix = result.get("entry_projection", {}).get("source_prefix")
    projected_e0 = result.get("entry_projection", {}).get("projected_e0")
    terminal_seam = result.get("terminal_seam")
    audited_identity = right_manifest.get("audited_state_identity")
    _require(type(source_prefix) is dict and
             source_prefix.get("endpoint_label") == "canonical_e0_q90_held" and
             type(audited_identity) is dict and
             audited_identity.get("endpoint_rows_unchanged_from_8263") is True,
             "DELEGATION_PREFIX_SEAM")
    q90 = audited_identity.get("q90_canonical_e0")
    _require(type(q90) is dict and type(projected_e0) is dict and
             type(terminal_seam) is dict,
             "DELEGATION_PREFIX_SEAM_STATE")
    right_arm = q90.get("right_arm_rad")
    pad_sliders = q90.get("right_pad_slider_mm")
    _require(type(right_arm) is list and len(right_arm) == 7 and
             type(pad_sliders) is list and len(pad_sliders) == 2,
             "DELEGATION_PREFIX_SEAM_DIMENSIONS")
    canonical_e0 = {
        "right_arm_rad": [
            _finite(value, f"DELEGATION_PREFIX_SEAM_RIGHT_ARM_{index}")
            for index, value in enumerate(right_arm)],
        "right_fingers_m": {
            "right_dex1_finger_1_joint": _finite(
                pad_sliders[0], "DELEGATION_PREFIX_SEAM_PAD_1") / 1000.0,
            "right_dex1_finger_2_joint": _finite(
                pad_sliders[1], "DELEGATION_PREFIX_SEAM_PAD_2") / 1000.0,
        },
        "torso_yaw_rad": math.radians(_finite(
            q90.get("torso_yaw_deg"), "DELEGATION_PREFIX_SEAM_TORSO")),
    }
    _require(_close(math.radians(_finite(
                 q90.get("handle_angle_deg"),
                 "DELEGATION_PREFIX_SEAM_HANDLE")), math.pi / 2.0) and
             _close(_finite(
                 result["entry_projection"].get("projected_handle_angle_rad"),
                 "DELEGATION_PROJECTED_HANDLE"), math.pi / 2.0),
             "DELEGATION_PREFIX_SEAM_HANDLE")

    def require_same_e0(value: Any, status: str) -> None:
        _require(type(value) is dict and
                 type(value.get("right_arm_rad")) is list and
                 len(value["right_arm_rad"]) == 7 and
                 type(value.get("right_fingers_m")) is dict,
                 status)
        for index, expected_value in enumerate(canonical_e0["right_arm_rad"]):
            _require(_close(_finite(
                value["right_arm_rad"][index], f"{status}_ARM_{index}"),
                expected_value), status)
        _require(set(value["right_fingers_m"]) ==
                 set(canonical_e0["right_fingers_m"]), status)
        for name, expected_value in canonical_e0["right_fingers_m"].items():
            _require(_close(_finite(
                value["right_fingers_m"][name], f"{status}_{name}"),
                expected_value), status)
        _require(_close(_finite(
            value.get("torso_yaw_rad"), f"{status}_TORSO"),
            canonical_e0["torso_yaw_rad"]), status)

    _require(projected_e0.get("label") == "canonical_e0_q90_held",
             "DELEGATION_PROJECTED_E0_LABEL")
    require_same_e0(projected_e0, "DELEGATION_PROJECTED_E0")
    require_same_e0(
        terminal_seam.get("entry_e0"), "DELEGATION_TERMINAL_ENTRY_E0")
    require_same_e0(
        terminal_seam.get("return_e0"), "DELEGATION_TERMINAL_RETURN_E0")
    sealed_contact = right_manifest.get(
        "sealed_bindings", {}).get("contact_path")
    _require(type(sealed_contact) is dict and
             sealed_contact.get("sha256") ==
             plan["expected_exact_fcl_file_sha256"]["contact_path"] and
             sealed_contact.get("schema") ==
             "h1_moveit/right_handle_variable_torso_path/v4",
             "DELEGATION_RIGHT_CONTACT_PATH")
    expected_left_gripper = []
    expected_phases = []
    for phase in middle_plan.get("phases", []):
        if type(phase) is not dict:
            raise LiveCertificateError("DELEGATION_MIDDLE_PHASE_TYPE")
        for command in phase.get("commands", []):
            if (type(command) is dict and command.get("topic") ==
                    "/left_gripper_controller/joint_trajectory"):
                expected_phases.append(phase.get("name"))
                sealed_trajectory = _validate_trajectory(
                    {key: command[key] for key in ("joint_names", "points")},
                    "/left_gripper_controller/joint_trajectory",
                    "DELEGATION_LEFT_GRIPPER_COMMAND")
                expected_left_gripper.append({
                    "joint_names": sealed_trajectory["joint_names"],
                    "points": [{
                        "positions": point["positions"],
                        "time_from_start_ns": (
                            MIDDLE_EXECUTION_LEAD_NS + int(round(
                                point["time_from_start_ns"] /
                                middle_speed_scale))),
                    } for point in sealed_trajectory["points"]],
                })
    _require(expected_phases == ["cube_contact", "cube_release"] and
             len(expected_left_gripper) == 2,
             "DELEGATION_LEFT_GRIPPER_TOPOLOGY")
    return {
        "middle_plan_sha256": bindings["middle_plan"]["sha256"],
        "middle_exact_result_sha256":
            bindings["middle_exact_result"]["sha256"],
        "middle_certificate_sha256":
            bindings["middle_certificate"]["sha256"],
        "support_contract_sha256": support_contract_sha256,
        "scene_cdr_sha256": scene["scene_cdr_sha256"],
        "dynamic_cube_object_ids": cube_objects,
        "cube_contact_window_ids": [
            "left_cube_grasp_carry", "left_cube_release_retreat"],
        "terminal_cube_target": {
            "position_m": list(release_pose["position"]),
            "orientation_xyzw": list(release_pose["orientation"]),
            "position_tolerance_m": WORKCELL_POSITION_TOLERANCE_M,
            "orientation_tolerance_rad":
                WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD,
        },
        "expected_left_gripper_payload_sha256": _canonical_hash(
            expected_left_gripper),
        "middle_speed_scale": middle_speed_scale,
        "left_gripper_execution_lead_ns": MIDDLE_EXECUTION_LEAD_NS,
        "source_prefix_endpoint": "canonical_e0_q90_held",
        "canonical_e0_state_sha256": _canonical_hash(canonical_e0),
        "right_v4_endpoint_rows_unchanged_from_8263": True,
        "controller_commands_issued": 0,
        "offline_only": True,
        "valid": True,
        "status": "PASS",
    }


def _contact_pairs(value: Any, status: str) -> list[list[str]]:
    _require(type(value) is list, f"{status}_TYPE")
    result = []
    seen = set()
    for index, item in enumerate(value):
        _require(type(item) is list and len(item) == 2 and all(
            type(name) is str and name != "" for name in item),
            f"{status}_{index}")
        pair = (item[0], item[1])
        _require(pair not in seen, f"{status}_{index}_DUPLICATE")
        seen.add(pair)
        result.append([item[0], item[1]])
    return result


def _parse_machine_line(
    line: str, prefix: str, keys: Sequence[str], status: str,
) -> Mapping[str, str]:
    tokens = line.split(" ")
    _require(len(tokens) == len(keys) + 1 and tokens[0] == prefix and
             all(token != "" for token in tokens), f"{status}_TOKENS")
    result: Dict[str, str] = {}
    for index, token in enumerate(tokens[1:]):
        _require(token.count("=") == 1, f"{status}_{index}_TOKEN")
        key, value = token.split("=", 1)
        _require(key == keys[index] and value != "" and key not in result,
                 f"{status}_{index}_KEY")
        result[key] = value
    return result


def _machine_bool(value: str, status: str) -> bool:
    _require(value in ("true", "false"), status)
    return value == "true"


def _machine_integer(value: str, status: str) -> int:
    _require(re.fullmatch(r"0|[1-9][0-9]*", value) is not None, status)
    return int(value)


def _machine_float(value: str, status: str) -> float:
    _require(value != "" and not value[0].isspace() and
             not value[-1].isspace(), status)
    try:
        result = float(value)
    except ValueError as error:
        raise LiveCertificateError(status) from error
    _require(math.isfinite(result), status)
    return result


def _close(left: float, right: float, tolerance: float = 1e-11) -> bool:
    return abs(left - right) <= tolerance * max(1.0, abs(left), abs(right))


def _validate_exact_fcl_replay_legacy(
    value: Mapping[str, Any], replay_evidence: FileEvidence,
    plan: Mapping[str, Any], rosbag: FileEvidence,
) -> Mapping[str, Any]:
    """Validate geometric contact proof over the exact recorded db3 bytes.

    Native DART contact/force evidence is explicitly unavailable for the
    handle mesh.  Therefore this gate proves geometric contact continuity and
    exclusivity only; it cannot be promoted into a force or wrench claim.
    """
    certificate = _exact(value, (
        "schema", "run_id", "rosbag", "geometry", "auditor", "replay",
        "native_physics_contacts", "offline_replay",
        "live_telemetry_bound", "valid", "status",
    ), "EXACT_FCL_REPLAY")
    _require(certificate["schema"] == EXACT_FCL_REPLAY_SCHEMA,
             "EXACT_FCL_REPLAY_SCHEMA")
    _require(certificate["run_id"] == plan["run_id"],
             "EXACT_FCL_REPLAY_RUN_ID")
    replay_bag = _validate_file_binding(
        certificate["rosbag"], "EXACT_FCL_REPLAY_ROSBAG")
    _require(
        replay_bag.path == rosbag.path and
        replay_bag.sha256 == rosbag.sha256 and
        replay_bag.size_bytes == rosbag.size_bytes,
        "EXACT_FCL_REPLAY_ROSBAG_BINDING")

    geometry = _exact(certificate["geometry"], (
        "original_handle_stl", "scene_cdr", "urdf",
    ), "EXACT_FCL_REPLAY_GEOMETRY")
    geometry_files = {}
    for identifier in ("original_handle_stl", "scene_cdr", "urdf"):
        geometry_files[identifier] = _validate_file_binding(
            geometry[identifier], f"EXACT_FCL_REPLAY_{identifier}")
        _require(geometry_files[identifier].size_bytes > 0,
                 f"EXACT_FCL_REPLAY_{identifier}_EMPTY")

    auditor = _exact(certificate["auditor"], (
        "source", "binary", "argv", "argv_sha256", "exit_code",
        "stdout", "stderr",
    ), "EXACT_FCL_REPLAY_AUDITOR")
    source = _validate_file_binding(
        auditor["source"], "EXACT_FCL_REPLAY_AUDITOR_SOURCE")
    binary = _validate_file_binding(
        auditor["binary"], "EXACT_FCL_REPLAY_AUDITOR_BINARY")
    stdout = _validate_file_binding(
        auditor["stdout"], "EXACT_FCL_REPLAY_AUDITOR_STDOUT")
    stderr = _validate_file_binding(
        auditor["stderr"], "EXACT_FCL_REPLAY_AUDITOR_STDERR")
    _require(source.size_bytes > 0 and binary.size_bytes > 0,
             "EXACT_FCL_REPLAY_AUDITOR_EMPTY")
    _require(stdout.raw == b"EXACT_FCL_LIVE_REPLAY_829=PASS valid=true\n" and
             stderr.raw == b"", "EXACT_FCL_REPLAY_AUDITOR_OUTPUT")
    argv = auditor["argv"]
    _require(type(argv) is list and len(argv) > 0 and all(
        type(item) is str and item != "" and "\x00" not in item
        for item in argv), "EXACT_FCL_REPLAY_AUDITOR_ARGV")
    argv_sha = _argv_sha256(argv)
    _require(auditor["argv_sha256"] == argv_sha,
             "EXACT_FCL_REPLAY_AUDITOR_ARGV_SHA256")
    argv_template_sha = argv_sha
    _require(hmac.compare_digest(
        argv_template_sha,
        plan["expected_exact_fcl_replay_argv_sha256"]),
        "EXACT_FCL_REPLAY_AUDITOR_PLANNED_ARGV_TEMPLATE")
    _require(argv[0] == str(binary.path),
             "EXACT_FCL_REPLAY_AUDITOR_EXECUTABLE")
    expected_flags = {
        "--bag": str(rosbag.path),
        "--bag-sha256": rosbag.sha256,
        "--handle-stl": str(geometry_files["original_handle_stl"].path),
        "--handle-stl-sha256":
            geometry_files["original_handle_stl"].sha256,
        "--scene": str(geometry_files["scene_cdr"].path),
        "--scene-sha256": geometry_files["scene_cdr"].sha256,
        "--urdf": str(geometry_files["urdf"].path),
        "--urdf-sha256": geometry_files["urdf"].sha256,
        "--output": str(replay_evidence.path),
    }
    for flag, expected in expected_flags.items():
        _require(_flag_value(
            argv, flag, f"EXACT_FCL_REPLAY_AUDITOR_FLAG:{flag}") == expected,
            f"EXACT_FCL_REPLAY_AUDITOR_FLAG_VALUE:{flag}")
    _require(auditor["exit_code"] == 0,
             "EXACT_FCL_REPLAY_AUDITOR_EXIT")

    replay = _exact(certificate["replay"], (
        "source_joint_state_samples", "densified_sample_count",
        "maximum_source_gap_ns", "maximum_revolute_step_rad",
        "maximum_linear_step_m", "collision_free", "left_handle_checks",
        "left_handle_contact_count", "minimum_left_handle_clearance_m",
        "right_contact_required_samples", "right_patch_valid_samples",
        "right_pad_1_patch_valid_samples",
        "right_pad_2_patch_valid_samples",
        "right_nonpad_handle_contact_count",
        "minimum_right_nonpad_handle_clearance_m",
        "allowed_handle_contact_pairs", "observed_handle_contact_pairs",
        "unauthorized_handle_contact_pairs", "replay_transcript_sha256",
    ), "EXACT_FCL_REPLAY_METRICS")
    for key, minimum in (
        ("source_joint_state_samples", 2),
        ("densified_sample_count", 2),
        ("maximum_source_gap_ns", 1),
        ("left_handle_checks", 1),
        ("left_handle_contact_count", 0),
        ("right_contact_required_samples", 1),
        ("right_patch_valid_samples", 1),
        ("right_pad_1_patch_valid_samples", 0),
        ("right_pad_2_patch_valid_samples", 0),
        ("right_nonpad_handle_contact_count", 0),
    ):
        _integer(replay[key], f"EXACT_FCL_REPLAY_INTEGER:{key}", minimum)
    _sha(replay["replay_transcript_sha256"],
         "EXACT_FCL_REPLAY_TRANSCRIPT_SHA256")
    source_samples = _integer(
        replay["source_joint_state_samples"],
        "EXACT_FCL_REPLAY_SOURCE_SAMPLES", 2)
    dense_samples = _integer(
        replay["densified_sample_count"],
        "EXACT_FCL_REPLAY_DENSE_SAMPLES", source_samples)
    maximum_gap = _integer(
        replay["maximum_source_gap_ns"],
        "EXACT_FCL_REPLAY_MAXIMUM_GAP", 1)
    _require(maximum_gap <= MAX_JOINT_GAP_NS,
             "EXACT_FCL_REPLAY_MAXIMUM_GAP_LIMIT")
    revolute_step = _finite(
        replay["maximum_revolute_step_rad"],
        "EXACT_FCL_REPLAY_REVOLUTE_STEP")
    linear_step = _finite(
        replay["maximum_linear_step_m"],
        "EXACT_FCL_REPLAY_LINEAR_STEP")
    _require(0.0 < revolute_step <= MAX_REPLAY_REVOLUTE_STEP_RAD,
             "EXACT_FCL_REPLAY_REVOLUTE_STEP_LIMIT")
    _require(0.0 < linear_step <= MAX_REPLAY_LINEAR_STEP_M,
             "EXACT_FCL_REPLAY_LINEAR_STEP_LIMIT")
    _require(replay["collision_free"] is True,
             "EXACT_FCL_REPLAY_COLLISION")
    left_checks = _integer(
        replay["left_handle_checks"], "EXACT_FCL_REPLAY_LEFT_CHECKS", 1)
    _require(left_checks == dense_samples,
             "EXACT_FCL_REPLAY_LEFT_CHECK_COVERAGE")
    _require(replay["left_handle_contact_count"] == 0,
             "EXACT_FCL_REPLAY_LEFT_HANDLE_CONTACT")
    left_clearance = _finite(
        replay["minimum_left_handle_clearance_m"],
        "EXACT_FCL_REPLAY_LEFT_CLEARANCE")
    _require(left_clearance >= 0.05,
             "EXACT_FCL_REPLAY_LEFT_CLEARANCE_LIMIT")
    required_samples = _integer(
        replay["right_contact_required_samples"],
        "EXACT_FCL_REPLAY_RIGHT_REQUIRED", 1)
    valid_samples = _integer(
        replay["right_patch_valid_samples"],
        "EXACT_FCL_REPLAY_RIGHT_PATCH_VALID", 1)
    _require(valid_samples == required_samples,
             "EXACT_FCL_REPLAY_RIGHT_CONTACT_GAP")
    pad1 = _integer(
        replay["right_pad_1_patch_valid_samples"],
        "EXACT_FCL_REPLAY_PAD1", 1)
    pad2 = _integer(
        replay["right_pad_2_patch_valid_samples"],
        "EXACT_FCL_REPLAY_PAD2", 1)
    _require(max(pad1, pad2) <= required_samples and
             pad1 + pad2 >= required_samples,
             "EXACT_FCL_REPLAY_PAD_COUNTS")
    _require(replay["right_nonpad_handle_contact_count"] == 0,
             "EXACT_FCL_REPLAY_RIGHT_NONPAD_CONTACT")
    nonpad_clearance = _finite(
        replay["minimum_right_nonpad_handle_clearance_m"],
        "EXACT_FCL_REPLAY_RIGHT_NONPAD_CLEARANCE")
    _require(nonpad_clearance >= 0.0006,
             "EXACT_FCL_REPLAY_RIGHT_NONPAD_CLEARANCE_LIMIT")
    allowed_pairs = _contact_pairs(
        replay["allowed_handle_contact_pairs"],
        "EXACT_FCL_REPLAY_ALLOWED_PAIRS")
    _require(allowed_pairs == [
        list(pair) for pair in ALLOWED_HANDLE_CONTACT_PAIRS],
             "EXACT_FCL_REPLAY_ALLOWED_PAIRS_EXACT")
    observed_pairs = _contact_pairs(
        replay["observed_handle_contact_pairs"],
        "EXACT_FCL_REPLAY_OBSERVED_PAIRS")
    _require(len(observed_pairs) > 0 and all(
        tuple(pair) in ALLOWED_HANDLE_CONTACT_PAIRS
        for pair in observed_pairs),
        "EXACT_FCL_REPLAY_OBSERVED_PAIRS_EXCLUSIVE")
    _require(replay["unauthorized_handle_contact_pairs"] == [],
             "EXACT_FCL_REPLAY_UNAUTHORIZED_PAIRS")
    transcript_sha = _sha(
        replay["replay_transcript_sha256"],
        "EXACT_FCL_REPLAY_TRANSCRIPT_SHA256")

    native = _exact(certificate["native_physics_contacts"], (
        "backend", "available", "reason", "force_or_wrench_claimed",
    ), "EXACT_FCL_REPLAY_NATIVE_CONTACTS")
    _require(native == {
        "backend": "DART",
        "available": False,
        "reason": NATIVE_CONTACT_UNAVAILABLE_REASON,
        "force_or_wrench_claimed": False,
    }, "EXACT_FCL_REPLAY_NATIVE_CONTACTS_UNAVAILABLE")
    _require(certificate["offline_replay"] is True and
             certificate["live_telemetry_bound"] is True and
             certificate["valid"] is True and
             certificate["status"] == "PASS",
             "EXACT_FCL_REPLAY_NOT_PASS")
    geometry_bindings = {
        identifier: _binding(geometry_files[identifier])
        for identifier in ("original_handle_stl", "scene_cdr", "urdf")
    }
    return {
        "schema": EXACT_FCL_REPLAY_SCHEMA,
        "certificate_sha256": replay_evidence.sha256,
        "rosbag_sha256": rosbag.sha256,
        "geometry_bindings_sha256": _canonical_hash(geometry_bindings),
        "auditor_source_sha256": source.sha256,
        "auditor_binary_sha256": binary.sha256,
        "auditor_argv_sha256": argv_sha,
        "auditor_argv_template_sha256": argv_template_sha,
        "auditor_stdout_sha256": stdout.sha256,
        "auditor_stderr_sha256": stderr.sha256,
        "source_joint_state_samples": source_samples,
        "densified_sample_count": dense_samples,
        "maximum_source_gap_ns": maximum_gap,
        "maximum_revolute_step_rad": revolute_step,
        "maximum_linear_step_m": linear_step,
        "left_handle_contact_count": 0,
        "minimum_left_handle_clearance_m": left_clearance,
        "right_contact_required_samples": required_samples,
        "right_patch_valid_samples": valid_samples,
        "right_nonpad_handle_contact_count": 0,
        "minimum_right_nonpad_handle_clearance_m": nonpad_clearance,
        "observed_handle_contact_pairs": observed_pairs,
        "replay_transcript_sha256": transcript_sha,
        "geometric_contact_exclusivity_passed": True,
        "native_physics_contacts_available": False,
        "native_physics_backend": "DART",
        "native_physics_unavailable_reason":
            NATIVE_CONTACT_UNAVAILABLE_REASON,
        "force_or_wrench_claimed": False,
    }


def _validate_exact_fcl_replay(
    value: Mapping[str, Any], replay_evidence: FileEvidence,
    plan: Mapping[str, Any], rosbag: FileEvidence,
    expected_telemetry: ReplayTelemetry,
) -> Mapping[str, Any]:
    certificate = _exact(value, (
        "schema", "run_id", "scope", "rosbag", "telemetry_tsv",
        "geometry", "auditor", "replay", "native_physics_contacts",
        "offline_replay", "live_telemetry_bound", "valid", "status",
    ), "EXACT_FCL_REPLAY")
    _require(certificate["schema"] == EXACT_FCL_REPLAY_SCHEMA,
             "EXACT_FCL_REPLAY_SCHEMA")
    _require(certificate["run_id"] == plan["run_id"],
             "EXACT_FCL_REPLAY_RUN_ID")
    _require(certificate["scope"] == _exact_fcl_scope(),
             "EXACT_FCL_REPLAY_SCOPE")
    replay_bag = _validate_file_binding(
        certificate["rosbag"], "EXACT_FCL_REPLAY_ROSBAG")
    _require(replay_bag.path == rosbag.path and
             replay_bag.sha256 == rosbag.sha256 and
             replay_bag.size_bytes == rosbag.size_bytes,
             "EXACT_FCL_REPLAY_ROSBAG_BINDING")
    telemetry = _validate_file_binding(
        certificate["telemetry_tsv"], "EXACT_FCL_REPLAY_TELEMETRY_TSV")
    _require(telemetry.raw == expected_telemetry.raw,
             "EXACT_FCL_REPLAY_TELEMETRY_NOT_DERIVED_FROM_DB3")

    geometry = _exact(certificate["geometry"], (
        "contact_path", "original_handle_stl", "scene_cdr", "urdf", "srdf",
    ), "EXACT_FCL_REPLAY_GEOMETRY")
    geometry_files = {}
    for identifier in (
            "contact_path", "original_handle_stl", "scene_cdr", "urdf",
            "srdf"):
        geometry_files[identifier] = _validate_file_binding(
            geometry[identifier], f"EXACT_FCL_REPLAY_{identifier}")
        _require(geometry_files[identifier].size_bytes > 0,
                 f"EXACT_FCL_REPLAY_{identifier}_EMPTY")

    auditor = _exact(certificate["auditor"], (
        "source", "path_base_source", "middle_base_source", "binary",
        "argv", "argv_sha256", "exit_code", "stdout", "stderr",
    ), "EXACT_FCL_REPLAY_AUDITOR")
    source = _validate_file_binding(
        auditor["source"], "EXACT_FCL_REPLAY_AUDITOR_SOURCE")
    path_base_source = _validate_file_binding(
        auditor["path_base_source"],
        "EXACT_FCL_REPLAY_AUDITOR_PATH_BASE_SOURCE")
    middle_base_source = _validate_file_binding(
        auditor["middle_base_source"],
        "EXACT_FCL_REPLAY_AUDITOR_MIDDLE_BASE_SOURCE")
    binary = _validate_file_binding(
        auditor["binary"], "EXACT_FCL_REPLAY_AUDITOR_BINARY")
    stdout = _validate_file_binding(
        auditor["stdout"], "EXACT_FCL_REPLAY_AUDITOR_STDOUT")
    stderr = _validate_file_binding(
        auditor["stderr"], "EXACT_FCL_REPLAY_AUDITOR_STDERR")
    _require(source.size_bytes > 0 and path_base_source.size_bytes > 0 and
             middle_base_source.size_bytes > 0 and binary.size_bytes > 0,
             "EXACT_FCL_REPLAY_AUDITOR_EMPTY")
    _require(stderr.raw == b"", "EXACT_FCL_REPLAY_AUDITOR_STDERR_NONEMPTY")
    prebound_files = {
        "auditor_source": source.sha256,
        "auditor_path_base_source": path_base_source.sha256,
        "auditor_middle_base_source": middle_base_source.sha256,
        "auditor_binary": binary.sha256,
        "contact_path": geometry_files["contact_path"].sha256,
        "original_handle_stl": geometry_files["original_handle_stl"].sha256,
        "scene_cdr": geometry_files["scene_cdr"].sha256,
        "urdf": geometry_files["urdf"].sha256,
        "srdf": geometry_files["srdf"].sha256,
    }
    _require(prebound_files == plan["expected_exact_fcl_file_sha256"],
             "EXACT_FCL_REPLAY_PREBOUND_FILE_SHA256")
    argv = auditor["argv"]
    _require(type(argv) is list and len(argv) == 13 and all(
        type(item) is str and item != "" and "\x00" not in item
        for item in argv), "EXACT_FCL_REPLAY_AUDITOR_ARGV")
    argv_sha = _argv_sha256(argv)
    _require(auditor["argv_sha256"] == argv_sha,
             "EXACT_FCL_REPLAY_AUDITOR_ARGV_SHA256")
    _require(hmac.compare_digest(
        argv_sha, plan["expected_exact_fcl_replay_argv_sha256"]),
        "EXACT_FCL_REPLAY_AUDITOR_PLANNED_ARGV")
    _require(argv[0] == str(binary.path),
             "EXACT_FCL_REPLAY_AUDITOR_EXECUTABLE")
    expected_flags = {
        "--scene": str(geometry_files["scene_cdr"].path),
        "--path": str(geometry_files["contact_path"].path),
        "--telemetry-tsv": str(telemetry.path),
        "--urdf": str(geometry_files["urdf"].path),
        "--srdf": str(geometry_files["srdf"].path),
        "--handle-stl": str(geometry_files["original_handle_stl"].path),
    }
    _require(set(argv[1::2]) == set(expected_flags),
             "EXACT_FCL_REPLAY_AUDITOR_FLAGS")
    for flag, expected in expected_flags.items():
        _require(_flag_value(
            argv, flag, f"EXACT_FCL_REPLAY_AUDITOR_FLAG:{flag}") == expected,
            f"EXACT_FCL_REPLAY_AUDITOR_FLAG_VALUE:{flag}")
    _require(auditor["exit_code"] == 0,
             "EXACT_FCL_REPLAY_AUDITOR_EXIT")

    try:
        stdout_text = stdout.raw.decode("ascii")
    except UnicodeDecodeError as error:
        raise LiveCertificateError(
            f"EXACT_FCL_REPLAY_AUDITOR_STDOUT_ENCODING:{error}") from error
    _require(stdout_text.endswith("\n") and "\r" not in stdout_text and
             "\x00" not in stdout_text,
             "EXACT_FCL_REPLAY_AUDITOR_STDOUT_FRAMING")
    lines = stdout_text.splitlines()
    _require(len(lines) == len(expected_telemetry.rows) + 1,
             "EXACT_FCL_REPLAY_AUDITOR_STDOUT_LINE_COUNT")
    row_results = []
    grip_engaged_count = 0
    pad1_valid_count = 0
    pad2_valid_count = 0
    pad1_contact_observed = False
    pad2_contact_observed = False
    for index, (line, expected) in enumerate(zip(
            lines[:-1], expected_telemetry.rows)):
        row = _parse_machine_line(
            line, "RIGHT_HANDLE_LIVE_FCL_ROW_829", FCL_ROW_KEYS,
            f"EXACT_FCL_REPLAY_ROW_{index}")
        _require(row["id"] == expected["id"],
                 f"EXACT_FCL_REPLAY_ROW_{index}_ID")
        _require(row["status"] == "PASS" and
                 _machine_bool(row["valid"],
                               f"EXACT_FCL_REPLAY_ROW_{index}_VALID") and
                 _machine_bool(
                     row["collision_free"],
                     f"EXACT_FCL_REPLAY_ROW_{index}_COLLISION_FREE") and
                 _machine_bool(
                     row["geometric_only"],
                     f"EXACT_FCL_REPLAY_ROW_{index}_GEOMETRIC_ONLY") and
                 not _machine_bool(
                     row["force_or_wrench_claimed"],
                     f"EXACT_FCL_REPLAY_ROW_{index}_FORCE_CLAIM"),
                 f"EXACT_FCL_REPLAY_ROW_{index}_NOT_PASS")
        contact_required = _machine_bool(
            row["contact_required"],
            f"EXACT_FCL_REPLAY_ROW_{index}_CONTACT_REQUIRED")
        _require(contact_required is expected["contact_required"],
                 f"EXACT_FCL_REPLAY_ROW_{index}_CONTACT_WINDOW")
        door_deg = _machine_float(
            row["door_angle_deg"], f"EXACT_FCL_REPLAY_ROW_{index}_DOOR")
        handle_deg = _machine_float(
            row["handle_angle_deg"], f"EXACT_FCL_REPLAY_ROW_{index}_HANDLE")
        torso = _machine_float(
            row["torso_rad"], f"EXACT_FCL_REPLAY_ROW_{index}_TORSO")
        _require(_close(door_deg, math.degrees(expected["door_angle_rad"])) and
                 _close(handle_deg,
                        math.degrees(expected["handle_angle_rad"])) and
                 _close(torso, expected["positions"]["torso_joint"]),
                 f"EXACT_FCL_REPLAY_ROW_{index}_ECHO")
        _require(abs(torso) <= MAX_ABS_TORSO_RAD,
                 f"EXACT_FCL_REPLAY_ROW_{index}_TORSO_LIMIT")
        pad1_valid = _machine_bool(
            row["pad1_patch_valid"],
            f"EXACT_FCL_REPLAY_ROW_{index}_PAD1")
        pad2_valid = _machine_bool(
            row["pad2_patch_valid"],
            f"EXACT_FCL_REPLAY_ROW_{index}_PAD2")
        grip_engaged = _machine_bool(
            row["right_pad_grip_engaged"],
            f"EXACT_FCL_REPLAY_ROW_{index}_RIGHT_PAD_GRIP_ENGAGED")
        pad1_contacts = _machine_integer(
            row["pad1_contacts"],
            f"EXACT_FCL_REPLAY_ROW_{index}_PAD1_CONTACTS")
        pad2_contacts = _machine_integer(
            row["pad2_contacts"],
            f"EXACT_FCL_REPLAY_ROW_{index}_PAD2_CONTACTS")
        pad1_distance = _machine_float(
            row["pad1_handle_distance_m"],
            f"EXACT_FCL_REPLAY_ROW_{index}_PAD1_DISTANCE")
        pad2_distance = _machine_float(
            row["pad2_handle_distance_m"],
            f"EXACT_FCL_REPLAY_ROW_{index}_PAD2_DISTANCE")
        minimum_right_pad_distance = min(pad1_distance, pad2_distance)
        _require(
            grip_engaged is
            (minimum_right_pad_distance <= RIGHT_PAD_GRIP_PROXIMITY_M),
            f"EXACT_FCL_REPLAY_ROW_{index}_GRIP_DISTANCE_BINDING")
        if contact_required:
            _require(grip_engaged and
                     minimum_right_pad_distance <=
                     RIGHT_PAD_GRIP_PROXIMITY_M,
                     f"EXACT_FCL_REPLAY_ROW_{index}_RIGHT_GRIP_REQUIRED")
        else:
            _require(not grip_engaged and
                     pad1_distance > RIGHT_PAD_GRIP_PROXIMITY_M and
                     pad2_distance > RIGHT_PAD_GRIP_PROXIMITY_M,
                     f"EXACT_FCL_REPLAY_ROW_{index}_RIGHT_GRIP_FORBIDDEN")
            _require(pad1_contacts == 0 and pad2_contacts == 0,
                     f"EXACT_FCL_REPLAY_ROW_{index}_NONCONTACT_PAD_CONTACT")
        grip_engaged_count += int(grip_engaged)
        pad1_valid_count += int(pad1_valid)
        pad2_valid_count += int(pad2_valid)
        pad1_contact_observed = pad1_contact_observed or pad1_contacts > 0
        pad2_contact_observed = pad2_contact_observed or pad2_contacts > 0
        nonpad_clearance = _machine_float(
            row["nonpad_handle_clearance_m"],
            f"EXACT_FCL_REPLAY_ROW_{index}_NONPAD_CLEARANCE")
        adjacent_finger_clearance = _machine_float(
            row["adjacent_finger_handle_clearance_m"],
            f"EXACT_FCL_REPLAY_ROW_{index}_ADJACENT_FINGER_CLEARANCE")
        left_clearance = _machine_float(
            row["left_handle_clearance_m"],
            f"EXACT_FCL_REPLAY_ROW_{index}_LEFT_CLEARANCE")
        _require(
            nonpad_clearance >= BUFFERED_NONPAD_HANDLE_CLEARANCE_M and
            adjacent_finger_clearance >=
            ADJACENT_FINGER_HANDLE_CLEARANCE_M and
            left_clearance >= LEFT_HANDLE_CLEARANCE_M,
                 f"EXACT_FCL_REPLAY_ROW_{index}_CLEARANCE")
        metrics = {
            "contact_required": contact_required,
            "grip_engaged": grip_engaged,
            "pad1_valid": pad1_valid,
            "pad2_valid": pad2_valid,
            "pad1_contacts": pad1_contacts,
            "pad2_contacts": pad2_contacts,
            "pad1_distance": pad1_distance,
            "pad2_distance": pad2_distance,
            "minimum_right_pad_distance": minimum_right_pad_distance,
            "nonpad_clearance": nonpad_clearance,
            "adjacent_finger_clearance": adjacent_finger_clearance,
            "left_clearance": left_clearance,
            "patch_span": _machine_float(
                row["patch_span_m"],
                f"EXACT_FCL_REPLAY_ROW_{index}_PATCH_SPAN"),
            "patch_station": _machine_float(
                row["patch_station_m"],
                f"EXACT_FCL_REPLAY_ROW_{index}_PATCH_STATION"),
            "patch_radius": _machine_float(
                row["patch_radius_m"],
                f"EXACT_FCL_REPLAY_ROW_{index}_PATCH_RADIUS"),
        }
        row_results.append(metrics)

    summary = _parse_machine_line(
        lines[-1], "RIGHT_HANDLE_LIVE_FCL_SUMMARY_829", FCL_SUMMARY_KEYS,
        "EXACT_FCL_REPLAY_SUMMARY")
    evaluated = len(expected_telemetry.rows)
    required_count = sum(
        row["contact_required"] for row in expected_telemetry.rows)
    noncontact_count = evaluated - required_count
    _require(summary["status"] == "PASS" and
             _machine_bool(summary["valid"], "EXACT_FCL_SUMMARY_VALID") and
             _machine_bool(
                 summary["collision_free"],
                 "EXACT_FCL_SUMMARY_COLLISION_FREE") and
             _machine_bool(
                 summary["geometric_only"],
                 "EXACT_FCL_SUMMARY_GEOMETRIC_ONLY") and
             not _machine_bool(
                 summary["force_or_wrench_claimed"],
                 "EXACT_FCL_SUMMARY_FORCE_CLAIM") and
             summary["reason"] == "NONE",
             "EXACT_FCL_REPLAY_SUMMARY_NOT_PASS")
    expected_counts = {
        "evaluated": evaluated,
        "passed": evaluated,
        "contact_required": required_count,
        "contact_required_passed": required_count,
        "noncontact": noncontact_count,
        "noncontact_passed": noncontact_count,
        "pad1_patch_valid": pad1_valid_count,
        "pad2_patch_valid": pad2_valid_count,
        "right_pad_grip_engaged": grip_engaged_count,
    }
    for key, expected in expected_counts.items():
        _require(_machine_integer(
            summary[key], f"EXACT_FCL_SUMMARY_{key}") == expected,
            f"EXACT_FCL_REPLAY_SUMMARY_COUNT:{key}")
    maximum_torso = max(
        abs(row["positions"]["torso_joint"])
        for row in expected_telemetry.rows)
    summary_max_torso = _machine_float(
        summary["max_torso_rad"], "EXACT_FCL_SUMMARY_MAX_TORSO")
    maximum_contact_minimum_right_pad_distance = max(
        row["minimum_right_pad_distance"] for row in row_results
        if row["contact_required"])
    minimum_nonpad = min(row["nonpad_clearance"] for row in row_results)
    minimum_adjacent_finger = min(
        row["adjacent_finger_clearance"] for row in row_results)
    minimum_left = min(row["left_clearance"] for row in row_results)
    minimum_patch_span = min(
        row["patch_span"] for row in row_results
        if row["contact_required"])
    minimum_patch_station = min(
        row["patch_station"] for row in row_results
        if row["contact_required"])
    minimum_patch_radius = min(
        row["patch_radius"] for row in row_results
        if row["contact_required"])
    summary_metrics = {
        "max_torso_rad": maximum_torso,
        "max_contact_minimum_right_pad_distance_m":
            maximum_contact_minimum_right_pad_distance,
        "min_nonpad_handle_clearance_m": minimum_nonpad,
        "min_adjacent_finger_handle_clearance_m":
            minimum_adjacent_finger,
        "min_left_handle_clearance_m": minimum_left,
        "min_patch_span_m": minimum_patch_span,
        "min_patch_station_m": minimum_patch_station,
        "min_patch_radius_m": minimum_patch_radius,
    }
    for key, expected in summary_metrics.items():
        _require(_close(_machine_float(
            summary[key], f"EXACT_FCL_SUMMARY_{key}"), expected),
            f"EXACT_FCL_REPLAY_SUMMARY_METRIC:{key}")
    _require(summary_max_torso <= MAX_ABS_TORSO_RAD and
             _close(summary_max_torso, maximum_torso),
             "EXACT_FCL_REPLAY_SUMMARY_TORSO")
    expected_hashes = {
        "scene_sha256": geometry_files["scene_cdr"].sha256,
        "path_sha256": geometry_files["contact_path"].sha256,
        "telemetry_sha256": telemetry.sha256,
        "urdf_sha256": geometry_files["urdf"].sha256,
        "srdf_sha256": geometry_files["srdf"].sha256,
        "handle_stl_sha256": geometry_files["original_handle_stl"].sha256,
    }
    for key, expected in expected_hashes.items():
        _require(summary[key] == expected,
                 f"EXACT_FCL_REPLAY_SUMMARY_HASH:{key}")

    replay = _exact(certificate["replay"], (
        "source_joint_state_samples", "densified_sample_count",
        "maximum_source_gap_ns", "maximum_revolute_step_rad",
        "maximum_linear_step_m", "close_command_timestamp_ns",
        "close_pass_timestamp_ns", "release_command_timestamp_ns",
        "release_pass_timestamp_ns", "excluded_close_transition_rows",
        "excluded_release_transition_rows", "collision_free",
        "left_handle_checks", "left_handle_contact_count",
        "minimum_left_handle_clearance_m",
        "right_contact_required_samples", "right_grip_engaged_samples",
        "right_pad1_patch_valid_samples",
        "right_pad2_patch_valid_samples",
        "maximum_contact_minimum_right_pad_distance_m",
        "minimum_adjacent_finger_handle_clearance_m",
        "minimum_buffered_nonpad_handle_clearance_m",
        "allowed_handle_contact_pairs", "observed_handle_contact_pairs",
        "unauthorized_handle_contact_pairs", "replay_transcript_sha256",
    ), "EXACT_FCL_REPLAY_METRICS")
    _require(
        replay["source_joint_state_samples"] ==
        expected_telemetry.source_sample_count and
        replay["densified_sample_count"] == evaluated and
        replay["maximum_source_gap_ns"] ==
        expected_telemetry.maximum_source_gap_ns and
        _close(_finite(
            replay["maximum_revolute_step_rad"],
            "EXACT_FCL_REPLAY_REVOLUTE_STEP"),
            expected_telemetry.maximum_revolute_step_rad) and
        _close(_finite(
            replay["maximum_linear_step_m"],
            "EXACT_FCL_REPLAY_LINEAR_STEP"),
            expected_telemetry.maximum_linear_step_m),
        "EXACT_FCL_REPLAY_DENSIFICATION_BINDING")
    _require(
        replay["close_command_timestamp_ns"] ==
        expected_telemetry.close_command_timestamp_ns and
        replay["close_pass_timestamp_ns"] ==
        expected_telemetry.close_pass_timestamp_ns and
        replay["release_command_timestamp_ns"] ==
        expected_telemetry.release_command_timestamp_ns and
        replay["release_pass_timestamp_ns"] ==
        expected_telemetry.release_pass_timestamp_ns and
        replay["excluded_close_transition_rows"] ==
        expected_telemetry.excluded_close_transition_rows and
        replay["excluded_release_transition_rows"] ==
        expected_telemetry.excluded_release_transition_rows,
        "EXACT_FCL_REPLAY_TRANSITION_WINDOW_BINDING")
    _require(replay["left_handle_contact_count"] == 0,
             "EXACT_FCL_REPLAY_LEFT_HANDLE_CONTACT")
    right_required = _integer(
        replay["right_contact_required_samples"],
        "EXACT_FCL_REPLAY_RIGHT_REQUIRED", 1)
    right_grip_engaged = _integer(
        replay["right_grip_engaged_samples"],
        "EXACT_FCL_REPLAY_RIGHT_GRIP_ENGAGED", 1)
    right_pad1_patch_valid = _integer(
        replay["right_pad1_patch_valid_samples"],
        "EXACT_FCL_REPLAY_RIGHT_PAD1_PATCH_VALID")
    right_pad2_patch_valid = _integer(
        replay["right_pad2_patch_valid_samples"],
        "EXACT_FCL_REPLAY_RIGHT_PAD2_PATCH_VALID")
    replay_maximum_contact_minimum_right_pad_distance = _finite(
        replay["maximum_contact_minimum_right_pad_distance_m"],
        "EXACT_FCL_REPLAY_MAX_CONTACT_MINIMUM_RIGHT_PAD_DISTANCE")
    replay_minimum_adjacent_finger_clearance = _finite(
        replay["minimum_adjacent_finger_handle_clearance_m"],
        "EXACT_FCL_REPLAY_MINIMUM_ADJACENT_FINGER_CLEARANCE")
    replay_minimum_buffered_nonpad_clearance = _finite(
        replay["minimum_buffered_nonpad_handle_clearance_m"],
        "EXACT_FCL_REPLAY_MINIMUM_BUFFERED_NONPAD_CLEARANCE")
    _require(
        right_required == required_count and
        right_grip_engaged == required_count and
        replay_maximum_contact_minimum_right_pad_distance <=
        RIGHT_PAD_GRIP_PROXIMITY_M and
        replay_minimum_adjacent_finger_clearance >=
        ADJACENT_FINGER_HANDLE_CLEARANCE_M and
        replay_minimum_buffered_nonpad_clearance >=
        BUFFERED_NONPAD_HANDLE_CLEARANCE_M,
        "EXACT_FCL_REPLAY_RIGHT_GRIP_CONTRACT")
    _require(replay["collision_free"] is True and
             replay["left_handle_checks"] == evaluated and
             replay["left_handle_contact_count"] == 0 and
             _close(_finite(
                 replay["minimum_left_handle_clearance_m"],
                 "EXACT_FCL_REPLAY_LEFT_CLEARANCE"), minimum_left) and
             right_required == required_count and
             right_grip_engaged == grip_engaged_count and
             right_pad1_patch_valid == pad1_valid_count and
             right_pad2_patch_valid == pad2_valid_count and
             _close(replay_maximum_contact_minimum_right_pad_distance,
                    maximum_contact_minimum_right_pad_distance) and
             _close(replay_minimum_adjacent_finger_clearance,
                    minimum_adjacent_finger) and
             _close(replay_minimum_buffered_nonpad_clearance,
                    minimum_nonpad) and
             replay["replay_transcript_sha256"] == stdout.sha256,
             "EXACT_FCL_REPLAY_METRIC_BINDING")
    allowed_pairs = _contact_pairs(
        replay["allowed_handle_contact_pairs"],
        "EXACT_FCL_REPLAY_ALLOWED_PAIRS")
    _require(allowed_pairs == [
        list(pair) for pair in ALLOWED_HANDLE_CONTACT_PAIRS],
        "EXACT_FCL_REPLAY_ALLOWED_PAIRS_EXACT")
    expected_observed_pairs = []
    if pad1_contact_observed:
        expected_observed_pairs.append(list(ALLOWED_HANDLE_CONTACT_PAIRS[0]))
    if pad2_contact_observed:
        expected_observed_pairs.append(list(ALLOWED_HANDLE_CONTACT_PAIRS[1]))
    observed_pairs = _contact_pairs(
        replay["observed_handle_contact_pairs"],
        "EXACT_FCL_REPLAY_OBSERVED_PAIRS")
    _require(observed_pairs == expected_observed_pairs and
             replay["unauthorized_handle_contact_pairs"] == [],
             "EXACT_FCL_REPLAY_CONTACT_PAIR_BINDING")

    native = _exact(certificate["native_physics_contacts"], (
        "backend", "available", "reason", "force_or_wrench_claimed",
    ), "EXACT_FCL_REPLAY_NATIVE_CONTACTS")
    _require(native == {
        "backend": "DART",
        "available": False,
        "reason": NATIVE_CONTACT_UNAVAILABLE_REASON,
        "force_or_wrench_claimed": False,
    }, "EXACT_FCL_REPLAY_NATIVE_CONTACTS_UNAVAILABLE")
    _require(certificate["offline_replay"] is True and
             certificate["live_telemetry_bound"] is True and
             certificate["valid"] is True and
             certificate["status"] == "PASS",
             "EXACT_FCL_REPLAY_NOT_PASS")
    geometry_bindings = {
        identifier: _binding(geometry_files[identifier])
        for identifier in (
            "contact_path", "original_handle_stl", "scene_cdr", "urdf",
            "srdf")
    }
    return {
        "schema": EXACT_FCL_REPLAY_SCHEMA,
        "scope": _exact_fcl_scope(),
        "certificate_sha256": replay_evidence.sha256,
        "rosbag_sha256": rosbag.sha256,
        "telemetry_tsv_sha256": telemetry.sha256,
        "geometry_bindings_sha256": _canonical_hash(geometry_bindings),
        "auditor_source_sha256": source.sha256,
        "auditor_path_base_source_sha256": path_base_source.sha256,
        "auditor_middle_base_source_sha256": middle_base_source.sha256,
        "auditor_binary_sha256": binary.sha256,
        "auditor_argv_sha256": argv_sha,
        "auditor_stdout_sha256": stdout.sha256,
        "auditor_stderr_sha256": stderr.sha256,
        "source_joint_state_samples": expected_telemetry.source_sample_count,
        "densified_sample_count": evaluated,
        "maximum_source_gap_ns": expected_telemetry.maximum_source_gap_ns,
        "maximum_revolute_step_rad":
            expected_telemetry.maximum_revolute_step_rad,
        "maximum_linear_step_m": expected_telemetry.maximum_linear_step_m,
        "close_command_timestamp_ns":
            expected_telemetry.close_command_timestamp_ns,
        "close_pass_timestamp_ns":
            expected_telemetry.close_pass_timestamp_ns,
        "release_command_timestamp_ns":
            expected_telemetry.release_command_timestamp_ns,
        "release_pass_timestamp_ns":
            expected_telemetry.release_pass_timestamp_ns,
        "excluded_close_transition_rows":
            expected_telemetry.excluded_close_transition_rows,
        "excluded_release_transition_rows":
            expected_telemetry.excluded_release_transition_rows,
        "left_handle_contact_count": 0,
        "minimum_left_handle_clearance_m": minimum_left,
        "right_contact_required_samples": required_count,
        "right_grip_engaged_samples": grip_engaged_count,
        "right_pad1_patch_valid_samples": pad1_valid_count,
        "right_pad2_patch_valid_samples": pad2_valid_count,
        "maximum_contact_minimum_right_pad_distance_m":
            maximum_contact_minimum_right_pad_distance,
        "minimum_adjacent_finger_handle_clearance_m":
            minimum_adjacent_finger,
        "minimum_buffered_nonpad_handle_clearance_m": minimum_nonpad,
        "observed_handle_contact_pairs": observed_pairs,
        "replay_transcript_sha256": stdout.sha256,
        "geometric_contact_exclusivity_passed": True,
        "native_physics_contacts_available": False,
        "native_physics_backend": "DART",
        "native_physics_unavailable_reason":
            NATIVE_CONTACT_UNAVAILABLE_REASON,
        "force_or_wrench_claimed": False,
    }


def _decode(
    decoder: CdrDeserializer, type_name: str, payload: bytes,
    status: str,
) -> Mapping[str, Any]:
    _require(type(payload) is bytes and 0 < len(payload) <= MAX_CDR_BYTES,
             f"{status}_CDR_SIZE")
    try:
        value = decoder.deserialize(type_name, payload)
    except LiveCertificateError:
        raise
    except Exception as error:
        raise LiveCertificateError(f"{status}_CDR:{error}") from error
    _ensure_finite_tree(value, f"{status}_FINITE")
    _require(type(value) is dict, f"{status}_TYPE")
    return value


def _joint_positions(
    value: Mapping[str, Any], expected_names: Sequence[str], status: str,
) -> Tuple[int, Mapping[str, float]]:
    state = _exact(value, ("header_stamp_ns", "names", "positions"), status)
    header_stamp_ns = _integer(
        state["header_stamp_ns"], f"{status}_HEADER_STAMP_NS")
    names = state["names"]
    positions = state["positions"]
    _require(type(names) is list and type(positions) is list and
             len(names) == len(positions) and len(names) > 0 and
             all(type(name) is str and name != "" for name in names) and
             len(set(names)) == len(names), f"{status}_JOINTS")
    _require(names == list(expected_names), f"{status}_JOINT_ORDER")
    normalized = {}
    for index, position in enumerate(positions):
        normalized[names[index]] = _finite(
            position, f"{status}_POSITION_{index}")
    return header_stamp_ns, {
        name: normalized[name] for name in expected_names}


def _validate_robot_joint_state(
    value: Mapping[str, Any], status: str,
) -> Tuple[int, Mapping[str, float]]:
    header_stamp_ns, positions = _joint_positions(
        value, ALL_ROBOT_JOINTS, status)
    torso = positions["torso_joint"]
    _require(abs(torso) <= MAX_ABS_TORSO_RAD,
             f"{status}_TORSO_LIMIT_44_5_DEG")
    return header_stamp_ns, positions


def _validate_incubator_joint_state(
    value: Mapping[str, Any], status: str,
) -> Tuple[int, Mapping[str, float]]:
    return _joint_positions(value, INCUBATOR_JOINTS, status)


def _validate_trajectory(
    value: Mapping[str, Any], topic: str, status: str,
) -> Mapping[str, Any]:
    trajectory = _exact(value, ("joint_names", "points"), status)
    _require(trajectory["joint_names"] == list(CONTROLLER_JOINTS[topic]),
             f"{status}_JOINT_NAMES")
    points = trajectory["points"]
    _require(type(points) is list and len(points) > 0, f"{status}_POINTS")
    normalized_points = []
    previous_time = -1
    for index, item in enumerate(points):
        point = _exact(
            item, ("positions", "time_from_start_ns"),
            f"{status}_POINT_{index}")
        _require(type(point["positions"]) is list and
                 len(point["positions"]) == len(CONTROLLER_JOINTS[topic]),
                 f"{status}_POINT_{index}_POSITIONS")
        positions = [
            _finite(position, f"{status}_POINT_{index}_POSITION_{axis}")
            for axis, position in enumerate(point["positions"])
        ]
        time_ns = _integer(
            point["time_from_start_ns"],
            f"{status}_POINT_{index}_TIME", 1)
        _require(time_ns > previous_time, f"{status}_POINT_TIME_ORDER")
        previous_time = time_ns
        if topic == "/torso_controller/joint_trajectory":
            _require(abs(positions[0]) <= MAX_ABS_TORSO_RAD,
                     f"{status}_TORSO_LIMIT_44_5_DEG")
        normalized_points.append({
            "positions": positions,
            "time_from_start_ns": time_ns,
        })
    return {
        "joint_names": list(CONTROLLER_JOINTS[topic]),
        "points": normalized_points,
    }


def _validate_clock(value: Mapping[str, Any], status: str) -> int:
    clock = _exact(value, ("clock_ns",), status)
    return _integer(clock["clock_ns"], f"{status}_CLOCK_NS")


def _open_rosbag(path: Path) -> sqlite3.Connection:
    uri = f"file:{path.as_posix()}?mode=ro&immutable=1"
    try:
        connection = sqlite3.connect(uri, uri=True)
        connection.execute("PRAGMA query_only=ON")
        return connection
    except sqlite3.Error as error:
        raise LiveCertificateError(f"ROSBAG_OPEN:{error}") from error


def _decimal(value: float) -> str:
    _require(math.isfinite(value), "REPLAY_TSV_NONFINITE")
    return "0" if value == 0.0 else format(value, ".17g")


def _replay_row_line(row: Mapping[str, Any]) -> str:
    positions = row["positions"]
    vector_groups = (
        LEG_JOINTS,
        CONTROLLER_JOINTS["/left_arm_controller/joint_trajectory"],
        CONTROLLER_JOINTS["/right_arm_controller/joint_trajectory"],
        CONTROLLER_JOINTS["/left_gripper_controller/joint_trajectory"],
        CONTROLLER_JOINTS["/right_gripper_controller/joint_trajectory"],
    )
    vectors = [
        ",".join(_decimal(positions[name]) for name in names)
        for names in vector_groups
    ]
    fields = (
        row["id"], "1" if row["contact_required"] else "0",
        _decimal(math.degrees(row["door_angle_rad"])),
        _decimal(math.degrees(row["handle_angle_rad"])),
        _decimal(positions["torso_joint"]), *vectors,
    )
    return "\t".join(fields)


def _interpolate_replay_row(
    left: Mapping[str, Any], right: Mapping[str, Any], timestamp_ns: int,
    contact_start_ns: int, contact_end_ns: int,
) -> Mapping[str, Any]:
    if timestamp_ns == right["timestamp_ns"]:
        identifier = f"bag_{right['message_id']}"
    else:
        identifier = (
            f"interp_{left['message_id']}_{right['message_id']}_"
            f"{timestamp_ns - left['timestamp_ns']}")
    duration = right["timestamp_ns"] - left["timestamp_ns"]
    _require(duration > 0, "REPLAY_TSV_SOURCE_TIME_ORDER")
    alpha = (timestamp_ns - left["timestamp_ns"]) / duration
    _require(0.0 < alpha <= 1.0, "REPLAY_TSV_INTERPOLATION_ALPHA")
    storage_timestamp_ns = left["storage_timestamp_ns"] + round(
        alpha * (right["storage_timestamp_ns"] -
                 left["storage_timestamp_ns"]))
    return {
        "id": identifier,
        "message_id": right["message_id"],
        "timestamp_ns": timestamp_ns,
        "storage_timestamp_ns": storage_timestamp_ns,
        "contact_required": (
            contact_start_ns <= storage_timestamp_ns <= contact_end_ns),
        "door_angle_rad": (
            left["door_angle_rad"] + alpha *
            (right["door_angle_rad"] - left["door_angle_rad"])),
        "handle_angle_rad": (
            left["handle_angle_rad"] + alpha *
            (right["handle_angle_rad"] - left["handle_angle_rad"])),
        "positions": {
            name: left["positions"][name] + alpha *
            (right["positions"][name] - left["positions"][name])
            for name in ALL_ROBOT_JOINTS
        },
    }


def _build_replay_telemetry(
    robot_samples: Sequence[Mapping[str, Any]],
    incubator_samples: Sequence[Mapping[str, Any]],
    markers: Sequence[Mapping[str, Any]],
    right_gripper_timestamps: Sequence[int],
) -> ReplayTelemetry:
    _require(len(robot_samples) >= 2 and len(incubator_samples) >= 2,
             "REPLAY_TSV_SOURCE_SAMPLES")
    _require(all(
        right["timestamp_ns"] > left["timestamp_ns"]
        for left, right in zip(robot_samples, robot_samples[1:])),
        "REPLAY_TSV_ROBOT_SIM_TIME_ORDER")
    _require(all(
        right["timestamp_ns"] > left["timestamp_ns"]
        for left, right in zip(incubator_samples, incubator_samples[1:])),
        "REPLAY_TSV_INCUBATOR_SIM_TIME_ORDER")
    joined = []
    mechanism_index = 0
    for robot in robot_samples:
        timestamp = robot["timestamp_ns"]
        while mechanism_index + 1 < len(incubator_samples):
            current_distance = abs(
                incubator_samples[mechanism_index]["timestamp_ns"] - timestamp)
            next_distance = abs(
                incubator_samples[mechanism_index + 1]["timestamp_ns"] -
                timestamp)
            if next_distance >= current_distance:
                break
            mechanism_index += 1
        mechanism = incubator_samples[mechanism_index]
        _require(abs(mechanism["timestamp_ns"] - timestamp) <=
                 MAX_JOINT_GAP_NS, "REPLAY_TSV_MECHANISM_PAIRING_GAP")
        door = mechanism["positions"]["door_hinge_joint"]
        handle = mechanism["positions"]["handle_latch_joint"]
        _require(-MECHANISM_PHYSICS_RANGE_TOLERANCE_RAD <= door <=
                 math.radians(60.0) +
                 MECHANISM_PHYSICS_RANGE_TOLERANCE_RAD,
                 "REPLAY_TSV_DOOR_RANGE")
        _require(-MECHANISM_PHYSICS_RANGE_TOLERANCE_RAD <= handle <=
                 math.radians(90.0) +
                 MECHANISM_PHYSICS_RANGE_TOLERANCE_RAD,
                 "REPLAY_TSV_HANDLE_RANGE")
        joined.append({
            "message_id": robot["message_id"],
            "storage_timestamp_ns": robot["storage_timestamp_ns"],
            "timestamp_ns": timestamp,
            "door_angle_rad": door,
            "handle_angle_rad": handle,
            "positions": robot["positions"],
        })

    marker_by_name = _marker_map(markers)
    _require(len(right_gripper_timestamps) == 2,
             "REPLAY_TSV_RIGHT_GRIPPER_COMMAND_COUNT")
    close_command, release_command = right_gripper_timestamps
    close_pass = marker_by_name[
        "PREFIX_CLOSE_Q0_RIGHT_HAND"]["bag_timestamp_ns"]
    release_pass = marker_by_name[
        "SUFFIX_RELEASE_Q0_RIGHT_HAND"]["bag_timestamp_ns"]
    _require(close_command < close_pass < release_command < release_pass,
             "REPLAY_TSV_GRIPPER_TRANSITION_ORDER")
    contact_start = close_pass
    contact_end = release_command
    _require(all(
        right["storage_timestamp_ns"] > left["storage_timestamp_ns"]
        for left, right in zip(joined, joined[1:])),
        "REPLAY_TSV_SOURCE_STORAGE_TIME_ORDER")

    boundary_sim_to_storage = {}
    for boundary_storage in (
            close_command, close_pass, release_command, release_pass):
        _require(
            joined[0]["storage_timestamp_ns"] <= boundary_storage <=
            joined[-1]["storage_timestamp_ns"],
            "REPLAY_TSV_TRANSITION_STORAGE_ENVELOPE")
        boundary_sim = None
        for left, right in zip(joined, joined[1:]):
            if (left["storage_timestamp_ns"] <= boundary_storage <=
                    right["storage_timestamp_ns"]):
                if boundary_storage == left["storage_timestamp_ns"]:
                    boundary_sim = left["timestamp_ns"]
                elif boundary_storage == right["storage_timestamp_ns"]:
                    boundary_sim = right["timestamp_ns"]
                else:
                    storage_duration = (
                        right["storage_timestamp_ns"] -
                        left["storage_timestamp_ns"])
                    alpha = ((boundary_storage -
                              left["storage_timestamp_ns"]) /
                             storage_duration)
                    boundary_sim = left["timestamp_ns"] + round(
                        alpha * (right["timestamp_ns"] -
                                 left["timestamp_ns"]))
                break
        _require(boundary_sim is not None,
                 "REPLAY_TSV_TRANSITION_STORAGE_MAPPING")
        _require(boundary_sim not in boundary_sim_to_storage,
                 "REPLAY_TSV_TRANSITION_SIM_COLLISION")
        boundary_sim_to_storage[boundary_sim] = boundary_storage
    first = joined[0]
    rows = [{
        "id": f"bag_{first['message_id']}",
        "message_id": first["message_id"],
        "timestamp_ns": first["timestamp_ns"],
        "storage_timestamp_ns": first["storage_timestamp_ns"],
        "contact_required": (
            contact_start <= first["storage_timestamp_ns"] <= contact_end),
        "door_angle_rad": first["door_angle_rad"],
        "handle_angle_rad": first["handle_angle_rad"],
        "positions": first["positions"],
    }]
    revolute_names = (
        "torso_joint",
        *LEG_JOINTS,
        *CONTROLLER_JOINTS["/left_arm_controller/joint_trajectory"],
        *CONTROLLER_JOINTS["/right_arm_controller/joint_trajectory"],
    )
    linear_names = (
        *CONTROLLER_JOINTS["/left_gripper_controller/joint_trajectory"],
        *CONTROLLER_JOINTS["/right_gripper_controller/joint_trajectory"],
    )
    for left, right in zip(joined, joined[1:]):
        duration = right["timestamp_ns"] - left["timestamp_ns"]
        _require(0 < duration <= MAX_JOINT_GAP_NS,
                 "REPLAY_TSV_SOURCE_GAP")
        maximum_revolute_delta = max(
            [abs(right["positions"][name] - left["positions"][name])
             for name in revolute_names] +
            [abs(right["door_angle_rad"] - left["door_angle_rad"]),
             abs(right["handle_angle_rad"] - left["handle_angle_rad"])])
        maximum_linear_delta = max(
            abs(right["positions"][name] - left["positions"][name])
            for name in linear_names)
        subdivisions = max(
            1,
            math.ceil(maximum_revolute_delta /
                      MAX_REPLAY_REVOLUTE_STEP_RAD - 1e-12),
            math.ceil(maximum_linear_delta /
                      MAX_REPLAY_LINEAR_STEP_M - 1e-12),
        )
        timestamps = {
            left["timestamp_ns"] + round(duration * step / subdivisions)
            for step in range(1, subdivisions + 1)
        }
        for boundary_sim in boundary_sim_to_storage:
            if left["timestamp_ns"] < boundary_sim <= right["timestamp_ns"]:
                timestamps.add(boundary_sim)
        for timestamp in sorted(timestamps):
            row = dict(_interpolate_replay_row(
                left, right, timestamp, contact_start, contact_end))
            if timestamp in boundary_sim_to_storage:
                row["storage_timestamp_ns"] = boundary_sim_to_storage[
                    timestamp]
                row["contact_required"] = (
                    contact_start <= row["storage_timestamp_ns"] <=
                    contact_end)
            rows.append(row)
    filtered_rows = []
    excluded_close = 0
    excluded_release = 0
    for row in rows:
        storage_timestamp = row["storage_timestamp_ns"]
        if close_command <= storage_timestamp < close_pass:
            excluded_close += 1
            continue
        if release_command <= storage_timestamp <= release_pass:
            excluded_release += 1
            continue
        normalized = dict(row)
        if storage_timestamp < close_command:
            normalized["contact_required"] = False
            normalized["audit_interval"] = "pre_close_transition"
        elif close_pass <= storage_timestamp < release_command:
            normalized["contact_required"] = True
            normalized["audit_interval"] = "stable_contact"
        else:
            normalized["contact_required"] = False
            normalized["audit_interval"] = "post_release_transition"
        filtered_rows.append(normalized)
    rows = filtered_rows
    _require(excluded_close > 0 and excluded_release > 0,
             "REPLAY_TSV_TRANSITION_WINDOWS_EMPTY")
    _require(any(row["contact_required"] for row in rows) and
             any(not row["contact_required"] for row in rows),
             "REPLAY_TSV_CONTACT_CATEGORIES")

    maximum_revolute_step = 0.0
    maximum_linear_step = 0.0
    for left, right in zip(rows, rows[1:]):
        if left["audit_interval"] != right["audit_interval"]:
            continue
        maximum_revolute_step = max(
            maximum_revolute_step,
            *(abs(right["positions"][name] - left["positions"][name])
              for name in revolute_names),
            abs(right["door_angle_rad"] - left["door_angle_rad"]),
            abs(right["handle_angle_rad"] - left["handle_angle_rad"]),
        )
        maximum_linear_step = max(
            maximum_linear_step,
            *(abs(right["positions"][name] - left["positions"][name])
              for name in linear_names),
        )
    _require(maximum_revolute_step <= MAX_REPLAY_REVOLUTE_STEP_RAD + 1e-12,
             "REPLAY_TSV_REVOLUTE_STEP")
    _require(maximum_linear_step <= MAX_REPLAY_LINEAR_STEP_M + 1e-12,
             "REPLAY_TSV_LINEAR_STEP")
    raw = (REPLAY_TSV_HEADER + "\n" +
           "\n".join(_replay_row_line(row) for row in rows) + "\n").encode(
               "ascii")
    source_gaps = [
        right["timestamp_ns"] - left["timestamp_ns"]
        for left, right in zip(joined, joined[1:])]
    return ReplayTelemetry(
        raw=raw, rows=tuple(rows), source_sample_count=len(joined),
        maximum_source_gap_ns=max(source_gaps),
        maximum_revolute_step_rad=maximum_revolute_step,
        maximum_linear_step_m=maximum_linear_step,
        close_command_timestamp_ns=close_command,
        close_pass_timestamp_ns=close_pass,
        release_command_timestamp_ns=release_command,
        release_pass_timestamp_ns=release_pass,
        excluded_close_transition_rows=excluded_close,
        excluded_release_transition_rows=excluded_release)


def _validate_gripper_stage_topology(
    controller_times: Mapping[str, Sequence[int]],
    markers: Sequence[Mapping[str, Any]],
) -> list[Mapping[str, Any]]:
    """Bind each gripper command to its one authorized completed stage."""
    marker_by_name = _marker_map(markers)
    stages = (
        ("right_gripper", "/right_gripper_controller/joint_trajectory",
         "PREFIX_INGRESS_OPEN_TO_Q0", "PREFIX_CLOSE_Q0_RIGHT_HAND"),
        ("right_gripper", "/right_gripper_controller/joint_trajectory",
         "PREFIX_CLOSE_Q0_RIGHT_HAND", "PREFIX_CONTACT_ARC_Q0_Q90"),
        ("right_gripper", "/right_gripper_controller/joint_trajectory",
         "Q90_AFTER_MIDDLE", "SUFFIX_CONTACT_ARC_Q90_Q0"),
        ("right_gripper", "/right_gripper_controller/joint_trajectory",
         "SUFFIX_CONTACT_ARC_Q90_Q0", "SUFFIX_RELEASE_Q0_RIGHT_HAND"),
    )
    normalized = []
    assigned = {topic: 0 for topic in (
        "/right_gripper_controller/joint_trajectory",
        "/left_gripper_controller/joint_trajectory")}
    for label, topic, after_name, through_name in stages:
        after = marker_by_name[after_name]["bag_timestamp_ns"]
        through = marker_by_name[through_name]["bag_timestamp_ns"]
        timestamps = [
            timestamp for timestamp in controller_times[topic]
            if after < timestamp <= through
        ]
        _require(len(timestamps) == 1,
                 f"ROSBAG_{label.upper()}_STAGE_TOPOLOGY:{through_name}")
        assigned[topic] += 1
        normalized.append({
            "controller": label,
            "after_marker": after_name,
            "through_marker": through_name,
            "command_storage_timestamp_ns": timestamps[0],
        })
    for topic, count in assigned.items():
        if topic == "/right_gripper_controller/joint_trajectory":
            _require(count == len(controller_times[topic]),
                 f"ROSBAG_GRIPPER_UNASSIGNED_COMMAND:{topic}")
    left_topic = "/left_gripper_controller/joint_trajectory"
    middle_start = marker_by_name["Q90_BEFORE_MIDDLE"]["bag_timestamp_ns"]
    middle_end = marker_by_name["MIDDLE_PASS"]["bag_timestamp_ns"]
    left_timestamps = [
        timestamp for timestamp in controller_times[left_topic]
        if middle_start < timestamp <= middle_end
    ]
    _require(left_timestamps == list(controller_times[left_topic]) and
             len(left_timestamps) == 2 and
             left_timestamps[0] < left_timestamps[1],
             "ROSBAG_LEFT_GRIPPER_MIDDLE_WINDOW")
    normalized.append({
        "controller": "left_gripper",
        "after_marker": "Q90_BEFORE_MIDDLE",
        "through_marker": "MIDDLE_PASS",
        "command_storage_timestamps_ns": left_timestamps,
        "phase_attribution": "sealed_middle_plan_payload",
    })
    return normalized


def _analyze_rosbag(
    evidence: FileEvidence, markers: Sequence[Mapping[str, Any]],
    decoder: CdrDeserializer,
) -> Tuple[Mapping[str, Any], ReplayTelemetry]:
    connection = _open_rosbag(evidence.path)
    try:
        tables = {row[0] for row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}
        _require({"topics", "messages"}.issubset(tables),
                 "ROSBAG_TABLES")
        topic_columns = {row[1] for row in connection.execute(
            "PRAGMA table_info(topics)")}
        message_columns = {row[1] for row in connection.execute(
            "PRAGMA table_info(messages)")}
        _require({"id", "name", "type", "serialization_format"}.issubset(
            topic_columns), "ROSBAG_TOPIC_COLUMNS")
        _require({"id", "topic_id", "timestamp", "data"}.issubset(
            message_columns), "ROSBAG_MESSAGE_COLUMNS")
        topic_rows = list(connection.execute(
            "SELECT id,name,type,serialization_format FROM topics ORDER BY id"))
        topics: Dict[str, Tuple[int, str]] = {}
        ids = set()
        for topic_id, name, type_name, serialization_format in topic_rows:
            _require(type(topic_id) is int and topic_id > 0 and
                     topic_id not in ids, "ROSBAG_TOPIC_ID")
            ids.add(topic_id)
            _require(type(name) is str and name not in topics,
                     "ROSBAG_TOPIC_NAME")
            _require(type(type_name) is str and
                     serialization_format == "cdr", "ROSBAG_TOPIC_FORMAT")
            topics[name] = (topic_id, type_name)
        for name, type_name in TOPIC_TYPES.items():
            _require(name in topics and topics[name][1] == type_name,
                     f"ROSBAG_REQUIRED_TOPIC:{name}")
        trajectory_topics = {
            name for name, (_, type_name) in topics.items()
            if type_name == "trajectory_msgs/msg/JointTrajectory"}
        _require(trajectory_topics == set(CONTROLLER_COUNTS),
                 "ROSBAG_UNEXPECTED_TRAJECTORY_TOPIC")

        relevant_by_id = {
            topic_id: (name, type_name)
            for name, (topic_id, type_name) in topics.items()
            if name in TOPIC_TYPES
        }
        rows = connection.execute(
            "SELECT id,topic_id,timestamp,data FROM messages ORDER BY "
            "timestamp,id")
        controller_counts = {topic: 0 for topic in CONTROLLER_COUNTS}
        controller_times = {topic: [] for topic in CONTROLLER_COUNTS}
        transcript = []
        joint_storage_times = []
        joint_sim_times = []
        robot_samples = []
        incubator_storage_times = []
        incubator_sim_times = []
        incubator_samples = []
        torso_values = []
        clocks = []
        marker_messages = []
        first_bag_timestamp = None
        last_bag_timestamp = None
        previous_key = None
        seen_message_ids = set()
        for message_id, topic_id, timestamp, payload in rows:
            _require(type(message_id) is int and message_id > 0 and
                     message_id not in seen_message_ids,
                     "ROSBAG_MESSAGE_ID")
            seen_message_ids.add(message_id)
            _require(type(topic_id) is int and type(timestamp) is int and
                     timestamp > 0 and type(payload) is bytes,
                     "ROSBAG_MESSAGE_ROW")
            key = (timestamp, message_id)
            _require(previous_key is None or key > previous_key,
                     "ROSBAG_MESSAGE_ORDER")
            previous_key = key
            if first_bag_timestamp is None:
                first_bag_timestamp = timestamp
            last_bag_timestamp = timestamp
            if topic_id not in relevant_by_id:
                continue
            topic, type_name = relevant_by_id[topic_id]
            decoded = _decode(
                decoder, type_name, payload,
                f"ROSBAG_MESSAGE_{message_id}")
            if topic == "/joint_states":
                sim_timestamp, positions = _validate_robot_joint_state(
                    decoded, f"ROSBAG_JOINT_STATE_{message_id}")
                joint_storage_times.append(timestamp)
                joint_sim_times.append(sim_timestamp)
                torso_values.append(positions["torso_joint"])
                robot_samples.append({
                    "message_id": message_id,
                    "storage_timestamp_ns": timestamp,
                    "timestamp_ns": sim_timestamp,
                    "positions": positions,
                })
            elif topic == "/incubator/joint_states":
                sim_timestamp, positions = _validate_incubator_joint_state(
                    decoded, f"ROSBAG_INCUBATOR_STATE_{message_id}")
                incubator_storage_times.append(timestamp)
                incubator_sim_times.append(sim_timestamp)
                incubator_samples.append({
                    "message_id": message_id,
                    "storage_timestamp_ns": timestamp,
                    "timestamp_ns": sim_timestamp,
                    "positions": positions,
                })
            elif topic == "/clock":
                clocks.append((timestamp, _validate_clock(
                    decoded, f"ROSBAG_CLOCK_{message_id}")))
            elif topic == MARKER_TOPIC:
                marker_value = _exact(
                    decoded, ("data",), f"ROSBAG_MARKER_{message_id}")
                _require(type(marker_value["data"]) is str,
                         f"ROSBAG_MARKER_{message_id}_DATA")
                marker_messages.append((
                    message_id, timestamp, marker_value["data"]))
            else:
                normalized = _validate_trajectory(
                    decoded, topic, f"ROSBAG_TRAJECTORY_{message_id}")
                controller_counts[topic] += 1
                controller_times[topic].append(timestamp)
                transcript.append({
                    "message_id": message_id,
                    "timestamp_ns": timestamp,
                    "topic": topic,
                    "trajectory": normalized,
                })
    except sqlite3.Error as error:
        raise LiveCertificateError(f"ROSBAG_SQL:{error}") from error
    finally:
        connection.close()

    _require(first_bag_timestamp is not None and last_bag_timestamp is not None,
             "ROSBAG_EMPTY")
    _require(len(marker_messages) == len(markers),
             "ROSBAG_MARKER_COUNT")
    marker_payload_transcript = []
    for index, ((message_id, storage_timestamp, data), marker) in enumerate(
            zip(marker_messages, markers)):
        try:
            raw = data.encode("ascii")
        except UnicodeEncodeError as error:
            raise LiveCertificateError(
                f"ROSBAG_MARKER_{index}_ASCII:{error}") from error
        payload = _parse_json(raw, f"ROSBAG_MARKER_{index}")
        _ensure_finite_tree(payload, f"ROSBAG_MARKER_{index}_FINITE")
        expected_payload = {
            "schema": MARKER_PAYLOAD_SCHEMA,
            "run_id": marker["run_id"],
            "sequence": marker["sequence"],
            "monotonic_ns": marker["monotonic_ns"],
            "process_role": marker["process_role"],
            "name": marker["name"],
            "line": marker["line"],
        }
        _require(raw == _canonical_bytes(payload)[:-1] and
                 payload == expected_payload and
                 storage_timestamp == marker["bag_timestamp_ns"],
                 f"ROSBAG_MARKER_{index}_BINDING")
        marker_payload_transcript.append({
            "message_id": message_id,
            "storage_timestamp_ns": storage_timestamp,
            "payload": payload,
        })
    marker_start = markers[0]["bag_timestamp_ns"]
    marker_end = markers[-1]["bag_timestamp_ns"]
    _require(first_bag_timestamp <= marker_start and
             last_bag_timestamp >= marker_end,
             "ROSBAG_FULL_COVERAGE")
    _require(controller_counts == CONTROLLER_COUNTS,
             "ROSBAG_CONTROLLER_MESSAGE_COUNTS")
    for topic, timestamps in controller_times.items():
        _require(all(marker_start <= timestamp <= marker_end
                     for timestamp in timestamps),
                 f"ROSBAG_CONTROLLER_WINDOW:{topic}")
    _require(len(joint_storage_times) >= 2 and
             joint_storage_times[0] <= marker_start and
             joint_storage_times[-1] >= marker_end,
             "ROSBAG_JOINT_STATE_COVERAGE")
    joint_gaps = [right - left for left, right in zip(
        joint_sim_times, joint_sim_times[1:])]
    _require(all(gap == ROBOT_JOINT_PERIOD_NS for gap in joint_gaps),
             "ROSBAG_JOINT_STATE_SIM_TIME_GAP")
    _require(len(incubator_storage_times) >= 2 and
             incubator_storage_times[0] <= marker_start and
             incubator_storage_times[-1] >= marker_end,
             "ROSBAG_INCUBATOR_STATE_COVERAGE")
    incubator_gaps = [right - left for left, right in zip(
        incubator_sim_times, incubator_sim_times[1:])]
    _require(all(gap == NATIVE_SIM_PERIOD_NS for gap in incubator_gaps),
             "ROSBAG_INCUBATOR_STATE_SIM_TIME_GAP")
    _require(len(clocks) >= 2 and clocks[0][0] <= marker_start and
             clocks[-1][0] >= marker_end,
             "ROSBAG_CLOCK_COVERAGE")
    _require(all(right[0] > left[0] and
                 right[1] - left[1] == NATIVE_SIM_PERIOD_NS
                 for left, right in zip(clocks, clocks[1:])),
             "ROSBAG_CLOCK_ORDER")
    _require(
        clocks[0][1] <= joint_sim_times[0] <= joint_sim_times[-1] <=
        clocks[-1][1] and
        clocks[0][1] <= incubator_sim_times[0] <=
        incubator_sim_times[-1] <= clocks[-1][1],
        "ROSBAG_SIM_TIME_CLOCK_ENVELOPE")
    marker_by_name = _marker_map(markers)
    middle_start = marker_by_name["Q90_BEFORE_MIDDLE"]["bag_timestamp_ns"]
    middle_end = marker_by_name["Q90_AFTER_MIDDLE"]["bag_timestamp_ns"]
    middle_right_gripper = sum(
        middle_start <= timestamp <= middle_end
        for timestamp in controller_times[
            "/right_gripper_controller/joint_trajectory"])
    _require(middle_right_gripper == 0,
             "ROSBAG_MIDDLE_RIGHT_GRIPPER_NONZERO")
    gripper_topology = _validate_gripper_stage_topology(
        controller_times, markers)
    max_abs_torso = max(abs(value) for value in torso_values)
    _require(max_abs_torso <= MAX_ABS_TORSO_RAD,
             "ROSBAG_TORSO_LIMIT_44_5_DEG")
    initial_legs = robot_samples[0]["positions"]
    maximum_leg_joint_drift = max(
        abs(sample["positions"][name] - initial_legs[name])
        for sample in robot_samples for name in LEG_JOINTS)
    replay_telemetry = _build_replay_telemetry(
        robot_samples, incubator_samples, markers,
        (controller_times[
            "/right_gripper_controller/joint_trajectory"][0],
         controller_times[
            "/right_gripper_controller/joint_trajectory"][-1]))
    summary = {
        "topic_types": dict(sorted(TOPIC_TYPES.items())),
        "controller_message_counts": dict(sorted(controller_counts.items())),
        "total_controller_messages": sum(controller_counts.values()),
        "middle_right_gripper_messages": middle_right_gripper,
        "gripper_stage_topology": gripper_topology,
        "controller_transcript_sha256": _canonical_hash(transcript),
        "controller_payload_sha256_by_topic": {
            topic: _canonical_hash([
                row["trajectory"] for row in transcript
                if row["topic"] == topic])
            for topic in sorted(CONTROLLER_COUNTS)
        },
        "bag_marker_message_count": len(marker_messages),
        "bag_marker_payload_transcript_sha256": _canonical_hash(
            marker_payload_transcript),
        "joint_state_samples": len(joint_sim_times),
        "joint_state_first_sim_timestamp_ns": joint_sim_times[0],
        "joint_state_last_sim_timestamp_ns": joint_sim_times[-1],
        "joint_state_first_storage_timestamp_ns": joint_storage_times[0],
        "joint_state_last_storage_timestamp_ns": joint_storage_times[-1],
        "maximum_joint_state_gap_ns": max(joint_gaps),
        "maximum_abs_torso_rad": max_abs_torso,
        "torso_limit_rad": MAX_ABS_TORSO_RAD,
        "leg_joint_names": list(LEG_JOINTS),
        "maximum_leg_joint_drift_rad": maximum_leg_joint_drift,
        "torso_telemetry_sha256": _canonical_hash([
            [timestamp, value]
            for timestamp, value in zip(joint_sim_times, torso_values)]),
        "robot_joint_telemetry_sha256": _canonical_hash(robot_samples),
        "incubator_joint_state_samples": len(incubator_sim_times),
        "incubator_joint_state_first_sim_timestamp_ns":
            incubator_sim_times[0],
        "incubator_joint_state_last_sim_timestamp_ns":
            incubator_sim_times[-1],
        "incubator_joint_state_first_storage_timestamp_ns":
            incubator_storage_times[0],
        "incubator_joint_state_last_storage_timestamp_ns":
            incubator_storage_times[-1],
        "maximum_incubator_joint_state_gap_ns": max(incubator_gaps),
        "incubator_joint_telemetry_sha256": _canonical_hash(
            incubator_samples),
        "replay_telemetry_tsv_sha256": hashlib.sha256(
            replay_telemetry.raw).hexdigest(),
        "replay_source_joint_state_samples":
            replay_telemetry.source_sample_count,
        "replay_densified_sample_count": len(replay_telemetry.rows),
        "replay_close_transition_excluded_rows":
            replay_telemetry.excluded_close_transition_rows,
        "replay_release_transition_excluded_rows":
            replay_telemetry.excluded_release_transition_rows,
        "clock_samples": len(clocks),
        "clock_first_ns": clocks[0][1],
        "clock_last_ns": clocks[-1][1],
        "bag_first_timestamp_ns": first_bag_timestamp,
        "bag_last_timestamp_ns": last_bag_timestamp,
        "full_marker_coverage": True,
    }
    return summary, replay_telemetry


def _validate_video_audit(
    value: Mapping[str, Any], run_id: str,
    markers: Sequence[Mapping[str, Any]],
) -> Tuple[FileEvidence, Mapping[str, Any]]:
    audit = _exact(value, (
        "schema", "run_id", "video", "capture_started_monotonic_ns",
        "capture_ended_monotonic_ns", "ffprobe", "decode", "valid",
        "status",
    ), "VIDEO")
    _require(audit["schema"] == VIDEO_SCHEMA, "VIDEO_SCHEMA")
    _require(audit["run_id"] == run_id, "VIDEO_RUN_ID")
    video = _validate_file_binding(audit["video"], "VIDEO_FILE")
    _require(video.path.suffix.lower() == ".mp4", "VIDEO_SUFFIX")
    started = _integer(
        audit["capture_started_monotonic_ns"], "VIDEO_STARTED", 1)
    ended = _integer(
        audit["capture_ended_monotonic_ns"], "VIDEO_ENDED", 1)
    _require(started <= markers[0]["monotonic_ns"] and
             ended >= markers[-1]["monotonic_ns"] and started < ended,
             "VIDEO_FULL_COVERAGE")
    probe = _exact(audit["ffprobe"], (
        "exit_code", "stream_count", "codec_name", "width", "height",
        "pix_fmt", "avg_frame_rate_num", "avg_frame_rate_den",
        "duration_s", "frame_count",
    ), "VIDEO_FFPROBE")
    _require(probe["exit_code"] == 0 and probe["stream_count"] == 1 and
             probe["codec_name"] == "h264" and
             probe["pix_fmt"] == "yuv420p", "VIDEO_FFPROBE_FORMAT")
    width = _integer(probe["width"], "VIDEO_WIDTH", 1)
    height = _integer(probe["height"], "VIDEO_HEIGHT", 1)
    numerator = _integer(
        probe["avg_frame_rate_num"], "VIDEO_FPS_NUM", 1)
    denominator = _integer(
        probe["avg_frame_rate_den"], "VIDEO_FPS_DEN", 1)
    duration = _finite(probe["duration_s"], "VIDEO_DURATION")
    frames = _integer(probe["frame_count"], "VIDEO_FRAME_COUNT", 1)
    _require(duration > 0.0 and
             abs(frames - duration * numerator / denominator) <= 1.0,
             "VIDEO_FRAME_MATH")
    decode = _exact(audit["decode"], (
        "exit_code", "stderr_size_bytes", "stderr_sha256",
    ), "VIDEO_DECODE")
    _require(decode["exit_code"] == 0 and
             decode["stderr_size_bytes"] == 0 and
             decode["stderr_sha256"] == EMPTY_SHA256,
             "VIDEO_DECODE_FAILED")
    _require(audit["valid"] is True and audit["status"] == "PASS",
             "VIDEO_NOT_PASS")
    return video, {
        "capture_started_monotonic_ns": started,
        "capture_ended_monotonic_ns": ended,
        "codec_name": probe["codec_name"],
        "width": width,
        "height": height,
        "pix_fmt": probe["pix_fmt"],
        "avg_frame_rate_num": numerator,
        "avg_frame_rate_den": denominator,
        "duration_s": duration,
        "frame_count": frames,
        "full_marker_coverage": True,
        "full_decode_passed": True,
    }


def _validate_gui_camera_audit(
    value: Mapping[str, Any], run_id: str, plan_sha256: str,
    markers: Sequence[Mapping[str, Any]],
    process_summary: Mapping[str, Any],
) -> Mapping[str, Any]:
    audit = _exact(value, (
        "schema", "run_id", "sealed_plan_sha256",
        "captured_monotonic_ns", "source_command_argv",
        "source_command_argv_sha256", "request", "response_stdout",
        "response_stderr", "exit_code", "valid", "status",
    ), "GUI_CAMERA")
    _require(audit["schema"] == GUI_CAMERA_SCHEMA and
             audit["run_id"] == run_id and
             audit["sealed_plan_sha256"] == plan_sha256,
             "GUI_CAMERA_HEADER")
    _require(audit["source_command_argv"] == list(GUI_CAMERA_ARGV) and
             audit["source_command_argv_sha256"] ==
             _argv_sha256(GUI_CAMERA_ARGV) and
             audit["request"] == GUI_CAMERA_REQUEST,
             "GUI_CAMERA_COMMAND_BINDING")
    captured = _integer(
        audit["captured_monotonic_ns"], "GUI_CAMERA_CAPTURED", 1)
    launch_interval = process_summary["process_intervals"]["launch"]
    _require(
        launch_interval["started_monotonic_ns"] <= captured <
        process_summary["recorder_ready_monotonic_ns"] and
        captured < markers[0]["monotonic_ns"],
        "GUI_CAMERA_TIME_ORDER")
    response = audit["response_stdout"]
    _require(type(response) is str and
             re.search(r"(?:^|\n)\s*data:\s*true\s*(?:\n|$)", response)
             is not None and
             audit["response_stderr"] == "" and
             audit["exit_code"] == 0 and audit["valid"] is True and
             audit["status"] == "PASS", "GUI_CAMERA_NOT_PASS")
    return {
        "schema": GUI_CAMERA_SCHEMA,
        "captured_monotonic_ns": captured,
        "source_command_argv_sha256": _argv_sha256(GUI_CAMERA_ARGV),
        "service": "/gui/move_to/pose",
        "request": GUI_CAMERA_REQUEST,
        "response_data": True,
    }


def _validate_video_window_audit(
    value: Mapping[str, Any], run_id: str, plan: Mapping[str, Any],
    plan_sha256: str, markers: Sequence[Mapping[str, Any]],
    process_summary: Mapping[str, Any], camera_summary: Mapping[str, Any],
    video_file: FileEvidence, video_summary: Mapping[str, Any],
) -> Mapping[str, Any]:
    audit = _exact(value, (
        "schema", "run_id", "sealed_plan_sha256", "helper_source_sha256",
        "helper_process_argv", "helper_process_argv_sha256",
        "capture_started_monotonic_ns", "xwindow_id_hex", "xwindow_id",
        "wm_class", "display", "window_width", "window_height", "width",
        "height", "crop", "window_tree_argv",
        "window_tree_argv_sha256", "window_details_argv",
        "window_details_argv_sha256", "ffmpeg_argv",
        "ffmpeg_argv_sha256", "valid", "status",
    ), "VIDEO_WINDOW")
    supervisor = plan["evidence_tools"]["supervisor_source"]
    video_process_argv = process_summary["process_argv"]["video"]
    _require(audit["schema"] == VIDEO_WINDOW_SCHEMA and
             audit["run_id"] == run_id and
             audit["sealed_plan_sha256"] == plan_sha256 and
             audit["helper_source_sha256"] == supervisor["sha256"] and
             audit["helper_process_argv"] == video_process_argv and
             audit["helper_process_argv_sha256"] ==
             _argv_sha256(video_process_argv) ==
             plan["expected_process_argv_sha256"]["video"],
             "VIDEO_WINDOW_HELPER_BINDING")
    captured = _integer(
        audit["capture_started_monotonic_ns"], "VIDEO_WINDOW_CAPTURED", 1)
    video_interval = process_summary["process_intervals"]["video"]
    _require(camera_summary["captured_monotonic_ns"] < captured <=
             markers[0]["monotonic_ns"] and
             video_interval["started_monotonic_ns"] <= captured and
             video_interval["ended_monotonic_ns"] >=
             markers[-1]["monotonic_ns"],
             "VIDEO_WINDOW_TIME_ORDER")
    window_hex = audit["xwindow_id_hex"]
    window_id = _integer(audit["xwindow_id"], "VIDEO_WINDOW_ID", 1)
    _require(type(window_hex) is str and
             re.fullmatch(r"0x[0-9a-f]+", window_hex) is not None and
             int(window_hex, 16) == window_id and
             audit["wm_class"] == ["gz-sim-gui", "Gazebo GUI"] and
             type(audit["display"]) is str and audit["display"] != "",
             "VIDEO_WINDOW_UNIQUE_IDENTITY")
    _require(audit["window_width"] == 2000 and
             audit["window_height"] == 1690 and
             audit["width"] == video_summary["width"] == 1160 and
             audit["height"] == video_summary["height"] == 1530 and
             audit["crop"] == "1160:1530:0:80" and
             video_summary["avg_frame_rate_num"] == 12 and
             video_summary["avg_frame_rate_den"] == 1,
             "VIDEO_WINDOW_CROP_BINDING")
    xwininfo = _flag_value(
        video_process_argv, "--xwininfo", "VIDEO_WINDOW_XWININFO")
    ffmpeg = _flag_value(
        video_process_argv, "--ffmpeg", "VIDEO_WINDOW_FFMPEG")
    expected_tree = [xwininfo, "-root", "-tree"]
    expected_details = [xwininfo, "-id", window_hex]
    expected_ffmpeg = [
        ffmpeg, "-n", "-thread_queue_size", "512", "-f", "x11grab",
        "-window_id", str(window_id), "-draw_mouse", "0", "-framerate",
        "12", "-video_size", "2000x1690", "-i", audit["display"],
        "-an", "-vf", "crop=1160:1530:0:80", "-c:v", "h264_nvenc",
        "-preset", "p1", "-cq", "20", "-pix_fmt", "yuv420p",
        str(video_file.path),
    ]
    for name, actual, expected, digest in (
        ("TREE", audit["window_tree_argv"], expected_tree,
         audit["window_tree_argv_sha256"]),
        ("DETAILS", audit["window_details_argv"], expected_details,
         audit["window_details_argv_sha256"]),
        ("FFMPEG", audit["ffmpeg_argv"], expected_ffmpeg,
         audit["ffmpeg_argv_sha256"]),
    ):
        _require(actual == expected and digest == _argv_sha256(expected),
                 f"VIDEO_WINDOW_{name}_ARGV_BINDING")
    _require(_flag_value(
        video_process_argv, "--output", "VIDEO_WINDOW_VIDEO_OUTPUT") ==
        str(video_file.path) and audit["valid"] is True and
        audit["status"] == "CAPTURE_STARTED", "VIDEO_WINDOW_NOT_PASS")
    return {
        "schema": VIDEO_WINDOW_SCHEMA,
        "capture_started_monotonic_ns": captured,
        "helper_source_sha256": supervisor["sha256"],
        "helper_process_argv_sha256": _argv_sha256(video_process_argv),
        "xwindow_id_hex": window_hex,
        "xwindow_id": window_id,
        "wm_class": ["gz-sim-gui", "Gazebo GUI"],
        "display": audit["display"],
        "window_width": 2000,
        "window_height": 1690,
        "crop": "1160:1530:0:80",
        "output_width": 1160,
        "output_height": 1530,
        "ffmpeg_argv_sha256": _argv_sha256(expected_ffmpeg),
        "unique_window_passed": True,
    }


def _validate_postflight(
    value: Mapping[str, Any], run_id: str,
    markers: Sequence[Mapping[str, Any]],
) -> Mapping[str, Any]:
    postflight = _exact(value, (
        "schema", "run_id", "captured_monotonic_ns", "door_angle_rad",
        "handle_angle_rad", "torso_angle_rad", "latch_locked",
        "supports_detached", "robot_at_launch", "controller_count",
        "trajectory_publisher_count", "unexpected_executor_process_count",
        "clock_advancing", "valid", "status",
    ), "POSTFLIGHT")
    _require(postflight["schema"] == POSTFLIGHT_SCHEMA,
             "POSTFLIGHT_SCHEMA")
    _require(postflight["run_id"] == run_id, "POSTFLIGHT_RUN_ID")
    captured = _integer(
        postflight["captured_monotonic_ns"], "POSTFLIGHT_CAPTURED", 1)
    marker_by_name = _marker_map(markers)
    _require(
        marker_by_name["POSTFLIGHT_FINALIZE"]["monotonic_ns"] <= captured <=
        marker_by_name["CAPTURE_END"]["monotonic_ns"],
        "POSTFLIGHT_MARKER_WINDOW")
    door = _finite(postflight["door_angle_rad"], "POSTFLIGHT_DOOR")
    handle = _finite(postflight["handle_angle_rad"], "POSTFLIGHT_HANDLE")
    torso = _finite(postflight["torso_angle_rad"], "POSTFLIGHT_TORSO")
    _require(abs(door) <= 0.004 and abs(handle) <= 0.004,
             "POSTFLIGHT_Q0")
    _require(abs(torso) <= MAX_ABS_TORSO_RAD,
             "POSTFLIGHT_TORSO_LIMIT_44_5_DEG")
    _require(postflight["latch_locked"] is True and
             postflight["supports_detached"] is True and
             postflight["robot_at_launch"] is True and
             postflight["controller_count"] == 6 and
             postflight["trajectory_publisher_count"] == 0 and
             postflight["unexpected_executor_process_count"] == 0 and
             postflight["clock_advancing"] is True and
             postflight["valid"] is True and
             postflight["status"] == "PASS", "POSTFLIGHT_NOT_PASS")
    return {
        "captured_monotonic_ns": captured,
        "door_angle_rad": door,
        "handle_angle_rad": handle,
        "torso_angle_rad": torso,
        "latch_locked": True,
        "supports_detached": True,
        "robot_at_launch": True,
        "controller_count": 6,
        "trajectory_publisher_count": 0,
        "unexpected_executor_process_count": 0,
        "clock_advancing": True,
    }


def _pose_vector(value: Any, width: int, status: str) -> Tuple[float, ...]:
    _require(type(value) is list and len(value) == width, f"{status}_WIDTH")
    return tuple(_finite(item, f"{status}_{index}")
                 for index, item in enumerate(value))


def _pose_error(
    actual: Mapping[str, Any], expected: Mapping[str, Sequence[float]],
    status: str,
) -> Tuple[float, float]:
    pose = _exact(actual, (
        "entity_id", "position_m", "orientation_xyzw"), status)
    _integer(pose["entity_id"], f"{status}_ENTITY_ID", 1)
    position = _pose_vector(pose["position_m"], 3, f"{status}_POSITION")
    orientation = _pose_vector(
        pose["orientation_xyzw"], 4, f"{status}_ORIENTATION")
    norm = math.hypot(*orientation)
    _require(abs(norm - 1.0) <= 1e-6, f"{status}_QUATERNION_NORM")
    expected_position = tuple(expected["position_m"])
    expected_orientation = tuple(expected["orientation_xyzw"])
    position_error = math.hypot(*(
        position[index] - expected_position[index] for index in range(3)))
    expected_norm = math.hypot(*expected_orientation)
    _require(abs(expected_norm - 1.0) <= 1e-12,
             f"{status}_EXPECTED_QUATERNION")
    dot = abs(sum(
        orientation[index] * expected_orientation[index]
        for index in range(4)) / (norm * expected_norm))
    dot = min(1.0, max(0.0, dot))
    orientation_error = 2.0 * math.atan2(
        math.sqrt(max(0.0, 1.0 - dot * dot)), dot)
    return position_error, orientation_error


def _validate_workcell_pose_audit(
    value: Mapping[str, Any], run_id: str, plan_sha256: str,
    markers: Sequence[Mapping[str, Any]],
    delegation_summary: Mapping[str, Any],
) -> Mapping[str, Any]:
    audit = _exact(value, (
        "schema", "run_id", "sealed_plan_sha256", "world_name",
        "source_command_argv", "source_command_argv_sha256",
        "preflight_samples", "postflight_samples", "terminal_cube_target",
        "terminal_cube_sample_gates", "terminal_cube_gate_passed",
        "valid", "status",
    ), "WORKCELL_POSE")
    _require(audit["schema"] == WORKCELL_POSE_SCHEMA,
             "WORKCELL_POSE_SCHEMA")
    _require(audit["run_id"] == run_id and
             audit["sealed_plan_sha256"] == plan_sha256,
             "WORKCELL_POSE_RUN_BINDING")
    _require(audit["world_name"] == "h1_world" and
             audit["source_command_argv"] == list(WORKCELL_SOURCE_ARGV) and
             audit["source_command_argv_sha256"] ==
             _argv_sha256(WORKCELL_SOURCE_ARGV),
             "WORKCELL_POSE_SOURCE")
    preflight = audit["preflight_samples"]
    postflight = audit["postflight_samples"]
    _require(type(preflight) is list and len(preflight) == 3 and
             type(postflight) is list and len(postflight) == 3,
             "WORKCELL_POSE_SAMPLE_COUNT")
    expected = {
        "h1_2": {
            "position_m": (0.0, 0.0, 0.0),
            "orientation_xyzw": (0.0, 0.0, 0.0, 1.0)},
        "table": {
            "position_m": (0.600, 0.0, 0.0),
            "orientation_xyzw": (0.0, 0.0, 0.0, 1.0)},
        "incubator": {
            "position_m": (0.590, 0.0, 0.815),
            "orientation_xyzw": (0.0, 0.0, 0.0, 1.0)},
        "green_cube": {
            "position_m": (0.334, 0.377, 0.830),
            "orientation_xyzw": (
                0.0, 0.0, 0.7071067811865475,
                0.7071067811865476)},
    }
    delegated_target = delegation_summary["terminal_cube_target"]
    terminal_cube = {
        "position_m": tuple(delegated_target["position_m"]),
        "orientation_xyzw": tuple(delegated_target["orientation_xyzw"]),
    }
    target = _exact(audit["terminal_cube_target"], (
        "position_m", "orientation_xyzw", "position_tolerance_m",
        "orientation_tolerance_rad",
    ), "WORKCELL_POSE_TERMINAL_CUBE_TARGET")
    _require(target["position_m"] == list(terminal_cube["position_m"]) and
             target["orientation_xyzw"] ==
             list(terminal_cube["orientation_xyzw"]) and
             target["position_tolerance_m"] ==
             delegated_target["position_tolerance_m"] and
             target["orientation_tolerance_rad"] ==
             delegated_target["orientation_tolerance_rad"],
             "WORKCELL_POSE_TERMINAL_CUBE_TARGET_BINDING")
    marker_by_name = _marker_map(markers)
    windows = (
        (preflight, tuple(expected),
         marker_by_name["CAPTURE_START"]["monotonic_ns"],
         marker_by_name["SUPPORT_PRELOAD"]["monotonic_ns"], "PREFLIGHT"),
        (postflight, tuple(expected),
         marker_by_name["POSTFLIGHT_FINALIZE"]["monotonic_ns"],
         marker_by_name["CAPTURE_END"]["monotonic_ns"], "POSTFLIGHT"),
    )
    entity_ids: Dict[str, int] = {}
    maximum_position_error = 0.0
    maximum_orientation_error = 0.0
    first_entities: Mapping[str, Any] | None = None
    for samples, names, window_start, window_end, phase in windows:
        previous_timestamp = -1
        for index, sample_value in enumerate(samples):
            sample = _exact(sample_value, (
                "sequence", "monotonic_ns", "entities"),
                f"WORKCELL_POSE_{phase}_{index}")
            _require(sample["sequence"] == index,
                     f"WORKCELL_POSE_{phase}_{index}_SEQUENCE")
            timestamp = _integer(
                sample["monotonic_ns"],
                f"WORKCELL_POSE_{phase}_{index}_TIME", 1)
            _require(window_start <= timestamp < window_end and
                     timestamp > previous_timestamp,
                     f"WORKCELL_POSE_{phase}_{index}_WINDOW")
            previous_timestamp = timestamp
            entities = _exact(
                sample["entities"], names,
                f"WORKCELL_POSE_{phase}_{index}_ENTITIES")
            if phase == "PREFLIGHT" and index == 0:
                first_entities = entities
            _require(first_entities is not None,
                     "WORKCELL_POSE_PREFLIGHT_REFERENCE")
            ids_in_sample = set()
            for name in names:
                target_pose = expected[name] if phase == "PREFLIGHT" else (
                    terminal_cube if name == "green_cube" else {
                        "position_m": _pose_vector(
                            first_entities[name]["position_m"], 3,
                            f"WORKCELL_POSE_REFERENCE_{name}_POSITION"),
                        "orientation_xyzw": _pose_vector(
                            first_entities[name]["orientation_xyzw"], 4,
                            f"WORKCELL_POSE_REFERENCE_{name}_ORIENTATION"),
                    })
                position_error, orientation_error = _pose_error(
                    entities[name], target_pose,
                    f"WORKCELL_POSE_{phase}_{index}_{name}")
                entity_id = entities[name]["entity_id"]
                _require(entity_id not in ids_in_sample,
                         f"WORKCELL_POSE_{phase}_{index}_DUPLICATE_ID")
                ids_in_sample.add(entity_id)
                if name in entity_ids:
                    _require(entity_ids[name] == entity_id,
                             f"WORKCELL_POSE_{name}_IDENTITY")
                else:
                    entity_ids[name] = entity_id
                orientation_limit = (
                    WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD
                    if name == "green_cube" else
                    WORKCELL_STATIC_ORIENTATION_TOLERANCE_RAD)
                _require(position_error <= WORKCELL_POSITION_TOLERANCE_M and
                         orientation_error <= orientation_limit,
                         f"WORKCELL_POSE_{phase}_{index}_{name}_GATE")
                if phase == "PREFLIGHT" and index > 0:
                    stable_position, stable_orientation = _pose_error(
                        entities[name], first_entities[name],
                        f"WORKCELL_POSE_{phase}_{index}_{name}_STABILITY")
                    _require(
                        stable_position <= WORKCELL_POSITION_TOLERANCE_M and
                        stable_orientation <= orientation_limit,
                        f"WORKCELL_POSE_{phase}_{index}_{name}_STABILITY")
                maximum_position_error = max(
                    maximum_position_error, position_error)
                maximum_orientation_error = max(
                    maximum_orientation_error, orientation_error)
    recorded_gates = audit["terminal_cube_sample_gates"]
    _require(type(recorded_gates) is list and len(recorded_gates) == 3,
             "WORKCELL_POSE_TERMINAL_CUBE_GATE_COUNT")
    terminal_gates = []
    for index, (recorded, sample) in enumerate(zip(
            recorded_gates, postflight)):
        gate = _exact(recorded, (
            "sequence", "position_error_m", "orientation_error_rad",
            "position_gate_passed", "orientation_gate_passed", "passed",
        ), f"WORKCELL_POSE_TERMINAL_CUBE_GATE_{index}")
        position_error, orientation_error = _pose_error(
            sample["entities"]["green_cube"], terminal_cube,
            f"WORKCELL_POSE_TERMINAL_CUBE_GATE_{index}_POSE")
        _require(gate["sequence"] == index and
                 _close(_finite(
                     gate["position_error_m"],
                     f"WORKCELL_POSE_TERMINAL_CUBE_GATE_{index}_POSITION"),
                     position_error) and
                 _close(_finite(
                     gate["orientation_error_rad"],
                     f"WORKCELL_POSE_TERMINAL_CUBE_GATE_{index}_ORIENTATION"),
                     orientation_error) and
                 gate["position_gate_passed"] is True and
                 gate["orientation_gate_passed"] is True and
                 gate["passed"] is True and
                 position_error <= WORKCELL_POSITION_TOLERANCE_M and
                 orientation_error <=
                 WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD,
                 f"WORKCELL_POSE_TERMINAL_CUBE_GATE_{index}_BINDING")
        terminal_gates.append({
            "sequence": index,
            "position_error_m": position_error,
            "orientation_error_rad": orientation_error,
            "position_gate_passed": True,
            "orientation_gate_passed": True,
            "passed": True,
        })
    _require(audit["valid"] is True and audit["status"] == "PASS",
             "WORKCELL_POSE_NOT_PASS")
    _require(audit["terminal_cube_gate_passed"] is True,
             "WORKCELL_POSE_TERMINAL_CUBE_NOT_PASS")
    return {
        "schema": WORKCELL_POSE_SCHEMA,
        "world_name": "h1_world",
        "source_command_argv_sha256": _argv_sha256(WORKCELL_SOURCE_ARGV),
        "preflight_sample_count": 3,
        "postflight_sample_count": 3,
        "entity_ids": dict(sorted(entity_ids.items())),
        "expected_preflight_poses": {
            name: {
                "position_m": list(pose["position_m"]),
                "orientation_xyzw": list(pose["orientation_xyzw"]),
            } for name, pose in expected.items()
        },
        "expected_terminal_cube_pose": {
            "position_m": list(terminal_cube["position_m"]),
            "orientation_xyzw": list(terminal_cube["orientation_xyzw"]),
        },
        "terminal_cube_sample_gates": terminal_gates,
        "terminal_cube_gate_passed": True,
        "preflight_transcript_sha256": _canonical_hash(preflight),
        "postflight_transcript_sha256": _canonical_hash(postflight),
        "maximum_position_error_m": maximum_position_error,
        "maximum_orientation_error_rad": maximum_orientation_error,
        "initial_pose_gate_passed": True,
        "terminal_static_pose_gate_passed": True,
    }


def _load_inputs(
    *, plan_path: Path, plan_sha256: str, rosbag_path: Path,
    markers_path: Path, proc_snapshot_path: Path, video_audit_path: Path,
    postflight_path: Path, workcell_pose_audit_path: Path,
    gui_camera_audit_path: Path, video_window_audit_path: Path,
    formal_authorities_path: Path,
    exact_fcl_replay_path: Path,
    decoder: CdrDeserializer | None,
) -> Mapping[str, Any]:
    plan_evidence, plan_value = _load_canonical_json(
        plan_path, expected_sha256=plan_sha256, status="PLAN_FILE")
    plan = _validate_plan(plan_value)
    supplied_paths = {
        "rosbag": rosbag_path,
        "markers": markers_path,
        "proc_snapshot": proc_snapshot_path,
        "video_audit": video_audit_path,
        "postflight": postflight_path,
        "workcell_pose_audit": workcell_pose_audit_path,
        "gui_camera_audit": gui_camera_audit_path,
        "video_window_audit": video_window_audit_path,
        "formal_authorities": formal_authorities_path,
        "exact_fcl_replay": exact_fcl_replay_path,
    }
    for identifier, path in supplied_paths.items():
        _require(path.is_absolute() and str(path) ==
                 plan["input_paths"][identifier],
                 f"PLAN_INPUT_PATH_MISMATCH:{identifier}")

    rosbag = _read_sealed_file(
        rosbag_path, maximum_bytes=MAX_DB_BYTES, status="ROSBAG_FILE")
    for suffix in ("-wal", "-shm", "-journal"):
        sidecar = Path(str(rosbag_path) + suffix)
        _require(not sidecar.exists() and not sidecar.is_symlink(),
                 f"ROSBAG_UNSEALED_SIDECAR:{suffix}")
    markers_evidence, markers = _load_markers(
        markers_path, plan["run_id"])
    proc_evidence, proc_value = _load_canonical_json(
        proc_snapshot_path, status="PROC_FILE")
    video_audit_evidence, video_value = _load_canonical_json(
        video_audit_path, status="VIDEO_AUDIT_FILE")
    postflight_evidence, postflight_value = _load_canonical_json(
        postflight_path, status="POSTFLIGHT_FILE")
    workcell_evidence, workcell_value = _load_canonical_json(
        workcell_pose_audit_path, status="WORKCELL_POSE_FILE")
    camera_evidence, camera_value = _load_canonical_json(
        gui_camera_audit_path, status="GUI_CAMERA_FILE")
    window_evidence, window_value = _load_canonical_json(
        video_window_audit_path, status="VIDEO_WINDOW_FILE")
    formal_evidence, formal_value = _load_canonical_json(
        formal_authorities_path,
        expected_sha256=plan["formal_authorities_sha256"],
        status="FORMAL_FILE")
    replay_evidence, replay_value = _load_canonical_json(
        exact_fcl_replay_path, status="EXACT_FCL_REPLAY_FILE")

    formal_summary = _validate_formal_authorities(
        formal_value, plan["run_id"])
    process_summary = _validate_proc_snapshot(
        proc_value, plan, plan_evidence.sha256, plan_evidence.path, markers,
        formal_summary)
    delegation_summary = _validate_dynamic_cube_delegation(
        formal_summary, plan, process_summary["middle_speed_scale"])
    active_decoder = decoder if decoder is not None else RclpyCdrDeserializer()
    rosbag_summary, replay_telemetry = _analyze_rosbag(
        rosbag, markers, active_decoder)
    _require(
        rosbag_summary["controller_payload_sha256_by_topic"][
            "/left_gripper_controller/joint_trajectory"] ==
        delegation_summary["expected_left_gripper_payload_sha256"],
        "DELEGATION_LEFT_GRIPPER_LIVE_PAYLOAD_MISMATCH")
    replay_summary = _validate_exact_fcl_replay(
        replay_value, replay_evidence, plan, rosbag, replay_telemetry)
    _require(
        replay_summary["source_joint_state_samples"] ==
        rosbag_summary["joint_state_samples"] and
        replay_summary["maximum_source_gap_ns"] ==
        rosbag_summary["maximum_joint_state_gap_ns"],
        "EXACT_FCL_REPLAY_TELEMETRY_COVERAGE_MISMATCH")
    video_file, video_summary = _validate_video_audit(
        video_value, plan["run_id"], markers)
    camera_summary = _validate_gui_camera_audit(
        camera_value, plan["run_id"], plan_evidence.sha256, markers,
        process_summary)
    window_summary = _validate_video_window_audit(
        window_value, plan["run_id"], plan, plan_evidence.sha256, markers,
        process_summary, camera_summary, video_file, video_summary)
    workcell_summary = _validate_workcell_pose_audit(
        workcell_value, plan["run_id"], plan_evidence.sha256, markers,
        delegation_summary)
    postflight_summary = _validate_postflight(
        postflight_value, plan["run_id"], markers)
    evidence = {
        "plan": plan_evidence,
        "rosbag": rosbag,
        "markers": markers_evidence,
        "proc_snapshot": proc_evidence,
        "video_audit": video_audit_evidence,
        "postflight": postflight_evidence,
        "workcell_pose_audit": workcell_evidence,
        "gui_camera_audit": camera_evidence,
        "video_window_audit": window_evidence,
        "formal_authorities": formal_evidence,
        "exact_fcl_replay": replay_evidence,
        "video": video_file,
    }
    return {
        "plan": plan,
        "evidence": evidence,
        "markers": markers,
        "formal_summary": formal_summary,
        "delegation_summary": delegation_summary,
        "process_summary": process_summary,
        "rosbag_summary": rosbag_summary,
        "replay_summary": replay_summary,
        "video_summary": video_summary,
        "camera_summary": camera_summary,
        "window_summary": window_summary,
        "workcell_summary": workcell_summary,
        "postflight_summary": postflight_summary,
    }


def _certificate_value(loaded: Mapping[str, Any]) -> Mapping[str, Any]:
    plan = loaded["plan"]
    markers = loaded["markers"]
    evidence = loaded["evidence"]
    marker_by_name = _marker_map(markers)
    motion_start = marker_by_name["MOTION_WINDOW_START"]
    motion_end = marker_by_name["MOTION_WINDOW_END"]
    printed_start = int(motion_start["line"].rsplit("=", 1)[1])
    printed_end = int(motion_end["line"].rsplit("=", 1)[1])
    return {
        "schema": CERTIFICATE_SCHEMA,
        "verifier_version": VERIFIER_VERSION,
        "run_id": plan["run_id"],
        "inputs": {
            identifier: _binding(evidence[identifier])
            for identifier in (
                "plan", "rosbag", "markers", "proc_snapshot",
                "video_audit", "postflight", "workcell_pose_audit",
                "gui_camera_audit", "video_window_audit",
                "formal_authorities", "exact_fcl_replay", "video")
        },
        "formal_authorities": loaded["formal_summary"],
        "evidence_tools": {
            identifier: dict(plan["evidence_tools"][identifier])
            for identifier in EVIDENCE_TOOL_IDS
        },
        "dynamic_cube_delegation": loaded["delegation_summary"],
        "fresh_launch": loaded["process_summary"],
        "markers": {
            "count": len(markers),
            "ordered_names": list(MARKER_NAMES),
            "first_monotonic_ns": markers[0]["monotonic_ns"],
            "last_monotonic_ns": markers[-1]["monotonic_ns"],
            "first_bag_timestamp_ns": markers[0]["bag_timestamp_ns"],
            "last_bag_timestamp_ns": markers[-1]["bag_timestamp_ns"],
            "marker_transcript_sha256": _canonical_hash(list(markers)),
            "support_state_sequence": list(SUPPORT_SEQUENCE),
            "motion_window": {
                "maximum_ns": MAX_FAST_MOTION_WINDOW_NS,
                "printed_start_monotonic_ns": printed_start,
                "printed_end_monotonic_ns": printed_end,
                "printed_duration_ns": printed_end - printed_start,
                "observed_duration_ns": (
                    motion_end["monotonic_ns"] -
                    motion_start["monotonic_ns"]),
                "within_90_seconds": True,
            },
            "strict_order_passed": True,
        },
        "telemetry_and_commands": loaded["rosbag_summary"],
        "contact_evidence": loaded["replay_summary"],
        "workcell_pose": loaded["workcell_summary"],
        "gui_camera": loaded["camera_summary"],
        "video_window": loaded["window_summary"],
        "video": loaded["video_summary"],
        "postflight": loaded["postflight_summary"],
        "offline_only": False,
        "live_execution_passed": True,
        "valid": True,
        "status": "PASS",
    }


def _validate_certificate_shape(value: Mapping[str, Any]) -> None:
    certificate = _exact(value, (
        "schema", "verifier_version", "run_id", "inputs",
        "formal_authorities", "evidence_tools", "dynamic_cube_delegation",
        "fresh_launch", "markers",
        "telemetry_and_commands", "contact_evidence", "workcell_pose",
        "gui_camera", "video_window", "video", "postflight",
        "offline_only", "live_execution_passed", "valid", "status",
    ), "CERTIFICATE")
    _require(certificate["schema"] == CERTIFICATE_SCHEMA,
             "CERTIFICATE_SCHEMA")
    _require(certificate["verifier_version"] == VERIFIER_VERSION,
             "CERTIFICATE_VERSION")
    _run_id(certificate["run_id"], "CERTIFICATE_RUN_ID")
    _exact(certificate["inputs"], (
        "plan", "rosbag", "markers", "proc_snapshot", "video_audit",
        "postflight", "workcell_pose_audit", "gui_camera_audit",
        "video_window_audit", "formal_authorities", "exact_fcl_replay",
        "video",
    ), "CERTIFICATE_INPUTS")
    tools = _exact(
        certificate["evidence_tools"], EVIDENCE_TOOL_IDS,
        "CERTIFICATE_EVIDENCE_TOOLS")
    for identifier in EVIDENCE_TOOL_IDS:
        evidence = _validate_file_binding(
            tools[identifier], f"CERTIFICATE_EVIDENCE_TOOL:{identifier}")
        _require(evidence.mode == "0500",
                 f"CERTIFICATE_EVIDENCE_TOOL_MODE:{identifier}")
    for identifier, binding in certificate["inputs"].items():
        _exact(binding, ("path", "sha256", "size_bytes", "mode"),
               f"CERTIFICATE_INPUT_{identifier}")
        _canonical_absolute_path(
            binding["path"], f"CERTIFICATE_INPUT_{identifier}_PATH")
        _sha(binding["sha256"], f"CERTIFICATE_INPUT_{identifier}_SHA")
        _integer(binding["size_bytes"],
                 f"CERTIFICATE_INPUT_{identifier}_SIZE")
        _require(binding["mode"] == "0400",
                 f"CERTIFICATE_INPUT_{identifier}_MODE")
    _require(certificate["offline_only"] is False and
             certificate["live_execution_passed"] is True and
             certificate["valid"] is True and
             certificate["status"] == "PASS", "CERTIFICATE_NOT_PASS")
    _ensure_finite_tree(certificate, "CERTIFICATE_FINITE")


def _validate_output_parent(path: Path) -> None:
    _require(path.is_absolute(), "OUTPUT_NOT_ABSOLUTE")
    _require(os.path.normpath(str(path)) == str(path), "OUTPUT_PATH_ALIAS")
    parent = path.parent
    try:
        metadata = parent.lstat()
        resolved = parent.resolve(strict=True)
    except OSError as error:
        raise LiveCertificateError(f"OUTPUT_PARENT:{error}") from error
    _require(stat.S_ISDIR(metadata.st_mode) and not parent.is_symlink() and
             resolved == parent and metadata.st_uid == os.geteuid(),
             "OUTPUT_PARENT_IDENTITY")
    _require(not path.exists() and not path.is_symlink(), "OUTPUT_EXISTS")


def _atomic_publish_no_replace(path: Path, raw: bytes) -> None:
    _validate_output_parent(path)
    descriptor = -1
    temporary: Path | None = None
    linked = False
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.stage.", dir=path.parent)
        temporary = Path(temporary_name)
        offset = 0
        while offset < len(raw):
            written = os.write(descriptor, raw[offset:])
            _require(written > 0, "OUTPUT_SHORT_WRITE")
            offset += written
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o400)
        os.close(descriptor)
        descriptor = -1
        os.link(temporary, path, follow_symlinks=False)
        linked = True
        temporary.unlink()
        temporary = None
        directory_descriptor = os.open(
            path.parent,
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
            getattr(os, "O_CLOEXEC", 0))
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
        published = path.lstat()
        _require(stat.S_ISREG(published.st_mode) and
                 stat.S_IMODE(published.st_mode) == 0o400 and
                 published.st_nlink == 1 and
                 published.st_uid == os.geteuid(),
                 "OUTPUT_IDENTITY")
    except LiveCertificateError:
        if linked:
            try:
                path.unlink()
            except OSError:
                pass
        raise
    except OSError as error:
        if linked:
            try:
                path.unlink()
            except OSError:
                pass
        raise LiveCertificateError(f"OUTPUT_ATOMIC_NO_REPLACE:{error}") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def extract_fcl_telemetry(
    *, plan_path: Path, plan_sha256: str, rosbag_path: Path,
    markers_path: Path, output_path: Path,
    decoder: CdrDeserializer | None = None,
) -> CertificateArtifact:
    """Deterministically derive and seal the exact-FCL TSV from the db3."""
    plan_evidence, plan_value = _load_canonical_json(
        plan_path, expected_sha256=plan_sha256, status="PLAN_FILE")
    del plan_evidence
    plan = _validate_plan(plan_value)
    _require(str(rosbag_path) == plan["input_paths"]["rosbag"] and
             str(markers_path) == plan["input_paths"]["markers"],
             "EXTRACT_FCL_PLAN_PATHS")
    rosbag = _read_sealed_file(
        rosbag_path, maximum_bytes=MAX_DB_BYTES, status="ROSBAG_FILE")
    for suffix in ("-wal", "-shm", "-journal"):
        sidecar = Path(str(rosbag_path) + suffix)
        _require(not sidecar.exists() and not sidecar.is_symlink(),
                 f"ROSBAG_UNSEALED_SIDECAR:{suffix}")
    _, markers = _load_markers(markers_path, plan["run_id"])
    active_decoder = decoder if decoder is not None else RclpyCdrDeserializer()
    summary, telemetry = _analyze_rosbag(rosbag, markers, active_decoder)
    digest = hashlib.sha256(telemetry.raw).hexdigest()
    _atomic_publish_no_replace(output_path, telemetry.raw)
    return CertificateArtifact(summary, telemetry.raw, digest, output_path)


def wrap_exact_fcl_replay(
    *, plan_path: Path, plan_sha256: str, rosbag_path: Path,
    markers_path: Path, telemetry_tsv_path: Path, scene_path: Path,
    contact_path: Path, urdf_path: Path, srdf_path: Path,
    handle_stl_path: Path, auditor_source_path: Path,
    auditor_path_base_source_path: Path,
    auditor_middle_base_source_path: Path,
    auditor_binary_path: Path, stdout_path: Path, stderr_path: Path,
    exit_code: int, output_path: Path,
    decoder: CdrDeserializer | None = None,
) -> CertificateArtifact:
    """Bind one completed C++ run into the separate replay certificate."""
    plan_evidence, plan_value = _load_canonical_json(
        plan_path, expected_sha256=plan_sha256, status="PLAN_FILE")
    del plan_evidence
    plan = _validate_plan(plan_value)
    _require(str(rosbag_path) == plan["input_paths"]["rosbag"] and
             str(markers_path) == plan["input_paths"]["markers"] and
             str(output_path) == plan["input_paths"]["exact_fcl_replay"],
             "WRAP_FCL_PLAN_PATHS")
    rosbag = _read_sealed_file(
        rosbag_path, maximum_bytes=MAX_DB_BYTES, status="ROSBAG_FILE")
    for suffix in ("-wal", "-shm", "-journal"):
        sidecar = Path(str(rosbag_path) + suffix)
        _require(not sidecar.exists() and not sidecar.is_symlink(),
                 f"ROSBAG_UNSEALED_SIDECAR:{suffix}")
    _, markers = _load_markers(markers_path, plan["run_id"])
    active_decoder = decoder if decoder is not None else RclpyCdrDeserializer()
    _, expected_telemetry = _analyze_rosbag(
        rosbag, markers, active_decoder)
    telemetry = _read_sealed_file(
        telemetry_tsv_path, maximum_bytes=MAX_JSONL_BYTES,
        status="EXACT_FCL_REPLAY_TELEMETRY_TSV")
    _require(telemetry.raw == expected_telemetry.raw,
             "EXACT_FCL_REPLAY_TELEMETRY_NOT_DERIVED_FROM_DB3")
    files = {
        "contact_path": _read_sealed_file(
            contact_path, status="EXACT_FCL_REPLAY_contact_path"),
        "original_handle_stl": _read_sealed_file(
            handle_stl_path, status="EXACT_FCL_REPLAY_original_handle_stl"),
        "scene_cdr": _read_sealed_file(
            scene_path, status="EXACT_FCL_REPLAY_scene_cdr"),
        "urdf": _read_sealed_file(
            urdf_path, status="EXACT_FCL_REPLAY_urdf"),
        "srdf": _read_sealed_file(
            srdf_path, status="EXACT_FCL_REPLAY_srdf"),
        "auditor_source": _read_sealed_file(
            auditor_source_path, status="EXACT_FCL_REPLAY_AUDITOR_SOURCE"),
        "auditor_path_base_source": _read_sealed_file(
            auditor_path_base_source_path,
            status="EXACT_FCL_REPLAY_AUDITOR_PATH_BASE_SOURCE"),
        "auditor_middle_base_source": _read_sealed_file(
            auditor_middle_base_source_path,
            status="EXACT_FCL_REPLAY_AUDITOR_MIDDLE_BASE_SOURCE"),
        "auditor_binary": _read_sealed_file(
            auditor_binary_path, expected_mode="0500",
            status="EXACT_FCL_REPLAY_AUDITOR_BINARY"),
        "stdout": _read_sealed_file(
            stdout_path, maximum_bytes=MAX_DB_BYTES,
            status="EXACT_FCL_REPLAY_AUDITOR_STDOUT"),
        "stderr": _read_sealed_file(
            stderr_path, maximum_bytes=MAX_JSON_BYTES,
            status="EXACT_FCL_REPLAY_AUDITOR_STDERR"),
    }
    actual_prebound = {
        identifier: files[identifier].sha256
        for identifier in FCL_PREBOUND_FILE_IDS
    }
    _require(actual_prebound == plan["expected_exact_fcl_file_sha256"],
             "EXACT_FCL_REPLAY_PREBOUND_FILE_SHA256")
    argv = [
        str(auditor_binary_path),
        "--scene", str(scene_path),
        "--path", str(contact_path),
        "--telemetry-tsv", str(telemetry_tsv_path),
        "--urdf", str(urdf_path),
        "--srdf", str(srdf_path),
        "--handle-stl", str(handle_stl_path),
    ]
    _require(_argv_sha256(argv) ==
             plan["expected_exact_fcl_replay_argv_sha256"],
             "EXACT_FCL_REPLAY_AUDITOR_PLANNED_ARGV")
    _require(exit_code == 0, "EXACT_FCL_REPLAY_AUDITOR_EXIT")
    try:
        stdout_text = files["stdout"].raw.decode("ascii")
    except UnicodeDecodeError as error:
        raise LiveCertificateError(
            f"EXACT_FCL_REPLAY_AUDITOR_STDOUT_ENCODING:{error}") from error
    _require(stdout_text.endswith("\n"),
             "EXACT_FCL_REPLAY_AUDITOR_STDOUT_FRAMING")
    stdout_lines = stdout_text.splitlines()
    _require(len(stdout_lines) == len(expected_telemetry.rows) + 1,
             "EXACT_FCL_REPLAY_AUDITOR_STDOUT_LINE_COUNT")
    summary = _parse_machine_line(
        stdout_lines[-1], "RIGHT_HANDLE_LIVE_FCL_SUMMARY_829",
        FCL_SUMMARY_KEYS, "EXACT_FCL_REPLAY_SUMMARY")
    required_count = _machine_integer(
        summary["contact_required"], "EXACT_FCL_SUMMARY_CONTACT_REQUIRED")
    pad1_count = _machine_integer(
        summary["pad1_patch_valid"], "EXACT_FCL_SUMMARY_PAD1")
    pad2_count = _machine_integer(
        summary["pad2_patch_valid"], "EXACT_FCL_SUMMARY_PAD2")
    grip_engaged_count = _machine_integer(
        summary["right_pad_grip_engaged"],
        "EXACT_FCL_SUMMARY_RIGHT_PAD_GRIP_ENGAGED")
    pad1_contact_observed = False
    pad2_contact_observed = False
    for index, line in enumerate(stdout_lines[:-1]):
        row = _parse_machine_line(
            line, "RIGHT_HANDLE_LIVE_FCL_ROW_829", FCL_ROW_KEYS,
            f"EXACT_FCL_REPLAY_ROW_{index}")
        pad1_contact_observed = pad1_contact_observed or (
            _machine_integer(
                row["pad1_contacts"],
                f"EXACT_FCL_REPLAY_ROW_{index}_PAD1_CONTACTS") > 0)
        pad2_contact_observed = pad2_contact_observed or (
            _machine_integer(
                row["pad2_contacts"],
                f"EXACT_FCL_REPLAY_ROW_{index}_PAD2_CONTACTS") > 0)
    observed_pairs = []
    if pad1_contact_observed:
        observed_pairs.append(list(ALLOWED_HANDLE_CONTACT_PAIRS[0]))
    if pad2_contact_observed:
        observed_pairs.append(list(ALLOWED_HANDLE_CONTACT_PAIRS[1]))
    geometry = {
        "contact_path": _binding(files["contact_path"]),
        "original_handle_stl": _binding(files["original_handle_stl"]),
        "scene_cdr": _binding(files["scene_cdr"]),
        "urdf": _binding(files["urdf"]),
        "srdf": _binding(files["srdf"]),
    }
    replay = {
        "schema": EXACT_FCL_REPLAY_SCHEMA,
        "run_id": plan["run_id"],
        "scope": _exact_fcl_scope(),
        "rosbag": _binding(rosbag),
        "telemetry_tsv": _binding(telemetry),
        "geometry": geometry,
        "auditor": {
            "source": _binding(files["auditor_source"]),
            "path_base_source": _binding(files["auditor_path_base_source"]),
            "middle_base_source": _binding(
                files["auditor_middle_base_source"]),
            "binary": _binding(files["auditor_binary"]),
            "argv": argv,
            "argv_sha256": _argv_sha256(argv),
            "exit_code": exit_code,
            "stdout": _binding(files["stdout"]),
            "stderr": _binding(files["stderr"]),
        },
        "replay": {
            "source_joint_state_samples":
                expected_telemetry.source_sample_count,
            "densified_sample_count": len(expected_telemetry.rows),
            "maximum_source_gap_ns":
                expected_telemetry.maximum_source_gap_ns,
            "maximum_revolute_step_rad":
                expected_telemetry.maximum_revolute_step_rad,
            "maximum_linear_step_m":
                expected_telemetry.maximum_linear_step_m,
            "close_command_timestamp_ns":
                expected_telemetry.close_command_timestamp_ns,
            "close_pass_timestamp_ns":
                expected_telemetry.close_pass_timestamp_ns,
            "release_command_timestamp_ns":
                expected_telemetry.release_command_timestamp_ns,
            "release_pass_timestamp_ns":
                expected_telemetry.release_pass_timestamp_ns,
            "excluded_close_transition_rows":
                expected_telemetry.excluded_close_transition_rows,
            "excluded_release_transition_rows":
                expected_telemetry.excluded_release_transition_rows,
            "collision_free": True,
            "left_handle_checks": len(expected_telemetry.rows),
            "left_handle_contact_count": 0,
            "minimum_left_handle_clearance_m": _machine_float(
                summary["min_left_handle_clearance_m"],
                "EXACT_FCL_SUMMARY_LEFT_CLEARANCE"),
            "right_contact_required_samples": required_count,
            "right_grip_engaged_samples": grip_engaged_count,
            "right_pad1_patch_valid_samples": pad1_count,
            "right_pad2_patch_valid_samples": pad2_count,
            "maximum_contact_minimum_right_pad_distance_m": _machine_float(
                summary["max_contact_minimum_right_pad_distance_m"],
                "EXACT_FCL_SUMMARY_MAX_CONTACT_MINIMUM_RIGHT_PAD_DISTANCE"),
            "minimum_adjacent_finger_handle_clearance_m": _machine_float(
                summary["min_adjacent_finger_handle_clearance_m"],
                "EXACT_FCL_SUMMARY_ADJACENT_FINGER_CLEARANCE"),
            "minimum_buffered_nonpad_handle_clearance_m": _machine_float(
                summary["min_nonpad_handle_clearance_m"],
                "EXACT_FCL_SUMMARY_BUFFERED_NONPAD_CLEARANCE"),
            "allowed_handle_contact_pairs": [
                list(pair) for pair in ALLOWED_HANDLE_CONTACT_PAIRS],
            "observed_handle_contact_pairs": observed_pairs,
            "unauthorized_handle_contact_pairs": [],
            "replay_transcript_sha256": files["stdout"].sha256,
        },
        "native_physics_contacts": {
            "backend": "DART",
            "available": False,
            "reason": NATIVE_CONTACT_UNAVAILABLE_REASON,
            "force_or_wrench_claimed": False,
        },
        "offline_replay": True,
        "live_telemetry_bound": True,
        "valid": True,
        "status": "PASS",
    }
    raw = _canonical_bytes(replay)
    digest = hashlib.sha256(raw).hexdigest()
    synthetic_evidence = FileEvidence(
        output_path, raw, digest, len(raw), "0400")
    _validate_exact_fcl_replay(
        replay, synthetic_evidence, plan, rosbag, expected_telemetry)
    _atomic_publish_no_replace(output_path, raw)
    return CertificateArtifact(replay, raw, digest, output_path)


def assemble(
    *, plan_path: Path, plan_sha256: str, rosbag_path: Path,
    markers_path: Path, proc_snapshot_path: Path, video_audit_path: Path,
    postflight_path: Path, workcell_pose_audit_path: Path,
    gui_camera_audit_path: Path, video_window_audit_path: Path,
    formal_authorities_path: Path,
    exact_fcl_replay_path: Path, output_path: Path,
    decoder: CdrDeserializer | None = None,
) -> CertificateArtifact:
    """Recompute all live gates and atomically publish one certificate."""
    _validate_output_parent(output_path)
    loaded = _load_inputs(
        plan_path=plan_path, plan_sha256=plan_sha256,
        rosbag_path=rosbag_path, markers_path=markers_path,
        proc_snapshot_path=proc_snapshot_path,
        video_audit_path=video_audit_path, postflight_path=postflight_path,
        workcell_pose_audit_path=workcell_pose_audit_path,
        gui_camera_audit_path=gui_camera_audit_path,
        video_window_audit_path=video_window_audit_path,
        formal_authorities_path=formal_authorities_path,
        exact_fcl_replay_path=exact_fcl_replay_path, decoder=decoder)
    value = _certificate_value(loaded)
    _validate_certificate_shape(value)
    raw = _canonical_bytes(value)
    digest = hashlib.sha256(raw).hexdigest()
    _atomic_publish_no_replace(output_path, raw)
    return CertificateArtifact(value, raw, digest, output_path)


def verify(
    *, certificate_path: Path, certificate_sha256: str,
    plan_sha256: str, decoder: CdrDeserializer | None = None,
) -> Mapping[str, Any]:
    """Independently reopen every binding and recompute the certificate."""
    certificate_evidence, value = _load_canonical_json(
        certificate_path, expected_sha256=certificate_sha256,
        status="CERTIFICATE_FILE")
    _validate_certificate_shape(value)
    inputs = value["inputs"]
    _require(inputs["plan"]["sha256"] == plan_sha256,
             "VERIFY_PLAN_SHA256")
    for identifier, binding in inputs.items():
        _validate_file_binding(binding, f"VERIFY_INPUT_{identifier}")
    loaded = _load_inputs(
        plan_path=Path(inputs["plan"]["path"]),
        plan_sha256=plan_sha256,
        rosbag_path=Path(inputs["rosbag"]["path"]),
        markers_path=Path(inputs["markers"]["path"]),
        proc_snapshot_path=Path(inputs["proc_snapshot"]["path"]),
        video_audit_path=Path(inputs["video_audit"]["path"]),
        postflight_path=Path(inputs["postflight"]["path"]),
        workcell_pose_audit_path=Path(
            inputs["workcell_pose_audit"]["path"]),
        gui_camera_audit_path=Path(inputs["gui_camera_audit"]["path"]),
        video_window_audit_path=Path(
            inputs["video_window_audit"]["path"]),
        formal_authorities_path=Path(inputs["formal_authorities"]["path"]),
        exact_fcl_replay_path=Path(inputs["exact_fcl_replay"]["path"]),
        decoder=decoder)
    recomputed = _certificate_value(loaded)
    _require(value == recomputed, "CERTIFICATE_RECOMPUTE_MISMATCH")
    _require(certificate_evidence.raw == _canonical_bytes(recomputed),
             "CERTIFICATE_BYTES_MISMATCH")
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    extract_parser = commands.add_parser("extract-fcl-tsv")
    extract_parser.add_argument("--plan", type=Path, required=True)
    extract_parser.add_argument("--plan-sha256", required=True)
    extract_parser.add_argument("--rosbag", type=Path, required=True)
    extract_parser.add_argument("--markers", type=Path, required=True)
    extract_parser.add_argument("--output", type=Path, required=True)
    wrap_parser = commands.add_parser("wrap-exact-fcl-replay")
    wrap_parser.add_argument("--plan", type=Path, required=True)
    wrap_parser.add_argument("--plan-sha256", required=True)
    wrap_parser.add_argument("--rosbag", type=Path, required=True)
    wrap_parser.add_argument("--markers", type=Path, required=True)
    wrap_parser.add_argument("--telemetry-tsv", type=Path, required=True)
    wrap_parser.add_argument("--scene", type=Path, required=True)
    wrap_parser.add_argument("--path", dest="contact_path", type=Path,
                             required=True)
    wrap_parser.add_argument("--urdf", type=Path, required=True)
    wrap_parser.add_argument("--srdf", type=Path, required=True)
    wrap_parser.add_argument("--handle-stl", type=Path, required=True)
    wrap_parser.add_argument("--auditor-source", type=Path, required=True)
    wrap_parser.add_argument(
        "--auditor-path-base-source", type=Path, required=True)
    wrap_parser.add_argument(
        "--auditor-middle-base-source", type=Path, required=True)
    wrap_parser.add_argument("--auditor-binary", type=Path, required=True)
    wrap_parser.add_argument("--stdout", type=Path, required=True)
    wrap_parser.add_argument("--stderr", type=Path, required=True)
    wrap_parser.add_argument("--exit-code", type=int, required=True)
    wrap_parser.add_argument("--output", type=Path, required=True)
    assemble_parser = commands.add_parser("assemble")
    assemble_parser.add_argument("--plan", type=Path, required=True)
    assemble_parser.add_argument("--plan-sha256", required=True)
    assemble_parser.add_argument("--rosbag", type=Path, required=True)
    assemble_parser.add_argument("--markers", type=Path, required=True)
    assemble_parser.add_argument("--proc-snapshot", type=Path, required=True)
    assemble_parser.add_argument("--video-audit", type=Path, required=True)
    assemble_parser.add_argument("--postflight", type=Path, required=True)
    assemble_parser.add_argument(
        "--workcell-pose-audit", type=Path, required=True)
    assemble_parser.add_argument(
        "--gui-camera-audit", type=Path, required=True)
    assemble_parser.add_argument(
        "--video-window-audit", type=Path, required=True)
    assemble_parser.add_argument(
        "--formal-authorities", type=Path, required=True)
    assemble_parser.add_argument(
        "--exact-fcl-replay", type=Path, required=True)
    assemble_parser.add_argument("--output", type=Path, required=True)
    verify_parser = commands.add_parser("verify")
    verify_parser.add_argument("--certificate", type=Path, required=True)
    verify_parser.add_argument("--certificate-sha256", required=True)
    verify_parser.add_argument("--plan-sha256", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "extract-fcl-tsv":
            artifact = extract_fcl_telemetry(
                plan_path=args.plan, plan_sha256=args.plan_sha256,
                rosbag_path=args.rosbag, markers_path=args.markers,
                output_path=args.output)
            print(
                "CANDIDATE3_LIVE_CERTIFICATE_829=PASS "
                "action=extract-fcl-tsv "
                f"sha256={artifact.sha256} valid=true")
        elif args.command == "wrap-exact-fcl-replay":
            artifact = wrap_exact_fcl_replay(
                plan_path=args.plan, plan_sha256=args.plan_sha256,
                rosbag_path=args.rosbag, markers_path=args.markers,
                telemetry_tsv_path=args.telemetry_tsv,
                scene_path=args.scene, contact_path=args.contact_path,
                urdf_path=args.urdf, srdf_path=args.srdf,
                handle_stl_path=args.handle_stl,
                auditor_source_path=args.auditor_source,
                auditor_path_base_source_path=args.auditor_path_base_source,
                auditor_middle_base_source_path=
                    args.auditor_middle_base_source,
                auditor_binary_path=args.auditor_binary,
                stdout_path=args.stdout, stderr_path=args.stderr,
                exit_code=args.exit_code, output_path=args.output)
            print(
                "CANDIDATE3_LIVE_CERTIFICATE_829=PASS "
                "action=wrap-exact-fcl-replay "
                f"sha256={artifact.sha256} valid=true")
        elif args.command == "assemble":
            artifact = assemble(
                plan_path=args.plan, plan_sha256=args.plan_sha256,
                rosbag_path=args.rosbag, markers_path=args.markers,
                proc_snapshot_path=args.proc_snapshot,
                video_audit_path=args.video_audit,
                postflight_path=args.postflight,
                workcell_pose_audit_path=args.workcell_pose_audit,
                gui_camera_audit_path=args.gui_camera_audit,
                video_window_audit_path=args.video_window_audit,
                formal_authorities_path=args.formal_authorities,
                exact_fcl_replay_path=args.exact_fcl_replay,
                output_path=args.output)
            print(
                "CANDIDATE3_LIVE_CERTIFICATE_829=PASS action=assemble "
                f"sha256={artifact.sha256} valid=true")
        elif args.command == "verify":
            value = verify(
                certificate_path=args.certificate,
                certificate_sha256=args.certificate_sha256,
                plan_sha256=args.plan_sha256)
            print(
                "CANDIDATE3_LIVE_CERTIFICATE_829=PASS action=verify "
                f"run_id={value['run_id']} valid=true")
        else:  # pragma: no cover - argparse enforces the command choices.
            raise LiveCertificateError("UNKNOWN_COMMAND")
        return 0
    except LiveCertificateError as error:
        print(f"CANDIDATE3_LIVE_CERTIFICATE_829=FAIL:{error}", file=os.sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
