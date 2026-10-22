#!/usr/bin/python3
"""Deterministically retarget the sealed 828 right-handle path below 44.5 deg.

The tool is deliberately offline.  It preserves every contact-geometry field and
both endpoint rows, computes each original ``right_dex1_tcp`` pose by FK, fixes
only positive torso values above 44.25 degrees, and solves the seven right-arm
joints with PyKDL.  The result is a fail-closed v4 *candidate*: collision and
dense-topology certification must be performed by the separate exact auditors.

All inputs are SHA-256 bound.  Outputs are fully staged, published with
no-replace hard links, made read-only, and never replace an existing path.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import hmac
import json
import math
import os
import re
import stat
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping, Sequence


SOURCE_SCHEMA = "h1_moveit/right_handle_variable_torso_path/v3"
OUTPUT_SCHEMA = "h1_moveit/right_handle_variable_torso_path/v4"
SUMMARY_SCHEMA = "h1_moveit/right_handle_torso_margin_summary/v1"
EXECUTION_TOPOLOGY = "right_q0_q90_hold_middle_hold_q90_q0_release/v1"
GENERATOR_ID = "h1_right_handle_torso_margin_829/v1"
OUTPUT_ASSET_ID = "right_handle_829_table_xplus100.variable_torso_v4_44p25_candidate"

DEFAULT_SOURCE_SHA256 = (
    "19d7db327498b1d14f9fe5ebfb2ed3fe741e7ed166cf4b01eade6fc369cf2e98"
)
DEFAULT_URDF_SHA256 = (
    "3aad4bdc038ff6cde3bef221bb15b8134bb45f7a87ef5dfa0fadc2126bd44929"
)
EXPECTED_ROW_COUNT = 921

PLANNED_TORSO_LIMIT_DEG = 44.25
FORMAL_TORSO_CONTRACT_DEG = 44.5
MAXIMUM_ADJACENT_STEP_DEG = 4.9
MINIMUM_ARM_MARGIN_DEG = 2.0
POSITION_TOLERANCE_M = 1.0e-7
ORIENTATION_TOLERANCE_RAD = 1.0e-7
IK_EPSILON = 1.0e-10
IK_MAX_ITERATIONS = 1000
MAX_JSON_BYTES = 64 * 1024 * 1024
MAX_URDF_BYTES = 32 * 1024 * 1024
SHA256_RE = re.compile(r"[0-9a-f]{64}")
RFC3339_RE = re.compile(
    r"[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}"
    r"(?:\.[0-9]+)?(?:Z|[+-][0-9]{2}:[0-9]{2})"
)

BASE_LINK = "pelvis"
TORSO_LINK = "torso_link"
TIP_LINK = "right_dex1_tcp"
TORSO_JOINT = "torso_joint"
RIGHT_ARM_JOINTS = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

ROOT_KEYS = frozenset({
    "$schema", "asset_id", "contract", "created_at", "endpoints",
    "execution_topology", "invalidated_endpoints", "metrics", "provenance",
    "rows", "schema_version", "scope", "status",
})
ROW_KEYS = frozenset({
    "action", "contact_index", "geometry_source", "handle_angle_deg", "index",
    "local_x_offset_mm", "original_stl_gate", "outward_offset_mm",
    "palm_down_tilt_deg", "phase_deg", "right_arm_rad",
    "right_pad1_slider_mm", "right_pad2_slider_mm", "side_tilt_deg",
    "torso_yaw_deg", "z_mm",
})
GEOMETRY_KEYS = tuple(sorted(ROW_KEYS - {"torso_yaw_deg", "right_arm_rad"}))
CONTRACT_KEYS = frozenset({
    "all_nonpad_handle_contact_forbidden", "authorized_load_contact",
    "handle_end_deg", "handle_start_deg", "left_hand_handle_contact_forbidden",
    "maximum_adjacent_joint_step_deg",
    "maximum_dense_contact_geometry_angular_step_deg",
    "maximum_dense_contact_geometry_linear_step_mm",
    "maximum_dense_slider_step_mm", "middle_initial", "middle_terminal",
    "minimum_arm_joint_limit_margin_deg", "opposite_right_pad_contact_permitted",
    "partial_path_forbidden", "prefix_terminal", "q90_regrasp_commands",
    "q90_release_commands", "suffix_initial", "suffix_release_only_after_q0",
    "torso_abs_limit_deg",
})
GATE_KEYS = frozenset({
    "left_hand_handle_contact_forbidden", "nonpad_contact_forbidden",
    "pad1_contact_authorized", "pass_",
})


class MarginGenerationError(RuntimeError):
    """Fail-closed input, IK, or publication error."""


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    joint_type: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float | None
    upper: float | None


@dataclass(frozen=True)
class ChainBundle:
    chain: Any
    joint_names: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]


@dataclass(frozen=True)
class SolveResult:
    arm_rad: tuple[float, ...]
    position_error_m: float
    orientation_error_rad: float
    seed_label: str


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise MarginGenerationError(message)


def _number(value: Any, label: str) -> float:
    _require(type(value) in (int, float), f"{label}: expected finite number")
    result = float(value)
    _require(math.isfinite(result), f"{label}: expected finite number")
    return result


def _integer(value: Any, label: str) -> int:
    _require(type(value) is int, f"{label}: expected integer")
    return value


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    _require(type(value) is dict, f"{label}: expected object")
    return value


def _exact_keys(value: Mapping[str, Any], expected: frozenset[str], label: str) -> None:
    actual = frozenset(value)
    missing = sorted(expected - actual)
    extra = sorted(actual - expected)
    _require(not missing and not extra, f"{label}: shape mismatch missing={missing} extra={extra}")


def _reject_duplicate_keys(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise MarginGenerationError(f"JSON_DUPLICATE_KEY:{key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise MarginGenerationError(f"JSON_NONFINITE_CONSTANT:{value}")


def _validate_sha256(value: str, label: str) -> str:
    _require(type(value) is str and SHA256_RE.fullmatch(value) is not None,
             f"{label}: expected 64 lowercase hexadecimal characters")
    return value


def _read_bound_regular_file(
    path: Path, expected_sha256: str, maximum_bytes: int, label: str,
) -> tuple[bytes, str]:
    expected = _validate_sha256(expected_sha256, f"{label}_SHA256")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise MarginGenerationError(f"{label}: cannot open protected input {path}: {exc}") from exc
    try:
        before = os.fstat(descriptor)
        _require(stat.S_ISREG(before.st_mode), f"{label}: input is not a regular file")
        _require(0 < before.st_size <= maximum_bytes,
                 f"{label}: input size {before.st_size} is outside 1..{maximum_bytes}")
        chunks: list[bytes] = []
        total = 0
        while True:
            chunk = os.read(descriptor, min(1024 * 1024, maximum_bytes + 1 - total))
            if not chunk:
                break
            chunks.append(chunk)
            total += len(chunk)
            _require(total <= maximum_bytes, f"{label}: input grew beyond size limit")
        after = os.fstat(descriptor)
        identity_before = (
            before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns,
            before.st_ctime_ns,
        )
        identity_after = (
            after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns,
            after.st_ctime_ns,
        )
        _require(identity_before == identity_after and total == before.st_size,
                 f"{label}: input changed while being read")
    finally:
        os.close(descriptor)
    raw = b"".join(chunks)
    digest = hashlib.sha256(raw).hexdigest()
    _require(hmac.compare_digest(digest, expected),
             f"{label}: SHA256 mismatch expected={expected} actual={digest}")
    return raw, digest


def _parse_json(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MarginGenerationError(f"{label}: invalid strict UTF-8 JSON: {exc}") from exc
    return dict(_mapping(value, label))


def load_source(path: Path, expected_sha256: str) -> tuple[dict[str, Any], str]:
    raw, digest = _read_bound_regular_file(
        path, expected_sha256, MAX_JSON_BYTES, "SOURCE_PATH")
    return _parse_json(raw, "SOURCE_PATH"), digest


def _assert_json_finite(value: Any, label: str = "root") -> None:
    if type(value) is float:
        _require(math.isfinite(value), f"{label}: non-finite number")
    elif type(value) in (str, int, bool) or value is None:
        return
    elif type(value) is list:
        for index, child in enumerate(value):
            _assert_json_finite(child, f"{label}[{index}]")
    elif type(value) is dict:
        for key, child in value.items():
            _require(type(key) is str, f"{label}: non-string object key")
            _assert_json_finite(child, f"{label}.{key}")
    else:
        raise MarginGenerationError(f"{label}: unsupported JSON value type {type(value).__name__}")


def _maximum_adjacent_step_deg(rows: Sequence[Mapping[str, Any]]) -> tuple[float, int, int]:
    maximum = 0.0
    maximum_row = 0
    maximum_axis = 0
    for row_index, (first, second) in enumerate(zip(rows, rows[1:]), start=1):
        first_values = [math.radians(float(first["torso_yaw_deg"])), *first["right_arm_rad"]]
        second_values = [math.radians(float(second["torso_yaw_deg"])), *second["right_arm_rad"]]
        for axis, (left, right) in enumerate(zip(first_values, second_values)):
            delta = math.degrees(abs(float(right) - float(left)))
            if delta > maximum:
                maximum = delta
                maximum_row = row_index
                maximum_axis = axis
    return maximum, maximum_row, maximum_axis


def validate_source_document(
    document: Mapping[str, Any], expected_row_count: int = EXPECTED_ROW_COUNT,
) -> None:
    root = _mapping(document, "root")
    _exact_keys(root, ROOT_KEYS, "root")
    _assert_json_finite(root)
    _require(root["$schema"] == SOURCE_SCHEMA, "SOURCE_SCHEMA")
    _require(_integer(root["schema_version"], "schema_version") == 3,
             "SOURCE_SCHEMA_VERSION")
    _require(root["execution_topology"] == EXECUTION_TOPOLOGY,
             "SOURCE_EXECUTION_TOPOLOGY")
    _require(type(root["asset_id"]) is str and bool(root["asset_id"]), "SOURCE_ASSET_ID")
    _require(type(root["created_at"]) is str and bool(root["created_at"]), "SOURCE_CREATED_AT")
    _require(type(root["invalidated_endpoints"]) is list, "SOURCE_INVALIDATED_ENDPOINTS")
    _mapping(root["metrics"], "metrics")
    _mapping(root["provenance"], "provenance")
    _mapping(root["scope"], "scope")
    status = _mapping(root["status"], "status")
    _require(status.get("torso_variable") is True, "SOURCE_TORSO_VARIABLE")
    _require(status.get("fail_closed") is True, "SOURCE_FAIL_CLOSED")

    contract = _mapping(root["contract"], "contract")
    _exact_keys(contract, CONTRACT_KEYS, "contract")
    _require(_number(contract["torso_abs_limit_deg"], "contract.torso_abs_limit_deg") == 45.0,
             "SOURCE_TORSO_LIMIT")
    _require(_number(contract["maximum_adjacent_joint_step_deg"],
                     "contract.maximum_adjacent_joint_step_deg") == MAXIMUM_ADJACENT_STEP_DEG,
             "SOURCE_CONTINUITY_LIMIT")
    _require(_number(contract["minimum_arm_joint_limit_margin_deg"],
                     "contract.minimum_arm_joint_limit_margin_deg") >= MINIMUM_ARM_MARGIN_DEG,
             "SOURCE_ARM_MARGIN")
    _require(_number(contract["handle_start_deg"], "contract.handle_start_deg") == 0.0,
             "SOURCE_HANDLE_START")
    _require(_number(contract["handle_end_deg"], "contract.handle_end_deg") == 90.0,
             "SOURCE_HANDLE_END")

    rows = root["rows"]
    _require(type(rows) is list and len(rows) == expected_row_count,
             f"SOURCE_ROW_COUNT: expected={expected_row_count} actual="
             f"{len(rows) if type(rows) is list else 'not-list'}")
    previous_handle: float | None = None
    previous_contact = -1
    for index, raw_row in enumerate(rows):
        row = _mapping(raw_row, f"rows[{index}]")
        _exact_keys(row, ROW_KEYS, f"rows[{index}]")
        _require(_integer(row["index"], f"rows[{index}].index") == index,
                 f"SOURCE_ROW_INDEX:{index}")
        contact = _integer(row["contact_index"], f"rows[{index}].contact_index")
        _require(contact >= previous_contact, f"SOURCE_CONTACT_INDEX_ORDER:{index}")
        previous_contact = contact
        handle = _number(row["handle_angle_deg"], f"rows[{index}].handle_angle_deg")
        _require(-1.0e-12 <= handle <= 90.0 + 1.0e-12,
                 f"SOURCE_HANDLE_RANGE:{index}")
        if previous_handle is not None:
            _require(handle > previous_handle,
                     f"SOURCE_HANDLE_NOT_STRICTLY_INCREASING:{index}")
            _require(handle - previous_handle <= 0.100000000001,
                     f"SOURCE_HANDLE_STEP:{index}")
        previous_handle = handle
        torso = _number(row["torso_yaw_deg"], f"rows[{index}].torso_yaw_deg")
        _require(abs(torso) <= 45.0 + 1.0e-12, f"SOURCE_TORSO_RANGE:{index}")
        arm = row["right_arm_rad"]
        _require(type(arm) is list and len(arm) == len(RIGHT_ARM_JOINTS),
                 f"SOURCE_ARM_SHAPE:{index}")
        for joint_index, value in enumerate(arm):
            _number(value, f"rows[{index}].right_arm_rad[{joint_index}]")
        for key in (
            "phase_deg", "z_mm", "palm_down_tilt_deg", "side_tilt_deg",
            "outward_offset_mm", "local_x_offset_mm", "right_pad1_slider_mm",
            "right_pad2_slider_mm",
        ):
            _number(row[key], f"rows[{index}].{key}")
        _require(type(row["action"]) is str and bool(row["action"]),
                 f"SOURCE_ACTION:{index}")
        _require(type(row["geometry_source"]) is str and bool(row["geometry_source"]),
                 f"SOURCE_GEOMETRY_SOURCE:{index}")
        gate = _mapping(row["original_stl_gate"], f"rows[{index}].original_stl_gate")
        _exact_keys(gate, GATE_KEYS, f"rows[{index}].original_stl_gate")
        _require(all(gate[key] is True for key in GATE_KEYS), f"SOURCE_STL_GATE:{index}")

    _require(abs(float(rows[0]["handle_angle_deg"])) <= 1.0e-12, "SOURCE_Q0_HANDLE")
    _require(abs(float(rows[-1]["handle_angle_deg"]) - 90.0) <= 1.0e-12,
             "SOURCE_Q90_HANDLE")
    _require(float(rows[0]["torso_yaw_deg"]) <= PLANNED_TORSO_LIMIT_DEG,
             "SOURCE_Q0_ENDPOINT_REQUIRES_RETARGET")
    _require(float(rows[-1]["torso_yaw_deg"]) <= PLANNED_TORSO_LIMIT_DEG,
             "SOURCE_Q90_ENDPOINT_REQUIRES_RETARGET")
    endpoints = _mapping(root["endpoints"], "endpoints")
    _require(frozenset(endpoints) == frozenset({"q0", "q90_canonical_e0"}),
             "SOURCE_ENDPOINT_SHAPE")
    _require(endpoints["q0"] == rows[0], "SOURCE_Q0_ENDPOINT_MISMATCH")
    _require(endpoints["q90_canonical_e0"] == rows[-1],
             "SOURCE_Q90_ENDPOINT_MISMATCH")
    maximum, row_index, axis = _maximum_adjacent_step_deg(rows)
    _require(maximum <= MAXIMUM_ADJACENT_STEP_DEG + 1.0e-12,
             f"SOURCE_CONTINUITY row={row_index} axis={axis} step_deg={maximum:.17g}")
    metrics = root["metrics"]
    _require(_integer(metrics.get("row_count"), "metrics.row_count") == len(rows),
             "SOURCE_METRIC_ROW_COUNT")
    measured_torso = max(abs(float(row["torso_yaw_deg"])) for row in rows)
    _require(abs(_number(metrics.get("maximum_abs_torso_deg"),
                         "metrics.maximum_abs_torso_deg") - measured_torso) <= 1.0e-12,
             "SOURCE_METRIC_TORSO")


def _parse_triplet(
    value: str | None,
    default: tuple[float, float, float],
    label: str,
) -> tuple[float, float, float]:
    if value is None:
        return default
    fields = value.split()
    _require(len(fields) == 3, f"URDF_{label}_SHAPE")
    try:
        result = tuple(_number(float(field), f"URDF_{label}") for field in fields)
    except ValueError as exc:
        raise MarginGenerationError(f"URDF_{label}_NUMBER") from exc
    return result  # type: ignore[return-value]


def _parse_urdf_joints(raw: bytes) -> dict[str, JointSpec]:
    _require(b"<!DOCTYPE" not in raw and b"<!ENTITY" not in raw, "URDF_DTD_FORBIDDEN")
    try:
        root = ET.fromstring(raw.decode("utf-8"))
    except (UnicodeDecodeError, ET.ParseError) as exc:
        raise MarginGenerationError(f"URDF_PARSE:{exc}") from exc
    _require(root.tag == "robot", "URDF_ROOT")
    result: dict[str, JointSpec] = {}
    children: set[str] = set()
    for element in root.findall("joint"):
        name = element.get("name")
        joint_type = element.get("type")
        _require(type(name) is str and bool(name) and name not in result,
                 "URDF_JOINT_NAME")
        _require(joint_type in {"fixed", "revolute", "prismatic"},
                 f"URDF_JOINT_TYPE:{name}")
        parent_element = element.find("parent")
        child_element = element.find("child")
        _require(parent_element is not None and child_element is not None,
                 f"URDF_JOINT_LINKS:{name}")
        parent = parent_element.get("link")
        child = child_element.get("link")
        _require(type(parent) is str and bool(parent) and type(child) is str and bool(child),
                 f"URDF_JOINT_LINK_NAMES:{name}")
        _require(child not in children, f"URDF_MULTIPLE_PARENTS:{child}")
        children.add(child)
        origin = element.find("origin")
        xyz = _parse_triplet(origin.get("xyz") if origin is not None else None,
                             (0.0, 0.0, 0.0), f"ORIGIN_XYZ:{name}")
        rpy = _parse_triplet(origin.get("rpy") if origin is not None else None,
                             (0.0, 0.0, 0.0), f"ORIGIN_RPY:{name}")
        axis_element = element.find("axis")
        axis = _parse_triplet(axis_element.get("xyz") if axis_element is not None else None,
                              (1.0, 0.0, 0.0), f"AXIS:{name}")
        lower: float | None = None
        upper: float | None = None
        if joint_type in {"revolute", "prismatic"}:
            limit = element.find("limit")
            has_bounds = limit is not None
            if limit is not None:
                has_bounds = (
                    limit.get("lower") is not None and limit.get("upper") is not None)
            _require(has_bounds, f"URDF_LIMIT:{name}")
            try:
                lower = _number(float(limit.get("lower", "nan")), f"URDF_LOWER:{name}")
                upper = _number(float(limit.get("upper", "nan")), f"URDF_UPPER:{name}")
            except ValueError as exc:
                raise MarginGenerationError(f"URDF_LIMIT_NUMBER:{name}") from exc
            _require(lower < upper, f"URDF_LIMIT_ORDER:{name}")
            norm = math.sqrt(sum(component * component for component in axis))
            _require(abs(norm - 1.0) <= 1.0e-9, f"URDF_AXIS_UNIT:{name}")
        result[name] = JointSpec(
            name=name, parent=parent, child=child, joint_type=joint_type,
            xyz=xyz, rpy=rpy, axis=axis, lower=lower, upper=upper,
        )
    _require(bool(result), "URDF_NO_JOINTS")
    return result


def _joint_path(joints: Mapping[str, JointSpec], base: str, tip: str) -> list[JointSpec]:
    by_child = {joint.child: joint for joint in joints.values()}
    reverse: list[JointSpec] = []
    visited: set[str] = set()
    current = tip
    while current != base:
        _require(current not in visited, f"URDF_CHAIN_CYCLE:{current}")
        visited.add(current)
        joint = by_child.get(current)
        _require(joint is not None, f"URDF_CHAIN_DISCONNECTED:{base}:{tip}:{current}")
        reverse.append(joint)
        current = joint.parent
    return list(reversed(reverse))


def _load_pykdl() -> Any:
    try:
        import PyKDL as kdl  # type: ignore[import-not-found]
    except ImportError as exc:
        raise MarginGenerationError(
            "PYKDL_UNAVAILABLE: run with /usr/bin/python3 and install python3-pykdl") from exc
    return kdl


def _build_chain(kdl: Any, path: Sequence[JointSpec]) -> ChainBundle:
    chain = kdl.Chain()
    names: list[str] = []
    lower: list[float] = []
    upper: list[float] = []
    for joint in path:
        origin = kdl.Frame(
            kdl.Rotation.RPY(*joint.rpy), kdl.Vector(*joint.xyz))
        if joint.joint_type == "fixed":
            chain.addSegment(kdl.Segment(
                joint.child, kdl.Joint(joint.name, kdl.Joint.Fixed), origin))
            continue
        chain.addSegment(kdl.Segment(
            f"{joint.name}__origin", kdl.Joint(kdl.Joint.Fixed), origin))
        variable_type = (
            kdl.Joint.RotAxis if joint.joint_type == "revolute"
            else kdl.Joint.TransAxis)
        variable_joint = kdl.Joint(
            joint.name, kdl.Vector(0.0, 0.0, 0.0), kdl.Vector(*joint.axis),
            variable_type)
        chain.addSegment(kdl.Segment(joint.child, variable_joint, kdl.Frame.Identity()))
        names.append(joint.name)
        _require(joint.lower is not None and joint.upper is not None,
                 f"URDF_LIMIT_INTERNAL:{joint.name}")
        lower.append(joint.lower)
        upper.append(joint.upper)
    _require(chain.getNrOfJoints() == len(names), "KDL_CHAIN_JOINT_COUNT")
    return ChainBundle(chain, tuple(names), tuple(lower), tuple(upper))


class KdlRetargetModel:
    """Strict pelvis/torso/right-arm KDL model bound to one URDF."""

    def __init__(self, urdf_raw: bytes):
        self.kdl = _load_pykdl()
        module_path = Path(getattr(self.kdl, "__file__", ""))
        _require(module_path.is_file() and not module_path.is_symlink(),
                 "PYKDL_MODULE_FILE")
        self.solver_identity = {
            "implementation": "PyKDL/ChainIkSolverPos_NR_JL",
            "module_sha256": hashlib.sha256(module_path.read_bytes()).hexdigest(),
            "python_implementation": sys.implementation.name,
            "python_version": ".".join(str(value) for value in sys.version_info[:3]),
            "version": str(getattr(self.kdl, "__version__", "unknown")),
        }
        joints = _parse_urdf_joints(urdf_raw)
        self.full = _build_chain(
            self.kdl, _joint_path(joints, BASE_LINK, TIP_LINK))
        self.torso = _build_chain(
            self.kdl, _joint_path(joints, BASE_LINK, TORSO_LINK))
        self.arm = _build_chain(
            self.kdl, _joint_path(joints, TORSO_LINK, TIP_LINK))
        _require(self.full.joint_names == (TORSO_JOINT, *RIGHT_ARM_JOINTS),
                 f"URDF_FULL_CHAIN:{self.full.joint_names}")
        _require(self.torso.joint_names == (TORSO_JOINT,),
                 f"URDF_TORSO_CHAIN:{self.torso.joint_names}")
        _require(self.arm.joint_names == RIGHT_ARM_JOINTS,
                 f"URDF_ARM_CHAIN:{self.arm.joint_names}")
        margin = math.radians(MINIMUM_ARM_MARGIN_DEG)
        inset_lower = tuple(value + margin for value in self.arm.lower)
        inset_upper = tuple(value - margin for value in self.arm.upper)
        _require(all(left < right for left, right in zip(inset_lower, inset_upper)),
                 "URDF_ARM_LIMIT_MARGIN_EMPTY")
        self.inset_lower = inset_lower
        self.inset_upper = inset_upper
        self._full_fk = self.kdl.ChainFkSolverPos_recursive(self.full.chain)
        self._torso_fk = self.kdl.ChainFkSolverPos_recursive(self.torso.chain)
        self._arm_fk = self.kdl.ChainFkSolverPos_recursive(self.arm.chain)
        # NR_JL stores references to both subordinate solvers in C++; retain
        # them for exactly as long as the position solver to avoid dangling
        # pybind-held references.
        self._q_min = self._joints(inset_lower)
        self._q_max = self._joints(inset_upper)
        self._velocity = self.kdl.ChainIkSolverVel_pinv(
            self.arm.chain, 1.0e-12, 500)
        self._arm_ik = self.kdl.ChainIkSolverPos_NR_JL(
            self.arm.chain, self._q_min, self._q_max, self._arm_fk,
            self._velocity,
            IK_MAX_ITERATIONS, IK_EPSILON)

    @property
    def arm_limits(self) -> tuple[tuple[float, float], ...]:
        return tuple(zip(self.arm.lower, self.arm.upper))

    def _joints(self, values: Sequence[float]) -> Any:
        array = self.kdl.JntArray(len(values))
        for index, value in enumerate(values):
            array[index] = float(value)
        return array

    def _fk(self, solver: Any, values: Sequence[float], label: str) -> Any:
        frame = self.kdl.Frame.Identity()
        code = solver.JntToCart(self._joints(values), frame)
        _require(code >= 0, f"KDL_FK:{label}:code={code}")
        return frame

    @staticmethod
    def _twist_norms(kdl: Any, actual: Any, expected: Any) -> tuple[float, float]:
        error = kdl.diff(actual, expected)
        return float(error.vel.Norm()), float(error.rot.Norm())

    @staticmethod
    def _seed_key(values: Sequence[float]) -> tuple[int, ...]:
        return tuple(round(float(value) * 1.0e12) for value in values)

    def validate_source_limits(self, rows: Sequence[Mapping[str, Any]]) -> None:
        margin = math.radians(MINIMUM_ARM_MARGIN_DEG)
        for row_index, row in enumerate(rows):
            for joint_index, (value, lower, upper) in enumerate(
                zip(row["right_arm_rad"], self.arm.lower, self.arm.upper)
            ):
                _require(lower + margin - 1.0e-12 <= value <= upper - margin + 1.0e-12,
                         f"SOURCE_ARM_LIMIT_MARGIN:row={row_index}:joint="
                         f"{RIGHT_ARM_JOINTS[joint_index]}")

    def retarget(
        self,
        source_torso_rad: float,
        source_arm_rad: Sequence[float],
        target_torso_rad: float,
        previous_values: Sequence[float],
        next_values: Sequence[float],
    ) -> SolveResult:
        source_world = self._fk(
            self._full_fk, (source_torso_rad, *source_arm_rad), "SOURCE_WORLD")
        target_torso_frame = self._fk(
            self._torso_fk, (target_torso_rad,), "TARGET_TORSO")
        target_arm_frame = target_torso_frame.Inverse() * source_world

        previous_arm = tuple(float(value) for value in previous_values[1:])
        next_arm = tuple(float(value) for value in next_values[1:])
        source_arm = tuple(float(value) for value in source_arm_rad)
        midpoint = tuple((left + right) * 0.5 for left, right in zip(previous_arm, source_arm))
        raw_seeds = (
            ("previous", previous_arm),
            ("source", source_arm),
            ("midpoint", midpoint),
            ("next_source", next_arm),
        )
        seeds: list[tuple[str, tuple[float, ...]]] = []
        seen: set[tuple[int, ...]] = set()
        for label, values in raw_seeds:
            clamped = tuple(
                min(max(value, lower), upper)
                for value, lower, upper in zip(values, self.inset_lower, self.inset_upper)
            )
            key = self._seed_key(clamped)
            if key not in seen:
                seen.add(key)
                seeds.append((label, clamped))

        candidates: list[tuple[tuple[float, float, float, tuple[float, ...]], SolveResult]] = []
        failures: list[str] = []
        for seed_label, seed in seeds:
            output = self.kdl.JntArray(len(RIGHT_ARM_JOINTS))
            code = self._arm_ik.CartToJnt(self._joints(seed), target_arm_frame, output)
            if code < 0:
                failures.append(f"{seed_label}:code={code}")
                continue
            arm = tuple(float(output[index]) for index in range(len(RIGHT_ARM_JOINTS)))
            if not all(math.isfinite(value) for value in arm):
                failures.append(f"{seed_label}:nonfinite")
                continue
            if not all(lower - 1.0e-12 <= value <= upper + 1.0e-12
                       for value, lower, upper in zip(
                           arm, self.inset_lower, self.inset_upper)):
                failures.append(f"{seed_label}:limit")
                continue
            actual_world = self._fk(
                self._full_fk, (target_torso_rad, *arm), f"CANDIDATE_{seed_label}")
            position_error, orientation_error = self._twist_norms(
                self.kdl, actual_world, source_world)
            pose_outside_tolerance = any((
                position_error > POSITION_TOLERANCE_M,
                orientation_error > ORIENTATION_TOLERANCE_RAD,
            ))
            if pose_outside_tolerance:
                failures.append(
                    f"{seed_label}:pose={position_error:.17g},{orientation_error:.17g}")
                continue
            candidate_values = (target_torso_rad, *arm)
            previous_step = max(
                math.degrees(abs(left - right))
                for left, right in zip(candidate_values, previous_values))
            if previous_step > MAXIMUM_ADJACENT_STEP_DEG + 1.0e-12:
                failures.append(f"{seed_label}:previous_step={previous_step:.17g}")
                continue
            next_step = max(
                math.degrees(abs(left - right))
                for left, right in zip(candidate_values, next_values))
            source_distance = math.sqrt(sum(
                (left - right) ** 2 for left, right in zip(arm, source_arm)))
            result = SolveResult(arm, position_error, orientation_error, seed_label)
            score = (max(previous_step, next_step), previous_step, source_distance, arm)
            candidates.append((score, result))
        _require(bool(candidates), "KDL_IK_NO_SOLUTION:" + ";".join(failures))
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]


def load_model(path: Path, expected_sha256: str) -> tuple[KdlRetargetModel, str]:
    raw, digest = _read_bound_regular_file(
        path, expected_sha256, MAX_URDF_BYTES, "URDF")
    return KdlRetargetModel(raw), digest


def _minimum_arm_margin_deg(
    rows: Sequence[Mapping[str, Any]], limits: Sequence[tuple[float, float]],
) -> tuple[float, int, int]:
    minimum = math.inf
    minimum_row = 0
    minimum_joint = 0
    for row_index, row in enumerate(rows):
        for joint_index, (value, (lower, upper)) in enumerate(
            zip(row["right_arm_rad"], limits)
        ):
            margin = math.degrees(min(float(value) - lower, upper - float(value)))
            if margin < minimum:
                minimum = margin
                minimum_row = row_index
                minimum_joint = joint_index
    return minimum, minimum_row, minimum_joint


def _planned_torso_deg(source_value: float) -> float:
    return PLANNED_TORSO_LIMIT_DEG if source_value > PLANNED_TORSO_LIMIT_DEG else source_value


def generate_candidate(
    source: Mapping[str, Any],
    model: Any,
    source_sha256: str,
    urdf_sha256: str,
    generator_sha256: str,
    created_at: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    _validate_sha256(source_sha256, "SOURCE_SHA256")
    _validate_sha256(urdf_sha256, "URDF_SHA256")
    _validate_sha256(generator_sha256, "GENERATOR_SHA256")
    _require(type(created_at) is str and RFC3339_RE.fullmatch(created_at) is not None,
             "CREATED_AT_RFC3339")
    try:
        parsed_created_at = datetime.fromisoformat(created_at.replace("Z", "+00:00"))
    except ValueError as exc:
        raise MarginGenerationError("CREATED_AT_RFC3339") from exc
    _require(parsed_created_at.tzinfo is not None, "CREATED_AT_RFC3339")
    rows = source["rows"]
    model.validate_source_limits(rows)
    solver_identity = _mapping(
        getattr(model, "solver_identity", None), "solver_identity")
    _assert_json_finite(solver_identity, "solver_identity")
    output_rows = copy.deepcopy(rows)
    retargeted: list[int] = []
    seed_counts: dict[str, int] = {}
    maximum_position_error = 0.0
    maximum_orientation_error = 0.0

    for index, source_row in enumerate(rows):
        source_torso_deg = float(source_row["torso_yaw_deg"])
        if source_torso_deg <= PLANNED_TORSO_LIMIT_DEG:
            continue
        _require(0 < index < len(rows) - 1, f"ENDPOINT_RETARGET_FORBIDDEN:{index}")
        target_torso_rad = math.radians(PLANNED_TORSO_LIMIT_DEG)
        previous_row = output_rows[index - 1]
        next_row = rows[index + 1]
        previous_values = (
            math.radians(float(previous_row["torso_yaw_deg"])),
            *[float(value) for value in previous_row["right_arm_rad"]],
        )
        next_values = (
            math.radians(_planned_torso_deg(float(next_row["torso_yaw_deg"]))),
            *[float(value) for value in next_row["right_arm_rad"]],
        )
        try:
            result = model.retarget(
                math.radians(source_torso_deg), source_row["right_arm_rad"],
                target_torso_rad, previous_values, next_values)
        except MarginGenerationError as exc:
            raise MarginGenerationError(
                f"ROW_IK_FAILED:index={index}:handle_deg="
                f"{float(source_row['handle_angle_deg']):.17g}:{exc}") from exc
        output_rows[index]["torso_yaw_deg"] = PLANNED_TORSO_LIMIT_DEG
        output_rows[index]["right_arm_rad"] = list(result.arm_rad)
        retargeted.append(index)
        seed_counts[result.seed_label] = seed_counts.get(result.seed_label, 0) + 1
        maximum_position_error = max(maximum_position_error, result.position_error_m)
        maximum_orientation_error = max(
            maximum_orientation_error, result.orientation_error_rad)

    for index, (before, after) in enumerate(zip(rows, output_rows)):
        for key in GEOMETRY_KEYS:
            _require(after[key] == before[key], f"GEOMETRY_CHANGED:row={index}:field={key}")
        if float(before["torso_yaw_deg"]) <= PLANNED_TORSO_LIMIT_DEG:
            _require(after == before, f"UNAFFECTED_ROW_CHANGED:{index}")
    _require(output_rows[0] == rows[0] and output_rows[-1] == rows[-1],
             "ENDPOINT_ROWS_CHANGED")
    maximum_step, maximum_step_row, maximum_step_axis = _maximum_adjacent_step_deg(output_rows)
    _require(maximum_step <= MAXIMUM_ADJACENT_STEP_DEG + 1.0e-12,
             f"OUTPUT_CONTINUITY:row={maximum_step_row}:axis={maximum_step_axis}:"
             f"step_deg={maximum_step:.17g}")
    maximum_torso = max(abs(float(row["torso_yaw_deg"])) for row in output_rows)
    _require(maximum_torso <= FORMAL_TORSO_CONTRACT_DEG,
             f"OUTPUT_TORSO_CONTRACT:{maximum_torso:.17g}")
    minimum_margin, minimum_margin_row, minimum_margin_joint = _minimum_arm_margin_deg(
        output_rows, model.arm_limits)
    _require(minimum_margin >= MINIMUM_ARM_MARGIN_DEG - 1.0e-10,
             f"OUTPUT_ARM_MARGIN:row={minimum_margin_row}:joint={minimum_margin_joint}:"
             f"margin_deg={minimum_margin:.17g}")

    candidate = copy.deepcopy(dict(source))
    candidate["$schema"] = OUTPUT_SCHEMA
    candidate["schema_version"] = 4
    candidate["asset_id"] = OUTPUT_ASSET_ID
    candidate["created_at"] = created_at
    candidate["rows"] = output_rows
    candidate["endpoints"] = copy.deepcopy(source["endpoints"])
    candidate["contract"] = copy.deepcopy(source["contract"])
    candidate["contract"]["torso_abs_limit_deg"] = FORMAL_TORSO_CONTRACT_DEG
    candidate["metrics"] = {
        "maximum_abs_torso_deg": maximum_torso,
        "maximum_adjacent_8dof_step_deg": maximum_step,
        "maximum_tcp_orientation_retarget_error_deg": math.degrees(
            maximum_orientation_error),
        "maximum_tcp_position_retarget_error_mm": maximum_position_error * 1000.0,
        "minimum_arm_margin_deg_discrete": minimum_margin,
        "retargeted_row_count": len(retargeted),
        "row_count": len(output_rows),
    }
    inherited_provenance_sha = hashlib.sha256(
        _canonical_json_bytes(source["provenance"])).hexdigest()
    candidate["provenance"] = {
        "generator": GENERATOR_ID,
        "generator_sha256": generator_sha256,
        "inherited_source_provenance_sha256": inherited_provenance_sha,
        "source_asset_id": source["asset_id"],
        "source_path_sha256": source_sha256,
        "source_schema": source["$schema"],
        "urdf_sha256": urdf_sha256,
        "fixed_torso_ik": {
            "arm_dof": 7,
            "base_link": BASE_LINK,
            "formal_contract_deg": FORMAL_TORSO_CONTRACT_DEG,
            "ik_epsilon": IK_EPSILON,
            "ik_max_iterations": IK_MAX_ITERATIONS,
            "orientation_tolerance_rad": ORIENTATION_TOLERANCE_RAD,
            "planned_limit_deg": PLANNED_TORSO_LIMIT_DEG,
            "position_tolerance_m": POSITION_TOLERANCE_M,
            "right_arm_joint_order": list(RIGHT_ARM_JOINTS),
            "solver": copy.deepcopy(dict(solver_identity)),
            "target_definition": "preserve_source_right_dex1_tcp_fk_pose",
            "tip_link": TIP_LINK,
            "torso_joint": TORSO_JOINT,
        },
    }
    candidate["status"] = {
        "canonical_e0_q90_endpoint_present": True,
        "exact_q90_endpoint_present": True,
        "execution_ready": False,
        "fail_closed": True,
        "forward_dense_exact_complete": False,
        "forward_discrete_path_valid": False,
        "full_execution_topology_exact_complete": False,
        "gazebo_slow_contact_dynamics_complete": False,
        "ik_retarget_complete": True,
        "offline_kinematic_path_valid": False,
        "reason": (
            "Deterministic fixed-torso PyKDL IK candidate only; exact pointwise collision, "
            "dense bidirectional topology, ingress/egress, and Gazebo validation are pending."
        ),
        "reverse_dense_exact_complete": False,
        "reverse_discrete_path_valid": False,
        "torso_variable": True,
    }
    _assert_json_finite(candidate)
    _require(candidate["endpoints"] == source["endpoints"], "OUTPUT_ENDPOINT_METADATA_CHANGED")

    summary = {
        "$schema": SUMMARY_SCHEMA,
        "candidate_asset_id": OUTPUT_ASSET_ID,
        "contract": {
            "formal_torso_abs_limit_deg": FORMAL_TORSO_CONTRACT_DEG,
            "maximum_adjacent_8dof_step_deg": MAXIMUM_ADJACENT_STEP_DEG,
            "minimum_arm_joint_limit_margin_deg": MINIMUM_ARM_MARGIN_DEG,
            "planned_torso_limit_deg": PLANNED_TORSO_LIMIT_DEG,
        },
        "created_at": created_at,
        "generator": GENERATOR_ID,
        "generator_sha256": generator_sha256,
        "solver": copy.deepcopy(dict(solver_identity)),
        "input": {
            "row_count": len(rows),
            "source_asset_id": source["asset_id"],
            "source_path_sha256": source_sha256,
            "urdf_sha256": urdf_sha256,
        },
        "result": {
            "first_retargeted_row": retargeted[0] if retargeted else None,
            "last_retargeted_row": retargeted[-1] if retargeted else None,
            "maximum_abs_torso_deg": maximum_torso,
            "maximum_adjacent_8dof_step_deg": maximum_step,
            "maximum_adjacent_step_axis": maximum_step_axis,
            "maximum_adjacent_step_row": maximum_step_row,
            "maximum_tcp_orientation_retarget_error_deg": math.degrees(
                maximum_orientation_error),
            "maximum_tcp_position_retarget_error_mm": maximum_position_error * 1000.0,
            "minimum_arm_margin_deg": minimum_margin,
            "minimum_arm_margin_joint": RIGHT_ARM_JOINTS[minimum_margin_joint],
            "minimum_arm_margin_row": minimum_margin_row,
            "retargeted_row_count": len(retargeted),
            "row_count": len(output_rows),
            "seed_counts": dict(sorted(seed_counts.items())),
            "status": "IK_CANDIDATE_REQUIRES_EXACT_AUDIT",
        },
    }
    _assert_json_finite(summary)
    return candidate, summary


def _format_float(value: Any) -> str:
    number = _number(value, "TSV_VALUE")
    return format(number, ".17g")


def render_batch_tsv(rows: Sequence[Mapping[str, Any]]) -> bytes:
    lines = [
        "# id handle_deg torso_deg phase_deg z_mm palm_tilt_deg side_tilt_deg "
        "outward_mm local_x_mm q1_mm q2_mm arm_rad_csv"
    ]
    for index, row in enumerate(rows):
        arm = ",".join(_format_float(value) for value in row["right_arm_rad"])
        fields = (
            f"P{index:04d}", _format_float(row["handle_angle_deg"]),
            _format_float(row["torso_yaw_deg"]), _format_float(row["phase_deg"]),
            _format_float(row["z_mm"]), _format_float(row["palm_down_tilt_deg"]),
            _format_float(row["side_tilt_deg"]),
            _format_float(row["outward_offset_mm"]),
            _format_float(row["local_x_offset_mm"]),
            _format_float(row["right_pad1_slider_mm"]),
            _format_float(row["right_pad2_slider_mm"]), arm,
        )
        lines.append("\t".join(fields))
    return ("\n".join(lines) + "\n").encode("ascii")


def _canonical_json_bytes(value: Any) -> bytes:
    try:
        text = json.dumps(
            value, ensure_ascii=True, allow_nan=False, sort_keys=True,
            separators=(",", ":"))
    except (TypeError, ValueError) as exc:
        raise MarginGenerationError(f"JSON_SERIALIZATION:{exc}") from exc
    return (text + "\n").encode("ascii")


def _write_staged(path: Path, raw: bytes) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags, 0o600)
    except OSError as exc:
        raise MarginGenerationError(f"OUTPUT_STAGE_CREATE:{path}:{exc}") from exc
    complete = False
    try:
        offset = 0
        while offset < len(raw):
            written = os.write(descriptor, raw[offset:])
            _require(written > 0, f"OUTPUT_SHORT_WRITE:{path}")
            offset += written
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o400)
        complete = True
    finally:
        os.close(descriptor)
        if not complete:
            try:
                os.unlink(path)
            except OSError:
                pass


def publish_outputs_no_replace(payloads: Mapping[Path, bytes]) -> None:
    _require(len(payloads) == 3, "OUTPUT_COUNT")
    normalized = [Path(os.path.abspath(os.fspath(path))) for path in payloads]
    _require(len(set(normalized)) == len(normalized), "OUTPUT_PATHS_NOT_DISTINCT")
    for path in normalized:
        _require(path.parent.is_dir(), f"OUTPUT_PARENT_NOT_DIRECTORY:{path.parent}")
        _require(not os.path.lexists(path), f"OUTPUT_EXISTS:{path}")

    token = f"{os.getpid()}.{hashlib.sha256(os.urandom(32)).hexdigest()[:16]}"
    staged = {
        path: path.parent / f".{path.name}.stage.{token}" for path in normalized
    }
    created: list[Path] = []
    try:
        for original, path in zip(payloads, normalized):
            _write_staged(staged[path], payloads[original])
        for path in normalized:
            try:
                os.link(staged[path], path, follow_symlinks=False)
            except OSError as exc:
                raise MarginGenerationError(f"OUTPUT_NO_REPLACE_PUBLISH:{path}:{exc}") from exc
            created.append(path)
        for parent in sorted({path.parent for path in normalized}, key=str):
            descriptor = os.open(parent, os.O_RDONLY | getattr(os, "O_CLOEXEC", 0))
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
    except BaseException:
        for path in reversed(created):
            try:
                os.unlink(path)
            except OSError:
                pass
        raise
    finally:
        for stage in staged.values():
            try:
                os.unlink(stage)
            except FileNotFoundError:
                pass


def _script_sha256() -> str:
    path = Path(__file__)
    raw = path.read_bytes()
    return hashlib.sha256(raw).hexdigest()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path,
                        help="sealed v3 contact_path JSON")
    parser.add_argument("--input-sha256", default=DEFAULT_SOURCE_SHA256,
                        help="required SHA-256 binding for --input")
    parser.add_argument("--urdf", required=True, type=Path,
                        help="expanded robot URDF used for FK and fixed-torso IK")
    parser.add_argument("--urdf-sha256", default=DEFAULT_URDF_SHA256,
                        help="required SHA-256 binding for --urdf")
    parser.add_argument("--created-at", required=True,
                        help="explicit RFC3339 timestamp; no wall clock is sampled")
    parser.add_argument("--output-json", required=True, type=Path)
    parser.add_argument("--output-tsv", required=True, type=Path)
    parser.add_argument("--output-summary", required=True, type=Path)
    return parser


def run(arguments: argparse.Namespace) -> dict[str, Any]:
    source, source_sha = load_source(arguments.input, arguments.input_sha256)
    validate_source_document(source)
    model, urdf_sha = load_model(arguments.urdf, arguments.urdf_sha256)
    generator_sha = _script_sha256()
    candidate, summary = generate_candidate(
        source, model, source_sha, urdf_sha, generator_sha, arguments.created_at)
    candidate_raw = _canonical_json_bytes(candidate)
    tsv_raw = render_batch_tsv(candidate["rows"])
    summary = copy.deepcopy(summary)
    summary["outputs"] = {
        "candidate_json_sha256": hashlib.sha256(candidate_raw).hexdigest(),
        "candidate_json_size_bytes": len(candidate_raw),
        "batch_tsv_sha256": hashlib.sha256(tsv_raw).hexdigest(),
        "batch_tsv_size_bytes": len(tsv_raw),
    }
    summary_raw = _canonical_json_bytes(summary)
    publish_outputs_no_replace({
        arguments.output_json: candidate_raw,
        arguments.output_tsv: tsv_raw,
        arguments.output_summary: summary_raw,
    })
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    try:
        arguments = _parser().parse_args(argv)
        summary = run(arguments)
    except MarginGenerationError as exc:
        print(f"H1_RIGHT_HANDLE_TORSO_MARGIN_829=ERROR:{exc}", file=sys.stderr)
        return 2
    result = summary["result"]
    outputs = summary["outputs"]
    print(
        "H1_RIGHT_HANDLE_TORSO_MARGIN_829=PASS "
        f"rows={result['row_count']} retargeted={result['retargeted_row_count']} "
        f"maximum_abs_torso_deg={result['maximum_abs_torso_deg']:.17g} "
        f"maximum_step_deg={result['maximum_adjacent_8dof_step_deg']:.17g} "
        f"candidate_sha256={outputs['candidate_json_sha256']} "
        f"tsv_sha256={outputs['batch_tsv_sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
