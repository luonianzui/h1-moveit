#!/usr/bin/python3
"""Capture, inspect, and deliberately replay H1 controller trajectories.

Replay is fail-closed: merely selecting the replay subcommand never publishes.
Both ``--execute`` and a matching ``--expected-stage`` are required.  This
tool does not attach, detach, reset, or command any Gazebo model directly.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import re
import signal
import stat
import sys
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Mapping, Sequence, Tuple


SCHEMA = "h1_joint_trajectory_asset/v1"
STAGES = ("handle", "left_clear", "door", "cube")
CONTROLLER_TOPICS = (
    "/right_arm_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/right_gripper_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
)
CONTROLLER_JOINTS = {
    "/right_arm_controller/joint_trajectory": (
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ),
    "/left_arm_controller/joint_trajectory": (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ),
    "/right_gripper_controller/joint_trajectory": (
        "right_dex1_finger_1_joint",
        "right_dex1_finger_2_joint",
    ),
    "/left_gripper_controller/joint_trajectory": (
        "left_dex1_finger_1_joint",
        "left_dex1_finger_2_joint",
    ),
}
CONTROLLER_STATE_TOPICS = {
    topic: topic.removesuffix("/joint_trajectory") + "/controller_state"
    for topic in CONTROLLER_TOPICS
}
START_TOLERANCE_RAD = 0.003
ENDPOINT_TOLERANCE_RAD = 0.003
CUBE_CLOSE_CONTACT_ENDPOINT_KIND = "cube_gripper_contact_close"
# The 80 mm cube stops the originally captured +3 mm preload command near
# -15 mm.  The scoped capture option may bind that stable physical endpoint,
# but never beyond this hard desired/actual separation and never for another
# stage or controller.  The deterministic 8251 cache builder below uses a
# much smaller positive contact preload to avoid ejecting the 12 g cube.
CUBE_CLOSE_CONTACT_MAX_ERROR_HARD_LIMIT_RAD = 0.020
CUBE_CLOSE_CONTACT_MIN_ACTUAL_TRAVEL_RAD = 0.002
MAX_ASSET_BYTES = 1024 * 1024 * 1024
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")
MESSAGE_TIME_SCALE_ITEM_PATTERN = re.compile(
    r"(?P<index>0|[1-9][0-9]*):(?P<scale>0(?:\.[0-9]+)?|1(?:\.0+)?)\Z"
)

# Position bounds come from the H1-2 URDF.  Velocity and acceleration bounds
# are the stricter planning limits used by this workspace.  They are applied
# only to messages explicitly named by --message-time-scales, preserving the
# legacy replay contract when that new option is absent.
JOINT_RETIME_LIMITS: Mapping[str, Tuple[float, float, float, float]] = {
    "left_shoulder_pitch_joint": (-3.14, 1.57, 1.0, 2.0),
    "left_shoulder_roll_joint": (-0.38, 3.4, 1.0, 2.0),
    "left_shoulder_yaw_joint": (-2.66, 3.01, 1.2, 2.5),
    "left_elbow_joint": (-0.95, 3.18, 1.2, 2.5),
    "left_wrist_roll_joint": (-3.01, 2.75, 1.5, 3.0),
    "left_wrist_pitch_joint": (-0.4625, 0.4625, 1.5, 3.0),
    "left_wrist_yaw_joint": (-1.27, 1.27, 1.5, 3.0),
    "right_shoulder_pitch_joint": (-3.14, 1.57, 1.0, 2.0),
    "right_shoulder_roll_joint": (-3.4, 0.38, 1.0, 2.0),
    "right_shoulder_yaw_joint": (-3.01, 2.66, 1.2, 2.5),
    "right_elbow_joint": (-0.95, 3.18, 1.2, 2.5),
    "right_wrist_roll_joint": (-2.75, 3.01, 1.5, 3.0),
    "right_wrist_pitch_joint": (-0.4625, 0.4625, 1.5, 3.0),
    "right_wrist_yaw_joint": (-1.27, 1.27, 1.5, 3.0),
    "left_dex1_finger_1_joint": (-0.02, 0.0245, 0.20, 0.40),
    "left_dex1_finger_2_joint": (-0.02, 0.0245, 0.20, 0.40),
    "right_dex1_finger_1_joint": (-0.02, 0.0245, 0.20, 0.40),
    "right_dex1_finger_2_joint": (-0.02, 0.0245, 0.20, 0.40),
}
JOINT_LIMIT_TOLERANCE = 1.0e-9
LEFT_RACK_Q60_8251_CACHE_SHA256 = (
    "37fbd0a66c274079ef8ec04254be545cc8ec7f800a99588f4f85fcdf016cec66"
)
LEFT_RACK_Q60_8251_STAGE_COUNTS = {
    "approach": 30,
    "carry": 269,
    "support": 12,
    "retreat": 299,
}
LEFT_RACK_Q60_8251_CONTACT_ENDPOINT = (-0.014942, -0.014942)
LEFT_RACK_Q60_8251_COMMAND_ENDPOINT = (-0.01472, -0.01472)
LEFT_RACK_CACHE_CONTRACT_SCHEMA = "h1_left_rack_cube_cache_contract/v1"
LEFT_RACK_CACHE_PROVENANCE_NAMES = {
    "legacy70": "candidate3_cabinet_front_positive_x_legacy70_v1",
    "parallel90": "candidate3_cabinet_front_positive_x_parallel90_v1",
}
LEFT_RACK_CACHE_UNSCALED_ARM_CEILINGS_S = {
    "legacy70": 35.0,
    # The parallel route contains an explicit outside-cabinet reorientation
    # and therefore more conservative source knots.  The 8254 wrapper applies
    # its separately sealed per-message scales; this ceiling only prevents an
    # unexpectedly longer position cache from being mislabeled here.
    "parallel90": 50.0,
}
# The first sixteen fields are the historical C++ task_contract order.  The
# parallel route additionally seals every grasp/orientation parameter that can
# change object_in_tcp or the visible rack attitude.  Fields 25 and 26 are the
# scoped carry-only self-clearance threshold and the level physics-settle
# release height in metres.  Its cache remains deliberately unbound below until
# the reviewed file digest and stage counts are available.
LEFT_RACK_CACHE_TASK_CONTRACTS: Mapping[str, Tuple[float, ...]] = {
    "legacy70": (
        0.08, 0.16, 0.03, 0.334, 0.377, 0.83, 90.0, 60.0,
        90.0, 0.610890542898, -0.029527393333, 0.995407082351,
        70.0, 0.340, 1.020, 1.0,
    ),
    "parallel90": (
        0.08, 0.16, 0.03, 0.334, 0.377, 0.83, 90.0, 60.0,
        90.0, 0.610890542898, -0.029527393333, 0.995407082351,
        90.0, 0.340, 1.020, 1.0,
        32.0, 0.0, 75.0, 0.0, 0.0, 0.0, -90.0, 0.025, 0.0005, 0.0015,
    ),
}

# ``build-cube-cache`` accepts only reviewed cache contracts from this registry.
# The legacy 70-degree route remains the default for backward compatibility.
# A new route must stay explicitly unbound until both its exact file SHA-256 and
# exact per-stage point counts have been reviewed and entered here; there is no
# CLI option that can supply or override either value.
DEFAULT_LEFT_RACK_CACHE_VARIANT = "legacy70"
LEFT_RACK_CACHE_VARIANTS: Mapping[str, Mapping[str, Any]] = {
    "legacy70": {
        "sha256": LEFT_RACK_Q60_8251_CACHE_SHA256,
        "stage_counts": LEFT_RACK_Q60_8251_STAGE_COUNTS,
    },
    "parallel90": {
        "sha256": "dfc45b9ab0ad12e6a6e038ed0a2a3300838ec143ad8dd4c591b423273a352a02",
        "stage_counts": {
            "approach": 32,
            "carry": 360,
            "support": 28,
            "retreat": 413,
        },
    },
}


class AssetError(RuntimeError):
    """A validation or fail-closed execution error."""


def _left_rack_cache_variant_contract(
    cache_variant: str,
) -> Tuple[str, Mapping[str, int]]:
    if type(cache_variant) is not str or cache_variant not in LEFT_RACK_CACHE_VARIANTS:
        raise AssetError(f"unknown left-rack cache variant: {cache_variant!r}")
    contract = LEFT_RACK_CACHE_VARIANTS[cache_variant]
    if set(contract) != {"sha256", "stage_counts"}:
        raise AssetError(
            f"left-rack cache variant {cache_variant!r} registry contract is malformed"
        )
    digest = contract["sha256"]
    stage_counts = contract["stage_counts"]
    if digest is None or stage_counts is None:
        raise AssetError(
            f"left-rack cache variant {cache_variant!r} is not bound to a reviewed "
            "SHA-256 and exact stage-count contract"
        )
    if type(digest) is not str or SHA256_PATTERN.fullmatch(digest) is None:
        raise AssetError(
            f"left-rack cache variant {cache_variant!r} has an invalid registry SHA-256"
        )
    expected_stages = ("approach", "carry", "support", "retreat")
    if type(stage_counts) is not dict or tuple(stage_counts) != expected_stages:
        raise AssetError(
            f"left-rack cache variant {cache_variant!r} has an invalid stage registry"
        )
    for stage, count in stage_counts.items():
        if type(count) is not int or count < 2:
            raise AssetError(
                f"left-rack cache variant {cache_variant!r} has an invalid "
                f"{stage} point count"
            )
    return digest, stage_counts


def _left_rack_task_contract(
    cache_variant: str, values: Sequence[float]
) -> Tuple[float, ...]:
    if cache_variant not in LEFT_RACK_CACHE_TASK_CONTRACTS:
        raise AssetError(f"unknown left-rack cache variant: {cache_variant!r}")
    expected = LEFT_RACK_CACHE_TASK_CONTRACTS[cache_variant]
    if len(values) != len(expected):
        raise AssetError(
            f"left-rack cache variant {cache_variant!r} task contract has "
            f"{len(values)} fields, expected {len(expected)}"
        )
    normalized: List[float] = []
    for index, (raw_value, expected_value) in enumerate(zip(values, expected)):
        if type(raw_value) not in (int, float) or not math.isfinite(float(raw_value)):
            raise AssetError(
                f"left-rack cache task contract field {index + 1} is non-finite"
            )
        value = float(raw_value)
        if abs(value - expected_value) > 1.0e-12:
            raise AssetError(
                f"left-rack cache variant {cache_variant!r} task contract field "
                f"{index + 1} is {value:.17g}, expected {expected_value:.17g}"
            )
        normalized.append(value)
    return tuple(normalized)


def _cube_cache_contract_record(
    cache_variant: str,
    cache_digest: str,
    stage_counts: Mapping[str, int],
    task_contract: Sequence[float],
) -> Dict[str, Any]:
    values = _left_rack_task_contract(cache_variant, task_contract)
    route_contract: Dict[str, Any] = {
        "provenance_name": LEFT_RACK_CACHE_PROVENANCE_NAMES[cache_variant],
        "task_contract_values": list(values),
    }
    if cache_variant == "parallel90":
        route_contract["carry_self_clearance_m"] = values[24]
        route_contract["support_release_height_m"] = values[25]
        route_contract["unscaled_arm_duration_ceiling_s"] = (
            LEFT_RACK_CACHE_UNSCALED_ARM_CEILINGS_S[cache_variant]
        )
    return {
        "schema": LEFT_RACK_CACHE_CONTRACT_SCHEMA,
        "cache_variant": cache_variant,
        "cache_sha256": cache_digest,
        "stage_point_counts": dict(stage_counts),
        "object_size_xyz_m": list(values[0:3]),
        "spawn_pose_world": {
            "xyz_m": list(values[3:6]),
            "rpy_rad": [0.0, 0.0, math.radians(values[6])],
        },
        "target_pose_world": {
            "xyz_m": list(values[9:12]),
            "rpy_rad": [0.0, 0.0, math.radians(values[12])],
        },
        "route_contract": route_contract,
    }


def _validate_cube_cache_contract(value: Any) -> None:
    contract = _exact_keys(
        value,
        (
            "schema", "cache_variant", "cache_sha256", "stage_point_counts",
            "object_size_xyz_m", "spawn_pose_world", "target_pose_world",
            "route_contract",
        ),
        "payload.cube_cache_contract",
    )
    if contract["schema"] != LEFT_RACK_CACHE_CONTRACT_SCHEMA:
        raise AssetError("payload.cube_cache_contract.schema is wrong")
    variant = contract["cache_variant"]
    if type(variant) is not str or variant not in LEFT_RACK_CACHE_VARIANTS:
        raise AssetError("payload.cube_cache_contract.cache_variant is unknown")
    digest = contract["cache_sha256"]
    if type(digest) is not str or SHA256_PATTERN.fullmatch(digest) is None:
        raise AssetError("payload.cube_cache_contract.cache_sha256 is malformed")
    registered_digest, registered_counts = _left_rack_cache_variant_contract(variant)
    if not hmac.compare_digest(digest, registered_digest):
        raise AssetError("payload.cube_cache_contract.cache_sha256 is not registered")
    counts = _exact_keys(
        contract["stage_point_counts"], tuple(registered_counts),
        "payload.cube_cache_contract.stage_point_counts",
    )
    if counts != dict(registered_counts):
        raise AssetError("payload.cube_cache_contract stage counts are not registered")
    route_keys = ("provenance_name", "task_contract_values")
    if variant == "parallel90":
        route_keys += (
            "carry_self_clearance_m", "support_release_height_m",
            "unscaled_arm_duration_ceiling_s",
        )
    route = _exact_keys(
        contract["route_contract"], route_keys,
        "payload.cube_cache_contract.route_contract",
    )
    if route["provenance_name"] != LEFT_RACK_CACHE_PROVENANCE_NAMES[variant]:
        raise AssetError("payload.cube_cache_contract provenance name is wrong")
    values = _left_rack_task_contract(variant, route["task_contract_values"])
    expected_record = _cube_cache_contract_record(
        variant, registered_digest, registered_counts, values
    )
    if contract != expected_record:
        raise AssetError(
            "payload.cube_cache_contract named semantic fields disagree with its "
            "sealed task contract"
        )


def _exact_keys(value: Any, expected: Iterable[str], context: str) -> Mapping[str, Any]:
    if type(value) is not dict:
        raise AssetError(f"{context} must be a JSON object")
    expected_set = set(expected)
    actual_set = set(value)
    if actual_set != expected_set:
        missing = sorted(expected_set - actual_set)
        extra = sorted(actual_set - expected_set)
        raise AssetError(f"{context} keys mismatch: missing={missing}, extra={extra}")
    return value


def _integer(value: Any, context: str, minimum: int = 0, maximum: int = 2**63 - 1) -> int:
    if type(value) is not int or value < minimum or value > maximum:
        raise AssetError(f"{context} must be an integer in [{minimum}, {maximum}]")
    return value


def _finite_vector(value: Any, context: str, allowed_lengths: Sequence[int]) -> List[float]:
    if type(value) is not list or len(value) not in allowed_lengths:
        raise AssetError(
            f"{context} must be a JSON array with length in {sorted(set(allowed_lengths))}"
        )
    result: List[float] = []
    for index, item in enumerate(value):
        if type(item) not in (int, float) or not math.isfinite(float(item)):
            raise AssetError(f"{context}[{index}] must be a finite number")
        result.append(float(item))
    return result


def _duration_ns(value: Any, context: str) -> int:
    duration = _exact_keys(value, ("sec", "nanosec"), context)
    sec = _integer(duration["sec"], f"{context}.sec", 0, 2**31 - 1)
    nanosec = _integer(duration["nanosec"], f"{context}.nanosec", 0, 999_999_999)
    return sec * 1_000_000_000 + nanosec


def _validate_message(
    value: Any,
    stage: str,
    expected_sequence: int,
    previous_wall_offset_ns: int,
    context: str,
) -> Tuple[int, int]:
    message_keys = (
        "sequence",
        "topic",
        "wall_offset_ns",
        "joint_names",
        "points",
    )
    if type(value) is dict and "expected_actual_endpoint" in value:
        message_keys += ("expected_actual_endpoint",)
    message = _exact_keys(
        value,
        message_keys,
        context,
    )
    sequence = _integer(message["sequence"], f"{context}.sequence")
    if sequence != expected_sequence:
        raise AssetError(
            f"{context}.sequence is {sequence}, expected contiguous value {expected_sequence}"
        )
    topic = message["topic"]
    if type(topic) is not str or topic not in CONTROLLER_TOPICS:
        raise AssetError(f"{context}.topic is not an allowed controller topic")
    wall_offset_ns = _integer(message["wall_offset_ns"], f"{context}.wall_offset_ns")
    if wall_offset_ns < previous_wall_offset_ns:
        raise AssetError(f"{context}.wall_offset_ns is not monotonic")

    joint_names = message["joint_names"]
    if type(joint_names) is not list or not joint_names:
        raise AssetError(f"{context}.joint_names must be a non-empty JSON array")
    if any(type(name) is not str or not name for name in joint_names):
        raise AssetError(f"{context}.joint_names contains an invalid name")
    if len(set(joint_names)) != len(joint_names):
        raise AssetError(f"{context}.joint_names contains duplicates")

    points = message["points"]
    if type(points) is not list or not points:
        raise AssetError(f"{context}.points must be a non-empty JSON array")
    joint_count = len(joint_names)
    previous_time_ns = -1
    for point_index, raw_point in enumerate(points):
        point_context = f"{context}.points[{point_index}]"
        point = _exact_keys(
            raw_point,
            (
                "positions",
                "velocities",
                "accelerations",
                "effort",
                "time_from_start",
            ),
            point_context,
        )
        _finite_vector(point["positions"], f"{point_context}.positions", (joint_count,))
        _finite_vector(
            point["velocities"], f"{point_context}.velocities", (0, joint_count)
        )
        _finite_vector(
            point["accelerations"], f"{point_context}.accelerations", (0, joint_count)
        )
        _finite_vector(point["effort"], f"{point_context}.effort", (0, joint_count))
        point_time_ns = _duration_ns(
            point["time_from_start"], f"{point_context}.time_from_start"
        )
        if point_time_ns < previous_time_ns or (
            point_index > 0 and point_time_ns == previous_time_ns
        ):
            raise AssetError(f"{point_context}.time_from_start is not strictly increasing")
        previous_time_ns = point_time_ns

    if "expected_actual_endpoint" in message:
        if stage != "cube" or topic != "/left_gripper_controller/joint_trajectory":
            raise AssetError(
                f"{context}.expected_actual_endpoint is allowed only for a cube-stage "
                "left-gripper close message"
            )
        if tuple(joint_names) != CONTROLLER_JOINTS[topic] or len(points) < 2:
            raise AssetError(
                f"{context}.expected_actual_endpoint requires the canonical left gripper "
                "joint order and at least two desired points"
            )
        endpoint = _exact_keys(
            message["expected_actual_endpoint"],
            ("kind", "positions", "desired_error_limit_rad"),
            f"{context}.expected_actual_endpoint",
        )
        if endpoint["kind"] != CUBE_CLOSE_CONTACT_ENDPOINT_KIND:
            raise AssetError(
                f"{context}.expected_actual_endpoint.kind must be "
                f"{CUBE_CLOSE_CONTACT_ENDPOINT_KIND!r}"
            )
        error_limit = endpoint["desired_error_limit_rad"]
        if type(error_limit) not in (int, float) or not math.isfinite(float(error_limit)):
            raise AssetError(
                f"{context}.expected_actual_endpoint.desired_error_limit_rad must be finite"
            )
        error_limit = float(error_limit)
        if not (
            ENDPOINT_TOLERANCE_RAD < error_limit
            <= CUBE_CLOSE_CONTACT_MAX_ERROR_HARD_LIMIT_RAD
        ):
            raise AssetError(
                f"{context}.expected_actual_endpoint.desired_error_limit_rad must be in "
                f"({ENDPOINT_TOLERANCE_RAD}, "
                f"{CUBE_CLOSE_CONTACT_MAX_ERROR_HARD_LIMIT_RAD}]"
            )
        actual_positions = _finite_vector(
            endpoint["positions"],
            f"{context}.expected_actual_endpoint.positions",
            (joint_count,),
        )
        start_positions = _finite_vector(
            points[0]["positions"], f"{context}.points[0].positions", (joint_count,)
        )
        desired_positions = _finite_vector(
            points[-1]["positions"],
            f"{context}.points[-1].positions",
            (joint_count,),
        )
        for joint_index, joint_name in enumerate(joint_names):
            lower, upper, _velocity, _acceleration = JOINT_RETIME_LIMITS[joint_name]
            start = start_positions[joint_index]
            desired = desired_positions[joint_index]
            actual = actual_positions[joint_index]
            if actual < lower - JOINT_LIMIT_TOLERANCE or actual > upper + JOINT_LIMIT_TOLERANCE:
                raise AssetError(
                    f"{context}.expected_actual_endpoint.positions[{joint_index}] is "
                    f"outside the sealed limit for {joint_name}"
                )
            if desired - start <= CUBE_CLOSE_CONTACT_MIN_ACTUAL_TRAVEL_RAD:
                raise AssetError(
                    f"{context} is not a closing left-gripper command"
                )
            if actual - start < CUBE_CLOSE_CONTACT_MIN_ACTUAL_TRAVEL_RAD:
                raise AssetError(
                    f"{context}.expected_actual_endpoint does not certify physical closing "
                    f"travel for {joint_name}"
                )
            desired_error = desired - actual
            if desired_error < -JOINT_LIMIT_TOLERANCE or desired_error > error_limit + 1e-12:
                raise AssetError(
                    f"{context}.expected_actual_endpoint desired error for {joint_name} "
                    f"is outside [0, {error_limit}]"
                )
    return wall_offset_ns, previous_time_ns


def _canonical_bytes(payload: Mapping[str, Any]) -> bytes:
    try:
        text = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except (TypeError, ValueError) as exc:
        raise AssetError(f"asset is not canonically serializable: {exc}") from exc
    return text.encode("utf-8")


def _validate_payload(payload: Any) -> List[int]:
    payload_keys = ("schema", "stage", "controller_topics", "clock", "messages")
    if type(payload) is dict and "cube_cache_contract" in payload:
        payload_keys += ("cube_cache_contract",)
    root = _exact_keys(
        payload,
        payload_keys,
        "payload",
    )
    if root["schema"] != SCHEMA:
        raise AssetError(f"unsupported schema: {root['schema']!r}")
    if root["stage"] not in STAGES:
        raise AssetError(f"invalid stage: {root['stage']!r}")
    if "cube_cache_contract" in root:
        if root["stage"] != "cube":
            raise AssetError("cube_cache_contract is allowed only for the cube stage")
        _validate_cube_cache_contract(root["cube_cache_contract"])
    if root["controller_topics"] != list(CONTROLLER_TOPICS):
        raise AssetError("controller_topics does not exactly match the four allowed topics")
    clock = _exact_keys(
        root["clock"], ("wall_offset_unit", "time_from_start_unit"), "payload.clock"
    )
    if clock["wall_offset_unit"] != "nanoseconds":
        raise AssetError("payload.clock.wall_offset_unit must be 'nanoseconds'")
    if clock["time_from_start_unit"] != "sec_nanosec":
        raise AssetError("payload.clock.time_from_start_unit must be 'sec_nanosec'")
    messages = root["messages"]
    if type(messages) is not list or not messages:
        raise AssetError("payload.messages must be a non-empty JSON array")
    durations: List[int] = []
    previous_wall_offset_ns = -1
    for index, message in enumerate(messages):
        previous_wall_offset_ns, duration_ns = _validate_message(
            message, root["stage"], index, previous_wall_offset_ns,
            f"payload.messages[{index}]"
        )
        durations.append(duration_ns)

    contact_indices = [
        index for index, message in enumerate(messages)
        if "expected_actual_endpoint" in message
    ]
    if contact_indices:
        if len(contact_indices) != 1:
            raise AssetError(
                "payload may contain exactly one cube close expected-actual endpoint"
            )
        contact_index = contact_indices[0]
        left_gripper_indices = [
            index for index, message in enumerate(messages)
            if message["topic"] == "/left_gripper_controller/joint_trajectory"
        ]
        if not left_gripper_indices or contact_index != left_gripper_indices[0]:
            raise AssetError(
                "expected_actual_endpoint must belong to the first left-gripper message"
            )
        later_gripper_indices = [
            index for index in left_gripper_indices if index > contact_index
        ]
        if not later_gripper_indices:
            raise AssetError(
                "cube close expected_actual_endpoint requires a later left-gripper open message"
            )
        open_message = messages[later_gripper_indices[0]]
        expected_actual = messages[contact_index]["expected_actual_endpoint"]["positions"]
        open_start = open_message["points"][0]["positions"]
        if max(
            abs(float(actual) - float(start))
            for actual, start in zip(expected_actual, open_start)
        ) > ENDPOINT_TOLERANCE_RAD:
            raise AssetError(
                "the next left-gripper message does not start at the sealed contact endpoint"
            )
        open_finish = open_message["points"][-1]["positions"]
        if any(
            float(finish) >= float(start) - CUBE_CLOSE_CONTACT_MIN_ACTUAL_TRAVEL_RAD
            for finish, start in zip(open_finish, open_start)
        ):
            raise AssetError(
                "the next left-gripper message is not an opening command"
            )
    return durations


def _reject_duplicate_keys(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise AssetError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise AssetError(f"invalid non-finite JSON constant: {value}")


def _read_protected_file(path: Path) -> bytes:
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        raise AssetError(f"cannot securely open asset {path}: {exc}") from exc
    try:
        metadata = os.fstat(fd)
        if not stat.S_ISREG(metadata.st_mode):
            raise AssetError(f"asset is not a regular file: {path}")
        if stat.S_IMODE(metadata.st_mode) != 0o400:
            raise AssetError(f"asset mode must be exactly 0400: {path}")
        if metadata.st_uid != os.geteuid():
            raise AssetError(f"asset is not owned by the current user: {path}")
        if metadata.st_nlink != 1:
            raise AssetError(f"asset must have exactly one hard link: {path}")
        if metadata.st_size <= 0 or metadata.st_size > MAX_ASSET_BYTES:
            raise AssetError(
                f"asset size must be in [1, {MAX_ASSET_BYTES}] bytes: {metadata.st_size}"
            )
        chunks: List[bytes] = []
        remaining = metadata.st_size
        while remaining:
            chunk = os.read(fd, min(1024 * 1024, remaining))
            if not chunk:
                raise AssetError("asset ended before its recorded file size")
            chunks.append(chunk)
            remaining -= len(chunk)
        if os.read(fd, 1):
            raise AssetError("asset grew while it was being read")
        return b"".join(chunks)
    finally:
        os.close(fd)


def load_asset(path: Path) -> Tuple[Mapping[str, Any], List[int], str]:
    raw = _read_protected_file(path)
    try:
        document = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise AssetError(f"asset is not strict UTF-8 JSON: {exc}") from exc
    root = _exact_keys(document, ("payload", "sha256"), "document")
    digest = root["sha256"]
    if type(digest) is not str or SHA256_PATTERN.fullmatch(digest) is None:
        raise AssetError("document.sha256 must be 64 lowercase hexadecimal characters")
    calculated = hashlib.sha256(_canonical_bytes(root["payload"])).hexdigest()
    if not hmac.compare_digest(digest, calculated):
        raise AssetError("asset SHA-256 mismatch")
    durations = _validate_payload(root["payload"])
    return root["payload"], durations, digest


def _exclusive_write(path: Path, document: Mapping[str, Any]) -> None:
    if os.path.lexists(path):
        raise AssetError(f"capture target already exists: {path}")
    parent = path.parent if str(path.parent) else Path(".")
    if not parent.is_dir():
        raise AssetError(f"capture target parent is not an existing directory: {parent}")
    data = _canonical_bytes(document) + b"\n"
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    created = False
    try:
        fd = os.open(path, flags, 0o400)
        created = True
    except OSError as exc:
        raise AssetError(f"exclusive capture target creation failed for {path}: {exc}") from exc
    try:
        view = memoryview(data)
        while view:
            written = os.write(fd, view)
            if written <= 0:
                raise AssetError(f"short write while creating {path}")
            view = view[written:]
        os.fsync(fd)
        os.fchmod(fd, 0o400)
    except BaseException:
        os.close(fd)
        if created:
            try:
                os.unlink(path)
            except OSError:
                pass
        raise
    else:
        os.close(fd)


def _record_duration(nanoseconds: int) -> Dict[str, int]:
    if nanoseconds < 0:
        raise AssetError("trajectory time may not be negative")
    return {
        "sec": nanoseconds // 1_000_000_000,
        "nanosec": nanoseconds % 1_000_000_000,
    }


def _position_point_record(
    positions: Sequence[float], time_from_start_ns: int
) -> Dict[str, Any]:
    return {
        "positions": [float(value) for value in positions],
        "velocities": [],
        "accelerations": [],
        "effort": [],
        "time_from_start": _record_duration(time_from_start_ns),
    }


def _read_left_rack_q60_8251_cache(
    cache_path: Path,
    cache_variant: str = DEFAULT_LEFT_RACK_CACHE_VARIANT,
) -> Tuple[Dict[str, List[List[float]]], Tuple[float, ...]]:
    expected_digest, stage_counts = _left_rack_cache_variant_contract(cache_variant)
    raw = _read_protected_file(cache_path)
    digest = hashlib.sha256(raw).hexdigest()
    if not hmac.compare_digest(digest, expected_digest):
        raise AssetError(
            f"left-rack cache variant {cache_variant!r} SHA-256 mismatch: expected "
            f"{expected_digest}, got {digest}"
        )
    try:
        lines = raw.decode("utf-8").splitlines()
    except UnicodeDecodeError as exc:
        raise AssetError(f"left-rack cache is not UTF-8: {exc}") from exc
    if not lines or lines[0] != "H1_LEFT_RACK_POSITION_CACHE_V1":
        raise AssetError("left-rack cache magic is wrong")
    if len(lines) < 3 or lines[1] != "joint_count 7":
        raise AssetError("left-rack cache joint count is wrong")
    if lines[2].split()[1:] != list(
        CONTROLLER_JOINTS["/left_arm_controller/joint_trajectory"]
    ):
        raise AssetError("left-rack cache joint order is wrong")

    task_lines = [line for line in lines if line.startswith("task_contract ")]
    if len(task_lines) != 1:
        raise AssetError("left-rack cache must contain exactly one task contract")
    try:
        task_contract = tuple(float(value) for value in task_lines[0].split()[1:])
    except ValueError as exc:
        raise AssetError("left-rack cache task contract is malformed") from exc
    task_contract = _left_rack_task_contract(cache_variant, task_contract)

    result: Dict[str, List[List[float]]] = {}
    stage_headers = [
        (index, line.split())
        for index, line in enumerate(lines)
        if line.startswith("stage ")
    ]
    if [fields[1] for _index, fields in stage_headers] != list(stage_counts):
        raise AssetError("left-rack cache stage order is wrong")
    for stage_index, fields in stage_headers:
        if len(fields) != 4 or fields[1] not in stage_counts:
            raise AssetError("left-rack cache stage declaration is malformed")
        stage = fields[1]
        expected_count = stage_counts[stage]
        try:
            point_count = int(fields[2])
            boundary_count = int(fields[3])
        except ValueError as exc:
            raise AssetError(f"left-rack cache {stage} counts are malformed") from exc
        if point_count != expected_count or boundary_count < 2:
            raise AssetError(f"left-rack cache {stage} count contract is wrong")
        if stage_index + 4 >= len(lines) or lines[stage_index + 4] != "points":
            raise AssetError(f"left-rack cache {stage} points marker is missing")
        boundary_fields = lines[stage_index + 1].split()
        if (
            len(boundary_fields) != boundary_count + 1
            or boundary_fields[0] != "boundaries"
            or boundary_fields[1] != "0"
            or boundary_fields[-1] != str(point_count - 1)
        ):
            raise AssetError(f"left-rack cache {stage} boundaries are malformed")
        start_fields = lines[stage_index + 2].split()
        finish_fields = lines[stage_index + 3].split()
        if (
            len(start_fields) != 8
            or start_fields[0] != "start"
            or len(finish_fields) != 8
            or finish_fields[0] != "finish"
        ):
            raise AssetError(f"left-rack cache {stage} endpoint declarations are malformed")
        point_lines = lines[stage_index + 5 : stage_index + 5 + point_count]
        if len(point_lines) != point_count:
            raise AssetError(f"left-rack cache {stage} points are truncated")
        points: List[List[float]] = []
        for point_index, line in enumerate(point_lines):
            fields = line.split()
            if len(fields) != 7:
                raise AssetError(
                    f"left-rack cache {stage} point {point_index} has the wrong width"
                )
            try:
                point = [float(value) for value in fields]
            except ValueError as exc:
                raise AssetError(
                    f"left-rack cache {stage} point {point_index} is malformed"
                ) from exc
            if not all(math.isfinite(value) for value in point):
                raise AssetError(
                    f"left-rack cache {stage} point {point_index} is non-finite"
                )
            points.append(point)
        declared_start = [float(value) for value in start_fields[1:]]
        declared_finish = [float(value) for value in finish_fields[1:]]
        if points[0] != declared_start or points[-1] != declared_finish:
            raise AssetError(f"left-rack cache {stage} endpoint record disagrees with points")
        result[stage] = points
    return result, task_contract


def _retime_left_rack_q60_8251_path(
    points: Sequence[Sequence[float]], stage: str
) -> Tuple[List[Dict[str, Any]], int, float, float]:
    """Mirror the C++ candidate-cache retimer at 34/95%, then prove 35/100%."""
    joint_names = CONTROLLER_JOINTS["/left_arm_controller/joint_trajectory"]
    if len(points) < 2 or any(len(point) != len(joint_names) for point in points):
        raise AssetError(f"left-rack {stage} cannot be retimed")
    differences = [
        [float(next_value) - float(value) for value, next_value in zip(first, second)]
        for first, second in zip(points[:-1], points[1:])
    ]
    durations = [0.020 for _ in differences]
    converged = False
    for _iteration in range(64):
        velocities = [
            [difference / duration_value for difference in row]
            for row, duration_value in zip(differences, durations)
        ]
        scales = [1.0 for _ in differences]
        adjustment = False
        for segment, velocity_row in enumerate(velocities):
            for joint_index, velocity in enumerate(velocity_row):
                velocity_limit = JOINT_RETIME_LIMITS[joint_names[joint_index]][2]
                ratio = abs(velocity) / (velocity_limit * 0.34)
                if ratio > 1.0 + 1.0e-6:
                    scales[segment] = max(scales[segment], ratio)
                    adjustment = True
        segment_count = len(differences)
        for knot in range(segment_count + 1):
            first = knot == 0
            last = knot == segment_count
            previous = 0 if first else knot - 1
            following = segment_count - 1 if last else knot
            acceleration_dt = (
                durations[0]
                if first
                else durations[-1]
                if last
                else 0.5 * (durations[previous] + durations[following])
            )
            for joint_index in range(len(joint_names)):
                prior_velocity = 0.0 if first else velocities[previous][joint_index]
                next_velocity = 0.0 if last else velocities[following][joint_index]
                acceleration_limit = JOINT_RETIME_LIMITS[joint_names[joint_index]][3]
                ratio = (
                    abs(next_velocity - prior_velocity)
                    / acceleration_dt
                    / (acceleration_limit * 0.95)
                )
                if ratio > 1.0 + 1.0e-6:
                    scale = math.sqrt(ratio)
                    scales[previous] = max(scales[previous], scale)
                    scales[following] = max(scales[following], scale)
                    adjustment = True
        if not adjustment:
            converged = True
            break
        durations = [value * scale for value, scale in zip(durations, scales)]
    if not converged:
        raise AssetError(f"left-rack {stage} retiming did not converge")

    peak_velocity_fraction = 0.0
    peak_acceleration_fraction = 0.0
    previous_velocity = [0.0 for _ in joint_names]
    previous_dt = 0.0
    for segment, (difference_row, dt) in enumerate(zip(differences, durations)):
        velocity_row: List[float] = []
        for joint_index, difference in enumerate(difference_row):
            velocity_limit = JOINT_RETIME_LIMITS[joint_names[joint_index]][2]
            acceleration_limit = JOINT_RETIME_LIMITS[joint_names[joint_index]][3]
            velocity = difference / dt
            velocity_row.append(velocity)
            peak_velocity_fraction = max(
                peak_velocity_fraction, abs(velocity) / velocity_limit
            )
            acceleration_dt = dt if segment == 0 else 0.5 * (previous_dt + dt)
            peak_acceleration_fraction = max(
                peak_acceleration_fraction,
                abs(velocity - previous_velocity[joint_index])
                / acceleration_dt
                / acceleration_limit,
            )
        previous_velocity = velocity_row
        previous_dt = dt
    for joint_index, velocity in enumerate(previous_velocity):
        acceleration_limit = JOINT_RETIME_LIMITS[joint_names[joint_index]][3]
        peak_acceleration_fraction = max(
            peak_acceleration_fraction,
            abs(velocity) / previous_dt / acceleration_limit,
        )
    if peak_velocity_fraction > 0.35 + 1.0e-9:
        raise AssetError(f"left-rack {stage} exceeds the 35% velocity hard limit")
    if peak_acceleration_fraction > 1.00 + 1.0e-9:
        raise AssetError(f"left-rack {stage} exceeds the 100% acceleration hard limit")

    elapsed = 0.0
    records: List[Dict[str, Any]] = []
    for index, positions in enumerate(points):
        records.append(_position_point_record(positions, round(elapsed * 1_000_000_000)))
        if index < len(durations):
            elapsed += durations[index]
    return records, round(elapsed * 1_000_000_000), peak_velocity_fraction, peak_acceleration_fraction


def _linear_gripper_points(
    start: Sequence[float], finish: Sequence[float], total_seconds: float
) -> List[Dict[str, Any]]:
    if len(start) != 2 or len(finish) != 2 or total_seconds < 0.2:
        raise AssetError("cube gripper trajectory contract is malformed")
    result: List[Dict[str, Any]] = []
    for step in range(11):
        alpha = step / 10.0
        positions = [
            float(first) + alpha * (float(last) - float(first))
            for first, last in zip(start, finish)
        ]
        point_time = max(0.05, alpha * total_seconds)
        result.append(_position_point_record(positions, round(point_time * 1_000_000_000)))
    return result


def build_cube_cache_asset(args: argparse.Namespace) -> int:
    cache_variant = getattr(
        args, "cache_variant", DEFAULT_LEFT_RACK_CACHE_VARIANT
    )
    cache_digest, stage_counts = _left_rack_cache_variant_contract(cache_variant)
    output_path = Path(args.asset)
    cache_path = Path(args.cache)
    if os.path.lexists(output_path):
        raise AssetError(f"cube-cache target already exists: {output_path}")
    if not cache_path.is_absolute():
        raise AssetError("left-rack cache path must be absolute")
    paths, task_contract = _read_left_rack_q60_8251_cache(
        cache_path, cache_variant
    )
    arm_topic = "/left_arm_controller/joint_trajectory"
    gripper_topic = "/left_gripper_controller/joint_trajectory"
    retimed: Dict[str, Tuple[List[Dict[str, Any]], int, float, float]] = {}
    for stage in stage_counts:
        retimed[stage] = _retime_left_rack_q60_8251_path(paths[stage], stage)

    open_position = (-0.019, -0.019)
    # Stop only 0.250 mm/joint beyond the modeled -14.970 mm pad contact
    # (0.222 mm beyond the captured -14.942 mm rigid-body endpoint).
    # The old +3 mm command asked for another 17.942 mm/joint and could eject
    # the lightweight cube before the opposite pad arrived under a loaded
    # simulator.  This endpoint retains real symmetric preload and the sealed
    # physical-feedback certificate without changing any arm position.
    preload_position = LEFT_RACK_Q60_8251_COMMAND_ENDPOINT
    contact_position = LEFT_RACK_Q60_8251_CONTACT_ENDPOINT
    close_points = _linear_gripper_points(open_position, preload_position, 1.0)
    open_points = _linear_gripper_points(contact_position, open_position, 0.8)
    sequence_contract = (
        (arm_topic, retimed["approach"][0], retimed["approach"][1]),
        (gripper_topic, close_points, 1_000_000_000),
        (arm_topic, retimed["carry"][0], retimed["carry"][1]),
        (arm_topic, retimed["support"][0], retimed["support"][1]),
        (gripper_topic, open_points, 800_000_000),
        (arm_topic, retimed["retreat"][0], retimed["retreat"][1]),
    )
    messages: List[Dict[str, Any]] = []
    wall_offset_ns = 0
    for sequence, (topic, points, message_duration_ns) in enumerate(sequence_contract):
        record: Dict[str, Any] = {
            "sequence": sequence,
            "topic": topic,
            "wall_offset_ns": wall_offset_ns,
            "joint_names": list(CONTROLLER_JOINTS[topic]),
            "points": points,
        }
        if sequence == 1:
            record["expected_actual_endpoint"] = {
                "kind": CUBE_CLOSE_CONTACT_ENDPOINT_KIND,
                "positions": list(contact_position),
                "desired_error_limit_rad": 0.020,
            }
        messages.append(record)
        wall_offset_ns += message_duration_ns + 200_000_000
    payload: Dict[str, Any] = {
        "schema": SCHEMA,
        "stage": "cube",
        "controller_topics": list(CONTROLLER_TOPICS),
        "clock": {
            "wall_offset_unit": "nanoseconds",
            "time_from_start_unit": "sec_nanosec",
        },
        "messages": messages,
        "cube_cache_contract": _cube_cache_contract_record(
            cache_variant, cache_digest, stage_counts, task_contract
        ),
    }
    durations = _validate_payload(payload)
    for index, record in enumerate(messages):
        _retimed_message_points(
            record,
            1.0,
            True,
            f"cube-cache message[{index}]",
        )
    arm_duration_ns = sum(
        durations[index] for index in (0, 2, 3, 5)
    )
    arm_ceiling_ns = round(
        LEFT_RACK_CACHE_UNSCALED_ARM_CEILINGS_S[cache_variant] * 1_000_000_000
    )
    if arm_duration_ns > arm_ceiling_ns:
        raise AssetError(
            "cube-cache arm critical duration exceeds its registered "
            f"{arm_ceiling_ns / 1_000_000_000:.0f} simulation-second ceiling"
        )
    digest = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    _exclusive_write(output_path, {"payload": payload, "sha256": digest})
    try:
        loaded_payload, loaded_durations, loaded_digest = load_asset(output_path)
        if (
            loaded_payload != payload
            or loaded_durations != durations
            or not hmac.compare_digest(loaded_digest, digest)
        ):
            raise AssetError("cube-cache asset round-trip identity mismatch")
    except BaseException:
        try:
            os.unlink(output_path)
        except OSError:
            pass
        raise
    print(
        "BUILD_CUBE_CACHE_ASSET=PASS "
        f"messages=6 arm_sim_seconds={arm_duration_ns / 1e9:.6f} "
        f"cache_variant={cache_variant} cache_sha256={cache_digest} "
        f"sha256={digest} path={output_path}",
        flush=True,
    )
    for stage in stage_counts:
        _records, duration_ns, peak_velocity, peak_acceleration = retimed[stage]
        print(
            f"BUILD_CUBE_CACHE_RETIME=PASS stage={stage} "
            f"sim_seconds={duration_ns / 1e9:.6f} "
            f"velocity_fraction={peak_velocity:.6f} "
            f"acceleration_fraction={peak_acceleration:.6f}",
            flush=True,
        )
    return 0


def _reverse_position_record(record: Mapping[str, Any], context: str) -> Dict[str, Any]:
    """Return a position-only time-correct reverse of one sealed message."""
    points = record["points"]
    if len(points) < 2:
        raise AssetError(f"{context} must contain at least two points")
    total_ns = _duration_ns(
        points[-1]["time_from_start"], f"{context}.points[-1].time_from_start"
    )
    result_points: List[Dict[str, Any]] = []
    previous_ns = -1
    for reverse_index, point in enumerate(reversed(points)):
        original_ns = _duration_ns(
            point["time_from_start"],
            f"{context}.points[{len(points) - 1 - reverse_index}].time_from_start",
        )
        reverse_ns = total_ns - original_ns
        if reverse_ns <= previous_ns:
            raise AssetError(f"{context} cannot be reversed with strictly increasing time")
        result_points.append(
            {
                "positions": [float(value) for value in point["positions"]],
                "velocities": [],
                "accelerations": [],
                "effort": [],
                "time_from_start": {
                    "sec": reverse_ns // 1_000_000_000,
                    "nanosec": reverse_ns % 1_000_000_000,
                },
            }
        )
        previous_ns = reverse_ns
    return {
        "topic": str(record["topic"]),
        "joint_names": list(record["joint_names"]),
        "points": result_points,
    }


def _position_only_copy_message(record: Mapping[str, Any]) -> Dict[str, Any]:
    """Copy one sealed message while removing all derivative and effort fields."""
    copied: Dict[str, Any] = {
        "sequence": record["sequence"],
        "topic": record["topic"],
        "wall_offset_ns": record["wall_offset_ns"],
        "joint_names": list(record["joint_names"]),
        "points": [
            {
                "positions": list(point["positions"]),
                "velocities": [],
                "accelerations": [],
                "effort": [],
                "time_from_start": {
                    "sec": point["time_from_start"]["sec"],
                    "nanosec": point["time_from_start"]["nanosec"],
                },
            }
            for point in record["points"]
        ],
    }
    if "expected_actual_endpoint" in record:
        endpoint = record["expected_actual_endpoint"]
        copied["expected_actual_endpoint"] = {
            "kind": endpoint["kind"],
            "positions": list(endpoint["positions"]),
            "desired_error_limit_rad": endpoint["desired_error_limit_rad"],
        }
    return copied


def build_position_only_copy_asset(args: argparse.Namespace) -> int:
    """Build an exact-order position-only copy of one explicitly bound asset."""
    output_path = Path(args.asset)
    source_path = Path(args.source)
    if not source_path.is_absolute():
        raise AssetError("position-only source path must be absolute")
    if type(args.expected_source_sha256) is not str or SHA256_PATTERN.fullmatch(
        args.expected_source_sha256
    ) is None:
        raise AssetError(
            "--expected-source-sha256 must be 64 lowercase hexadecimal characters"
        )

    source_payload, _source_durations, source_digest = load_asset(source_path)
    if not hmac.compare_digest(source_digest, args.expected_source_sha256):
        raise AssetError(
            "position-only source payload SHA-256 differs from the explicit binding"
        )
    if source_payload["stage"] != args.expected_stage:
        raise AssetError(
            f"position-only source stage {source_payload['stage']!r} differs from "
            f"expected stage {args.expected_stage!r}"
        )

    messages = [
        _position_only_copy_message(record) for record in source_payload["messages"]
    ]
    payload: Dict[str, Any] = {
        "schema": source_payload["schema"],
        "stage": source_payload["stage"],
        "controller_topics": list(source_payload["controller_topics"]),
        "clock": {
            "wall_offset_unit": source_payload["clock"]["wall_offset_unit"],
            "time_from_start_unit": source_payload["clock"]["time_from_start_unit"],
        },
        "messages": messages,
    }
    durations = _validate_payload(payload)
    for index, record in enumerate(messages):
        _retimed_message_points(
            record, 1.0, True, f"position-only message[{index}]"
        )

    digest = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    _exclusive_write(output_path, {"payload": payload, "sha256": digest})
    try:
        loaded_payload, loaded_durations, loaded_digest = load_asset(output_path)
        if (
            loaded_payload != payload
            or loaded_durations != durations
            or not hmac.compare_digest(loaded_digest, digest)
        ):
            raise AssetError("position-only asset round-trip identity mismatch")
    except BaseException:
        try:
            os.unlink(output_path)
        except OSError:
            pass
        raise

    print(
        "BUILD_POSITION_ONLY_COPY_ASSET=PASS "
        f"stage={args.expected_stage} messages={len(messages)} "
        f"source_sha256={source_digest} sha256={digest} path={output_path}",
        flush=True,
    )
    return 0


def _maximum_endpoint_error(
    first: Mapping[str, Any], second: Mapping[str, Any]
) -> float | None:
    """Return same-controller endpoint error, or None for independent controllers."""
    if first["topic"] != second["topic"]:
        return None
    if first["joint_names"] != second["joint_names"]:
        raise AssetError("reverse recipe changes joint order on one controller")
    finish = first["points"][-1]["positions"]
    start = second["points"][0]["positions"]
    return max(abs(float(a) - float(b)) for a, b in zip(finish, start))


def build_close_reverse_asset(args: argparse.Namespace) -> int:
    """Build one deterministic reverse component from an admitted 8251 asset."""
    output_path = Path(args.asset)
    source_path = Path(args.source)
    if not source_path.is_absolute():
        raise AssetError("close-reverse source path must be absolute")
    if type(args.expected_source_sha256) is not str or SHA256_PATTERN.fullmatch(
        args.expected_source_sha256
    ) is None:
        raise AssetError("--expected-source-sha256 must be 64 lowercase hexadecimal characters")
    source_payload, _source_durations, source_digest = load_asset(source_path)
    if not hmac.compare_digest(source_digest, args.expected_source_sha256):
        raise AssetError(
            "close-reverse source payload SHA-256 differs from the explicit binding"
        )

    source_messages = source_payload["messages"]
    recipes: Mapping[str, Tuple[str, int, Tuple[int, ...]]] = {
        # Close with the seven certified supported arc messages.  The wrapper
        # detaches at this exact split while the real fingers are still at the
        # sealed clamp endpoint.  Message 5 then loosens them by 1 mm before
        # reverse ingress (4..1), and message 0 opens fully only at neutral.
        "door_close": ("door", 13, (12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)),
        # Neutral/open -> q90 contact/open -> q90 clamped.
        "left_regrasp": ("left_clear", 2, (1, 0)),
        # q90 clamped -> q0 clamped -> open -> reverse approach to neutral.
        "handle_relock": ("handle", 5, (4, 3, 2, 1, 0)),
    }
    expected_stage, expected_count, indices = recipes[args.recipe]
    if source_payload["stage"] != expected_stage or len(source_messages) != expected_count:
        raise AssetError(
            f"{args.recipe} requires a {expected_stage} source with exactly "
            f"{expected_count} messages"
        )
    expected_topics: Mapping[str, Tuple[str, ...]] = {
        "door_close": (
            *("/right_arm_controller/joint_trajectory",) * 7,
            "/right_gripper_controller/joint_trajectory",
            *("/right_arm_controller/joint_trajectory",) * 4,
            "/right_gripper_controller/joint_trajectory",
        ),
        "left_regrasp": (
            "/left_arm_controller/joint_trajectory",
            "/left_gripper_controller/joint_trajectory",
        ),
        "handle_relock": (
            "/left_arm_controller/joint_trajectory",
            "/left_gripper_controller/joint_trajectory",
            "/left_arm_controller/joint_trajectory",
            "/left_arm_controller/joint_trajectory",
            "/left_arm_controller/joint_trajectory",
        ),
    }

    reversed_records = [
        _reverse_position_record(source_messages[index], f"source message[{index}]")
        for index in indices
    ]
    actual_topics = tuple(record["topic"] for record in reversed_records)
    if actual_topics != expected_topics[args.recipe]:
        raise AssetError(
            f"{args.recipe} source controller topology differs from the admitted recipe"
        )
    for record in reversed_records:
        topic = record["topic"]
        if tuple(record["joint_names"]) != CONTROLLER_JOINTS[topic]:
            raise AssetError(f"{args.recipe} source has noncanonical joints for {topic}")

    messages: List[Dict[str, Any]] = []
    wall_offset_ns = 0
    maximum_same_controller_gap = 0.0
    last_by_topic: Dict[str, Mapping[str, Any]] = {}
    for sequence, record in enumerate(reversed_records):
        previous = last_by_topic.get(record["topic"])
        if previous is not None:
            error = _maximum_endpoint_error(previous, record)
            assert error is not None
            maximum_same_controller_gap = max(maximum_same_controller_gap, error)
            if error > ENDPOINT_TOLERANCE_RAD:
                raise AssetError(
                    f"{args.recipe} same-controller reverse gap {error:.9f} rad exceeds "
                    f"{ENDPOINT_TOLERANCE_RAD:.3f} rad"
                )
        message = {
            "sequence": sequence,
            "topic": record["topic"],
            "wall_offset_ns": wall_offset_ns,
            "joint_names": record["joint_names"],
            "points": record["points"],
        }
        messages.append(message)
        duration_ns = _duration_ns(
            record["points"][-1]["time_from_start"],
            f"reverse message[{sequence}].points[-1].time_from_start",
        )
        wall_offset_ns += duration_ns + 50_000_000
        last_by_topic[record["topic"]] = record

    payload: Dict[str, Any] = {
        "schema": SCHEMA,
        "stage": expected_stage,
        "controller_topics": list(CONTROLLER_TOPICS),
        "clock": {
            "wall_offset_unit": "nanoseconds",
            "time_from_start_unit": "sec_nanosec",
        },
        "messages": messages,
    }
    durations = _validate_payload(payload)
    for index, record in enumerate(messages):
        _retimed_message_points(
            record, 1.0, True, f"close-reverse message[{index}]"
        )
    digest = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    _exclusive_write(output_path, {"payload": payload, "sha256": digest})
    try:
        loaded_payload, loaded_durations, loaded_digest = load_asset(output_path)
        if (
            loaded_payload != payload
            or loaded_durations != durations
            or not hmac.compare_digest(loaded_digest, digest)
        ):
            raise AssetError("close-reverse asset round-trip identity mismatch")
    except BaseException:
        try:
            os.unlink(output_path)
        except OSError:
            pass
        raise
    print(
        "BUILD_CLOSE_REVERSE_ASSET=PASS "
        f"recipe={args.recipe} stage={expected_stage} messages={len(messages)} "
        f"source_sha256={source_digest} maximum_same_controller_gap_rad="
        f"{maximum_same_controller_gap:.9g} sha256={digest} path={output_path}",
        flush=True,
    )
    return 0


def _message_record(topic: str, message: Any, sequence: int, wall_offset_ns: int) -> Dict[str, Any]:
    points: List[Dict[str, Any]] = []
    for point in message.points:
        points.append(
            {
                "positions": [float(value) for value in point.positions],
                "velocities": [float(value) for value in point.velocities],
                "accelerations": [float(value) for value in point.accelerations],
                "effort": [float(value) for value in point.effort],
                "time_from_start": {
                    "sec": int(point.time_from_start.sec),
                    "nanosec": int(point.time_from_start.nanosec),
                },
            }
        )
    return {
        "sequence": sequence,
        "topic": topic,
        "wall_offset_ns": wall_offset_ns,
        "joint_names": list(message.joint_names),
        "points": points,
    }


def _install_stop_handlers(stop_event: threading.Event) -> Dict[int, Any]:
    previous: Dict[int, Any] = {}

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    return previous


def _restore_stop_handlers(previous: Mapping[int, Any]) -> None:
    for signum, handler in previous.items():
        signal.signal(signum, handler)


def capture_asset(args: argparse.Namespace) -> int:
    output_path = Path(args.asset)
    if os.path.lexists(output_path):
        raise AssetError(f"capture target must not exist: {output_path}")
    parent = output_path.parent if str(output_path.parent) else Path(".")
    if not parent.is_dir():
        raise AssetError(f"capture target parent is not an existing directory: {parent}")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from trajectory_msgs.msg import JointTrajectory

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(f"h1_joint_trajectory_capture_{os.getpid()}")
    messages: List[Dict[str, Any]] = []
    fatal_errors: List[str] = []
    capture_started_ns = time.monotonic_ns()
    stop_event = threading.Event()
    handlers = _install_stop_handlers(stop_event)
    subscriptions: List[Any] = []
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_ALL,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    def receive(topic: str, message: Any) -> None:
        if stop_event.is_set():
            return
        try:
            wall_offset_ns = time.monotonic_ns() - capture_started_ns
            record = _message_record(topic, message, len(messages), wall_offset_ns)
            previous_offset = messages[-1]["wall_offset_ns"] if messages else -1
            _validate_message(
                record,
                len(messages),
                previous_offset,
                f"captured message[{len(messages)}]",
            )
            messages.append(record)
        except (AssetError, ValueError, TypeError) as exc:
            fatal_errors.append(str(exc))
            stop_event.set()

    for topic in CONTROLLER_TOPICS:
        subscriptions.append(
            node.create_subscription(
                JointTrajectory,
                topic,
                lambda message, bound_topic=topic: receive(bound_topic, message),
                qos,
            )
        )

    print(
        f"capture stage={args.stage} topics={len(CONTROLLER_TOPICS)}; "
        "send SIGINT or SIGTERM to seal the new asset",
        flush=True,
    )
    try:
        while not stop_event.is_set():
            if not rclpy.ok():
                raise AssetError("ROS context stopped before a capture termination signal")
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        _restore_stop_handlers(handlers)
        node.destroy_node()
        rclpy.try_shutdown()

    if fatal_errors:
        raise AssetError(f"capture rejected a malformed trajectory: {fatal_errors[0]}")
    if not messages:
        raise AssetError("capture received no trajectory messages; no file was created")

    payload: Dict[str, Any] = {
        "schema": SCHEMA,
        "stage": args.stage,
        "controller_topics": list(CONTROLLER_TOPICS),
        "clock": {
            "wall_offset_unit": "nanoseconds",
            "time_from_start_unit": "sec_nanosec",
        },
        "messages": messages,
    }
    _validate_payload(payload)
    digest = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    document = {"payload": payload, "sha256": digest}
    _exclusive_write(output_path, document)
    print(
        f"CAPTURE_ASSET=PASS stage={args.stage} messages={len(messages)} "
        f"sha256={digest} path={output_path}",
        flush=True,
    )
    return 0


def _display_trajectory_record(
    message: Any,
    *,
    expected_points: int,
    expected_duration_ns: int,
) -> Dict[str, Any]:
    """Convert one exact MoveIt display trajectory into a position-only record."""
    if message.model_id != "h1_2":
        raise AssetError(
            f"display trajectory model_id {message.model_id!r} differs from 'h1_2'"
        )
    if len(message.trajectory) != 1:
        raise AssetError("display trajectory must contain exactly one robot trajectory")
    trajectory = message.trajectory[0]
    if (
        trajectory.multi_dof_joint_trajectory.joint_names
        or trajectory.multi_dof_joint_trajectory.points
    ):
        raise AssetError("display trajectory unexpectedly contains a multi-DOF path")
    joint_trajectory = trajectory.joint_trajectory
    topic = "/left_arm_controller/joint_trajectory"
    if tuple(joint_trajectory.joint_names) != CONTROLLER_JOINTS[topic]:
        raise AssetError("display trajectory is not the exact ordered seven-joint left arm")
    if len(joint_trajectory.points) != expected_points:
        raise AssetError(
            f"display trajectory has {len(joint_trajectory.points)} points; "
            f"expected exactly {expected_points}"
        )
    points: List[Dict[str, Any]] = []
    previous_ns = -1
    for index, point in enumerate(joint_trajectory.points):
        point_ns = int(point.time_from_start.sec) * 1_000_000_000 + int(
            point.time_from_start.nanosec
        )
        if point_ns <= previous_ns:
            raise AssetError(
                f"display trajectory point[{index}] time is not strictly increasing"
            )
        previous_ns = point_ns
        if len(point.positions) != len(joint_trajectory.joint_names):
            raise AssetError(
                f"display trajectory point[{index}] does not contain seven positions"
            )
        positions = [float(value) for value in point.positions]
        if not all(math.isfinite(value) for value in positions):
            raise AssetError(f"display trajectory point[{index}] is nonfinite")
        points.append(
            {
                "positions": positions,
                "velocities": [],
                "accelerations": [],
                "effort": [],
                "time_from_start": {
                    "sec": point_ns // 1_000_000_000,
                    "nanosec": point_ns % 1_000_000_000,
                },
            }
        )
    if previous_ns != expected_duration_ns:
        raise AssetError(
            f"display trajectory duration is {previous_ns} ns; "
            f"expected exactly {expected_duration_ns} ns"
        )
    return {
        "sequence": 0,
        "topic": topic,
        "wall_offset_ns": 0,
        "joint_names": list(joint_trajectory.joint_names),
        "points": points,
    }


def capture_display_path_asset(args: argparse.Namespace) -> int:
    """Seal one explicitly identified retained MoveIt path without publishing."""
    output_path = Path(args.asset)
    if os.path.lexists(output_path):
        raise AssetError(f"capture-display-path target must not exist: {output_path}")
    parent = output_path.parent if str(output_path.parent) else Path(".")
    if not parent.is_dir():
        raise AssetError(
            f"capture-display-path target parent is not an existing directory: {parent}"
        )
    source_path = Path(args.source)
    if not source_path.is_absolute():
        raise AssetError("capture-display-path source path must be absolute")
    if SHA256_PATTERN.fullmatch(args.expected_source_sha256) is None:
        raise AssetError(
            "--expected-source-sha256 must be 64 lowercase hexadecimal characters"
        )
    source_payload, _source_durations, source_digest = load_asset(source_path)
    if not hmac.compare_digest(source_digest, args.expected_source_sha256):
        raise AssetError(
            "capture-display-path source payload SHA-256 differs from the explicit binding"
        )
    if source_payload["stage"] != "cube":
        raise AssetError("capture-display-path source must be a cube-stage asset")
    if not 0 <= args.source_message_index < len(source_payload["messages"]):
        raise AssetError("--source-message-index is outside the source asset")
    source_message = source_payload["messages"][args.source_message_index]
    if source_message["topic"] != "/left_arm_controller/joint_trajectory":
        raise AssetError("capture-display-path source message is not a left-arm trajectory")
    source_endpoint = source_message["points"][-1]["positions"]

    import rclpy
    from moveit_msgs.msg import DisplayTrajectory
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(f"h1_display_path_capture_{os.getpid()}")
    received: List[Any] = []
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    subscription = node.create_subscription(
        DisplayTrajectory, "/display_planned_path", received.append, qos
    )
    try:
        deadline = time.monotonic() + args.discovery_timeout
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not received:
            raise AssetError("no retained /display_planned_path message was received")
        record = _display_trajectory_record(
            received[-1],
            expected_points=args.expected_points,
            expected_duration_ns=args.expected_duration_ns,
        )
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()

    maximum_start_error = max(
        abs(float(actual) - float(expected))
        for actual, expected in zip(record["points"][0]["positions"], source_endpoint)
    )
    if maximum_start_error > args.endpoint_tolerance:
        raise AssetError(
            f"display path start differs from bound cube endpoint by "
            f"{maximum_start_error:.12g} rad"
        )
    neutral_target = (0.0, 0.30, 0.0, 0.60, 0.0, 0.0, 0.0)
    maximum_neutral_error = max(
        abs(float(actual) - expected)
        for actual, expected in zip(record["points"][-1]["positions"], neutral_target)
    )
    if maximum_neutral_error > args.endpoint_tolerance:
        raise AssetError(
            f"display path end differs from the exact left-neutral target by "
            f"{maximum_neutral_error:.12g} rad"
        )

    payload: Dict[str, Any] = {
        "schema": SCHEMA,
        "stage": "left_clear",
        "controller_topics": list(CONTROLLER_TOPICS),
        "clock": {
            "wall_offset_unit": "nanoseconds",
            "time_from_start_unit": "sec_nanosec",
        },
        "messages": [record],
    }
    durations = _validate_payload(payload)
    _retimed_message_points(record, 1.0, True, "captured display trajectory")
    digest = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    _exclusive_write(output_path, {"payload": payload, "sha256": digest})
    try:
        loaded_payload, loaded_durations, loaded_digest = load_asset(output_path)
        if (
            loaded_payload != payload
            or loaded_durations != durations
            or not hmac.compare_digest(loaded_digest, digest)
        ):
            raise AssetError("capture-display-path asset round-trip identity mismatch")
    except BaseException:
        try:
            os.unlink(output_path)
        except OSError:
            pass
        raise
    print(
        "CAPTURE_DISPLAY_PATH_ASSET=PASS "
        f"stage=left_clear messages=1 points={args.expected_points} "
        f"duration_ns={args.expected_duration_ns} "
        f"start_error_rad={maximum_start_error:.12g} "
        f"neutral_error_rad={maximum_neutral_error:.12g} "
        f"sha256={digest} path={output_path}",
        flush=True,
    )
    return 0


def _state_point(message: Any, modern_name: str, legacy_name: str) -> Any:
    point = getattr(message, modern_name, None)
    if point is None:
        point = getattr(message, legacy_name, None)
    if point is None:
        raise AssetError(
            f"controller-state message has neither {modern_name!r} nor {legacy_name!r}"
        )
    return point


def _controller_sample(topic: str, message: Any) -> Dict[str, Any]:
    expected_joints = CONTROLLER_JOINTS[topic]
    if tuple(message.joint_names) != expected_joints:
        raise AssetError(
            f"{CONTROLLER_STATE_TOPICS[topic]} joint order mismatch: "
            f"expected={list(expected_joints)}, received={list(message.joint_names)}"
        )
    desired = _state_point(message, "reference", "desired")
    actual = _state_point(message, "feedback", "actual")
    count = len(expected_joints)
    desired_positions = _finite_vector(
        list(desired.positions), f"{topic} desired.positions", (count,)
    )
    desired_velocities = _finite_vector(
        list(desired.velocities), f"{topic} desired.velocities", (0, count)
    )
    desired_accelerations = _finite_vector(
        list(desired.accelerations), f"{topic} desired.accelerations", (0, count)
    )
    desired_effort = _finite_vector(
        list(desired.effort), f"{topic} desired.effort", (0, count)
    )
    actual_positions = _finite_vector(
        list(actual.positions), f"{topic} actual.positions", (count,)
    )
    # Validate all available actual derivatives even though only actual position
    # is used to admit the reconstructed command.
    _finite_vector(list(actual.velocities), f"{topic} actual.velocities", (0, count))
    _finite_vector(
        list(actual.accelerations), f"{topic} actual.accelerations", (0, count)
    )
    stamp_sec = _integer(int(message.header.stamp.sec), f"{topic} header.stamp.sec", 0, 2**31 - 1)
    stamp_nanosec = _integer(
        int(message.header.stamp.nanosec),
        f"{topic} header.stamp.nanosec",
        0,
        999_999_999,
    )
    timestamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
    if timestamp_ns <= 0:
        raise AssetError(f"{topic} controller-state has a zero simulation timestamp")
    return {
        "timestamp_ns": timestamp_ns,
        "desired_positions": desired_positions,
        "desired_velocities": desired_velocities,
        "desired_accelerations": desired_accelerations,
        "desired_effort": desired_effort,
        "actual_positions": actual_positions,
    }


def _maximum_position_delta(first: Sequence[float], second: Sequence[float]) -> float:
    return max(abs(float(a) - float(b)) for a, b in zip(first, second))


def _position_span(samples: Sequence[Mapping[str, Any]], field: str) -> float:
    joint_count = len(samples[0][field])
    return max(
        max(float(sample[field][joint]) for sample in samples)
        - min(float(sample[field][joint]) for sample in samples)
        for joint in range(joint_count)
    )


def _rdp_indices(
    timestamps_ns: Sequence[int], positions: Sequence[Sequence[float]], tolerance: float
) -> List[int]:
    """Joint-space RDP using time-linear interpolation."""
    if len(timestamps_ns) <= 2:
        return list(range(len(timestamps_ns)))
    retained = {0, len(timestamps_ns) - 1}
    stack = [(0, len(timestamps_ns) - 1)]
    while stack:
        lower, upper = stack.pop()
        elapsed = timestamps_ns[upper] - timestamps_ns[lower]
        if upper <= lower + 1 or elapsed <= 0:
            continue
        worst_error = -1.0
        worst_index = -1
        for index in range(lower + 1, upper):
            fraction = (timestamps_ns[index] - timestamps_ns[lower]) / elapsed
            error = max(
                abs(
                    float(positions[index][joint])
                    - (
                        float(positions[lower][joint])
                        + fraction
                        * (float(positions[upper][joint]) - float(positions[lower][joint]))
                    )
                )
                for joint in range(len(positions[index]))
            )
            if error > worst_error:
                worst_error = error
                worst_index = index
        if worst_error > tolerance and worst_index > lower:
            retained.add(worst_index)
            stack.append((lower, worst_index))
            stack.append((worst_index, upper))
    return sorted(retained)


def _fill_maximum_period(
    retained: Sequence[int], timestamps_ns: Sequence[int], maximum_period_ns: int
) -> List[int]:
    result = {int(index) for index in retained}
    ordered = sorted(result)
    for lower, upper in zip(ordered, ordered[1:]):
        cursor = lower
        while timestamps_ns[upper] - timestamps_ns[cursor] > maximum_period_ns:
            target = timestamps_ns[cursor] + maximum_period_ns
            next_index = cursor + 1
            while next_index + 1 < upper and timestamps_ns[next_index + 1] <= target:
                next_index += 1
            if next_index <= cursor:
                next_index = cursor + 1
            result.add(next_index)
            cursor = next_index
    return sorted(result)


def _stable_endpoint_index(
    samples: Sequence[Mapping[str, Any]],
    last_active_index: int,
    boundary_index: int,
    stable_window_ns: int,
    stable_tolerance: float,
    actual_tolerance: float,
    topic: str,
) -> int:
    earliest_time = samples[last_active_index]["timestamp_ns"] + stable_window_ns
    for candidate in range(last_active_index, boundary_index):
        if samples[candidate]["timestamp_ns"] < earliest_time:
            continue
        window_start_time = samples[candidate]["timestamp_ns"] - stable_window_ns
        window_start = candidate
        while (
            window_start > last_active_index
            and samples[window_start - 1]["timestamp_ns"] >= window_start_time
        ):
            window_start -= 1
        window = samples[window_start : candidate + 1]
        if _position_span(window, "desired_positions") > stable_tolerance:
            continue
        if _position_span(window, "actual_positions") > stable_tolerance:
            continue
        if (
            _maximum_position_delta(
                samples[candidate]["desired_positions"],
                samples[candidate]["actual_positions"],
            )
            > actual_tolerance
        ):
            continue
        return candidate
    raise AssetError(
        f"{topic} motion segment ending at sample {last_active_index} lacks a stable actual endpoint"
    )


def _scale_nanoseconds(value_ns: int, time_scale: float, context: str) -> int:
    scaled = int(round(value_ns * time_scale))
    if value_ns > 0 and scaled <= 0:
        raise AssetError(f"{context} collapsed to zero at time scale {time_scale}")
    return scaled


def _parse_message_time_scales(value: str) -> Dict[int, float]:
    """Parse a canonical comma-separated INDEX:SCALE replay override list."""
    if type(value) is not str or not value:
        raise AssetError("--message-time-scales must be a non-empty INDEX:SCALE list")
    result: Dict[int, float] = {}
    for item_number, item in enumerate(value.split(",")):
        match = MESSAGE_TIME_SCALE_ITEM_PATTERN.fullmatch(item)
        if match is None:
            raise AssetError(
                "--message-time-scales item "
                f"{item_number} must exactly match INDEX:SCALE with 0 < SCALE <= 1"
            )
        index = int(match.group("index"), 10)
        scale = float(match.group("scale"))
        if not math.isfinite(scale) or scale <= 0.0 or scale > 1.0:
            raise AssetError(
                f"--message-time-scales index {index} scale must satisfy 0 < scale <= 1"
            )
        if index in result:
            raise AssetError(f"--message-time-scales duplicates message index {index}")
        result[index] = scale
    return result


def _selected_message_time_scales(
    parsed: Mapping[int, float], start_index: int, end_index: int, message_count: int
) -> Dict[int, float]:
    """Reject unused or out-of-range overrides before any ROS publisher exists."""
    selected: Dict[int, float] = {}
    for index, scale in parsed.items():
        if index < 0 or index >= message_count:
            raise AssetError(
                f"--message-time-scales index {index} is outside asset range [0,{message_count})"
            )
        if index < start_index or index >= end_index:
            raise AssetError(
                f"--message-time-scales index {index} is outside selected replay range "
                f"[{start_index},{end_index})"
            )
        selected[index] = scale
    if not selected:
        raise AssetError("--message-time-scales selected no replay messages")
    return selected


def _duration_object(value_ns: int) -> Dict[str, int]:
    return {"sec": value_ns // 1_000_000_000, "nanosec": value_ns % 1_000_000_000}


def _reconstruct_controller_segments(
    topic: str, samples: Sequence[Mapping[str, Any]], args: argparse.Namespace
) -> List[Tuple[int, Dict[str, Any]]]:
    if len(samples) < 3:
        return []
    if _position_span(samples, "desired_positions") <= args.motion_threshold:
        return []
    active = [
        index
        for index in range(1, len(samples))
        if _maximum_position_delta(
            samples[index - 1]["desired_positions"], samples[index]["desired_positions"]
        )
        > args.sample_motion_epsilon
    ]
    if not active:
        return []
    segment_gap_ns = int(round(args.segment_gap * 1_000_000_000))
    stable_window_ns = int(round(args.stable_window * 1_000_000_000))
    maximum_period_ns = int(round(args.max_point_period * 1_000_000_000))
    groups: List[List[int]] = [[active[0]]]
    for index in active[1:]:
        previous = groups[-1][-1]
        if samples[index]["timestamp_ns"] - samples[previous]["timestamp_ns"] <= segment_gap_ns:
            groups[-1].append(index)
        else:
            groups.append([index])

    reconstructed: List[Tuple[int, Dict[str, Any]]] = []
    for group_index, group in enumerate(groups):
        first_active = group[0]
        last_active = group[-1]
        start_index = first_active - 1
        boundary = groups[group_index + 1][0] if group_index + 1 < len(groups) else len(samples)
        contact_error_limit = getattr(args, "cube_close_contact_max_error", None)
        seal_cube_close_contact = (
            contact_error_limit is not None
            and args.stage == "cube"
            and topic == "/left_gripper_controller/joint_trajectory"
            and group_index == 0
        )
        resume_from_cube_close_contact = (
            contact_error_limit is not None
            and args.stage == "cube"
            and topic == "/left_gripper_controller/joint_trajectory"
            and group_index == 1
        )
        end_index = _stable_endpoint_index(
            samples,
            last_active,
            boundary,
            stable_window_ns,
            args.stable_tolerance,
            float(contact_error_limit) if seal_cube_close_contact else args.actual_tolerance,
            topic,
        )
        if (
            not resume_from_cube_close_contact
            and _maximum_position_delta(
                samples[start_index]["desired_positions"],
                samples[start_index]["actual_positions"],
            )
            > args.actual_tolerance
        ):
            raise AssetError(f"{topic} segment {group_index} actual initial state is not settled")

        segment_samples = samples[start_index : end_index + 1]
        relative_times = [
            sample["timestamp_ns"] - segment_samples[0]["timestamp_ns"]
            for sample in segment_samples
        ]
        command_positions = [list(sample["desired_positions"]) for sample in segment_samples]
        # The replay admission point must be the physical state that actually
        # existed before this segment, not an idealized reference sample.
        command_positions[0] = list(segment_samples[0]["actual_positions"])
        retained = _rdp_indices(relative_times, command_positions, args.downsample_tolerance)
        retained = _fill_maximum_period(retained, relative_times, maximum_period_ns)
        points: List[Dict[str, Any]] = []
        previous_scaled_time = -1
        for retained_index in retained:
            sample = segment_samples[retained_index]
            scaled_time = _scale_nanoseconds(
                relative_times[retained_index], args.time_scale, "trajectory point time"
            )
            if scaled_time <= previous_scaled_time:
                raise AssetError(f"{topic} scaled trajectory point times are not strictly increasing")
            previous_scaled_time = scaled_time
            if retained_index == 0:
                velocities: List[float] = []
                accelerations: List[float] = []
            else:
                velocities = [
                    float(value) / args.time_scale for value in sample["desired_velocities"]
                ]
                accelerations = [
                    float(value) / (args.time_scale * args.time_scale)
                    for value in sample["desired_accelerations"]
                ]
            if retained_index == retained[-1]:
                velocities = [0.0 for _ in velocities]
                accelerations = [0.0 for _ in accelerations]
            points.append(
                {
                    "positions": command_positions[retained_index],
                    "velocities": velocities,
                    "accelerations": accelerations,
                    "effort": list(sample["desired_effort"]),
                    "time_from_start": _duration_object(scaled_time),
                }
            )
        if len(points) < 2:
            raise AssetError(f"{topic} reconstructed segment has fewer than two points")
        record: Dict[str, Any] = {
            "sequence": -1,
            "topic": topic,
            "wall_offset_ns": -1,
            "joint_names": list(CONTROLLER_JOINTS[topic]),
            "points": points,
        }
        if seal_cube_close_contact:
            record["expected_actual_endpoint"] = {
                "kind": CUBE_CLOSE_CONTACT_ENDPOINT_KIND,
                "positions": [
                    float(value) for value in samples[end_index]["actual_positions"]
                ],
                "desired_error_limit_rad": float(contact_error_limit),
            }
        reconstructed.append(
            (
                segment_samples[0]["timestamp_ns"],
                record,
            )
        )
    return reconstructed


def _state_samples_to_payload(
    samples_by_topic: Mapping[str, Sequence[Mapping[str, Any]]], args: argparse.Namespace
) -> Dict[str, Any]:
    segments: List[Tuple[int, int, Dict[str, Any]]] = []
    topic_order = {topic: index for index, topic in enumerate(CONTROLLER_TOPICS)}
    for topic in CONTROLLER_TOPICS:
        for start_time_ns, record in _reconstruct_controller_segments(
            topic, samples_by_topic[topic], args
        ):
            segments.append((start_time_ns, topic_order[topic], record))
    if not segments:
        raise AssetError("capture-state observed no controller motion; no file was created")
    segments.sort(key=lambda item: (item[0], item[1]))
    first_start_ns = segments[0][0]
    messages: List[Dict[str, Any]] = []
    for sequence, (start_time_ns, _topic_order, record) in enumerate(segments):
        record["sequence"] = sequence
        record["wall_offset_ns"] = _scale_nanoseconds(
            start_time_ns - first_start_ns, args.time_scale, "segment wall offset"
        )
        messages.append(record)
    if getattr(args, "cube_close_contact_max_error", None) is not None:
        contact_count = sum(
            "expected_actual_endpoint" in message for message in messages
        )
        if contact_count != 1:
            raise AssetError(
                "--cube-close-contact-max-error did not identify exactly one first "
                "left-gripper close segment"
            )
    payload: Dict[str, Any] = {
        "schema": SCHEMA,
        "stage": args.stage,
        "controller_topics": list(CONTROLLER_TOPICS),
        "clock": {
            "wall_offset_unit": "nanoseconds",
            "time_from_start_unit": "sec_nanosec",
        },
        "messages": messages,
    }
    _validate_payload(payload)
    return payload


def capture_state_asset(args: argparse.Namespace) -> int:
    output_path = Path(args.asset)
    if os.path.lexists(output_path):
        raise AssetError(f"capture-state target must not exist: {output_path}")
    parent = output_path.parent if str(output_path.parent) else Path(".")
    if not parent.is_dir():
        raise AssetError(f"capture-state target parent is not an existing directory: {parent}")

    import rclpy
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(
        f"h1_joint_trajectory_state_capture_{os.getpid()}",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        automatically_declare_parameters_from_overrides=True,
    )
    stop_event = threading.Event()
    handlers = _install_stop_handlers(stop_event)
    fatal_errors: List[str] = []
    latest_ready: Dict[str, Mapping[str, Any]] = {}
    samples_by_topic: Dict[str, List[Mapping[str, Any]]] = {
        topic: [] for topic in CONTROLLER_TOPICS
    }
    last_timestamp: Dict[str, int] = {}
    armed = False
    qos = QoSProfile(
        depth=1000,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscriptions: List[Any] = []

    def receive(topic: str, message: Any) -> None:
        nonlocal armed
        if stop_event.is_set():
            return
        try:
            sample = _controller_sample(topic, message)
            latest_ready[topic] = sample
            if not armed:
                return
            timestamp_ns = sample["timestamp_ns"]
            previous_timestamp = last_timestamp.get(topic)
            if previous_timestamp is not None and timestamp_ns < previous_timestamp:
                raise AssetError(f"{topic} simulation timestamp moved backwards")
            if previous_timestamp == timestamp_ns:
                samples_by_topic[topic][-1] = sample
            else:
                samples_by_topic[topic].append(sample)
                last_timestamp[topic] = timestamp_ns
        except (AssetError, ValueError, TypeError) as exc:
            fatal_errors.append(str(exc))
            stop_event.set()

    for trajectory_topic in CONTROLLER_TOPICS:
        state_topic = CONTROLLER_STATE_TOPICS[trajectory_topic]
        subscriptions.append(
            node.create_subscription(
                JointTrajectoryControllerState,
                state_topic,
                lambda message, bound_topic=trajectory_topic: receive(bound_topic, message),
                qos,
            )
        )

    try:
        discovery_deadline = time.monotonic() + args.discovery_timeout
        while True:
            if fatal_errors:
                raise AssetError(f"capture-state admission failed: {fatal_errors[0]}")
            if stop_event.is_set() or not rclpy.ok():
                raise AssetError("capture-state interrupted before all controller states were ready")
            clock = node.get_clock()
            clock_ready = bool(getattr(clock, "ros_time_is_active", False)) and clock.now().nanoseconds > 0
            if clock_ready and len(latest_ready) == len(CONTROLLER_TOPICS):
                break
            if time.monotonic() >= discovery_deadline:
                missing = sorted(set(CONTROLLER_TOPICS) - set(latest_ready))
                raise AssetError(f"controller-state discovery timeout; missing={missing}")
            rclpy.spin_once(node, timeout_sec=0.05)

        # Discard transient-local history used for discovery.  Only samples
        # arriving after this explicit marker belong to the requested stage.
        samples_by_topic = {topic: [] for topic in CONTROLLER_TOPICS}
        last_timestamp.clear()
        armed = True
        print(
            f"CAPTURE_STATE_READY=PASS stage={args.stage} "
            "source=controller_state(reference/feedback); send SIGINT/SIGTERM after stable end",
            flush=True,
        )
        while not stop_event.is_set():
            if fatal_errors:
                raise AssetError(f"capture-state rejected a sample: {fatal_errors[0]}")
            if not rclpy.ok():
                raise AssetError("ROS context stopped before a capture-state termination signal")
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        _restore_stop_handlers(handlers)
        node.destroy_node()
        rclpy.try_shutdown()

    if fatal_errors:
        raise AssetError(f"capture-state rejected a sample: {fatal_errors[0]}")
    if any(not samples_by_topic[topic] for topic in CONTROLLER_TOPICS):
        missing = [topic for topic in CONTROLLER_TOPICS if not samples_by_topic[topic]]
        raise AssetError(f"capture-state has no post-arm samples for: {missing}")
    payload = _state_samples_to_payload(samples_by_topic, args)
    digest = hashlib.sha256(_canonical_bytes(payload)).hexdigest()
    _exclusive_write(output_path, {"payload": payload, "sha256": digest})
    print(
        f"CAPTURE_STATE_ASSET=PASS stage={args.stage} messages={len(payload['messages'])} "
        f"time_scale={args.time_scale:.9g} sha256={digest} path={output_path}",
        flush=True,
    )
    return 0


def inspect_asset(args: argparse.Namespace) -> int:
    payload, durations, digest = load_asset(Path(args.asset))
    messages = payload["messages"]
    print(f"schema={payload['schema']}")
    print(f"stage={payload['stage']}")
    print(f"sha256={digest}")
    print(f"message_count={len(messages)}")
    for index, (message, duration_ns) in enumerate(zip(messages, durations)):
        wall_offset_ns = int(message["wall_offset_ns"])
        print(
            f"trajectory[{index}] topic={message['topic']} points={len(message['points'])} "
            f"wall_offset_seconds={wall_offset_ns / 1_000_000_000.0:.9f} "
            f"sim_duration_seconds={duration_ns / 1_000_000_000.0:.9f} "
            f"end_offset_seconds={(wall_offset_ns + duration_ns) / 1_000_000_000.0:.9f}"
        )
    return 0


class JointFeedback:
    """Latest finite joint positions, with per-joint wall freshness."""

    def __init__(self) -> None:
        self.positions: Dict[str, float] = {}
        self.received_ns: Dict[str, int] = {}
        self.error: str = ""

    def update(self, message: Any) -> None:
        if len(message.name) != len(message.position):
            self.error = "received /joint_states with name/position length mismatch"
            return
        if len(set(message.name)) != len(message.name):
            self.error = "received /joint_states with duplicate joint names"
            return
        arrival_ns = time.monotonic_ns()
        for name, raw_position in zip(message.name, message.position):
            position = float(raw_position)
            if not name or not math.isfinite(position):
                self.error = "received /joint_states with an invalid name or position"
                return
            self.positions[name] = position
            self.received_ns[name] = arrival_ns


def _spin_until(
    node: Any,
    stop_event: threading.Event,
    predicate: Callable[[], bool],
    wall_timeout_sec: float,
    description: str,
) -> None:
    import rclpy

    deadline = time.monotonic() + wall_timeout_sec
    while not predicate():
        if stop_event.is_set() or not rclpy.ok():
            raise AssetError(f"interrupted while waiting for {description}")
        if time.monotonic() >= deadline:
            raise AssetError(f"wall timeout while waiting for {description}")
        rclpy.spin_once(node, timeout_sec=0.02)


def _fresh_positions(
    node: Any,
    feedback: JointFeedback,
    joint_names: Sequence[str],
    stop_event: threading.Event,
    timeout_sec: float,
    newer_than_ns: int = 0,
) -> Dict[str, float]:
    freshness_ns = int(timeout_sec * 1_000_000_000)

    def ready() -> bool:
        if feedback.error:
            raise AssetError(feedback.error)
        now_ns = time.monotonic_ns()
        return all(
            name in feedback.positions
            and feedback.received_ns[name] >= newer_than_ns
            and now_ns - feedback.received_ns[name] <= freshness_ns
            for name in joint_names
        )

    _spin_until(node, stop_event, ready, timeout_sec, "fresh joint-state feedback")
    return {name: feedback.positions[name] for name in joint_names}


def _require_close_to_positions(
    actual: Mapping[str, float],
    joint_names: Sequence[str],
    expected: Sequence[float],
    tolerance: float,
    context: str,
) -> None:
    errors = [abs(actual[name] - float(target)) for name, target in zip(joint_names, expected)]
    worst_index = max(range(len(errors)), key=errors.__getitem__)
    if errors[worst_index] > tolerance:
        raise AssetError(
            f"{context} error {errors[worst_index]:.9f} rad exceeds {tolerance:.3f} rad "
            f"at joint {joint_names[worst_index]}"
        )


def _retimed_message_points(
    record: Mapping[str, Any],
    time_scale: float,
    enforce_joint_limits: bool,
    context: str,
) -> List[Dict[str, Any]]:
    if not math.isfinite(time_scale) or time_scale <= 0.0:
        raise AssetError(f"{context} time scale must be finite and greater than zero")
    topic = str(record["topic"])
    joint_names = list(record["joint_names"])
    if enforce_joint_limits:
        expected_joint_names = CONTROLLER_JOINTS.get(topic)
        if expected_joint_names is None or tuple(joint_names) != expected_joint_names:
            raise AssetError(
                f"{context} cannot be safely retimed: joint order does not exactly match "
                f"controller topic {topic}"
            )

    result: List[Dict[str, Any]] = []
    previous_time_ns = -1
    previous_positions: List[float] | None = None
    for point_index, stored_point in enumerate(record["points"]):
        positions = [float(value) for value in stored_point["positions"]]
        velocities = [float(value) / time_scale for value in stored_point["velocities"]]
        accelerations = [
            float(value) / (time_scale * time_scale)
            for value in stored_point["accelerations"]
        ]
        if not all(math.isfinite(value) for value in velocities + accelerations):
            raise AssetError(f"{context}.points[{point_index}] retiming produced a non-finite value")
        stored_time_ns = _duration_ns(
            stored_point["time_from_start"],
            f"{context}.points[{point_index}].time_from_start",
        )
        scaled_time_ns = _scale_nanoseconds(
            stored_time_ns,
            time_scale,
            f"{context}.points[{point_index}] replay time",
        )
        if scaled_time_ns <= previous_time_ns:
            raise AssetError(
                f"{context}.points[{point_index}] retimed time_from_start is not strictly "
                "increasing"
            )

        if enforce_joint_limits:
            elapsed_ns = scaled_time_ns - previous_time_ns if previous_time_ns >= 0 else 0
            for joint_index, joint_name in enumerate(joint_names):
                if joint_name not in JOINT_RETIME_LIMITS:
                    raise AssetError(f"{context} has no sealed retiming limits for {joint_name}")
                lower, upper, maximum_velocity, maximum_acceleration = JOINT_RETIME_LIMITS[
                    joint_name
                ]
                position = positions[joint_index]
                if (
                    position < lower - JOINT_LIMIT_TOLERANCE
                    or position > upper + JOINT_LIMIT_TOLERANCE
                ):
                    raise AssetError(
                        f"{context}.points[{point_index}] {joint_name} position {position:.9g} "
                        f"is outside [{lower:.9g},{upper:.9g}]"
                    )
                if velocities and abs(velocities[joint_index]) > (
                    maximum_velocity + JOINT_LIMIT_TOLERANCE
                ):
                    raise AssetError(
                        f"{context}.points[{point_index}] {joint_name} retimed velocity "
                        f"{velocities[joint_index]:.9g} exceeds {maximum_velocity:.9g}"
                    )
                if accelerations and abs(accelerations[joint_index]) > (
                    maximum_acceleration + JOINT_LIMIT_TOLERANCE
                ):
                    raise AssetError(
                        f"{context}.points[{point_index}] {joint_name} retimed acceleration "
                        f"{accelerations[joint_index]:.9g} exceeds {maximum_acceleration:.9g}"
                    )
                if previous_positions is not None and elapsed_ns > 0:
                    average_velocity = (
                        abs(position - previous_positions[joint_index])
                        * 1_000_000_000.0
                        / elapsed_ns
                    )
                    if average_velocity > maximum_velocity + JOINT_LIMIT_TOLERANCE:
                        raise AssetError(
                            f"{context}.points[{point_index}] {joint_name} retimed segment "
                            f"average velocity {average_velocity:.9g} exceeds "
                            f"{maximum_velocity:.9g}"
                        )

        result.append(
            {
                "positions": positions,
                "velocities": velocities,
                "accelerations": accelerations,
                "effort": [float(value) for value in stored_point["effort"]],
                "time_ns": scaled_time_ns,
            }
        )
        previous_time_ns = scaled_time_ns
        previous_positions = positions
    return result


def _trajectory_message(
    record: Mapping[str, Any],
    time_scale: float,
    enforce_joint_limits: bool = False,
    context: str = "stored message",
) -> Any:
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    retimed_points = _retimed_message_points(
        record, time_scale, enforce_joint_limits, context
    )
    message = JointTrajectory()
    message.joint_names = list(record["joint_names"])
    for retimed in retimed_points:
        point = JointTrajectoryPoint()
        point.positions = retimed["positions"]
        point.velocities = retimed["velocities"]
        point.accelerations = retimed["accelerations"]
        point.effort = retimed["effort"]
        scaled_time_ns = int(retimed["time_ns"])
        point.time_from_start.sec = scaled_time_ns // 1_000_000_000
        point.time_from_start.nanosec = scaled_time_ns % 1_000_000_000
        message.points.append(point)
    return message


def _selected_replay_range(args: argparse.Namespace, message_count: int) -> Tuple[int, int]:
    segmented = args.start_index is not None or args.end_index is not None
    if not segmented:
        return 0, message_count
    if args.expected_stage not in ("door", "cube"):
        raise AssetError(
            "--start-index/--end-index are permitted only for expected stage 'door' or 'cube'"
        )
    if args.start_index is None or args.end_index is None:
        raise AssetError("door/cube segmentation requires both --start-index and --end-index")
    if not (0 <= args.start_index < args.end_index <= message_count):
        raise AssetError(
            f"door/cube segment must satisfy 0 <= start < end <= {message_count}; "
            "end is exclusive"
        )
    return args.start_index, args.end_index


def _current_fresh_positions(
    feedback: JointFeedback,
    joint_names: Sequence[str],
    freshness_sec: float,
    newer_than_ns: int = 0,
) -> Dict[str, float] | None:
    if feedback.error:
        raise AssetError(feedback.error)
    now_ns = time.monotonic_ns()
    freshness_ns = int(freshness_sec * 1_000_000_000)
    if not all(
        name in feedback.positions
        and feedback.received_ns[name] >= newer_than_ns
        and now_ns - feedback.received_ns[name] <= freshness_ns
        for name in joint_names
    ):
        return None
    return {name: feedback.positions[name] for name in joint_names}


def _replay_overlap_groups(
    messages: Sequence[Mapping[str, Any]],
    durations: Sequence[int],
    start_index: int,
    end_index: int,
    time_scale: float,
    parallel_window_sec: float,
    message_time_scales: Mapping[int, float] | None = None,
) -> List[List[Dict[str, Any]]]:
    base_offset_ns = int(messages[start_index]["wall_offset_ns"])
    parallel_window_ns = int(round(parallel_window_sec * 1_000_000_000))
    wall_offset_scale = time_scale if message_time_scales is None else 1.0
    items: List[Dict[str, Any]] = []
    previous_end_by_topic: Dict[str, int] = {}
    for index in range(start_index, end_index):
        stored_start_ns = int(messages[index]["wall_offset_ns"]) - base_offset_ns
        stored_duration_ns = int(durations[index])
        stored_end_ns = stored_start_ns + stored_duration_ns
        topic = str(messages[index]["topic"])
        if topic in previous_end_by_topic and stored_start_ns < previous_end_by_topic[topic]:
            raise AssetError(
                f"trajectory[{index}] overlaps an earlier command on the same controller topic"
            )
        previous_end_by_topic[topic] = stored_end_ns
        duration_scale = (
            time_scale
            if message_time_scales is None
            else float(message_time_scales.get(index, 1.0))
        )
        items.append(
            {
                "index": index,
                "topic": topic,
                "start_ns": _scale_nanoseconds(
                    stored_start_ns, wall_offset_scale, "replay wall offset"
                ),
                "duration_ns": _scale_nanoseconds(
                    stored_duration_ns, duration_scale, "replay trajectory duration"
                ),
            }
        )

    groups: List[List[Dict[str, Any]]] = []
    group_end_ns = -1
    group_start_ns = -1
    for item in items:
        if not groups:
            groups.append([item])
            group_start_ns = item["start_ns"]
            group_end_ns = item["start_ns"] + item["duration_ns"]
            continue
        starts_in_parallel_window = item["start_ns"] - group_start_ns <= parallel_window_ns
        truly_overlaps = item["start_ns"] < group_end_ns
        if starts_in_parallel_window or truly_overlaps:
            groups[-1].append(item)
            group_end_ns = max(group_end_ns, item["start_ns"] + item["duration_ns"])
        else:
            groups.append([item])
            group_start_ns = item["start_ns"]
            group_end_ns = item["start_ns"] + item["duration_ns"]
    for group in groups:
        group_base_ns = group[0]["start_ns"]
        for item in group:
            item["local_start_ns"] = item["start_ns"] - group_base_ns
    return groups


def _replay_groups_critical_duration_ns(
    groups: Sequence[Sequence[Mapping[str, Any]]],
) -> int:
    total_ns = 0
    for group in groups:
        if not group:
            raise AssetError("replay timeline contains an empty overlap group")
        total_ns += max(
            int(item["local_start_ns"]) + int(item["duration_ns"]) for item in group
        )
    return total_ns


def _expected_feedback_endpoint(record: Mapping[str, Any]) -> Sequence[float]:
    """Return the sealed physical endpoint without changing the outgoing command."""
    endpoint = record.get("expected_actual_endpoint")
    if endpoint is not None:
        return endpoint["positions"]
    return record["points"][-1]["positions"]


def _require_record_feedback_endpoint(
    actual: Mapping[str, float],
    record: Mapping[str, Any],
    tolerance: float,
    context: str,
) -> None:
    _require_close_to_positions(
        actual,
        record["joint_names"],
        _expected_feedback_endpoint(record),
        tolerance,
        context,
    )


def _log_message_retime_plan(
    messages: Sequence[Mapping[str, Any]],
    durations: Sequence[int],
    start_index: int,
    end_index: int,
    message_time_scales: Mapping[int, float],
    parallel_window_sec: float,
) -> None:
    statistics = _message_retime_statistics(
        messages,
        durations,
        start_index,
        end_index,
        message_time_scales,
        parallel_window_sec,
    )
    for index in range(start_index, end_index):
        original_duration_ns = int(durations[index])
        scale = float(message_time_scales.get(index, 1.0))
        retimed_duration_ns = _scale_nanoseconds(
            original_duration_ns, scale, f"trajectory[{index}] retimed duration"
        )
        print(
            f"REPLAY_RETIME index={index} topic={messages[index]['topic']} "
            f"override={int(index in message_time_scales)} scale={scale:.9g} "
            f"original_duration_seconds={original_duration_ns / 1_000_000_000.0:.9f} "
            f"retimed_duration_seconds={retimed_duration_ns / 1_000_000_000.0:.9f}",
            flush=True,
        )
    print(
        f"REPLAY_RETIME_TOTAL=PASS messages={end_index - start_index} "
        f"original_duration_seconds={statistics['original_total_ns'] / 1_000_000_000.0:.9f} "
        f"retimed_duration_seconds={statistics['retimed_total_ns'] / 1_000_000_000.0:.9f} "
        "original_critical_path_seconds="
        f"{statistics['original_critical_ns'] / 1_000_000_000.0:.9f} "
        "retimed_critical_path_seconds="
        f"{statistics['retimed_critical_ns'] / 1_000_000_000.0:.9f}",
        flush=True,
    )


def _message_retime_statistics(
    messages: Sequence[Mapping[str, Any]],
    durations: Sequence[int],
    start_index: int,
    end_index: int,
    message_time_scales: Mapping[int, float],
    parallel_window_sec: float,
) -> Dict[str, int]:
    original_groups = _replay_overlap_groups(
        messages, durations, start_index, end_index, 1.0, parallel_window_sec
    )
    retimed_groups = _replay_overlap_groups(
        messages,
        durations,
        start_index,
        end_index,
        1.0,
        parallel_window_sec,
        message_time_scales,
    )
    original_total_ns = 0
    retimed_total_ns = 0
    for index in range(start_index, end_index):
        original_duration_ns = int(durations[index])
        scale = float(message_time_scales.get(index, 1.0))
        retimed_duration_ns = _scale_nanoseconds(
            original_duration_ns, scale, f"trajectory[{index}] retimed duration"
        )
        original_total_ns += original_duration_ns
        retimed_total_ns += retimed_duration_ns
    return {
        "original_total_ns": original_total_ns,
        "retimed_total_ns": retimed_total_ns,
        "original_critical_ns": _replay_groups_critical_duration_ns(original_groups),
        "retimed_critical_ns": _replay_groups_critical_duration_ns(retimed_groups),
    }


def _run_replay_group(
    node: Any,
    stop_event: threading.Event,
    feedback: JointFeedback,
    publishers: Mapping[str, Any],
    messages: Sequence[Mapping[str, Any]],
    outgoing_messages: Mapping[int, Any],
    group: Sequence[Mapping[str, Any]],
    args: argparse.Namespace,
    publication_callback: Callable[[int, str], None] | None = None,
) -> None:
    import rclpy

    # The normal replay path retains the audited 3 mrad limits.  A caller may
    # opt into a separately bounded simulation-only admission window without
    # changing the sealed trajectory or its retiming checks.
    start_tolerance_rad = float(
        getattr(args, "start_tolerance", START_TOLERANCE_RAD)
    )
    endpoint_tolerance_rad = float(
        getattr(args, "endpoint_tolerance", ENDPOINT_TOLERANCE_RAD)
    )
    group_start_sim_ns = node.get_clock().now().nanoseconds
    previous_sim_ns = group_start_sim_ns
    pending = {int(item["index"]): item for item in group}
    active: Dict[int, Dict[str, Any]] = {}
    due_since_wall_ns: Dict[int, int] = {}
    while pending or active:
        if stop_event.is_set() or not rclpy.ok():
            raise AssetError("interrupted during replay timeline")
        sim_now_ns = node.get_clock().now().nanoseconds
        if sim_now_ns < previous_sim_ns:
            raise AssetError("simulation clock moved backwards during replay timeline")
        previous_sim_ns = sim_now_ns
        group_elapsed_ns = sim_now_ns - group_start_sim_ns
        now_wall_ns = time.monotonic_ns()

        for index in sorted(list(pending)):
            item = pending[index]
            if group_elapsed_ns < item["local_start_ns"]:
                continue
            due_since_wall_ns.setdefault(index, now_wall_ns)
            # A delayed endpoint must not be preempted by another command on
            # the same controller, even when its recorded offsets touch.
            if any(entry["topic"] == item["topic"] for entry in active.values()):
                continue
            record = messages[index]
            joint_names = record["joint_names"]
            current = _current_fresh_positions(
                feedback, joint_names, args.state_timeout
            )
            if current is None:
                if (
                    now_wall_ns - due_since_wall_ns[index]
                    > int(args.state_timeout * 1_000_000_000)
                ):
                    raise AssetError(f"trajectory[{index}] lacks fresh start feedback")
                continue
            _require_close_to_positions(
                current,
                joint_names,
                record["points"][0]["positions"],
                start_tolerance_rad,
                f"trajectory[{index}] start",
            )
            if publication_callback is not None:
                publication_callback(index, item["topic"])
            publishers[item["topic"]].publish(outgoing_messages[index])
            publish_wall_ns = time.monotonic_ns()
            publish_sim_ns = node.get_clock().now().nanoseconds
            duration_ns = int(item["duration_ns"])
            timeout_sec = max(
                args.state_timeout,
                duration_ns / 1_000_000_000.0 * args.timeout_factor
                + args.timeout_padding,
            )
            active[index] = {
                "topic": item["topic"],
                "publish_wall_ns": publish_wall_ns,
                "publish_sim_ns": publish_sim_ns,
                "duration_ns": duration_ns,
                "deadline_wall_ns": publish_wall_ns + int(timeout_sec * 1_000_000_000),
            }
            del pending[index]

        for index in sorted(list(active)):
            entry = active[index]
            if sim_now_ns - entry["publish_sim_ns"] < entry["duration_ns"]:
                if now_wall_ns >= entry["deadline_wall_ns"]:
                    raise AssetError(f"trajectory[{index}] simulation duration wall timeout")
                continue
            record = messages[index]
            joint_names = record["joint_names"]
            actual = _current_fresh_positions(
                feedback,
                joint_names,
                args.state_timeout,
                newer_than_ns=entry["publish_wall_ns"],
            )
            endpoint_pass = False
            if actual is not None:
                try:
                    _require_record_feedback_endpoint(
                        actual,
                        record,
                        endpoint_tolerance_rad,
                        f"trajectory[{index}] endpoint",
                    )
                    endpoint_pass = True
                except AssetError:
                    endpoint_pass = False
            if endpoint_pass:
                print(
                    f"REPLAY_TRAJECTORY=PASS index={index} topic={record['topic']} "
                    f"sim_duration_seconds={entry['duration_ns'] / 1_000_000_000.0:.9f}",
                    flush=True,
                )
                del active[index]
            elif now_wall_ns >= entry["deadline_wall_ns"]:
                raise AssetError(
                    f"trajectory[{index}] did not reach its complete live endpoint before timeout"
                )
        if pending or active:
            rclpy.spin_once(node, timeout_sec=0.01)


def retime_check_asset(args: argparse.Namespace) -> int:
    """Validate an entire replay retiming plan without importing ROS."""
    payload, durations, digest = load_asset(Path(args.asset))
    if payload["stage"] != args.expected_stage:
        raise AssetError(
            f"stage mismatch: asset={payload['stage']!r}, expected={args.expected_stage!r}"
        )
    messages = payload["messages"]
    start_index, end_index = _selected_replay_range(args, len(messages))
    message_time_scales = _selected_message_time_scales(
        args.parsed_message_time_scales,
        start_index,
        end_index,
        len(messages),
    )
    # Validate every selected message while still entirely offline.  Explicit
    # overrides additionally opt into the sealed position/velocity/
    # acceleration limits; unlisted messages retain legacy scale 1.
    for index in range(start_index, end_index):
        _retimed_message_points(
            messages[index],
            message_time_scales.get(index, 1.0),
            index in message_time_scales,
            f"trajectory[{index}]",
        )
    statistics = _message_retime_statistics(
        messages,
        durations,
        start_index,
        end_index,
        message_time_scales,
        args.parallel_window,
    )
    print(
        f"RETIME_CHECK=PASS stage={args.expected_stage} "
        f"range=[{start_index},{end_index}) messages={end_index - start_index} "
        f"overrides={len(message_time_scales)} "
        f"original_duration_seconds={statistics['original_total_ns'] / 1_000_000_000.0:.9f} "
        f"retimed_duration_seconds={statistics['retimed_total_ns'] / 1_000_000_000.0:.9f} "
        "original_critical_path_seconds="
        f"{statistics['original_critical_ns'] / 1_000_000_000.0:.9f} "
        "retimed_critical_path_seconds="
        f"{statistics['retimed_critical_ns'] / 1_000_000_000.0:.9f} "
        f"sha256={digest}",
        flush=True,
    )
    return 0


def replay_asset(args: argparse.Namespace) -> int:
    payload, durations, digest = load_asset(Path(args.asset))
    if not args.execute:
        raise AssetError("replay refused: explicit --execute was not supplied")
    if args.expected_stage is None:
        raise AssetError("replay refused: explicit --expected-stage was not supplied")
    if payload["stage"] != args.expected_stage:
        raise AssetError(
            f"stage mismatch: asset={payload['stage']!r}, expected={args.expected_stage!r}"
        )
    messages = payload["messages"]
    start_index, end_index = _selected_replay_range(args, len(messages))
    message_time_scales: Dict[int, float] | None = None
    if args.message_time_scales is not None:
        message_time_scales = _selected_message_time_scales(
            args.parsed_message_time_scales,
            start_index,
            end_index,
            len(messages),
        )
        # Complete every explicit override check before importing rclpy or
        # creating a publisher.  Unlisted messages retain exact legacy scale
        # 1 behavior; listing an index opts that message into the stricter
        # joint-limit certificate, even when its requested scale is 1.
        for index, scale in message_time_scales.items():
            _retimed_message_points(
                messages[index], scale, True, f"trajectory[{index}]"
            )
        _log_message_retime_plan(
            messages,
            durations,
            start_index,
            end_index,
            message_time_scales,
            args.parallel_window,
        )
    replay_groups = _replay_overlap_groups(
        messages,
        durations,
        start_index,
        end_index,
        args.time_scale,
        args.parallel_window,
        message_time_scales,
    )

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(
        f"h1_joint_trajectory_replay_{os.getpid()}",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        automatically_declare_parameters_from_overrides=True,
    )
    stop_event = threading.Event()
    handlers = _install_stop_handlers(stop_event)
    feedback = JointFeedback()
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    joint_state_subscription = node.create_subscription(
        JointState, "/joint_states", feedback.update, qos_profile_sensor_data
    )
    selected_topics = {messages[index]["topic"] for index in range(start_index, end_index)}
    publishers = {
        topic: node.create_publisher(JointTrajectory, topic, qos) for topic in selected_topics
    }
    outgoing_messages = {
        index: _trajectory_message(
            messages[index],
            args.time_scale
            if message_time_scales is None
            else message_time_scales.get(index, 1.0),
            message_time_scales is not None and index in message_time_scales,
            f"trajectory[{index}]",
        )
        for index in range(start_index, end_index)
    }

    def publishers_ready() -> bool:
        return all(
            publisher.get_subscription_count() >= args.minimum_subscriptions
            for publisher in publishers.values()
        )

    try:
        _spin_until(
            node,
            stop_event,
            publishers_ready,
            args.discovery_timeout,
            "controller topic subscribers",
        )

        def simulated_clock_ready() -> bool:
            clock = node.get_clock()
            return bool(getattr(clock, "ros_time_is_active", False)) and clock.now().nanoseconds > 0

        _spin_until(
            node,
            stop_event,
            simulated_clock_ready,
            args.discovery_timeout,
            "active nonzero simulation clock",
        )
        all_selected_joints = sorted(
            {
                joint_name
                for index in range(start_index, end_index)
                for joint_name in messages[index]["joint_names"]
            }
        )
        _fresh_positions(
            node,
            feedback,
            all_selected_joints,
            stop_event,
            args.state_timeout,
        )
        for group_number, group in enumerate(replay_groups):
            print(
                f"REPLAY_GROUP_START={group_number} messages="
                f"{','.join(str(item['index']) for item in group)}",
                flush=True,
            )
            _run_replay_group(
                node,
                stop_event,
                feedback,
                publishers,
                messages,
                outgoing_messages,
                group,
                args,
            )
    finally:
        # No reset, detach, gripper release, or Gazebo command is sent here.
        _restore_stop_handlers(handlers)
        node.destroy_subscription(joint_state_subscription)
        for publisher in publishers.values():
            node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.try_shutdown()

    print(
        f"REPLAY_ASSET=PASS stage={args.expected_stage} range=[{start_index},{end_index}) "
        f"messages={end_index - start_index} sha256={digest}",
        flush=True,
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Capture, strictly inspect, or explicitly replay the four H1 joint-trajectory topics. "
            "The tool never implements Gazebo attach/detach."
        )
    )
    subparsers = parser.add_subparsers(dest="mode", required=True)

    capture = subparsers.add_parser(
        "capture",
        help="capture in memory until SIGINT/SIGTERM, then create a new mode-0400 JSON asset",
    )
    capture.add_argument("asset", help="output JSON path; the target must not exist")
    capture.add_argument("--stage", choices=STAGES, required=True, help="stage identity to seal")
    capture.set_defaults(handler=capture_asset)

    capture_state = subparsers.add_parser(
        "capture-state",
        help=(
            "reconstruct segmented commands from four controller-state reference/feedback streams"
        ),
    )
    capture_state.add_argument("asset", help="output JSON path; the target must not exist")
    capture_state.add_argument(
        "--stage", choices=STAGES, required=True, help="stage identity to seal"
    )
    capture_state.add_argument(
        "--time-scale",
        type=float,
        default=1.0,
        help="uniform multiplier for offsets and durations; below 1 is faster (default: 1)",
    )
    capture_state.add_argument(
        "--motion-threshold",
        type=float,
        default=0.0005,
        help="minimum controller joint span counted as real motion in radians (default: 0.0005)",
    )
    capture_state.add_argument(
        "--sample-motion-epsilon",
        type=float,
        default=0.000001,
        help="per-sample desired delta that marks activity in radians (default: 1e-6)",
    )
    capture_state.add_argument(
        "--segment-gap",
        type=float,
        default=0.12,
        help="simulation seconds of inactivity that split motion segments (default: 0.12)",
    )
    capture_state.add_argument(
        "--stable-window",
        type=float,
        default=0.05,
        help="required stable actual endpoint window in simulation seconds (default: 0.05)",
    )
    capture_state.add_argument(
        "--stable-tolerance",
        type=float,
        default=0.0005,
        help="maximum joint span in an endpoint stable window in radians (default: 0.0005)",
    )
    capture_state.add_argument(
        "--actual-tolerance",
        type=float,
        default=0.003,
        help="maximum desired/actual error at segment boundaries in radians (default: 0.003)",
    )
    capture_state.add_argument(
        "--cube-close-contact-max-error",
        type=float,
        help=(
            "explicit cube-stage-only upper bound for desired-versus-stable-actual error "
            "on the first left-gripper close segment; must be >0.003 and <=0.020 rad, "
            "is sealed into the asset, and does not relax any other segment"
        ),
    )
    capture_state.add_argument(
        "--downsample-tolerance",
        type=float,
        default=0.0002,
        help="joint-space RDP error limit in radians (default: 0.0002)",
    )
    capture_state.add_argument(
        "--max-point-period",
        type=float,
        default=0.05,
        help="maximum simulation seconds between retained points (default: 0.05)",
    )
    capture_state.add_argument(
        "--discovery-timeout",
        type=float,
        default=8.0,
        help="wall seconds to discover four state streams and nonzero simulation clock (default: 8)",
    )
    capture_state.set_defaults(handler=capture_state_asset)

    capture_display_path = subparsers.add_parser(
        "capture-display-path",
        help=(
            "read-only: seal one retained MoveIt display path whose point count, "
            "duration, source endpoint and neutral endpoint are explicitly bound"
        ),
    )
    capture_display_path.add_argument(
        "asset", help="output JSON path; the target must not exist"
    )
    capture_display_path.add_argument(
        "source", help="absolute mode-0400 cube asset that binds the path start"
    )
    capture_display_path.add_argument(
        "--expected-source-sha256",
        required=True,
        help="required embedded payload SHA-256 of the source cube asset",
    )
    capture_display_path.add_argument(
        "--source-message-index",
        type=int,
        required=True,
        help="source left-arm message whose final point must equal the display-path start",
    )
    capture_display_path.add_argument(
        "--expected-points",
        type=int,
        required=True,
        help="exact retained MoveIt path point count",
    )
    capture_display_path.add_argument(
        "--expected-duration-ns",
        type=int,
        required=True,
        help="exact retained MoveIt path duration in nanoseconds",
    )
    capture_display_path.add_argument(
        "--endpoint-tolerance",
        type=float,
        default=0.001,
        help="maximum per-joint source-start and neutral-end error in radians",
    )
    capture_display_path.add_argument(
        "--discovery-timeout",
        type=float,
        default=5.0,
        help="wall seconds to receive the retained display path (default: 5)",
    )
    capture_display_path.set_defaults(handler=capture_display_path_asset)

    build_cube_cache = subparsers.add_parser(
        "build-cube-cache",
        help=(
            "offline-only: build the six-message q60 cube asset from the exact "
            "candidate-3 left-rack position cache"
        ),
    )
    build_cube_cache.add_argument(
        "asset", help="output JSON path; the target must not exist"
    )
    build_cube_cache.add_argument(
        "cache", help="absolute mode-0400 q60 left-rack cache path"
    )
    build_cube_cache.add_argument(
        "--cache-variant",
        choices=tuple(LEFT_RACK_CACHE_VARIANTS),
        default=DEFAULT_LEFT_RACK_CACHE_VARIANT,
        help=(
            "reviewed cache contract registered in this tool "
            f"(default: {DEFAULT_LEFT_RACK_CACHE_VARIANT}); unbound registered "
            "variants fail closed"
        ),
    )
    build_cube_cache.set_defaults(handler=build_cube_cache_asset)

    build_close_reverse = subparsers.add_parser(
        "build-close-reverse",
        help=(
            "offline-only: derive one exact reverse close/regrasp/relock asset "
            "from an explicitly SHA-bound 8251 forward asset"
        ),
    )
    build_close_reverse.add_argument(
        "asset", help="output JSON path; the target must not exist"
    )
    build_close_reverse.add_argument(
        "source", help="absolute mode-0400 admitted 8251 source asset"
    )
    build_close_reverse.add_argument(
        "--recipe",
        choices=("door_close", "left_regrasp", "handle_relock"),
        required=True,
        help="exact reverse sequence to derive",
    )
    build_close_reverse.add_argument(
        "--expected-source-sha256",
        required=True,
        help="required embedded payload SHA-256 of the source asset",
    )
    build_close_reverse.set_defaults(handler=build_close_reverse_asset)

    build_position_only_copy = subparsers.add_parser(
        "build-position-only-copy",
        help=(
            "offline-only: copy one explicitly SHA-bound asset while preserving "
            "positions and timing and clearing derivatives and effort"
        ),
    )
    build_position_only_copy.add_argument(
        "asset", help="output JSON path; the target must not exist"
    )
    build_position_only_copy.add_argument(
        "source", help="absolute mode-0400 source asset"
    )
    build_position_only_copy.add_argument(
        "--expected-stage",
        choices=STAGES,
        required=True,
        help="required source stage identity",
    )
    build_position_only_copy.add_argument(
        "--expected-source-sha256",
        required=True,
        help="required embedded payload SHA-256 of the source asset",
    )
    build_position_only_copy.set_defaults(handler=build_position_only_copy_asset)

    inspect = subparsers.add_parser("inspect", help="strictly validate and summarize an asset")
    inspect.add_argument("asset", help="mode-0400 JSON asset")
    inspect.set_defaults(handler=inspect_asset)

    retime_check = subparsers.add_parser(
        "retime-check",
        help="offline-only validation of a per-message replay timing plan",
    )
    retime_check.add_argument("asset", help="strictly validated mode-0400 JSON asset")
    retime_check.add_argument(
        "--expected-stage",
        choices=STAGES,
        required=True,
        help="required asset stage identity",
    )
    retime_check.add_argument(
        "--start-index",
        type=int,
        help="door/cube only: inclusive first global message index (requires --end-index)",
    )
    retime_check.add_argument(
        "--end-index",
        type=int,
        help="door/cube only: exclusive final global message index (requires --start-index)",
    )
    retime_check.add_argument(
        "--message-time-scales",
        metavar="INDEX:SCALE,...",
        action="append",
        required=True,
        help=(
            "strict global-message-index overrides; each scale must satisfy 0 < scale <= 1 "
            "and unlisted selected messages retain scale 1"
        ),
    )
    retime_check.set_defaults(handler=retime_check_asset, parallel_window=0.02)

    replay = subparsers.add_parser(
        "replay", help="fail closed unless --execute and --expected-stage are both explicit"
    )
    replay.add_argument("asset", help="strictly validated mode-0400 JSON asset")
    replay.add_argument(
        "--execute", action="store_true", help="explicitly authorize controller publication"
    )
    replay.add_argument(
        "--expected-stage",
        choices=STAGES,
        help="required execution-stage identity; it must match the sealed asset",
    )
    replay.add_argument(
        "--start-index",
        type=int,
        help="door/cube only: inclusive first global message index (requires --end-index)",
    )
    replay.add_argument(
        "--end-index",
        type=int,
        help="door/cube only: exclusive final global message index (requires --start-index)",
    )
    replay_timing = replay.add_mutually_exclusive_group()
    replay_timing.add_argument(
        "--time-scale",
        type=float,
        default=1.0,
        help="uniform replay multiplier for offsets and durations; below 1 is faster (default: 1)",
    )
    replay_timing.add_argument(
        "--message-time-scales",
        metavar="INDEX:SCALE,...",
        action="append",
        help=(
            "strict global-message-index overrides; each scale must satisfy 0 < scale <= 1, "
            "unlisted selected messages retain scale 1, recorded inter-message starts stay "
            "unchanged, and this cannot be combined with --time-scale"
        ),
    )
    replay.add_argument(
        "--parallel-window",
        type=float,
        default=0.02,
        help="recorded start-time window treated as one parallel group in seconds (default: 0.02)",
    )
    replay.add_argument(
        "--discovery-timeout",
        type=float,
        default=8.0,
        help="wall seconds for controller discovery and simulation-clock admission (default: 8)",
    )
    replay.add_argument(
        "--minimum-subscriptions",
        type=int,
        default=1,
        help=(
            "minimum matched subscribers required on every selected controller topic "
            "before the first publish (default: 1)"
        ),
    )
    replay.add_argument(
        "--state-timeout",
        type=float,
        default=3.0,
        help="wall seconds to obtain fresh joint feedback (default: 3)",
    )
    replay.add_argument(
        "--timeout-factor",
        type=float,
        default=5.0,
        help="wall timeout multiplier for each stored simulation duration (default: 5)",
    )
    replay.add_argument(
        "--timeout-padding",
        type=float,
        default=10.0,
        help="wall timeout padding per trajectory in seconds (default: 10)",
    )
    replay.set_defaults(handler=replay_asset)
    return parser


def _validate_positive_cli_timeout(value: float, name: str) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise AssetError(f"{name} must be finite and greater than zero")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.mode == "capture-display-path":
            _validate_positive_cli_timeout(
                args.endpoint_tolerance, "--endpoint-tolerance"
            )
            _validate_positive_cli_timeout(
                args.discovery_timeout, "--discovery-timeout"
            )
            if args.endpoint_tolerance > 0.003:
                raise AssetError("--endpoint-tolerance must be <=0.003 rad")
            if args.expected_points < 2:
                raise AssetError("--expected-points must be at least two")
            if args.expected_duration_ns <= 0:
                raise AssetError("--expected-duration-ns must be positive")
        if args.mode == "capture-state":
            for name in (
                "time_scale",
                "motion_threshold",
                "sample_motion_epsilon",
                "segment_gap",
                "stable_window",
                "stable_tolerance",
                "actual_tolerance",
                "downsample_tolerance",
                "max_point_period",
                "discovery_timeout",
            ):
                _validate_positive_cli_timeout(getattr(args, name), f"--{name.replace('_', '-')}")
            if args.motion_threshold <= args.sample_motion_epsilon:
                raise AssetError("--motion-threshold must exceed --sample-motion-epsilon")
            if args.segment_gap < args.stable_window:
                raise AssetError("--segment-gap must be at least --stable-window")
            if args.cube_close_contact_max_error is not None:
                if args.stage != "cube":
                    raise AssetError(
                        "--cube-close-contact-max-error is allowed only with --stage cube"
                    )
                if (
                    not math.isfinite(args.cube_close_contact_max_error)
                    or not (
                        ENDPOINT_TOLERANCE_RAD
                        < args.cube_close_contact_max_error
                        <= CUBE_CLOSE_CONTACT_MAX_ERROR_HARD_LIMIT_RAD
                    )
                ):
                    raise AssetError(
                        "--cube-close-contact-max-error must be finite and in "
                        f"({ENDPOINT_TOLERANCE_RAD}, "
                        f"{CUBE_CLOSE_CONTACT_MAX_ERROR_HARD_LIMIT_RAD}]"
                    )
                if args.actual_tolerance > ENDPOINT_TOLERANCE_RAD:
                    raise AssetError(
                        "--actual-tolerance may not exceed 0.003 when the scoped cube "
                        "contact endpoint contract is enabled"
                    )
        if args.mode in ("replay", "retime-check"):
            if args.message_time_scales is not None and len(args.message_time_scales) != 1:
                raise AssetError("--message-time-scales may be supplied exactly once")
            args.parsed_message_time_scales = {}
            if args.message_time_scales is not None:
                args.parsed_message_time_scales = _parse_message_time_scales(
                    args.message_time_scales[0]
                )
        if args.mode == "replay":
            _validate_positive_cli_timeout(args.time_scale, "--time-scale")
            _validate_positive_cli_timeout(args.parallel_window, "--parallel-window")
            _validate_positive_cli_timeout(args.discovery_timeout, "--discovery-timeout")
            _validate_positive_cli_timeout(args.state_timeout, "--state-timeout")
            _validate_positive_cli_timeout(args.timeout_factor, "--timeout-factor")
            _validate_positive_cli_timeout(args.timeout_padding, "--timeout-padding")
            if args.minimum_subscriptions < 1:
                raise AssetError("--minimum-subscriptions must be at least one")
        return int(args.handler(args))
    except AssetError as exc:
        print(f"h1_joint_trajectory_asset: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
