# flake8: noqa
"""ROS-free semantic authority for the sealed 828 candidate-3 middle."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping, Sequence
from typing import Any


PLAN_SCHEMA = "h1_candidate3_middle_plan/828-e0-v1"
RESULT_SCHEMA = "h1_candidate3_middle_exact_result/828-e0-v1"
CERTIFICATE_SCHEMA = "h1_candidate3_middle_certificate/828-e0-v1"
AUDITOR_VERSION = "h1_candidate3_middle_e0_exact_auditor/828-e0-v1"
ASSET_SCHEMA = "h1_candidate3_middle_trajectory_asset/828-e0-v1"
ENTRY_PROJECTION_SCHEMA = "h1_candidate3_middle_entry_projection/828-e0-v1"
CONTACT_AUTHORIZATION_SCHEMA = (
    "h1_candidate3_middle_contact_authorizations/828-e0-v1")
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

E0_TORSO_RAD = math.radians(-2.7349115403864994)
E0_RIGHT_ARM_RAD = (
    -0.41373146194762,
    0.23755434038212944,
    1.1008080707922079,
    0.8747783439736734,
    -0.3046852067271319,
    0.4094159191441438,
    -0.9999635680199944,
)
E0_RIGHT_FINGERS = {
    "right_dex1_finger_1_joint": 0.0108,
    "right_dex1_finger_2_joint": 0.007,
}
Q0_RIGHT_ARM_RAD = (0.0, -0.25, 0.0, 0.60, 0.0, 0.0, 0.0)
OPEN_RIGHT_FINGERS = {
    "right_dex1_finger_1_joint": -0.019,
    "right_dex1_finger_2_joint": -0.019,
}
LAUNCH_LEFT_ARM_RAD = (
    -0.000012933885986937658,
    0.24998646984876535,
    0.000023774721497885907,
    0.6000753185207717,
    0.000023572560034304773,
    -0.00008730492997967213,
    -0.00001654021997178485,
)
PARK_LEFT_ARM_RAD = (
    LAUNCH_LEFT_ARM_RAD[0], 0.316, *LAUNCH_LEFT_ARM_RAD[2:]
)
OPEN_LEFT_FINGERS = {
    "left_dex1_finger_1_joint": -0.019,
    "left_dex1_finger_2_joint": -0.019,
}

RIGHT_PREFIX_BINDING = {
    "contact_path_path": (
        "runtime_assets/right_handle_8263_natural/variable_torso_v3_828/"
        "contact_path_v3.json"),
    "contact_path_sha256": (
        "19d7db327498b1d14f9fe5ebfb2ed3fe741e7ed166cf4b01eade6fc369cf2e98"),
    "exact_audit_path": (
        "runtime_assets/right_handle_8263_natural/variable_torso_v3_828/"
        "full_topology_exact_audit.json"),
    "exact_audit_sha256": (
        "042ae8ce8c20e15b35222854f84583d1e4cc88ada2e0d8d4bfdd7e53ab4235f2"),
    "authorization_manifest_path": (
        "runtime_assets/right_handle_8263_natural/variable_torso_v3_828/"
        "controller_execution_authorization.json"),
    "authorization_manifest_sha256": (
        "18456c1391057d25bbb9e21bd19f89277bc12576b16e7e720cafee110738c4a0"),
    "endpoint_label": "canonical_e0_q90_held",
}

WORLD_VALUE = {
    "world_name": "h1_world",
    "table_x_m": 0.600,
    "table_guard_primitive_xyz_m": [
        [0.600, 0.0, -0.2],
        [0.950, 0.550, -0.6],
        [0.950, -0.550, -0.6],
        [0.250, 0.550, -0.6],
        [0.250, -0.550, -0.6],
    ],
    "incubator_x_m": 0.590,
    "incubator_y_m": 0.0,
    "incubator_z_m": 0.815,
    "cube_spawn_pose_xyzyaw": [0.334, 0.377, 0.830, math.pi / 2.0],
}

MINIMUM_CLEARANCE_KEYS = frozenset((
    "unauthorized_environment", "table", "door", "handle", "cube",
    "self", "right_handle_nonpad", "left_handle",
))
RIGHT_HANDLE_PADS = frozenset((
    "right_dex1_finger_1_pad_link", "right_dex1_finger_2_pad_link",
))
CUBE_OBJECT_IDS = (
    "green_cube_dynamic_exact", "green_cube_10mm_guard",
)
LEFT_CUBE_TOUCH_LINKS = frozenset((
    "left_dex1_finger_1_slider_link", "left_dex1_finger_1_link",
    "left_dex1_finger_1_pad_link", "left_dex1_finger_2_slider_link",
    "left_dex1_finger_2_link", "left_dex1_finger_2_pad_link",
))

PLAN_KEYS = frozenset((
    "schema", "worker_version", "topology", "offline_only",
    "controller_commands_issued", "scene_binding", "entry_projection",
    "assets", "phase_names", "phases", "hooks",
    "support_contract_sha256", "release_owner", "terminal_seam",
    "final_state", "required_thresholds", "timing", "provenance",
))
RESULT_KEYS = frozenset((
    "schema", "auditor_version", "audit_method", "offline_only",
    "controller_commands_issued", "scene_binding", "entry_projection",
    "plan", "plan_file_sha256", "support_contract_sha256", "assets",
    "phase_results", "contact_authorizations", "metrics", "terminal_seam",
    "valid", "status",
))
CERTIFICATE_KEYS = frozenset((
    "schema", "auditor_version", "offline_only",
    "controller_commands_issued", "plan_schema", "plan_sha256",
    "exact_result_schema", "exact_result_sha256",
    "support_contract_sha256", "scene_binding_sha256",
    "entry_projection_sha256", "asset_bindings_sha256",
    "terminal_seam_sha256", "metrics", "valid", "status",
))

_SHA256 = re.compile(r"[0-9a-f]{64}\Z")
_FORBIDDEN = ("right_release", "right_retreat")


class MiddleExecutionContract828Error(ValueError):
    """Raised when an 828 plan/result/certificate is not authoritative."""


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise MiddleExecutionContract828Error(status)


def _exact(value: Any, keys: frozenset[str], status: str) -> Mapping:
    _require(type(value) is dict, f"{status}_TYPE")
    _require(frozenset(value) == keys, f"{status}_KEYS")
    return value


def _digest(value: Any, status: str) -> str:
    _require(type(value) is str and _SHA256.fullmatch(value) is not None,
             status)
    return value


def _finite(value: Any, status: str) -> float:
    _require(type(value) in (int, float) and type(value) is not bool, status)
    number = float(value)
    _require(math.isfinite(number), status)
    return number


def canonical_hash(value: Any) -> str:
    """Return the contract's canonical JSON SHA-256."""
    try:
        raw = json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
            allow_nan=False).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise MiddleExecutionContract828Error(
            f"CANONICAL_JSON:{error}") from error
    return hashlib.sha256(raw).hexdigest()


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


def _json_container(value: Any, status: str) -> None:
    _require(type(value) in (dict, list), status)
    # Canonicalization also rejects non-JSON values and non-finite numbers.
    canonical_hash(value)


def _close_vector(
    value: Any, expected: Sequence[float], status: str,
    tolerance: float = 1e-12,
) -> None:
    _require(type(value) is list and len(value) == len(expected), status)
    for index, (actual, target) in enumerate(zip(value, expected)):
        _require(abs(_finite(actual, f"{status}:{index}") - target) <= tolerance,
                 f"{status}:{index}")


def _close_fingers(
    value: Any, expected: Mapping[str, float], status: str,
    tolerance: float = 1e-12,
) -> None:
    _require(type(value) is dict and frozenset(value) == frozenset(expected),
             status)
    for name, target in expected.items():
        _require(abs(_finite(value[name], f"{status}:{name}") - target) <=
                 tolerance, f"{status}:{name}")


def _validate_robot_state(value: Any, *, projected: bool, status: str) -> None:
    state = _exact(value, frozenset((
        "label", "torso_yaw_rad", "right_arm_rad", "right_fingers_m",
        "left_arm_rad", "left_fingers_m",
    )), status)
    if projected:
        _require(state["label"] == "canonical_e0_q90_held",
                 f"{status}_LABEL")
        torso = E0_TORSO_RAD
        arm = E0_RIGHT_ARM_RAD
        fingers = E0_RIGHT_FINGERS
    else:
        _require(state["label"] == "raw_scene_q0", f"{status}_LABEL")
        torso = 0.0
        arm = Q0_RIGHT_ARM_RAD
        fingers = OPEN_RIGHT_FINGERS
    _require(abs(_finite(state["torso_yaw_rad"], f"{status}_TORSO") -
                 torso) <= 1e-12, f"{status}_TORSO")
    _close_vector(state["right_arm_rad"], arm, f"{status}_RIGHT_ARM")
    _close_fingers(
        state["right_fingers_m"], fingers, f"{status}_RIGHT_FINGERS")
    left_arm = PARK_LEFT_ARM_RAD if projected else LAUNCH_LEFT_ARM_RAD
    _close_vector(state["left_arm_rad"], left_arm,
                  f"{status}_LEFT_ARM")
    _close_fingers(
        state["left_fingers_m"], OPEN_LEFT_FINGERS,
        f"{status}_LEFT_FINGERS")


def _validate_scene_binding(value: Any) -> None:
    binding = _exact(value, frozenset((
        "scene_path", "scene_cdr_sha256", "scene_name", "raw_scene_hash",
        "planning_frame", "world", "required_object_ids",
        "removed_visual_mask_ids", "unknown_visual_mask_ids",
    )), "SCENE_BINDING")
    _require(type(binding["scene_path"]) is str and binding["scene_path"] and
             not binding["scene_path"].startswith("/"),
             "SCENE_BINDING_PATH")
    _digest(binding["scene_cdr_sha256"], "SCENE_BINDING_CDR_SHA256")
    _digest(binding["raw_scene_hash"], "SCENE_BINDING_RAW_HASH")
    _require(type(binding["scene_name"]) is str and binding["scene_name"],
             "SCENE_BINDING_NAME")
    _require(binding["planning_frame"] == "world",
             "SCENE_BINDING_PLANNING_FRAME")
    world = _exact(binding["world"], frozenset(WORLD_VALUE),
                   "SCENE_BINDING_WORLD")
    for name, target in WORLD_VALUE.items():
        actual = world[name]
        if type(target) is str:
            _require(actual == target, f"SCENE_BINDING_WORLD:{name}")
        elif type(target) is list:
            _require(actual == target, f"SCENE_BINDING_WORLD:{name}")
        else:
            _require(abs(_finite(actual, f"SCENE_BINDING_WORLD:{name}") -
                         target) <= 1e-12,
                     f"SCENE_BINDING_WORLD:{name}")
    required = binding["required_object_ids"]
    _require(type(required) is list and required and
             all(type(item) is str and item for item in required) and
             len(required) == len(frozenset(required)),
             "SCENE_BINDING_REQUIRED_OBJECT_IDS")
    for token in ("table", "incubator", "incubator_handle_exact", "green_cube"):
        _require(any(token == item or token in item for item in required),
                 f"SCENE_BINDING_REQUIRED:{token}")
    removed = binding["removed_visual_mask_ids"]
    unknown = binding["unknown_visual_mask_ids"]
    _require(type(removed) is list and
             all(type(item) is str and item for item in removed) and
             len(removed) == len(frozenset(removed)),
             "SCENE_BINDING_REMOVED_MASKS")
    _require(unknown == [], "SCENE_BINDING_UNKNOWN_MASKS")
    if removed:
        _require("incubator_handle_visual_mask" in removed and
                 "left_dex1_finger_2_visual_mask" in removed,
                 "SCENE_BINDING_MASK_CONFLICT_PAIR")


def _validate_entry_projection(value: Any, scene_binding: Mapping) -> None:
    projection = _exact(value, frozenset((
        "schema", "method", "source_scene_sha256", "raw_robot_state",
        "source_prefix", "projected_e0", "projected_door_angle_rad",
        "projected_handle_angle_rad", "max_abs_joint_error_rad", "valid",
    )), "ENTRY_PROJECTION")
    _require(projection["schema"] == ENTRY_PROJECTION_SCHEMA,
             "ENTRY_PROJECTION_SCHEMA")
    _require(projection["method"] == "prefix_certified_endpoint_projection",
             "ENTRY_PROJECTION_METHOD")
    _require(projection["source_scene_sha256"] ==
             scene_binding["scene_cdr_sha256"],
             "ENTRY_PROJECTION_SCENE_SHA256")
    _validate_robot_state(
        projection["raw_robot_state"], projected=False,
        status="ENTRY_PROJECTION_RAW_STATE")
    prefix = _exact(projection["source_prefix"],
                    frozenset(RIGHT_PREFIX_BINDING),
                    "ENTRY_PROJECTION_SOURCE_PREFIX")
    _require(prefix == RIGHT_PREFIX_BINDING,
             "ENTRY_PROJECTION_SOURCE_PREFIX_BINDING")
    _validate_robot_state(
        projection["projected_e0"], projected=True,
        status="ENTRY_PROJECTION_PROJECTED_E0")
    _require(abs(_finite(projection["projected_door_angle_rad"],
                         "ENTRY_PROJECTION_DOOR") - 0.0) <= 1e-12,
             "ENTRY_PROJECTION_DOOR")
    _require(abs(_finite(projection["projected_handle_angle_rad"],
                         "ENTRY_PROJECTION_HANDLE") - math.pi / 2.0) <= 1e-12,
             "ENTRY_PROJECTION_HANDLE")
    error = _finite(projection["max_abs_joint_error_rad"],
                    "ENTRY_PROJECTION_ERROR")
    _require(0.0 <= error <= 0.004, "ENTRY_PROJECTION_ERROR")
    _require(projection["valid"] is True, "ENTRY_PROJECTION_INVALID")


def _validate_assets(value: Any) -> None:
    _require(type(value) is dict and value, "PLAN_ASSETS")
    for asset_id, asset in value.items():
        _require(type(asset_id) is str and asset_id, "PLAN_ASSET_ID")
        binding = _exact(asset, frozenset((
            "path", "file_sha256", "payload_sha256", "schema",
        )), f"PLAN_ASSET:{asset_id}")
        _require(type(binding["path"]) is str and binding["path"] and
                 not binding["path"].startswith("/"),
                 f"PLAN_ASSET_PATH:{asset_id}")
        _digest(binding["file_sha256"], f"PLAN_ASSET_FILE:{asset_id}")
        _digest(binding["payload_sha256"], f"PLAN_ASSET_PAYLOAD:{asset_id}")
        _require(binding["schema"] == ASSET_SCHEMA,
                 f"PLAN_ASSET_SCHEMA:{asset_id}")


def _validate_terminal_seam(value: Any, status: str) -> None:
    seam = _exact(value, frozenset((
        "entry_e0", "return_e0", "left_park_open", "door_state",
        "support_state", "release_owner", "forbidden_middle_phases", "valid",
    )), status)
    for name in ("entry_e0", "return_e0"):
        state = _exact(seam[name], frozenset((
            "torso_yaw_rad", "right_arm_rad", "right_fingers_m",
            "max_abs_error",
        )), f"{status}_{name.upper()}")
        _require(abs(_finite(state["torso_yaw_rad"],
                             f"{status}_{name}_TORSO") - E0_TORSO_RAD) <= 1e-12,
                 f"{status}_{name}_TORSO")
        _close_vector(state["right_arm_rad"], E0_RIGHT_ARM_RAD,
                      f"{status}_{name}_RIGHT_ARM")
        _close_fingers(state["right_fingers_m"], E0_RIGHT_FINGERS,
                       f"{status}_{name}_RIGHT_FINGERS")
        maximum = _finite(state["max_abs_error"], f"{status}_{name}_ERROR")
        _require(0.0 <= maximum <= 0.004, f"{status}_{name}_ERROR")
    left = _exact(seam["left_park_open"], frozenset((
        "left_arm_rad", "left_fingers_m", "max_abs_error",
    )), f"{status}_LEFT_PARK_OPEN")
    _close_vector(left["left_arm_rad"], PARK_LEFT_ARM_RAD,
                  f"{status}_LEFT_ARM")
    _close_fingers(left["left_fingers_m"], OPEN_LEFT_FINGERS,
                   f"{status}_LEFT_FINGERS")
    left_error = _finite(left["max_abs_error"], f"{status}_LEFT_ERROR")
    _require(0.0 <= left_error <= 0.004, f"{status}_LEFT_ERROR")
    door = _exact(seam["door_state"], frozenset((
        "entry_angle_rad", "return_angle_rad", "handle_angle_rad",
    )), f"{status}_DOOR_STATE")
    _require(abs(_finite(door["entry_angle_rad"], f"{status}_DOOR_ENTRY"))
             <= 1e-12, f"{status}_DOOR_ENTRY")
    _require(abs(_finite(door["return_angle_rad"], f"{status}_DOOR_RETURN"))
             <= 1e-12, f"{status}_DOOR_RETURN")
    _require(abs(_finite(door["handle_angle_rad"], f"{status}_HANDLE") -
                 math.pi / 2.0) <= 1e-12, f"{status}_HANDLE")
    support = _exact(seam["support_state"], frozenset((
        "before_entry", "after_entry", "before_return", "after_return",
    )), f"{status}_SUPPORT_STATE")
    _require(support == {
        "before_entry": "detached",
        "after_entry": "attached",
        "before_return": "attached",
        "after_return": "detached",
    }, f"{status}_SUPPORT_STATE")
    _require(seam["release_owner"] == RELEASE_OWNER, f"{status}_RELEASE_OWNER")
    _require(seam["forbidden_middle_phases"] ==
             ["right_release", "right_retreat"],
             f"{status}_FORBIDDEN_PHASES")
    _require(seam["valid"] is True, f"{status}_INVALID")


def _validate_hooks(value: Any) -> None:
    _require(type(value) is list and len(value) == len(HOOK_EXPECTATIONS),
             "PLAN_HOOKS")
    identifiers: set[str] = set()
    for index, (hook, expected) in enumerate(zip(value, HOOK_EXPECTATIONS)):
        item = _exact(hook, frozenset((
            "id", "boundary", "action", "pre_state", "post_state",
            "require_verified",
        )), f"PLAN_HOOK:{index}")
        identifier = item["id"]
        _require(type(identifier) is str and identifier and
                 identifier not in identifiers, f"PLAN_HOOK_ID:{index}")
        identifiers.add(identifier)
        boundary = _exact(item["boundary"], frozenset((
            "kind", "phase_index", "phase_name",
        )), f"PLAN_HOOK_BOUNDARY:{index}")
        kind, phase_index, action, pre_state, post_state = expected
        _require(boundary == {
            "kind": kind,
            "phase_index": phase_index,
            "phase_name": PHASE_NAMES[phase_index],
        }, f"PLAN_HOOK_BOUNDARY:{index}")
        _require(item["action"] == action and
                 item["pre_state"] == pre_state and
                 item["post_state"] == post_state and
                 item["require_verified"] is True,
                 f"PLAN_HOOK_CONTRACT:{index}")


def _validate_phases(value: Any, assets: Mapping) -> None:
    _require(type(value) is list and len(value) == len(PHASE_NAMES),
             "PLAN_PHASES")
    for index, (phase, name, topics, support_expected) in enumerate(zip(
            value, PHASE_NAMES, PHASE_COMMAND_TOPICS, SUPPORT_EXPECTATIONS)):
        item = _exact(phase, frozenset((
            "name", "commanded", "asset", "message_range", "commands",
            "support", "entry_state_sha256", "exit_state_sha256",
        )), f"PLAN_PHASE:{index}")
        _require(item["name"] == name, f"PLAN_PHASE_NAME:{index}")
        _require(item["commanded"] is bool(topics),
                 f"PLAN_PHASE_COMMANDED:{index}")
        asset = _exact(item["asset"], frozenset((
            "path", "file_sha256", "payload_sha256", "schema",
        )), f"PLAN_PHASE_ASSET:{index}")
        _require(type(asset["path"]) is str and asset["path"] and
                 not asset["path"].startswith("/"),
                 f"PLAN_PHASE_ASSET_PATH:{index}")
        _digest(asset["file_sha256"], f"PLAN_PHASE_ASSET_FILE:{index}")
        _digest(asset["payload_sha256"],
                f"PLAN_PHASE_ASSET_PAYLOAD:{index}")
        _require(asset["schema"] == ASSET_SCHEMA,
                 f"PLAN_PHASE_ASSET_SCHEMA:{index}")
        _require(asset in assets.values(), f"PLAN_PHASE_ASSET_BINDING:{index}")
        message_range = item["message_range"]
        _require(type(message_range) is list and len(message_range) == 2 and
                 all(type(number) is int and number >= 0
                     for number in message_range) and
                 message_range[0] <= message_range[1],
                 f"PLAN_PHASE_RANGE:{index}")
        _digest(item["entry_state_sha256"], f"PLAN_PHASE_ENTRY:{index}")
        _digest(item["exit_state_sha256"], f"PLAN_PHASE_EXIT:{index}")
        support = _exact(item["support"], frozenset((
            "door_pre", "door_post", "cube_pre", "cube_post",
        )), f"PLAN_PHASE_SUPPORT:{index}")
        actual_support = tuple(support[key] for key in (
            "door_pre", "door_post", "cube_pre", "cube_post"))
        _require(actual_support == support_expected,
                 f"PLAN_PHASE_SUPPORT:{index}")
        commands = item["commands"]
        _require(type(commands) is list and len(commands) == len(topics),
                 f"PLAN_PHASE_COMMANDS:{index}")
        _require(message_range[1] - message_range[0] == len(commands),
                 f"PLAN_PHASE_RANGE_COMMAND_COUNT:{index}")
        for command_index, (command, topic) in enumerate(zip(commands, topics)):
            parsed = _exact(command, frozenset((
                "topic", "joint_names", "points",
            )), f"PLAN_PHASE_COMMAND:{index}:{command_index}")
            _require(parsed["topic"] == topic,
                     f"PLAN_PHASE_COMMAND:{index}:{command_index}_TOPIC")
            _require(type(parsed["joint_names"]) is list and
                     all(type(joint) is str and joint
                         for joint in parsed["joint_names"]),
                     f"PLAN_PHASE_COMMAND:{index}:{command_index}_JOINTS")
            points = parsed["points"]
            _require(type(points) is list and points,
                     f"PLAN_PHASE_COMMAND:{index}:{command_index}_POINTS")
            previous = -1
            for point_index, point in enumerate(points):
                point_value = _exact(point, frozenset((
                    "positions", "time_from_start_ns",
                )), f"PLAN_PHASE_POINT:{index}:{command_index}:{point_index}")
                _require(type(point_value["positions"]) is list and
                         len(point_value["positions"]) ==
                         len(parsed["joint_names"]),
                         f"PLAN_PHASE_POINT_POSITIONS:{index}:{command_index}")
                for position in point_value["positions"]:
                    _finite(position,
                            f"PLAN_PHASE_POINT_POSITION:{index}:{command_index}")
                duration = point_value["time_from_start_ns"]
                _require(type(duration) is int and duration > previous,
                         f"PLAN_PHASE_POINT_TIME:{index}:{command_index}")
                previous = duration


def validate_plan(plan: Mapping) -> Mapping:
    """Validate and return the root plan document (never a wrapper)."""
    value = _exact(plan, PLAN_KEYS, "PLAN")
    _require(value["schema"] == PLAN_SCHEMA, "PLAN_SCHEMA")
    _require(type(value["worker_version"]) is str and
             bool(value["worker_version"]), "PLAN_WORKER_VERSION")
    _require(
        (type(value["topology"]) is str and bool(value["topology"])) or
        type(value["topology"]) in (dict, list),
        "PLAN_TOPOLOGY")
    canonical_hash(value["topology"])
    _require(value["offline_only"] is True, "PLAN_OFFLINE_ONLY")
    _require(value["controller_commands_issued"] == 0,
             "PLAN_CONTROLLER_COMMANDS")
    _validate_scene_binding(value["scene_binding"])
    _validate_entry_projection(value["entry_projection"],
                               value["scene_binding"])
    _validate_assets(value["assets"])
    for name in ("final_state", "required_thresholds", "timing",
                 "provenance"):
        _json_container(value[name], f"PLAN_{name.upper()}")
    _require(tuple(value["phase_names"]) == PHASE_NAMES, "PLAN_PHASE_NAMES")
    _digest(value["support_contract_sha256"], "PLAN_SUPPORT_CONTRACT_SHA256")
    _require(value["release_owner"] == RELEASE_OWNER, "PLAN_RELEASE_OWNER")
    _validate_terminal_seam(value["terminal_seam"], "PLAN_TERMINAL_SEAM")
    _validate_hooks(value["hooks"])
    _validate_phases(value["phases"], value["assets"])
    for key, item in value.items():
        if key == "terminal_seam":
            continue
        for text in (key, *_all_strings(item)):
            _require(not any(forbidden in text for forbidden in _FORBIDDEN),
                     "PLAN_FORBIDDEN_RIGHT_RELEASE_RETREAT")
    return value


def _validate_clearances(value: Any, status: str) -> Mapping:
    clearances = _exact(value, MINIMUM_CLEARANCE_KEYS, status)
    for name in MINIMUM_CLEARANCE_KEYS:
        _require(_finite(clearances[name], f"{status}:{name}") >= 0.0,
                 f"{status}:{name}")
    _require(clearances["unauthorized_environment"] >= 0.010001,
             f"{status}:unauthorized_environment")
    _require(clearances["table"] >= 0.010001, f"{status}:table")
    _require(clearances["door"] >= 0.010001, f"{status}:door")
    _require(clearances["right_handle_nonpad"] >= 0.0006,
             f"{status}:right_handle_nonpad")
    _require(clearances["left_handle"] >= 0.05,
             f"{status}:left_handle")
    return clearances


def _asset_id_for_phase(plan: Mapping, phase: Mapping) -> str:
    matches = [name for name, binding in plan["assets"].items()
               if binding == phase["asset"]]
    _require(len(matches) == 1, "RESULT_PHASE_ASSET_BINDING")
    return matches[0]


def _validate_phase_results(value: Any, plan: Mapping) -> tuple[Mapping, ...]:
    _require(type(value) is list and len(value) == len(PHASE_NAMES),
             "RESULT_PHASE_RESULTS")
    output = []
    for index, (phase_result, name, plan_phase) in enumerate(zip(
            value, PHASE_NAMES, plan["phases"])):
        item = _exact(phase_result, frozenset((
            "index", "name", "asset_id", "message_range", "topics",
            "samples", "subdivisions", "minimum_clearances_m",
            "maximum_abs_torso_yaw_rad", "contact_window_ids",
            "endpoint_sha256", "valid", "status",
        )), f"RESULT_PHASE_RESULT:{index}")
        _require(item["index"] == index, f"RESULT_PHASE_INDEX:{index}")
        _require(item["name"] == name, f"RESULT_PHASE_NAME:{index}")
        _require(item["asset_id"] == _asset_id_for_phase(plan, plan_phase),
                 f"RESULT_PHASE_ASSET_ID:{index}")
        message_range = _exact(item["message_range"], frozenset((
            "start_message_index", "end_message_index_exclusive",
        )), f"RESULT_PHASE_RANGE:{index}")
        _require([
            message_range["start_message_index"],
            message_range["end_message_index_exclusive"],
        ] == plan_phase["message_range"], f"RESULT_PHASE_RANGE:{index}")
        _require(item["topics"] == [
            command["topic"] for command in plan_phase["commands"]
        ], f"RESULT_PHASE_TOPICS:{index}")
        samples = item["samples"]
        subdivisions = item["subdivisions"]
        _require(type(samples) is int and samples > 0,
                 f"RESULT_PHASE_SAMPLES:{index}")
        _require(type(subdivisions) is int and subdivisions > 0,
                 f"RESULT_PHASE_SUBDIVISIONS:{index}")
        _validate_clearances(
            item["minimum_clearances_m"],
            f"RESULT_PHASE_CLEARANCES:{index}")
        windows = item["contact_window_ids"]
        _require(type(windows) is list and
                 all(type(window) is str and window for window in windows) and
                 len(windows) == len(frozenset(windows)),
                 f"RESULT_PHASE_CONTACT_WINDOWS:{index}")
        maximum_torso = _finite(
            item["maximum_abs_torso_yaw_rad"],
            f"RESULT_PHASE_TORSO:{index}")
        _require(0.0 <= maximum_torso <= math.pi / 4.0,
                 f"RESULT_PHASE_TORSO:{index}")
        _require(item["endpoint_sha256"] == plan_phase["exit_state_sha256"],
                 f"RESULT_PHASE_ENDPOINT:{index}")
        _require(item["valid"] is True and item["status"] == "PASS",
                 f"RESULT_PHASE_NOT_PASS:{index}")
        output.append(item)
    return tuple(output)


def _observed_pair(value: Any, status: str) -> tuple[str, str]:
    _require(type(value) is list and len(value) == 2 and
             all(type(item) is str and item for item in value), status)
    return value[0], value[1]


def _validate_contact_authorizations(
    value: Any, phase_results: Sequence[Mapping],
) -> None:
    root = _exact(value, frozenset((
        "schema", "windows", "forbidden_left_handle",
        "unauthorized_contacts", "valid",
    )), "RESULT_CONTACT_AUTHORIZATIONS")
    _require(root["schema"] == CONTACT_AUTHORIZATION_SCHEMA,
             "RESULT_CONTACT_SCHEMA")
    windows = root["windows"]
    _require(type(windows) is list and len(windows) == 3,
             "RESULT_CONTACT_WINDOWS")
    identifiers: set[str] = set()
    handle_count = 0
    cube_count = 0
    retreat_count = 0
    window_by_id = {}
    binding_by_window = {}
    for index, window in enumerate(windows):
        item = _exact(window, frozenset((
            "id", "phase_bindings", "object_ids",
            "allowed_links", "penetration_policy",
            "maximum_observed_penetration_m", "observed_pairs",
            "authorized_sample_count", "valid",
        )), f"RESULT_CONTACT_WINDOW:{index}")
        identifier = item["id"]
        _require(type(identifier) is str and identifier and
                 identifier not in identifiers,
                 f"RESULT_CONTACT_WINDOW_ID:{index}")
        identifiers.add(identifier)
        window_by_id[identifier] = item
        phase_bindings = item["phase_bindings"]
        _require(type(phase_bindings) is list and phase_bindings,
                 f"RESULT_CONTACT_WINDOW_BINDINGS:{index}")
        binding_map = {}
        binding_names = []
        for binding_index, binding_value in enumerate(phase_bindings):
            binding = _exact(binding_value, frozenset((
                "phase_name", "asset_id", "message_range",
            )), f"RESULT_CONTACT_WINDOW_BINDING:{index}:{binding_index}")
            phase_name = binding["phase_name"]
            _require(phase_name in PHASE_NAMES and
                     phase_name not in binding_map,
                     f"RESULT_CONTACT_WINDOW_BINDING_PHASE:{index}:"
                     f"{binding_index}")
            _require(type(binding["asset_id"]) is str and
                     binding["asset_id"],
                     f"RESULT_CONTACT_WINDOW_BINDING_ASSET:{index}:"
                     f"{binding_index}")
            message_range = _exact(binding["message_range"], frozenset((
                "start_message_index", "end_message_index_exclusive",
            )), f"RESULT_CONTACT_WINDOW_BINDING_RANGE:{index}:"
                f"{binding_index}")
            start = message_range["start_message_index"]
            end = message_range["end_message_index_exclusive"]
            _require(type(start) is int and type(end) is int and
                     0 <= start <= end,
                     f"RESULT_CONTACT_WINDOW_BINDING_RANGE:{index}:"
                     f"{binding_index}")
            binding_names.append(phase_name)
            binding_map[phase_name] = binding
        _require(binding_names == [
            name for name in PHASE_NAMES if name in binding_map
        ], f"RESULT_CONTACT_WINDOW_BINDING_ORDER:{index}")
        binding_by_window[identifier] = binding_map
        objects = item["object_ids"]
        links = item["allowed_links"]
        _require(type(objects) is list and objects and
                 all(type(name) is str and name for name in objects) and
                 len(objects) == len(frozenset(objects)),
                 f"RESULT_CONTACT_WINDOW_OBJECTS:{index}")
        _require(type(links) is list and links and
                 all(type(name) is str and name for name in links) and
                 len(links) == len(frozenset(links)),
                 f"RESULT_CONTACT_WINDOW_LINKS:{index}")
        is_handle = objects == ["incubator_handle_exact"]
        is_cube = objects == list(CUBE_OBJECT_IDS)
        _require(is_handle != is_cube, f"RESULT_CONTACT_WINDOW_OBJECT_KIND:{index}")
        if is_handle:
            handle_count += 1
            _require(frozenset(links) == RIGHT_HANDLE_PADS,
                     f"RESULT_CONTACT_WINDOW_HANDLE:{index}")
        else:
            cube_count += 1
            _require(frozenset(links).issubset(LEFT_CUBE_TOUCH_LINKS),
                     f"RESULT_CONTACT_WINDOW_CUBE_LINKS:{index}")
        policy = item["penetration_policy"]
        _require(type(policy) is str and policy,
                 f"RESULT_CONTACT_WINDOW_POLICY:{index}")
        penetration = _finite(item["maximum_observed_penetration_m"],
                              f"RESULT_CONTACT_WINDOW_PENETRATION:{index}")
        is_retreat = is_cube and "cube_retreat" in binding_names
        if is_retreat:
            retreat_count += 1
            _require(policy == "NONPENETRATING" and penetration <= 0.0 and
                     frozenset(links) == LEFT_CUBE_TOUCH_LINKS,
                     f"RESULT_CONTACT_WINDOW_RETREAT:{index}")
        observed = item["observed_pairs"]
        _require(type(observed) is list,
                 f"RESULT_CONTACT_WINDOW_OBSERVED:{index}")
        for pair_index, pair_value in enumerate(observed):
            object_id, link = _observed_pair(
                pair_value, f"RESULT_CONTACT_WINDOW_PAIR:{index}:{pair_index}")
            _require(object_id in objects and link in links,
                     f"RESULT_CONTACT_WINDOW_PAIR_AUTH:{index}:{pair_index}")
        count = item["authorized_sample_count"]
        _require(type(count) is int and count > 0,
                 f"RESULT_CONTACT_WINDOW_SAMPLES:{index}")
        _require(item["valid"] is True,
                 f"RESULT_CONTACT_WINDOW_INVALID:{index}")
    _require((handle_count, cube_count, retreat_count) == (1, 2, 1),
             "RESULT_CONTACT_WINDOW_TOPOLOGY")
    references = {window_id: [] for window_id in window_by_id}
    for index, phase in enumerate(phase_results):
        active_windows = []
        for window_id in phase["contact_window_ids"]:
            _require(window_id in window_by_id,
                     f"RESULT_PHASE_CONTACT_WINDOW_BINDING:{index}")
            binding = binding_by_window[window_id].get(phase["name"])
            _require(binding is not None,
                     f"RESULT_PHASE_CONTACT_WINDOW_BINDING:{index}")
            _require(binding["asset_id"] == phase["asset_id"],
                     f"RESULT_PHASE_CONTACT_WINDOW_ASSET:{index}")
            _require(binding["message_range"] == phase["message_range"],
                     f"RESULT_PHASE_CONTACT_WINDOW_RANGE:{index}")
            references[window_id].append(phase["name"])
            active_windows.append(window_by_id[window_id])
        clearances = phase["minimum_clearances_m"]
        if not any("incubator_handle_exact" in window["object_ids"]
                   for window in active_windows):
            _require(clearances["handle"] >= 0.010001,
                     f"RESULT_PHASE_CLEARANCES:{index}:handle")
        if not any(window["object_ids"] == list(CUBE_OBJECT_IDS)
                   for window in active_windows):
            _require(clearances["cube"] >= 0.010001,
                     f"RESULT_PHASE_CLEARANCES:{index}:cube")
    for window_id, binding_map in binding_by_window.items():
        _require(references[window_id] == [
            name for name in PHASE_NAMES if name in binding_map
        ], f"RESULT_CONTACT_WINDOW_REFERENCE_SET:{window_id}")
    forbidden = _exact(root["forbidden_left_handle"], frozenset((
        "object_id", "link_prefix", "contact_count", "minimum_clearance_m",
        "required_minimum_clearance_m", "valid",
    )), "RESULT_FORBIDDEN_LEFT_HANDLE")
    _require(forbidden["object_id"] == "incubator_handle_exact" and
             forbidden["link_prefix"] == "left_" and
             forbidden["contact_count"] == 0,
             "RESULT_FORBIDDEN_LEFT_HANDLE_IDENTITY")
    minimum = _finite(forbidden["minimum_clearance_m"],
                      "RESULT_FORBIDDEN_LEFT_HANDLE_MINIMUM")
    required = _finite(forbidden["required_minimum_clearance_m"],
                       "RESULT_FORBIDDEN_LEFT_HANDLE_REQUIRED")
    _require(required == 0.05 and minimum >= required and
             forbidden["valid"] is True,
             "RESULT_FORBIDDEN_LEFT_HANDLE_CLEARANCE")
    unauthorized = _exact(root["unauthorized_contacts"], frozenset((
        "count", "pairs",
    )), "RESULT_UNAUTHORIZED_CONTACTS")
    _require(unauthorized["count"] == 0 and unauthorized["pairs"] == [],
             "RESULT_UNAUTHORIZED_CONTACTS")
    _require(root["valid"] is True, "RESULT_CONTACT_AUTHORIZATIONS_INVALID")


def _validate_metrics(value: Any) -> None:
    metrics = _exact(value, frozenset((
        "sample_count", "subdivision_count", "minimum_clearances_m",
        "maximum_abs_torso_yaw_rad", "cube_bump", "cube_release_retreat",
        "final_target", "all_thresholds_passed",
    )), "RESULT_METRICS")
    _require(type(metrics["sample_count"]) is int and
             metrics["sample_count"] > 0, "RESULT_METRICS_SAMPLE_COUNT")
    _require(type(metrics["subdivision_count"]) is int and
             metrics["subdivision_count"] > 0,
             "RESULT_METRICS_SUBDIVISION_COUNT")
    _validate_clearances(metrics["minimum_clearances_m"],
                         "RESULT_METRICS_CLEARANCES")
    torso = _finite(metrics["maximum_abs_torso_yaw_rad"],
                    "RESULT_METRICS_TORSO")
    _require(0.0 <= torso <= math.pi / 4.0, "RESULT_METRICS_TORSO")
    bump = _exact(metrics["cube_bump"], frozenset((
        "maximum_tcp_translation_m", "maximum_tcp_rotation_rad",
        "translation_limit_m", "rotation_limit_rad",
    )), "RESULT_METRICS_CUBE_BUMP")
    translation_limit = _finite(bump["translation_limit_m"],
                                "RESULT_METRICS_BUMP_TRANSLATION_LIMIT")
    rotation_limit = _finite(bump["rotation_limit_rad"],
                             "RESULT_METRICS_BUMP_ROTATION_LIMIT")
    _require(abs(translation_limit - 0.006) <= 1e-15 and
             abs(rotation_limit - 0.013962634015954637) <= 1e-15,
             "RESULT_METRICS_BUMP_LIMITS")
    translation = _finite(bump["maximum_tcp_translation_m"],
                          "RESULT_METRICS_BUMP_TRANSLATION")
    rotation = _finite(bump["maximum_tcp_rotation_rad"],
                       "RESULT_METRICS_BUMP_ROTATION")
    _require(0.0 <= translation <= translation_limit and
             0.0 <= rotation <= rotation_limit,
             "RESULT_METRICS_CUBE_BUMP")
    retreat = _exact(metrics["cube_release_retreat"], frozenset((
        "phase_name", "message_index", "start_point_index",
        "start_subdivision", "gate_point_index", "gate_subdivision",
        "window_sample_count", "minimum_exact_distance_m",
        "maximum_exact_distance_m", "final_exact_distance_m",
        "minimum_time_step_ns", "minimum_path_parameter_step",
        "minimum_tcp_separation_projection_step_m",
        "maximum_exact_penetration_m", "post_gate_minimum_exact_distance_m",
        "post_gate_minimum_guard_physical_lower_bound_m",
        "observed_exact_pairs", "observed_guard_pairs",
        "gate_never_regressed",
    )), "RESULT_METRICS_RELEASE_RETREAT")
    _require(retreat["phase_name"] == "cube_retreat",
             "RESULT_METRICS_RETREAT_PHASE")
    for name in ("message_index", "start_point_index", "start_subdivision",
                 "gate_point_index", "gate_subdivision"):
        _require(type(retreat[name]) is int and retreat[name] >= 0,
                 f"RESULT_METRICS_RETREAT_{name.upper()}")
    _require(type(retreat["window_sample_count"]) is int and
             retreat["window_sample_count"] > 0,
             "RESULT_METRICS_RETREAT_SAMPLE_COUNT")
    distances = {}
    for name in (
        "minimum_exact_distance_m", "maximum_exact_distance_m",
        "final_exact_distance_m", "minimum_path_parameter_step",
        "post_gate_minimum_exact_distance_m",
        "post_gate_minimum_guard_physical_lower_bound_m",
    ):
        distances[name] = _finite(retreat[name],
                                  f"RESULT_METRICS_RETREAT_{name.upper()}")
        _require(distances[name] >= 0.0,
                 f"RESULT_METRICS_RETREAT_{name.upper()}")
    _finite(
        retreat["minimum_tcp_separation_projection_step_m"],
        "RESULT_METRICS_RETREAT_MINIMUM_TCP_SEPARATION_PROJECTION_STEP_M")
    _require(distances["minimum_exact_distance_m"] <=
             distances["maximum_exact_distance_m"] and
             distances["minimum_exact_distance_m"] <=
             distances["final_exact_distance_m"],
             "RESULT_METRICS_RETREAT_DISTANCE_ORDER")
    _require(type(retreat["minimum_time_step_ns"]) is int and
             retreat["minimum_time_step_ns"] > 0,
             "RESULT_METRICS_RETREAT_TIME_STEP")
    penetration = _finite(retreat["maximum_exact_penetration_m"],
                          "RESULT_METRICS_RETREAT_PENETRATION")
    _require(penetration <= 0.0,
             "RESULT_METRICS_RETREAT_PENETRATION")
    for name in ("observed_exact_pairs", "observed_guard_pairs"):
        pairs = retreat[name]
        _require(type(pairs) is list, f"RESULT_METRICS_RETREAT_{name.upper()}")
        for index, pair in enumerate(pairs):
            _observed_pair(pair, f"RESULT_METRICS_RETREAT_{name}:{index}")
    _require(retreat["gate_never_regressed"] is True,
             "RESULT_METRICS_RETREAT_REGRESSION")
    target = _exact(metrics["final_target"], frozenset((
        "translation_error_m", "rotation_error_rad", "translation_limit_m",
        "rotation_limit_rad",
    )), "RESULT_METRICS_FINAL_TARGET")
    target_translation_limit = _finite(
        target["translation_limit_m"], "RESULT_FINAL_TRANSLATION_LIMIT")
    target_rotation_limit = _finite(
        target["rotation_limit_rad"], "RESULT_FINAL_ROTATION_LIMIT")
    _require(abs(target_translation_limit - 0.003) <= 1e-15 and
             abs(target_rotation_limit - 0.03490658503988659) <= 1e-15,
             "RESULT_FINAL_TARGET_LIMITS")
    translation_error = _finite(target["translation_error_m"],
                                "RESULT_FINAL_TRANSLATION_ERROR")
    rotation_error = _finite(target["rotation_error_rad"],
                             "RESULT_FINAL_ROTATION_ERROR")
    _require(0.0 <= translation_error <= target_translation_limit and
             0.0 <= rotation_error <= target_rotation_limit,
             "RESULT_FINAL_TARGET_ERROR")
    _require(metrics["all_thresholds_passed"] is True,
             "RESULT_METRICS_THRESHOLDS")


def validate_result(result: Mapping) -> Mapping:
    """Validate an exact result and return its embedded, complete plan."""
    value = _exact(result, RESULT_KEYS, "RESULT")
    _require(value["schema"] == RESULT_SCHEMA, "RESULT_SCHEMA")
    _require(value["auditor_version"] == AUDITOR_VERSION,
             "RESULT_AUDITOR_VERSION")
    _require(type(value["audit_method"]) is str and value["audit_method"],
             "RESULT_AUDIT_METHOD")
    _require(value["offline_only"] is True, "RESULT_OFFLINE_ONLY")
    _require(value["controller_commands_issued"] == 0,
             "RESULT_CONTROLLER_COMMANDS")
    plan = validate_plan(value["plan"])
    _digest(value["plan_file_sha256"], "RESULT_PLAN_FILE_SHA256")
    _digest(value["support_contract_sha256"],
            "RESULT_SUPPORT_CONTRACT_SHA256")
    _require(value["support_contract_sha256"] ==
             plan["support_contract_sha256"],
             "RESULT_PLAN_SUPPORT_CONTRACT")
    for name in ("scene_binding", "entry_projection", "assets",
                 "terminal_seam"):
        _require(value[name] == plan[name],
                 f"RESULT_PLAN_{name.upper()}")
    phase_results = _validate_phase_results(value["phase_results"], plan)
    _validate_contact_authorizations(
        value["contact_authorizations"], phase_results)
    _validate_metrics(value["metrics"])
    _require(value["valid"] is True and value["status"] == "PASS",
             "RESULT_NOT_PASS")
    _require("certificate" not in value, "RESULT_CERTIFICATE_FORBIDDEN")
    return plan


def validate_certificate(
    certificate: Mapping,
    *,
    plan_file_sha256: str,
    exact_result_file_sha256: str,
    result: Mapping,
    support_contract_sha256: str,
) -> Mapping:
    """Validate the external certificate against already read DAG nodes."""
    value = _exact(certificate, CERTIFICATE_KEYS, "CERTIFICATE")
    _digest(plan_file_sha256, "EXPECTED_PLAN_SHA256")
    _digest(exact_result_file_sha256, "EXPECTED_RESULT_SHA256")
    _digest(support_contract_sha256, "EXPECTED_SUPPORT_CONTRACT_SHA256")
    _require(value["schema"] == CERTIFICATE_SCHEMA, "CERTIFICATE_SCHEMA")
    _require(value["auditor_version"] == AUDITOR_VERSION,
             "CERTIFICATE_AUDITOR_VERSION")
    _require(value["offline_only"] is True, "CERTIFICATE_OFFLINE_ONLY")
    _require(value["controller_commands_issued"] == 0,
             "CERTIFICATE_CONTROLLER_COMMANDS")
    _require(value["plan_schema"] == PLAN_SCHEMA, "CERTIFICATE_PLAN_SCHEMA")
    _require(value["plan_sha256"] == plan_file_sha256,
             "CERTIFICATE_PLAN_SHA256")
    _require(value["exact_result_schema"] == RESULT_SCHEMA,
             "CERTIFICATE_RESULT_SCHEMA")
    _require(value["exact_result_sha256"] == exact_result_file_sha256,
             "CERTIFICATE_RESULT_SHA256")
    _require(value["support_contract_sha256"] == support_contract_sha256,
             "CERTIFICATE_SUPPORT_CONTRACT_SHA256")
    result_value = _exact(result, RESULT_KEYS, "RESULT")
    expected_bindings = {
        "scene_binding_sha256": canonical_hash(result_value["scene_binding"]),
        "entry_projection_sha256": canonical_hash(
            result_value["entry_projection"]),
        "asset_bindings_sha256": canonical_hash(result_value["assets"]),
        "terminal_seam_sha256": canonical_hash(result_value["terminal_seam"]),
    }
    for name, expected in expected_bindings.items():
        _require(value[name] == expected, f"CERTIFICATE_{name.upper()}")
    _require(value["metrics"] == result_value["metrics"],
             "CERTIFICATE_METRICS")
    _require(value["valid"] is True and value["status"] == "PASS",
             "CERTIFICATE_NOT_PASS")
    return value


__all__ = (
    "ASSET_SCHEMA", "AUDITOR_VERSION", "CERTIFICATE_KEYS", "CUBE_OBJECT_IDS",
    "CERTIFICATE_SCHEMA", "MiddleExecutionContract828Error", "PHASE_NAMES",
    "PLAN_KEYS", "PLAN_SCHEMA", "RELEASE_OWNER", "RESULT_KEYS",
    "RESULT_SCHEMA", "canonical_hash", "validate_certificate",
    "validate_plan", "validate_result",
)
