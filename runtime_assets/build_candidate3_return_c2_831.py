#!/usr/bin/python3
"""Build the plan-only C2/return middle trajectory bundle for result 831.

The builder treats the sealed 828 middle plan and its bound scene-model
provenance as read-only inputs.  It never authorizes execution: its output is
an explicit, deterministic description of the second-cycle middle sequence
that later runtime authorities may audit and consume.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import stat
from typing import Any, Iterable, Mapping, Sequence


WORKSPACE = Path(__file__).resolve().parents[1]
SOURCE_RELATIVE_PATH = Path(
    "runtime_assets/candidate3_middle_e0_828/"
    "candidate3_middle_plan_828.json")
OUTPUT_RELATIVE_PATH = Path(
    "runtime_assets/candidate3_return_c2_831/"
    "candidate3_return_plan_831.json")
EXPECTED_SOURCE_SHA256 = (
    "cb20f8303cb2e75a5fe45025ea4a119fa5ba81054c8cda55442d320052d1634e")

PLAN_SCHEMA = "h1_candidate3_return_plan/831-c2-v1"
BUILDER_VERSION = "h1_candidate3_return_plan_builder/831-c2-v1"
STAGE_NAMESPACE = "C2/return"
CONTINUITY_LIMIT_RAD_OR_M = 0.004

RIGHT_HANDLE_CONTROLLER_MESSAGES_PER_CYCLE = 14
RETURN_MIDDLE_CONTROLLER_MESSAGES = 16
RETURN_CYCLE_CONTROLLER_MESSAGES = 30
TWO_CYCLE_CONTROLLER_MESSAGES = 60

TRANSFORM_COPY = "copy_exact"
TRANSFORM_REVERSE = "reverse_points_mirror_time"


PHASE_SPECS: tuple[Mapping[str, Any], ...] = (
    {
        "name": "c2_return_door_pre_support",
        "semantic": "attach door support before reopening",
        "commands": (),
    },
    {
        "name": "c2_return_door_open_supported",
        "semantic": "reopen door while the right hand remains on the handle",
        "commands": (
            ("door_open_supported", 0, TRANSFORM_COPY),
            ("door_open_supported", 1, TRANSFORM_COPY),
        ),
    },
    {
        "name": "c2_return_door_open_seam_to_workspace",
        "semantic": "reuse the certified open-door seam into cube workspace",
        "commands": (
            ("door60_seam_to_cube", 0, TRANSFORM_COPY),
            ("door60_seam_to_cube", 1, TRANSFORM_COPY),
        ),
    },
    {
        "name": "c2_return_left_ready",
        "semantic": "move the open left hand from park to ready",
        "commands": (("left_cube_ready", 0, TRANSFORM_COPY),),
    },
    {
        "name": "c2_return_enter_inside",
        "semantic": "reverse the old retreat and enter the incubator",
        "commands": (("cube_retreat", 0, TRANSFORM_REVERSE),),
    },
    {
        "name": "c2_return_grasp_inside",
        "semantic": "close the left gripper on the cube at the inside pose",
        "commands": (("cube_release", 0, TRANSFORM_REVERSE),),
    },
    {
        "name": "c2_return_carry_to_origin",
        "semantic": "reverse both carry segments from inside to origin",
        "commands": (
            ("cube_carry_place", 1, TRANSFORM_REVERSE),
            ("cube_carry_place", 0, TRANSFORM_REVERSE),
        ),
    },
    {
        "name": "c2_return_release_at_origin",
        "semantic": "open the left gripper after support detaches at origin",
        "commands": (("cube_release", 0, TRANSFORM_COPY),),
    },
    {
        "name": "c2_return_retreat_from_origin",
        "semantic": "reverse the old contact arm motion away from origin",
        "commands": (("cube_contact", 0, TRANSFORM_REVERSE),),
    },
    {
        "name": "c2_return_left_park",
        "semantic": "return the open left hand from ready to park",
        "commands": (("left_cube_park", 0, TRANSFORM_COPY),),
    },
    {
        "name": "c2_return_door_seam_to_hinge",
        "semantic": "reuse the certified seam back to the hinge path",
        "commands": (
            ("door60_seam_to_hinge", 0, TRANSFORM_COPY),
            ("door60_seam_to_hinge", 1, TRANSFORM_COPY),
        ),
    },
    {
        "name": "c2_return_door_close_supported",
        "semantic": "close the door while the right hand remains on the handle",
        "commands": (
            ("door_close_supported", 0, TRANSFORM_COPY),
            ("door_close_supported", 1, TRANSFORM_COPY),
        ),
    },
)


HOOK_SPECS: tuple[Mapping[str, Any], ...] = (
    {
        "semantic_event": "C2_RETURN_DOOR_ATTACH",
        "action": "door_attach",
        "kind": "before_phase",
        "phase_index": 0,
        "location": "door_handle_support",
        "pre_state": "detached",
        "post_state": "attached",
    },
    {
        "semantic_event": "C2_RETURN_CUBE_ATTACH_INSIDE",
        "action": "cube_attach",
        "kind": "after_phase",
        "phase_index": 5,
        "location": "inside",
        "pre_state": "detached",
        "post_state": "attached",
    },
    {
        "semantic_event": "C2_RETURN_CUBE_DETACH_ORIGIN",
        "action": "cube_detach",
        "kind": "after_phase",
        "phase_index": 6,
        "location": "origin",
        "pre_state": "attached",
        "post_state": "detached",
    },
    {
        "semantic_event": "C2_RETURN_DOOR_DETACH",
        "action": "door_detach",
        "kind": "terminal_post",
        "phase_index": 11,
        "location": "door_closed",
        "pre_state": "attached",
        "post_state": "detached",
    },
)


CONTROLLER_INITIAL_STATE_FIELDS: Mapping[str, tuple[str, str]] = {
    "/torso_controller/joint_trajectory": ("projected_e0", "torso_yaw_rad"),
    "/right_arm_controller/joint_trajectory": ("projected_e0", "right_arm_rad"),
    "/left_arm_controller/joint_trajectory": ("left_park_open", "left_arm_rad"),
    "/left_gripper_controller/joint_trajectory": (
        "left_park_open", "left_fingers_m"),
}


def _sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _canonical(value: Any) -> bytes:
    return (json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def _canonical_sha256(value: Any) -> str:
    return _sha256(_canonical(value))


def _unique_object(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise RuntimeError(f"DUPLICATE_JSON_KEY:{key}")
        result[key] = value
    return result


def _read_stable_regular(path: Path, expected_sha256: str | None = None) -> bytes:
    path = path.resolve(strict=True)
    before = path.lstat()
    if (path.is_symlink() or not stat.S_ISREG(before.st_mode) or
            before.st_nlink != 1):
        raise RuntimeError(f"INPUT_IDENTITY:{path}")
    raw = path.read_bytes()
    after = path.lstat()
    identity = lambda value: (
        value.st_dev, value.st_ino, value.st_size,
        value.st_mtime_ns, value.st_ctime_ns)
    if identity(before) != identity(after) or len(raw) != after.st_size:
        raise RuntimeError(f"INPUT_CHANGED_DURING_READ:{path}")
    actual_sha256 = _sha256(raw)
    if expected_sha256 is not None and actual_sha256 != expected_sha256:
        raise RuntimeError(
            f"INPUT_SHA256:{path}:{actual_sha256}:{expected_sha256}")
    return raw


def _parse_json(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=_unique_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                RuntimeError(f"NONFINITE_JSON:{label}:{token}")))
    except UnicodeDecodeError as error:
        raise RuntimeError(f"JSON_ENCODING:{label}") from error
    if type(value) is not dict:
        raise RuntimeError(f"JSON_ROOT_NOT_OBJECT:{label}")
    return value


def _phase_map(source: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    phases = source.get("phases")
    if type(phases) is not list:
        raise RuntimeError("SOURCE_PHASES")
    result: dict[str, Mapping[str, Any]] = {}
    for phase in phases:
        if type(phase) is not dict or type(phase.get("name")) is not str:
            raise RuntimeError("SOURCE_PHASE")
        name = phase["name"]
        if name in result:
            raise RuntimeError(f"SOURCE_PHASE_DUPLICATE:{name}")
        result[name] = phase
    return result


def _source_command(
        phases: Mapping[str, Mapping[str, Any]], phase_name: str,
        command_index: int) -> Mapping[str, Any]:
    if phase_name not in phases:
        raise RuntimeError(f"SOURCE_PHASE_MISSING:{phase_name}")
    commands = phases[phase_name].get("commands")
    if type(commands) is not list or not (0 <= command_index < len(commands)):
        raise RuntimeError(
            f"SOURCE_COMMAND_MISSING:{phase_name}:{command_index}")
    command = commands[command_index]
    if type(command) is not dict:
        raise RuntimeError(f"SOURCE_COMMAND_TYPE:{phase_name}:{command_index}")
    return command


def _validate_command(command: Mapping[str, Any], label: str) -> None:
    if (type(command.get("topic")) is not str or
            type(command.get("joint_names")) is not list or
            not command["joint_names"] or
            any(type(name) is not str for name in command["joint_names"])):
        raise RuntimeError(f"COMMAND_HEADER:{label}")
    points = command.get("points")
    if type(points) is not list or not points:
        raise RuntimeError(f"COMMAND_POINTS:{label}")
    previous_time = -1
    for point_index, point in enumerate(points):
        if (type(point) is not dict or
                type(point.get("positions")) is not list or
                len(point["positions"]) != len(command["joint_names"]) or
                any(type(value) not in (int, float)
                    for value in point["positions"]) or
                type(point.get("time_from_start_ns")) is not int or
                point["time_from_start_ns"] < previous_time):
            raise RuntimeError(f"COMMAND_POINT:{label}:{point_index}")
        previous_time = point["time_from_start_ns"]


def _reverse_command(command: Mapping[str, Any]) -> dict[str, Any]:
    """Reverse point order and mirror timestamps over the same time interval."""
    result = copy.deepcopy(command)
    points = command["points"]
    first_time = points[0]["time_from_start_ns"]
    last_time = points[-1]["time_from_start_ns"]
    reversed_points = []
    for source_point in reversed(points):
        target_point = copy.deepcopy(source_point)
        target_point["time_from_start_ns"] = (
            first_time + last_time - source_point["time_from_start_ns"])
        reversed_points.append(target_point)
    result["points"] = reversed_points
    return result


def _state_value(
        source: Mapping[str, Any], topic: str,
        joint_names: Sequence[str]) -> list[float]:
    section_name, field_name = CONTROLLER_INITIAL_STATE_FIELDS[topic]
    if section_name == "projected_e0":
        section = source["entry_projection"]["projected_e0"]
    else:
        section = source["terminal_seam"]["left_park_open"]
    value = section[field_name]
    if type(value) is dict:
        try:
            return [float(value[name]) for name in joint_names]
        except KeyError as error:
            raise RuntimeError(f"INITIAL_STATE_JOINT:{topic}") from error
    if type(value) is list and len(value) == len(joint_names):
        return [float(item) for item in value]
    if type(value) in (int, float) and len(joint_names) == 1:
        return [float(value)]
    raise RuntimeError(f"INITIAL_STATE_SHAPE:{topic}")


def _state_document(
        joint_names: Mapping[str, Sequence[str]],
        positions: Mapping[str, Sequence[float]]) -> dict[str, Any]:
    return {
        topic: {
            "joint_names": list(joint_names[topic]),
            "positions": list(positions[topic]),
        }
        for topic in sorted(positions)
    }


def _maximum_difference(first: Iterable[float], second: Iterable[float]) -> float:
    pairs = list(zip(first, second))
    if not pairs:
        return 0.0
    return max(abs(float(left) - float(right)) for left, right in pairs)


def _support_for_phase(index: int) -> dict[str, bool]:
    return {
        "door_pre": index != 0,
        "door_post": True,
        "cube_pre": index == 6,
        "cube_post": index == 5,
    }


def _validate_source_contract(source: Mapping[str, Any]) -> None:
    if source.get("schema") != "h1_candidate3_middle_plan/828-e0-v1":
        raise RuntimeError("SOURCE_SCHEMA")
    if source.get("offline_only") is not True:
        raise RuntimeError("SOURCE_NOT_OFFLINE")
    if source.get("controller_commands_issued") != 0:
        raise RuntimeError("SOURCE_COMMANDS_ALREADY_ISSUED")
    if source.get("entry_projection", {}).get("valid") is not True:
        raise RuntimeError("SOURCE_ENTRY_PROJECTION")
    if source.get("terminal_seam", {}).get("valid") is not True:
        raise RuntimeError("SOURCE_TERMINAL_SEAM")


def build_return_plan(
        source_path: Path | None = None,
        workspace: Path | None = None) -> dict[str, Any]:
    """Build and return the deterministic plan value without writing files."""
    workspace = (WORKSPACE if workspace is None else workspace).resolve()
    source_path = ((workspace / SOURCE_RELATIVE_PATH) if source_path is None
                   else source_path).resolve()
    source_raw = _read_stable_regular(source_path, EXPECTED_SOURCE_SHA256)
    source = _parse_json(source_raw, "SOURCE_MIDDLE_PLAN")
    _validate_source_contract(source)
    phases_by_name = _phase_map(source)

    provenance_binding = source.get("provenance", {}).get(
        "scene_model_provenance")
    if type(provenance_binding) is not dict:
        raise RuntimeError("SOURCE_PROVENANCE_BINDING")
    provenance_path = (workspace / provenance_binding["path"]).resolve()
    provenance_raw = _read_stable_regular(
        provenance_path, provenance_binding["file_sha256"])
    provenance = _parse_json(provenance_raw, "SCENE_MODEL_PROVENANCE")
    if provenance.get("schema") != provenance_binding.get("schema"):
        raise RuntimeError("SOURCE_PROVENANCE_SCHEMA")
    cube_poses = provenance.get("cube_poses")
    if (type(cube_poses) is not dict or
            type(cube_poses.get("contact")) is not dict or
            type(cube_poses.get("release")) is not dict):
        raise RuntimeError("SOURCE_CUBE_POSES")

    controller_joint_names: dict[str, list[str]] = {}
    for spec in PHASE_SPECS:
        for source_phase, source_index, _ in spec["commands"]:
            command = _source_command(
                phases_by_name, source_phase, source_index)
            _validate_command(command, f"{source_phase}:{source_index}")
            topic = command["topic"]
            names = list(command["joint_names"])
            if topic in controller_joint_names and controller_joint_names[topic] != names:
                raise RuntimeError(f"CONTROLLER_JOINT_ORDER:{topic}")
            controller_joint_names[topic] = names
    if set(controller_joint_names) != set(CONTROLLER_INITIAL_STATE_FIELDS):
        raise RuntimeError("CONTROLLER_TOPIC_SET")

    controller_positions = {
        topic: _state_value(source, topic, names)
        for topic, names in controller_joint_names.items()
    }
    initial_state = _state_document(
        controller_joint_names, controller_positions)
    target_phases = []
    continuity = []
    duration_rows = []
    message_cursor = 0

    for phase_index, spec in enumerate(PHASE_SPECS):
        entry_state_sha256 = _canonical_sha256(_state_document(
            controller_joint_names, controller_positions))
        commands = []
        command_lineage = []
        maximum_first_point_error = 0.0
        maximum_command_duration_ns = 0
        for source_phase, source_index, transform in spec["commands"]:
            source_command = _source_command(
                phases_by_name, source_phase, source_index)
            if transform == TRANSFORM_COPY:
                target_command = copy.deepcopy(source_command)
            elif transform == TRANSFORM_REVERSE:
                target_command = _reverse_command(source_command)
            else:
                raise RuntimeError(f"UNKNOWN_TRANSFORM:{transform}")
            _validate_command(
                target_command,
                f"{spec['name']}:{len(commands)}")
            topic = target_command["topic"]
            first_positions = target_command["points"][0]["positions"]
            maximum_first_point_error = max(
                maximum_first_point_error,
                _maximum_difference(controller_positions[topic], first_positions))
            controller_positions[topic] = list(
                target_command["points"][-1]["positions"])
            maximum_command_duration_ns = max(
                maximum_command_duration_ns,
                target_command["points"][-1]["time_from_start_ns"])
            commands.append(target_command)
            command_lineage.append({
                "source_phase": source_phase,
                "source_phase_index": source["phase_names"].index(source_phase),
                "source_command_index": source_index,
                "source_command_sha256": _canonical_sha256(source_command),
                "source_point_count": len(source_command["points"]),
                "transform": transform,
                "derived_command_sha256": _canonical_sha256(target_command),
            })
        if maximum_first_point_error > CONTINUITY_LIMIT_RAD_OR_M:
            raise RuntimeError(
                f"FIRST_POINT_DISCONTINUITY:{spec['name']}:"
                f"{maximum_first_point_error:.17g}")
        range_start = message_cursor
        message_cursor += len(commands)
        exit_state_sha256 = _canonical_sha256(_state_document(
            controller_joint_names, controller_positions))
        target_phases.append({
            "name": spec["name"],
            "stage": f"{STAGE_NAMESPACE}/{spec['name'][10:]}",
            "semantic": spec["semantic"],
            "commanded": bool(commands),
            "message_range": [range_start, message_cursor],
            "commands": commands,
            "command_lineage": command_lineage,
            "entry_state_sha256": entry_state_sha256,
            "exit_state_sha256": exit_state_sha256,
            "support": _support_for_phase(phase_index),
        })
        continuity.append({
            "phase": spec["name"],
            "maximum_first_point_error": maximum_first_point_error,
            "limit": CONTINUITY_LIMIT_RAD_OR_M,
        })
        duration_rows.append({
            "phase": spec["name"],
            "duration_ns": maximum_command_duration_ns,
        })

    if message_cursor != RETURN_MIDDLE_CONTROLLER_MESSAGES:
        raise RuntimeError(
            f"RETURN_MIDDLE_MESSAGE_COUNT:{message_cursor}:"
            f"{RETURN_MIDDLE_CONTROLLER_MESSAGES}")

    hooks = []
    for hook_index, spec in enumerate(HOOK_SPECS):
        phase_index = spec["phase_index"]
        hooks.append({
            "id": f"c2-return-hook-{hook_index}",
            "semantic_event": spec["semantic_event"],
            "action": spec["action"],
            "boundary": {
                "kind": spec["kind"],
                "phase_index": phase_index,
                "phase_name": PHASE_SPECS[phase_index]["name"],
            },
            "location": spec["location"],
            "pre_state": spec["pre_state"],
            "post_state": spec["post_state"],
            "require_verified": True,
        })

    final_state = _state_document(
        controller_joint_names, controller_positions)
    return {
        "schema": PLAN_SCHEMA,
        "builder_version": BUILDER_VERSION,
        "cycle_id": "C2",
        "operation": "return",
        "stage_namespace": STAGE_NAMESPACE,
        "topology": (
            "right_door_reopen_left_cube_return_to_origin_"
            "right_door_relock_831_c2"),
        "offline_only": True,
        "controller_commands_issued": 0,
        "authorization": {
            "status": "PLAN_ONLY",
            "execution_authorized": False,
            "requires_fresh_runtime_audit": True,
        },
        "source_binding": {
            "middle_plan": {
                "path": source_path.relative_to(workspace).as_posix(),
                "file_sha256": _sha256(source_raw),
                "schema": source["schema"],
            },
            "scene_model_provenance": {
                "path": provenance_path.relative_to(workspace).as_posix(),
                "file_sha256": _sha256(provenance_raw),
                "schema": provenance["schema"],
            },
        },
        "scene_binding": copy.deepcopy(source["scene_binding"]),
        "cube_pose_contract": {
            "origin": {
                "source_label": "contact",
                "pose": copy.deepcopy(cube_poses["contact"]),
                "pose_sha256": _canonical_sha256(cube_poses["contact"]),
                "terminal_role": "C2/return/detach_target",
            },
            "inside": {
                "source_label": "release",
                "pose": copy.deepcopy(cube_poses["release"]),
                "pose_sha256": _canonical_sha256(cube_poses["release"]),
                "terminal_role": "C2/return/attach_source",
            },
            "attach_event": "C2_RETURN_CUBE_ATTACH_INSIDE",
            "detach_event": "C2_RETURN_CUBE_DETACH_ORIGIN",
            "final_pose": "origin",
        },
        "path_contract": {
            "candidate3_only": True,
            "contains_partial": False,
            "partial_execution_allowed": False,
            "all_segments_required": True,
            "inverse_transform": TRANSFORM_REVERSE,
        },
        "message_contract": {
            "return_middle": RETURN_MIDDLE_CONTROLLER_MESSAGES,
            "right_handle_per_cycle": RIGHT_HANDLE_CONTROLLER_MESSAGES_PER_CYCLE,
            "return_cycle_total": RETURN_CYCLE_CONTROLLER_MESSAGES,
            "two_cycle_total": TWO_CYCLE_CONTROLLER_MESSAGES,
        },
        "initial_controller_state": initial_state,
        "initial_controller_state_sha256": _canonical_sha256(initial_state),
        "phase_names": [spec["name"] for spec in PHASE_SPECS],
        "phases": target_phases,
        "hooks": hooks,
        "timing": {
            "first_point_continuity_limit": CONTINUITY_LIMIT_RAD_OR_M,
            "first_point_continuity": continuity,
            "phase_durations_ns": duration_rows,
            "sequential_total_duration_ns": sum(
                row["duration_ns"] for row in duration_rows),
        },
        "terminal_contract": {
            "cube_pose": "origin",
            "cube_support": "detached",
            "door": "closed_supported_pending_right_suffix_relock",
            "door_support": "detached_after_terminal_hook",
            "left_hand": "open_and_parked",
            "right_hand": "retains_handle_until_right_suffix",
            "final_controller_state": final_state,
            "final_controller_state_sha256": _canonical_sha256(final_state),
        },
    }


def _write_exclusive(path: Path, raw: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
        getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0), 0o444)
    try:
        view = memoryview(raw)
        while view:
            count = os.write(descriptor, view)
            if count <= 0:
                raise RuntimeError(f"SHORT_WRITE:{path}")
            view = view[count:]
        os.fchmod(descriptor, 0o444)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    if _read_stable_regular(path) != raw:
        raise RuntimeError(f"OUTPUT_READBACK:{path}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", type=Path, default=WORKSPACE / SOURCE_RELATIVE_PATH)
    parser.add_argument(
        "--output", type=Path, default=WORKSPACE / OUTPUT_RELATIVE_PATH)
    arguments = parser.parse_args(argv)
    plan = build_return_plan(arguments.source, WORKSPACE)
    raw = _canonical(plan)
    output = arguments.output.resolve()
    if output.exists() or output.is_symlink():
        if _read_stable_regular(output) != raw:
            raise RuntimeError(f"OUTPUT_EXISTS_WITH_DIFFERENT_CONTENT:{output}")
        status = "MATCH"
    else:
        _write_exclusive(output, raw)
        status = "CREATED"
    print(
        f"C2_RETURN_PLAN={status} path={output} sha256={_sha256(raw)} "
        f"middle_messages={RETURN_MIDDLE_CONTROLLER_MESSAGES} "
        f"return_cycle_messages={RETURN_CYCLE_CONTROLLER_MESSAGES} "
        f"two_cycle_messages={TWO_CYCLE_CONTROLLER_MESSAGES}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
