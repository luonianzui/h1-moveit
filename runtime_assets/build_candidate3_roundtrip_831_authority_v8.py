#!/usr/bin/python3
"""Build the non-overwriting 831 round-trip authority v8.

V8 preserves every v7 waypoint, controller-message count, support transition,
fresh-feedback gate, and endpoint tolerance.  It only retimes the four right
hand stage classes inside analytically checked velocity/acceleration limits.
The loaded-cube middle trajectories are deliberately unchanged.
"""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V7_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v7.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v8_base", V7_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V8_BASE_BUILDER_IMPORT")
V7 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V7
SPEC.loader.exec_module(V7)
BUILDER = V7.BUILDER


V7_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v7")
V7_MANIFEST_SHA256 = (
    "814a90cfca9e1c52c9ef2e536ac5b27f3bdb45bf21e318fcd3b807046fafc40d")
V7_COMMANDS_SHA256 = (
    "f5d2f2aff2da41142b36dfc4acc7dd5acb5eb8129caeccb4203c78dce0c0053f")

RIGHT_STAGE_TIME_SCALES = {
    "prepare_left_park": 0.82,
    "ingress_open_to_q0": 0.82,
    "close_q0_right_hand": 0.87,
    "contact_arc_q0_to_q90_hold": 0.99,
    "contact_arc_q90_hold_to_q0": 0.99,
    "release_q0_right_hand": 0.87,
    "egress_open_from_q0": 0.82,
    "return_left_launch_seam": 0.82,
}

BASE_RIGHT_TRANSFORM = BUILDER.IMMUTABLE_BYTE_TRANSFORMS[
    "h1_right_handle_executor_828.py"]
BASE_MAKE_COMMANDS = BUILDER._make_commands
BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
BASE_SELF_TEST = BUILDER.self_test
V7_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)


def _replace_once(raw: bytes, old: bytes, new: bytes, label: str) -> bytes:
    if raw.count(old) != 1:
        raise RuntimeError(f"V8_RIGHT_RETIME_SOURCE:{label}")
    return raw.replace(old, new)


def _retime_right_executor(raw: bytes) -> bytes:
    """Apply v5's settle extension, then the v8 per-stage retiming."""
    transformed = BASE_RIGHT_TRANSFORM(raw)
    transformed = _replace_once(
        transformed,
        b"MAX_DENSE_POINTS_PER_STAGE = 100_000\n"
        b"REQUIRED_CONTROLLER_AND_RECORDER_SUBSCRIPTIONS = 2\n",
        b"MAX_DENSE_POINTS_PER_STAGE = 100_000\n"
        b"REQUIRED_CONTROLLER_AND_RECORDER_SUBSCRIPTIONS = 2\n"
        b"# V8 changes timing only; every dense waypoint remains identical.\n"
        b"HARD_STAGE_TIME_SCALES = {\n"
        b"    \"prepare_left_park\": 0.82,\n"
        b"    \"ingress_open_to_q0\": 0.82,\n"
        b"    \"close_q0_right_hand\": 0.87,\n"
        b"    \"contact_arc_q0_to_q90_hold\": 0.99,\n"
        b"    \"contact_arc_q90_hold_to_q0\": 0.99,\n"
        b"    \"release_q0_right_hand\": 0.87,\n"
        b"    \"egress_open_from_q0\": 0.82,\n"
        b"    \"return_left_launch_seam\": 0.82,\n"
        b"}\n",
        "constants")
    transformed = _replace_once(
        transformed,
        b"            dense = densify_stage(stage, bundle.limits)\n"
        b"            sim_duration = (bundle.limits.lead_time_s +\n"
        b"                            (len(dense) - 1) *\n"
        b"                            bundle.limits.point_dt_s * time_scale)\n",
        b"            dense = densify_stage(stage, bundle.limits)\n"
        b"            stage_time_scale = HARD_STAGE_TIME_SCALES.get(stage.name)\n"
        b"            _require(stage_time_scale is not None,\n"
        b"                     f\"STAGE_TIME_SCALE:{stage.name}\")\n"
        b"            point_dt_s = (bundle.limits.point_dt_s * time_scale *\n"
        b"                          stage_time_scale)\n"
        b"            sim_duration = (bundle.limits.lead_time_s +\n"
        b"                            (len(dense) - 1) * point_dt_s)\n",
        "execute-duration")
    transformed = _replace_once(
        transformed,
        b"                joint_names = CONTROLLER_JOINTS[topic]\n"
        b"                point_dt_s = bundle.limits.point_dt_s * time_scale\n"
        b"                velocities = _pchip_velocities(dense, joint_names, point_dt_s)\n",
        b"                joint_names = CONTROLLER_JOINTS[topic]\n"
        b"                velocities = _pchip_velocities(dense, joint_names, point_dt_s)\n",
        "execute-profile")
    transformed = _replace_once(
        transformed,
        b"                    seconds = (bundle.limits.lead_time_s +\n"
        b"                               index * bundle.limits.point_dt_s * time_scale)\n",
        b"                    seconds = (bundle.limits.lead_time_s +\n"
        b"                               index * point_dt_s)\n",
        "execute-message-time")
    transformed = _replace_once(
        transformed,
        b"        for stage in stages:\n"
        b"            dense = densify_stage(stage, bundle.limits)\n"
        b"            dense_count += len(dense)\n"
        b"            for topic in stage.topics:\n"
        b"                names = CONTROLLER_JOINTS[topic]\n"
        b"                velocities = _pchip_velocities(\n"
        b"                    dense, names, bundle.limits.point_dt_s)\n"
        b"                _validate_fast_cubic_profile(\n"
        b"                    dense, velocities, names, bundle.limits.point_dt_s,\n"
        b"                    stage.name)\n",
        b"        for stage in stages:\n"
        b"            dense = densify_stage(stage, bundle.limits)\n"
        b"            dense_count += len(dense)\n"
        b"            stage_time_scale = HARD_STAGE_TIME_SCALES.get(stage.name)\n"
        b"            _require(stage_time_scale is not None,\n"
        b"                     f\"STAGE_TIME_SCALE:{stage.name}\")\n"
        b"            point_dt_s = bundle.limits.point_dt_s * stage_time_scale\n"
        b"            for topic in stage.topics:\n"
        b"                names = CONTROLLER_JOINTS[topic]\n"
        b"                velocities = _pchip_velocities(\n"
        b"                    dense, names, point_dt_s)\n"
        b"                _validate_fast_cubic_profile(\n"
        b"                    dense, velocities, names, point_dt_s, stage.name)\n",
        "offline-profile")
    transformed = _replace_once(
        transformed,
        b"                  \"controller_commands=0 fast_point_dt_s=0.004\")\n",
        b"                  \"controller_commands=0 base_point_dt_s=0.004 \"\n"
        b"                  \"stage_retiming=v8\")\n",
        "offline-marker")
    return transformed


def _make_commands(
    run_id: str,
    socket_path: Path,
    immutable: Mapping[str, Any],
    snapshots: Mapping[str, Any],
    external: Mapping[str, Any],
    return_plan: Any,
    return_authorization_sha256: str,
) -> dict[str, Any]:
    commands = BASE_MAKE_COMMANDS(
        run_id, socket_path, immutable, snapshots, external,
        return_plan, return_authorization_sha256)
    timing = dict(commands["timing_contract"])
    timing.update({
        "trajectory_time_scales_unchanged": False,
        "trajectory_geometry_unchanged": True,
        "middle_trajectory_timing_unchanged": True,
        "right_stage_time_scales": dict(RIGHT_STAGE_TIME_SCALES),
        "analytic_velocity_acceleration_validation_required": True,
    })
    commands["timing_contract"] = timing
    return commands


def prepare_authority() -> Any:
    v7_manifest = BUILDER.read_regular(
        V7_AUTHORITY / "manifest.json", V7_MANIFEST_SHA256,
        allowed_modes=(0o400,))
    v7_commands = BUILDER.read_regular(
        V7_AUTHORITY / "commands_831.json", V7_COMMANDS_SHA256,
        allowed_modes=(0o400,))
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract.update({
        "trajectory_time_scales_unchanged": False,
        "trajectory_geometry_unchanged": True,
        "middle_trajectory_timing_unchanged": True,
        "right_stage_time_scales": dict(RIGHT_STAGE_TIME_SCALES),
        "right_profile_limits": {
            "revolute_velocity_rad_s": 2.0,
            "revolute_acceleration_rad_s2": 750.0,
            "prismatic_velocity_m_s": 0.10,
            "prismatic_acceleration_m_s2": 40.0,
        },
    })
    manifest["execution_contract"] = contract
    manifest["derived_from"] = {
        "path": BUILDER._relative(V7_AUTHORITY),
        "manifest_sha256": v7_manifest.sha256,
        "commands_sha256": v7_commands.sha256,
        "semantic_changes": [
            "retime only the eight right-hand stage classes",
            "retain every waypoint, message count, safety gate, and tolerance",
            "retain all loaded-cube and table-contact middle timing",
            "analytically validate every retimed cubic before publication",
        ],
    }
    output_files = dict(prepared.output_files)
    output_files["manifest.json"] = BUILDER.OutputBlob(
        BUILDER.pretty(manifest), 0o400, "generated")
    return BUILDER.PreparedAuthority(
        run_id=prepared.run_id,
        socket_path=prepared.socket_path,
        output_files=output_files,
        commands=prepared.commands,
        return_authorization=prepared.return_authorization,
        manifest=manifest,
    )


def self_test() -> Mapping[str, Any]:
    # The v6 inherited test intentionally asserts its historical timing
    # dictionary byte-for-byte.  Exercise that baseline with the v7 command
    # builder, then exercise the v8 extension below.
    current_make_commands = BUILDER._make_commands
    BUILDER._make_commands = BASE_MAKE_COMMANDS
    try:
        result = dict(BASE_SELF_TEST())
    finally:
        BUILDER._make_commands = current_make_commands
    prepared = prepare_authority()
    source = prepared.output_files["h1_right_handle_executor_828.py"].raw
    for token in (
        b"HARD_STAGE_TIME_SCALES = {",
        b'"contact_arc_q0_to_q90_hold": 0.99',
        b"point_dt_s = (bundle.limits.point_dt_s * time_scale *",
        b"stage_retiming=v8",
    ):
        if source.count(token) != 1:
            raise RuntimeError(f"V8_SELF_TEST_RIGHT_RETIME:{token!r}")
    if prepared.commands["timing_contract"][
            "right_stage_time_scales"] != RIGHT_STAGE_TIME_SCALES:
        raise RuntimeError("V8_SELF_TEST_COMMAND_TIMING")
    result.update({
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v8",
        "right_stage_time_scales": dict(RIGHT_STAGE_TIME_SCALES),
        "trajectory_geometry_unchanged": True,
        "middle_trajectory_timing_unchanged": True,
        "timed_maximum_seconds": 180.0,
    })
    return result


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v8")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r16"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v8"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v8"
BUILDER.IMMUTABLE_BYTE_TRANSFORMS = {
    "h1_right_handle_executor_828.py": _retime_right_executor,
}
BUILDER.RUN_ID_EXTRA_BINDING = V7_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING["right_stage_retiming_v8"] = {
    "stage_time_scales": dict(RIGHT_STAGE_TIME_SCALES),
    "trajectory_geometry_unchanged": True,
    "middle_trajectory_timing_unchanged": True,
    "profile_validation": "analytic cubic velocity and acceleration bounds",
}
BUILDER._make_commands = _make_commands
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
