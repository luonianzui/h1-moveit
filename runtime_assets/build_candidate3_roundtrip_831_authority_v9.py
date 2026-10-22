#!/usr/bin/python3
"""Build the contract-correct, non-overwriting 831 round-trip authority v9.

V9 keeps all v7 geometry, support gates, endpoint checks, and loaded-cube
timing.  Only unloaded right-hand approach/egress and right-gripper stages are
retimed.  Unlike the unused v8 draft, the controller authorization explicitly
binds the base point interval, minimum scheduled interval, and exact per-stage
scales.  Handle-contact arcs remain at the proven v7 timing.
"""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V8_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v8.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v9_base", V8_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V9_BASE_BUILDER_IMPORT")
V8 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V8
SPEC.loader.exec_module(V8)
BUILDER = V8.BUILDER


V8_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v8")
V8_MANIFEST_SHA256 = (
    "f44e905507297e44e1c9f52ed75f4a0a6b0e97ab5eb905ec0a09ae1526ad63ab")
V8_COMMANDS_SHA256 = (
    "8f5ee7e5ee5ef01a0513658ece6825cbcd12d815198a9d448f702794762e1a39")

BASE_POINT_DT_S = 0.004
MINIMUM_SCHEDULED_POINT_DT_S = 0.00328
RIGHT_STAGE_TIME_SCALES = {
    "prepare_left_park": 0.82,
    "ingress_open_to_q0": 0.82,
    "close_q0_right_hand": 0.87,
    "contact_arc_q0_to_q90_hold": 1.0,
    "contact_arc_q90_hold_to_q0": 1.0,
    "release_q0_right_hand": 0.87,
    "egress_open_from_q0": 0.82,
    "return_left_launch_seam": 0.82,
}
RETIMING_AUTHORIZATION = {
    "base_point_dt_s": BASE_POINT_DT_S,
    "minimum_scheduled_point_dt_s": MINIMUM_SCHEDULED_POINT_DT_S,
    "schema": "h1_right_handle_stage_retiming/831-v9",
    "stage_time_scales": dict(RIGHT_STAGE_TIME_SCALES),
    "trajectory_geometry_unchanged": True,
}

BASE_RIGHT_TRANSFORM = BUILDER.IMMUTABLE_BYTE_TRANSFORMS[
    "h1_right_handle_executor_828.py"]
BASE_MAKE_COMMANDS = BUILDER._make_commands
BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V8_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)


def _replace_exact(
    raw: bytes, old: bytes, new: bytes, count: int, label: str,
) -> bytes:
    if raw.count(old) != count:
        raise RuntimeError(f"V9_RIGHT_RETIME_SOURCE:{label}")
    return raw.replace(old, new)


def _retime_right_executor(raw: bytes) -> bytes:
    transformed = BASE_RIGHT_TRANSFORM(raw)
    transformed = _replace_exact(
        transformed,
        b'MANIFEST_SCHEMA = "h1_right_handle_controller_execution_authorization/v1"',
        b'MANIFEST_SCHEMA = "h1_right_handle_controller_execution_authorization/v2"',
        1, "authorization-schema")
    transformed = _replace_exact(
        transformed,
        b"HARD_MIN_POINT_DT_S = 0.004\n",
        b"HARD_BASE_POINT_DT_S = 0.004\n"
        b"HARD_MIN_SCHEDULED_POINT_DT_S = 0.00328\n",
        1, "point-dt-contract")
    transformed = _replace_exact(
        transformed,
        b'    "contact_arc_q0_to_q90_hold": 0.99,\n',
        b'    "contact_arc_q0_to_q90_hold": 1.0,\n',
        1, "forward-contact-scale")
    transformed = _replace_exact(
        transformed,
        b'    "contact_arc_q90_hold_to_q0": 0.99,\n',
        b'    "contact_arc_q90_hold_to_q0": 1.0,\n',
        1, "reverse-contact-scale")
    transformed = _replace_exact(
        transformed,
        b"    _require(abs(point_dt - HARD_MIN_POINT_DT_S) <= 1e-15,\n"
        b"             \"LIMIT_POINT_DT_HARD\")\n",
        b"    _require(abs(point_dt - HARD_BASE_POINT_DT_S) <= 1e-15,\n"
        b"             \"LIMIT_BASE_POINT_DT_HARD\")\n",
        1, "base-point-dt-validation")
    transformed = _replace_exact(
        transformed,
        b"        \"execution_topology\",\n"
        b"    ), \"MANIFEST\")\n"
        b"    _require(root.get(\"schema\") == MANIFEST_SCHEMA, \"MANIFEST_SCHEMA\")\n",
        b"        \"execution_topology\", \"retiming\",\n"
        b"    ), \"MANIFEST\")\n"
        b"    retiming = _exact_keys(root.get(\"retiming\"), (\n"
        b"        \"schema\", \"base_point_dt_s\",\n"
        b"        \"minimum_scheduled_point_dt_s\", \"stage_time_scales\",\n"
        b"        \"trajectory_geometry_unchanged\",\n"
        b"    ), \"MANIFEST_RETIMING\")\n"
        b"    _require(\n"
        b"        retiming.get(\"schema\") ==\n"
        b"        \"h1_right_handle_stage_retiming/831-v9\" and\n"
        b"        abs(_finite(retiming.get(\"base_point_dt_s\"),\n"
        b"                    \"RETIMING_BASE_POINT_DT\") -\n"
        b"            HARD_BASE_POINT_DT_S) <= 1e-15 and\n"
        b"        abs(_finite(retiming.get(\"minimum_scheduled_point_dt_s\"),\n"
        b"                    \"RETIMING_MINIMUM_POINT_DT\") -\n"
        b"            HARD_MIN_SCHEDULED_POINT_DT_S) <= 1e-15 and\n"
        b"        retiming.get(\"trajectory_geometry_unchanged\") is True,\n"
        b"        \"MANIFEST_RETIMING_CONTRACT\")\n"
        b"    scales = _exact_keys(\n"
        b"        retiming.get(\"stage_time_scales\"),\n"
        b"        tuple(HARD_STAGE_TIME_SCALES), \"MANIFEST_STAGE_TIME_SCALES\")\n"
        b"    for stage_name, expected_scale in HARD_STAGE_TIME_SCALES.items():\n"
        b"        _require(\n"
        b"            abs(_finite(scales.get(stage_name),\n"
        b"                        f\"STAGE_TIME_SCALE:{stage_name}\") -\n"
        b"                expected_scale) <= 1e-15,\n"
        b"            f\"MANIFEST_STAGE_TIME_SCALE:{stage_name}\")\n"
        b"    _require(root.get(\"schema\") == MANIFEST_SCHEMA, \"MANIFEST_SCHEMA\")\n",
        1, "authorization-retiming-validation")
    mapping_end = (
        b'    "return_left_launch_seam": 0.82,\n'
        b'}\n')
    transformed = _replace_exact(
        transformed,
        mapping_end,
        mapping_end +
        b"\n\n"
        b"def _scheduled_point_dt(\n"
        b"    base_point_dt_s: float, stage_name: str, outer_scale: float = 1.0,\n"
        b") -> float:\n"
        b"    stage_scale = HARD_STAGE_TIME_SCALES.get(stage_name)\n"
        b"    _require(stage_scale is not None, f\"STAGE_TIME_SCALE:{stage_name}\")\n"
        b"    _require(math.isfinite(outer_scale) and\n"
        b"             abs(outer_scale - 1.0) <= 1e-15,\n"
        b"             \"STAGE_OUTER_TIME_SCALE\")\n"
        b"    point_dt_s = base_point_dt_s * stage_scale\n"
        b"    _require(\n"
        b"        math.isfinite(point_dt_s) and\n"
        b"        HARD_MIN_SCHEDULED_POINT_DT_S - 1e-15 <= point_dt_s <=\n"
        b"        HARD_BASE_POINT_DT_S + 1e-15,\n"
        b"        f\"STAGE_SCHEDULED_POINT_DT:{stage_name}\")\n"
        b"    return point_dt_s\n",
        1, "scheduled-point-dt-helper")
    transformed = _replace_exact(
        transformed,
        b"            stage_time_scale = HARD_STAGE_TIME_SCALES.get(stage.name)\n"
        b"            _require(stage_time_scale is not None,\n"
        b"                     f\"STAGE_TIME_SCALE:{stage.name}\")\n"
        b"            point_dt_s = (bundle.limits.point_dt_s * time_scale *\n"
        b"                          stage_time_scale)\n",
        b"            point_dt_s = _scheduled_point_dt(\n"
        b"                bundle.limits.point_dt_s, stage.name, time_scale)\n",
        1, "live-scheduled-point-dt")
    transformed = _replace_exact(
        transformed,
        b"            stage_time_scale = HARD_STAGE_TIME_SCALES.get(stage.name)\n"
        b"            _require(stage_time_scale is not None,\n"
        b"                     f\"STAGE_TIME_SCALE:{stage.name}\")\n"
        b"            point_dt_s = bundle.limits.point_dt_s * stage_time_scale\n",
        b"            point_dt_s = _scheduled_point_dt(\n"
        b"                bundle.limits.point_dt_s, stage.name)\n",
        1, "offline-scheduled-point-dt")
    transformed = _replace_exact(
        transformed, b"stage_retiming=v8", b"stage_retiming=v9", 1,
        "offline-marker")
    return transformed


def _retime_controller_authorization(raw: bytes) -> bytes:
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"V9_AUTHORIZATION_JSON:{error}") from error
    if (
        type(value) is not dict or
        value.get("schema") !=
            "h1_right_handle_controller_execution_authorization/v1" or
        "retiming" in value or
        value.get("limits", {}).get("point_dt_s") != BASE_POINT_DT_S
    ):
        raise RuntimeError("V9_AUTHORIZATION_INPUT")
    value["schema"] = "h1_right_handle_controller_execution_authorization/v2"
    value["retiming"] = copy.deepcopy(RETIMING_AUTHORIZATION)
    return BUILDER.pretty(value)


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
        "right_base_point_dt_s": BASE_POINT_DT_S,
        "right_minimum_scheduled_point_dt_s":
            MINIMUM_SCHEDULED_POINT_DT_S,
        "right_stage_time_scales": dict(RIGHT_STAGE_TIME_SCALES),
        "analytic_velocity_acceleration_validation_required": True,
    })
    commands["timing_contract"] = timing
    return commands


def prepare_authority() -> Any:
    v8_manifest = BUILDER.read_regular(
        V8_AUTHORITY / "manifest.json", V8_MANIFEST_SHA256,
        allowed_modes=(0o400,))
    v8_commands = BUILDER.read_regular(
        V8_AUTHORITY / "commands_831.json", V8_COMMANDS_SHA256,
        allowed_modes=(0o400,))
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract.update({
        "trajectory_time_scales_unchanged": False,
        "trajectory_geometry_unchanged": True,
        "middle_trajectory_timing_unchanged": True,
        "right_base_point_dt_s": BASE_POINT_DT_S,
        "right_minimum_scheduled_point_dt_s":
            MINIMUM_SCHEDULED_POINT_DT_S,
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
        "path": BUILDER._relative(V8_AUTHORITY),
        "manifest_sha256": v8_manifest.sha256,
        "commands_sha256": v8_commands.sha256,
        "semantic_changes": [
            "formally authorize base and minimum scheduled point intervals",
            "bind all eight exact stage scales in controller authorization",
            "restore both handle-contact arcs to v7 timing",
            "retain every waypoint, message count, safety gate, and tolerance",
            "retain all loaded-cube and table-contact middle timing",
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
    prepared = prepare_authority()
    source = prepared.output_files["h1_right_handle_executor_828.py"].raw
    authorization = json.loads(prepared.output_files[
        "controller_execution_authorization.json"].raw)
    if authorization.get("retiming") != RETIMING_AUTHORIZATION:
        raise RuntimeError("V9_SELF_TEST_AUTHORIZATION_RETIMING")
    if authorization.get("schema") != (
            "h1_right_handle_controller_execution_authorization/v2"):
        raise RuntimeError("V9_SELF_TEST_AUTHORIZATION_SCHEMA")
    for token, count in (
        (b"HARD_BASE_POINT_DT_S = 0.004", 1),
        (b"HARD_MIN_SCHEDULED_POINT_DT_S = 0.00328", 1),
        (b"def _scheduled_point_dt(", 1),
        (b"point_dt_s = _scheduled_point_dt(", 2),
        (b'"contact_arc_q0_to_q90_hold": 1.0', 1),
        (b'"contact_arc_q90_hold_to_q0": 1.0', 1),
        (b"stage_retiming=v9", 1),
    ):
        if source.count(token) != count:
            raise RuntimeError(f"V9_SELF_TEST_RIGHT_SOURCE:{token!r}")
    if b"HARD_MIN_POINT_DT_S" in source or b"stage_retiming=v8" in source:
        raise RuntimeError("V9_SELF_TEST_STALE_CONTRACT")
    # Independent closed-form check against the v7 per-stage analytic maxima.
    base_ratios = {
        "prepare_left_park": (0.289533027, 0.579066054),
        "ingress_open_to_q0": (0.290857706, 0.581715413),
        "close_q0_right_hand": (1.0 / 3.0, 0.625),
        "contact_arc_q0_to_q90_hold": (0.375, 0.9375),
        "contact_arc_q90_hold_to_q0": (0.375, 0.9375),
        "release_q0_right_hand": (1.0 / 3.0, 0.625),
        "egress_open_from_q0": (0.290857706, 0.581715413),
        "return_left_launch_seam": (0.289533027, 0.579066054),
    }
    peak_velocity_ratio = max(
        base_ratios[name][0] / scale
        for name, scale in RIGHT_STAGE_TIME_SCALES.items())
    peak_acceleration_ratio = max(
        base_ratios[name][1] / (scale * scale)
        for name, scale in RIGHT_STAGE_TIME_SCALES.items())
    if peak_velocity_ratio >= 1.0 or peak_acceleration_ratio >= 1.0:
        raise RuntimeError("V9_SELF_TEST_PROFILE_LIMIT")
    points = {
        "prepare_left_park": 39,
        "ingress_open_to_q0": 773,
        "close_q0_right_hand": 344,
        "contact_arc_q0_to_q90_hold": 2352,
        "contact_arc_q90_hold_to_q0": 2352,
        "release_q0_right_hand": 344,
        "egress_open_from_q0": 773,
        "return_left_launch_seam": 39,
    }
    saved_per_cycle_s = sum(
        (count - 1) * BASE_POINT_DT_S *
        (1.0 - RIGHT_STAGE_TIME_SCALES[name])
        for name, count in points.items())
    if abs(saved_per_cycle_s - 1.52312) > 1e-12:
        raise RuntimeError("V9_SELF_TEST_THEORETICAL_SAVING")
    if prepared.commands["timing_contract"][
            "right_stage_time_scales"] != RIGHT_STAGE_TIME_SCALES:
        raise RuntimeError("V9_SELF_TEST_COMMAND_TIMING")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v9",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "process_roles": list(BUILDER.PROCESS_ROLES),
        "right_stage_time_scales": dict(RIGHT_STAGE_TIME_SCALES),
        "right_nominal_two_cycles_saved_seconds": 2.0 * saved_per_cycle_s,
        "peak_velocity_limit_ratio": peak_velocity_ratio,
        "peak_acceleration_limit_ratio": peak_acceleration_ratio,
        "trajectory_geometry_unchanged": True,
        "middle_trajectory_timing_unchanged": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v9")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r17"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v9"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v9"
BUILDER.IMMUTABLE_BYTE_TRANSFORMS = {
    "controller_execution_authorization.json":
        _retime_controller_authorization,
    "h1_right_handle_executor_828.py": _retime_right_executor,
}
BUILDER.RUN_ID_EXTRA_BINDING = V8_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("right_stage_retiming_v8", None)
BUILDER.RUN_ID_EXTRA_BINDING["right_stage_retiming_v9"] = {
    "authorization": copy.deepcopy(RETIMING_AUTHORIZATION),
    "middle_trajectory_timing_unchanged": True,
    "profile_validation": "analytic cubic velocity and acceleration bounds",
}
BUILDER._make_commands = _make_commands
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
