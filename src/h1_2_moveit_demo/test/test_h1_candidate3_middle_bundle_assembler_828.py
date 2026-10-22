"""ROS-free tests for the fail-closed 828 middle bundle assembler."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
from types import ModuleType, SimpleNamespace

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = (WORKSPACE / "src/h1_2_moveit_demo/scripts/"
          "h1_candidate3_middle_bundle_assembler_828.py")

# Load verified source bytes without creating a pyc next to a sealed script.
MODULE = ModuleType("h1_candidate3_middle_bundle_assembler_828_test")
MODULE.__file__ = str(SOURCE)
sys.modules[MODULE.__name__] = MODULE
exec(compile(SOURCE.read_bytes(), str(SOURCE), "exec"), MODULE.__dict__)


def _canonical(value):
    return (json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def _sealed(path, value, mode=0o400):
    raw = value if type(value) is bytes else _canonical(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(raw)
    path.chmod(mode)
    return hashlib.sha256(raw).hexdigest()


def _copy_sealed(source, target, mode):
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    target.chmod(mode)
    return hashlib.sha256(target.read_bytes()).hexdigest()


def _pose(x, y, z):
    return {
        "position": [x, y, z],
        "orientation": [
            0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)],
    }


def _fixture(tmp_path, monkeypatch, *, publish=False):
    workspace = (tmp_path / "ws").resolve()
    runtime = workspace / "runtime"
    output = workspace / "output"
    runtime.mkdir(parents=True)
    output.mkdir()

    baseline = tmp_path / "826.3.tar.zst"
    baseline_sha = _sealed(baseline, b"synthetic-826.3-baseline", 0o444)
    monkeypatch.setattr(MODULE, "BASELINE_BACKUP_PATH", baseline)
    monkeypatch.setattr(MODULE, "BASELINE_BACKUP_SHA256", baseline_sha)

    source_asset_directory = (
        WORKSPACE / "runtime_assets/candidate3_middle_e0_828")
    asset_bindings = {}
    for asset_id in MODULE.ASSET_IDS:
        filename = f"{asset_id}.trajectory.json"
        target = runtime / filename
        file_sha = _copy_sealed(
            source_asset_directory / filename, target, 0o400)
        wrapper = json.loads(target.read_text(encoding="utf-8"))
        asset_bindings[asset_id] = {
            "path": filename,
            "file_sha256": file_sha,
            "payload_sha256": wrapper["sha256"],
            "schema": MODULE.ASSET_SCHEMA,
        }
    manifest_value = {
        "schema": MODULE.RAW_BINDINGS_SCHEMA,
        "controller_commands_issued": 0,
        "offline_only": True,
        "assets": asset_bindings,
        "phase_message_ranges": {
            name: list(message_range)
            for name, message_range in zip(
                MODULE.PHASE_NAMES, MODULE.PHASE_RANGES)
        },
    }
    manifest = runtime / "raw_asset_bindings.json"
    manifest_sha = _sealed(manifest, manifest_value)

    scene = runtime / "scene.cdr"
    scene_sha = _sealed(scene, b"synthetic-scene")
    mesh = runtime / "robot_model/mesh.stl"
    mesh_sha = _sealed(mesh, b"solid mesh\nendsolid mesh\n")
    urdf = runtime / "robot_model/robot.urdf"
    urdf_sha = _sealed(urdf, b"<robot name='test'/>\n")
    srdf = runtime / "robot_model/robot.srdf"
    srdf_sha = _sealed(srdf, b"<robot name='test'/>\n")
    inventory_value = {
        "schema": "h1_candidate3_middle_robot_model_snapshot/828-e0-v1",
        "source_urdf_sha256": "a" * 64,
        "source_srdf_sha256": srdf_sha,
        "derived_urdf_path": urdf.relative_to(workspace).as_posix(),
        "derived_urdf_sha256": urdf_sha,
        "srdf_path": srdf.relative_to(workspace).as_posix(),
        "srdf_sha256": srdf_sha,
        "mesh_files": [{
            "source_uri": "package://test/mesh.stl",
            "snapshot_path": mesh.relative_to(workspace).as_posix(),
            "resolved_file_uri": "file://" + str(mesh),
            "file_sha256": mesh_sha,
            "size_bytes": mesh.stat().st_size,
            "roles": ["collision"],
        }],
    }
    inventory = runtime / "robot_model/inventory.json"
    inventory_sha = _sealed(inventory, inventory_value)

    reference = runtime / "cube_reference_unmodified.trajectory.json"
    reference_sha = _copy_sealed(
        source_asset_directory / reference.name, reference, 0o400)
    reference_wrapper = json.loads(reference.read_text(encoding="utf-8"))
    reference_payload_sha = reference_wrapper["sha256"]

    plugin = workspace / MODULE.PLUGIN_RELATIVE_PATH
    plugin_sha = _sealed(plugin, b"synthetic-plugin", 0o755)
    snapshot = runtime / "snapshot.py"
    snapshot_sha = _sealed(
        snapshot, b"#!/usr/bin/python3\nprint('snapshot')\n", 0o500)

    authority_sources = {
        "contract": (
            WORKSPACE / "src/h1_2_vision/h1_2_vision/"
            "candidate3_middle_execution_contract_828.py"),
        "admission": (
            WORKSPACE / "src/h1_2_vision/h1_2_vision/"
            "candidate3_middle_admission.py"),
        "right_executor": (
            WORKSPACE / "src/h1_2_moveit_demo/scripts/"
            "h1_right_handle_executor_828.py"),
        "middle_executor": (
            WORKSPACE / "src/h1_2_moveit_demo/scripts/"
            "h1_candidate3_middle_executor_828.py"),
        "flow": (
            WORKSPACE / "src/h1_2_moveit_demo/scripts/"
            "h1_candidate3_flow_828.py"),
        "support": (
            WORKSPACE / "src/h1_2_moveit_demo/scripts/"
            "h1_support_orchestrator_828.py"),
    }
    authorities = {}
    for identifier, source in authority_sources.items():
        target = workspace / "authorities" / source.name
        digest = _copy_sealed(source, target, 0o500)
        authorities[identifier] = {
            "path": target.relative_to(workspace).as_posix(),
            "sha256": digest,
        }

    auditor_source = runtime / "auditor.cpp"
    auditor_source_sha = _sealed(
        auditor_source, b"// synthetic exact auditor source\n")
    monkeypatch.setattr(
        MODULE, "RAW_AUDITOR_SOURCE_SHA256", auditor_source_sha)
    auditor_binary = runtime / "auditor.bin"
    auditor_binary_sha = _sealed(
        auditor_binary, b"synthetic exact auditor binary\n", 0o500)
    provenance_value = {
        "schema": MODULE.PROVENANCE_SCHEMA,
        "baseline_backup": {
            "path": str(baseline),
            "sha256": baseline_sha,
        },
        "scene": {
            "path": scene.relative_to(workspace).as_posix(),
            "scene_name": "synthetic_candidate3_scene",
            "planning_frame": "world",
            "required_object_ids": [
                "table", "incubator", "incubator_handle_exact",
                "green_cube_dynamic_exact", "green_cube_10mm_guard",
            ],
        },
        "robot_model": {
            "inventory_path": inventory.relative_to(workspace).as_posix(),
            "inventory_sha256": inventory_sha,
            "urdf_path": urdf.relative_to(workspace).as_posix(),
            "urdf_sha256": urdf_sha,
            "srdf_path": srdf.relative_to(workspace).as_posix(),
            "srdf_sha256": srdf_sha,
            "mesh_count": 1,
        },
        "cube_reference": {
            "path": reference.relative_to(workspace).as_posix(),
            "file_sha256": reference_sha,
            "payload_sha256": reference_payload_sha,
        },
        "auditor": {
            "source_path": auditor_source.relative_to(workspace).as_posix(),
            "source_sha256": auditor_source_sha,
            "binary_path": auditor_binary.relative_to(workspace).as_posix(),
            "binary_sha256": auditor_binary_sha,
        },
        "authorities": authorities,
        "support": {
            "plugin_path": MODULE.PLUGIN_RELATIVE_PATH,
            "plugin_sha256": plugin_sha,
            "ros_snapshot_path": snapshot.relative_to(workspace).as_posix(),
            "ros_snapshot_sha256": snapshot_sha,
        },
        "cube_poses": {
            "contact": _pose(0.334, 0.377, 0.830),
            "release": _pose(0.611, -0.030, 0.995),
        },
    }
    clearances = {
        "unauthorized_environment": 0.010001,
        "table": 0.010001,
        "door": 0.010001,
        "handle": 0.0,
        "cube": 0.0,
        "self": 0.010001,
        "right_handle_nonpad": 0.0006,
        "left_handle": 0.05,
    }
    per_phase = []
    for index, (name, asset_id, message_range) in enumerate(zip(
            MODULE.PHASE_NAMES, MODULE.PHASE_ASSET_IDS,
            MODULE.PHASE_RANGES)):
        phase_cube = 0.0 if index in (4, 5, 6, 7) else 0.010001
        per_phase.append({
            "phase": name,
            "asset_id": asset_id,
            "message_range": list(message_range),
            "samples": 1,
            "subdivisions": 1,
            "right_handle_authorized_samples": 1,
            "minimum_is_lower_bound": True,
            "minimum_environment_clearance_m": 0.010001,
            "minimum_table_clearance_m": 0.010001,
            "minimum_door_clearance_m": 0.010001,
            "minimum_handle_clearance_m": 0.0,
            "minimum_cube_clearance_m": phase_cube,
            "minimum_self_clearance_m": 0.010001,
            "minimum_left_handle_clearance_m": 0.05,
            "minimum_torso_rad": -0.13962634015954636,
            "maximum_torso_rad": 0.0,
            "minimum_right_handle_nonpad_clearance_m": 0.0006,
        })
    right_pairs = [[
        "incubator_handle_exact", "right_dex1_finger_1_pad_link"]]
    right_allowed_pairs = [
        ["incubator_handle_exact", "right_dex1_finger_1_pad_link"],
        ["incubator_handle_exact", "right_dex1_finger_2_pad_link"],
    ]
    cube_pairs = [[
        "green_cube_dynamic_exact", "left_dex1_finger_1_pad_link"]]
    guard_pairs = [[
        "green_cube_10mm_guard", "left_dex1_finger_1_pad_link"]]
    retreat_metric = {
        "phase_name": "cube_retreat",
        "message_index": 6,
        "start_point_index": 0,
        "start_subdivision": 0,
        "window_sample_count": 1,
        "gate_point_index": 1,
        "gate_subdivision": 1,
        "gate_ordinal": 1,
        "last_below_gate_ordinal": 0,
        "gate_reached": True,
        "minimum_exact_distance_m": 0.0,
        "maximum_exact_distance_m": 0.02,
        "final_exact_distance_m": 0.02,
        "maximum_exact_penetration_m": 0.0,
        "minimum_time_step_ns": 1,
        "minimum_path_parameter_step": 0.001,
        "minimum_tcp_separation_projection_step_m": -0.001,
        "final_tcp_projection_m": 0.02,
        "post_gate_minimum_exact_distance_m": 0.010001,
        "post_gate_minimum_guard_physical_lower_bound_m": 0.010001,
        "penetration_geometry": "green_cube_dynamic_exact_physical_only",
        "guard_geometry": "green_cube_10mm_guard_conservative_nonphysical",
        "observed_pair_semantics": (
            "nearest_signed_distance_observation_not_collision_event"),
        "observed_exact_pairs": cube_pairs,
        "observed_guard_pairs": guard_pairs,
        "observed_exact_links": ["left_dex1_finger_1_pad_link"],
        "observed_guard_links": ["left_dex1_finger_1_pad_link"],
        "gate_never_regressed": True,
    }
    raw_value = {
        "schema": MODULE.RAW_SCHEMA,
        "auditor_version": MODULE.RAW_AUDITOR_VERSION,
        "audit_method": "FCL_UNPADDED_DYNAMIC_EXACT_STL_DENSE_INTERPOLATION",
        "raw_asset_manifest_sha256": manifest_sha,
        "raw_scene_hash": "1" * 64,
        "physical_scene_hash": scene_sha,
        "raw_scene_cdr_sha256": scene_sha,
        "entry_projection": (
            "prefix_certified_endpoint_projection_to_canonical_e0_q90_held"),
        "robot_model_snapshot": {
            "inventory_sha256": inventory_sha,
            "urdf_sha256": urdf_sha,
            "srdf_sha256": srdf_sha,
            "mesh_count": 1,
        },
        "controller_commands_issued": 0,
        "offline_only": True,
        "sample_count": 11,
        "subdivision_count": 11,
        "collision_free": True,
        "cube_bump": {
            "reference_file_sha256": reference_sha,
            "reference_payload_sha256": reference_payload_sha,
            "only_modified_joint": "left_shoulder_roll_joint",
            "comparison": "same_start_state_dense_interpolation",
            "maximum_tcp_translation_m": 0.001,
            "maximum_tcp_rotation_rad": 0.001,
            "translation_limit_m": 0.006,
            "rotation_limit_rad": 0.013962634015954637,
        },
        "final_target": {
            "translation_error_m": 0.001,
            "rotation_error_rad": 0.001,
            "translation_limit_m": 0.003,
            "rotation_limit_rad": 0.03490658503988659,
        },
        "minimum_clearances_m": clearances,
        "cube_guard_exact_refinement_samples": 0,
        "minimum_cube_exact_noncontact_clearance_m": None,
        "minimum_environment_clearance_is_lower_bound": True,
        "park_roll_rad": 0.316,
        "environment_clearance_derivation": {
            "exact_world_objects": "direct_unpadded_FCL_distance",
            "environment_table_10mm_margin": "0.010_plus_guard_distance",
            "green_cube_10mm_guard": "0.010_plus_guard_distance",
            "self_pairs_excluded": True,
        },
        "park_static_clearance_required_m": 0.0105,
        "initial_park_exact_door_clearance": {
            "distance_m": 0.0105,
            "link": "right_dex1_finger_2_link",
        },
        "final_park_exact_door_clearance": {
            "distance_m": 0.0105,
            "link": "right_dex1_finger_2_link",
        },
        "right_handle_nonpad_clearance_required_m": 0.0006,
        "left_handle_clearance_required_m": 0.05,
        "dynamic_handle_exact": {
            "id": "incubator_handle_exact",
            "geometry_type": "mesh",
            "mesh_count": 1,
            "primitive_count": 0,
            "geometry_sha256": mesh_sha,
            "retained_every_phase": True,
            "audited_phases": list(MODULE.PHASE_NAMES),
            "angle_rad": math.pi / 2.0,
            "door_angle_min_rad": 0.0,
            "door_angle_max_rad": math.pi / 3.0,
        },
        "right_handle_allowed_contact_pairs": right_allowed_pairs,
        "right_door_allowed_contact_pairs": [],
        "left_cube_allowed_contact_links": [
            "left_dex1_finger_1_slider_link",
            "left_dex1_finger_1_link",
            "left_dex1_finger_1_pad_link",
            "left_dex1_finger_2_slider_link",
            "left_dex1_finger_2_link",
            "left_dex1_finger_2_pad_link",
        ],
        "left_cube_object_ids": [
            "green_cube_dynamic_exact", "green_cube_10mm_guard"],
        "contact_window_measurements": {
            "right_handle_hold": {
                "authorized_sample_count": 11,
                "maximum_observed_penetration_m": 0.0,
                "observed_pairs": right_pairs,
            },
            "cube_grasp_carry_release": {
                "authorized_sample_count": 3,
                "maximum_observed_penetration_m": 0.0,
                "observed_pairs": cube_pairs,
                "guard_nearest_pairs": guard_pairs,
            },
            "cube_release_retreat": {
                "authorized_sample_count": 1,
                "maximum_observed_penetration_m": 0.0,
                "penetration_geometry": (
                    "green_cube_dynamic_exact_physical_only"),
                "guard_observation_geometry": (
                    "green_cube_10mm_guard_conservative_nonphysical"),
                "observed_pair_semantics": (
                    "nearest_signed_distance_observation_not_collision_event"),
                "observed_pairs": cube_pairs,
                "guard_nearest_pairs": guard_pairs,
            },
        },
        "torso_joint": {
            "commanded": True,
            "minimum_rad": -0.13962634015954636,
            "maximum_rad": 0.0,
            "maximum_absolute_rad": 0.13962634015954636,
        },
        "left_handle": {
            "allowed_contact_pairs": [],
            "contact_count": 0,
            "minimum_clearance_m": 0.05,
        },
        "mask_projection": {
            "raw_scene_hash": "1" * 64,
            "physical_scene_hash": scene_sha,
            "removed_visual_mask_ids": [],
            "unknown_visual_mask_ids": [],
            "classification": "RAW_SCENE_ALREADY_PHYSICAL",
            "handle_exact_retained": True,
        },
        "mask_conflict_recheck": {
            "classification": "RAW_SCENE_ALREADY_PHYSICAL",
            "observed_visual_distance_m": None,
        },
        "cube_release_retreat": retreat_metric,
        "per_phase_metrics": per_phase,
        "raw_audit_complete": True,
        "geometry_valid": True,
        "valid": False,
        "status": MODULE.RAW_STATUS,
    }
    raw = runtime / "formal_run/raw_combined_exact.json"
    raw_sha = _sealed(raw, raw_value)
    monkeypatch.setattr(MODULE, "EXPECTED_RAW_FILE_SHA256", raw_sha)

    run_directory = runtime / "formal_run"
    run_plan = run_directory / "auditor_run_plan_828.json"
    run_result = run_directory / "auditor_run_result_828.json"
    proc_cmdline = run_directory / "proc_cmdline.bin"
    stdout = run_directory / "raw_combined_exact.stdout"
    stderr = run_directory / "raw_combined_exact.stderr"
    time_file = run_directory / "raw_combined_exact.time"
    run_raw_relative = raw.relative_to(workspace).as_posix()
    run_outputs = {
        "raw": run_raw_relative,
        "stderr": stderr.relative_to(workspace).as_posix(),
        "stdout": stdout.relative_to(workspace).as_posix(),
        "time": time_file.relative_to(workspace).as_posix(),
    }
    run_inputs = {
        "asset_manifest": {
            "path": manifest.relative_to(workspace).as_posix(),
            "sha256": manifest_sha,
        },
        "cube_reference": {
            "path": reference.relative_to(workspace).as_posix(),
            "sha256": reference_sha,
        },
        "handle_stl": {
            "path": mesh.relative_to(workspace).as_posix(),
            "sha256": mesh_sha,
        },
        "robot_model_inventory": {
            "path": inventory.relative_to(workspace).as_posix(),
            "sha256": inventory_sha,
        },
        "scene": {
            "path": scene.relative_to(workspace).as_posix(),
            "sha256": scene_sha,
        },
        "srdf": {
            "path": srdf.relative_to(workspace).as_posix(),
            "sha256": srdf_sha,
        },
        "urdf": {
            "path": urdf.relative_to(workspace).as_posix(),
            "sha256": urdf_sha,
        },
    }
    for run_id, asset_id in {
        "cube": "cube", "door_close": "door_close",
        "door_open": "door_open",
        "seam_to_cube": "door60_seam_to_cube",
        "seam_to_hinge": "door60_seam_to_hinge",
    }.items():
        run_inputs[run_id] = {
            "path": (runtime / asset_bindings[asset_id]["path"])
            .relative_to(workspace).as_posix(),
            "sha256": asset_bindings[asset_id]["file_sha256"],
        }
    argument_values = (
        run_inputs["scene"]["path"], run_outputs["raw"],
        run_inputs["asset_manifest"]["path"], "0.316",
        run_inputs["urdf"]["path"], run_inputs["srdf"]["path"],
        run_inputs["door_open"]["path"],
        run_inputs["seam_to_cube"]["path"], run_inputs["cube"]["path"],
        run_inputs["cube_reference"]["path"],
        run_inputs["seam_to_hinge"]["path"],
        run_inputs["door_close"]["path"], run_inputs["handle_stl"]["path"],
        run_inputs["robot_model_inventory"]["path"],
    )
    arguments = [
        item for pair in zip(MODULE.AUDITOR_RUN_ARGUMENT_FLAGS,
                             argument_values) for item in pair]
    run_plan_value = {
        "schema": MODULE.AUDITOR_RUN_PLAN_SCHEMA,
        "working_directory": str(workspace),
        "offline_only": True,
        "controller_commands_issued": 0,
        "auditor": dict(provenance_value["auditor"]),
        "wrapper": {
            "path": str(MODULE.TIME_WRAPPER_PATH),
            "sha256": MODULE.TIME_WRAPPER_SHA256,
            "time_format": MODULE.TIME_WRAPPER_FORMAT,
        },
        "inputs": run_inputs,
        "outputs": run_outputs,
        "arguments": arguments,
        "expected_raw": {
            "schema": MODULE.RAW_SCHEMA,
            "auditor_version": MODULE.RAW_AUDITOR_VERSION,
            "file_sha256": raw_sha,
            "status": MODULE.RAW_STATUS,
        },
    }
    run_plan_sha = _sealed(run_plan, run_plan_value)

    argv = [provenance_value["auditor"]["binary_path"], *arguments]
    proc_raw = b"\0".join(item.encode("utf-8") for item in argv) + b"\0"
    proc_sha = _sealed(proc_cmdline, proc_raw)
    normalized_sha = hashlib.sha256(
        ("\n".join(argv) + "\n").encode("utf-8")).hexdigest()
    stdout_sha = _sealed(
        stdout,
        ("CANDIDATE3_MIDDLE_EXACT_AUDIT=PASS samples=11 "
         "minimum_environment_clearance_m=0.010001 "
         "minimum_left_handle_clearance_m=0.05\n").encode("utf-8"))
    stderr_sha = _sealed(stderr, b"[WARN] synthetic loader warning\n")
    time_sha = _sealed(
        time_file,
        b"elapsed_seconds=2.5\nuser_seconds=2.0\n"
        b"system_seconds=0.25\nmax_rss_kb=1024\n")
    run_result_value = {
        "schema": MODULE.AUDITOR_RUN_RESULT_SCHEMA,
        "offline_only": True,
        "controller_commands_issued": 0,
        "plan": {
            "path": run_plan.relative_to(workspace).as_posix(),
            "sha256": run_plan_sha,
        },
        "execution": {
            "argc": len(argv),
            "executable_dev": auditor_binary.stat().st_dev,
            "executable_file_sha256": auditor_binary_sha,
            "executable_inode": auditor_binary.stat().st_ino,
            "executable_mode": "0500",
            "executable_nlink": 1,
            "exit_code": 0,
            "proc_cmdline_file_sha256": proc_sha,
            "proc_cmdline_normalized_sha256": normalized_sha,
            "proc_cmdline_path": proc_cmdline.relative_to(workspace).as_posix(),
        },
        "outputs": {
            "raw": {
                "path": run_outputs["raw"], "sha256": raw_sha,
                "size_bytes": raw.stat().st_size,
            },
            "stderr": {
                "path": run_outputs["stderr"], "sha256": stderr_sha,
                "size_bytes": stderr.stat().st_size,
            },
            "stdout": {
                "path": run_outputs["stdout"], "sha256": stdout_sha,
                "size_bytes": stdout.stat().st_size,
            },
            "time": {
                "path": run_outputs["time"], "sha256": time_sha,
                "size_bytes": time_file.stat().st_size,
            },
        },
        "raw_validation": {
            "schema": MODULE.RAW_SCHEMA,
            "auditor_version": MODULE.RAW_AUDITOR_VERSION,
            "file_sha256": raw_sha,
            "byte_equal_to_original_reviewed_raw": True,
            "strict_json_finite": True,
            "sample_count": 11,
            "subdivision_count": 11,
            "collision_free": True,
            "geometry_valid": True,
            "valid": False,
            "status": MODULE.RAW_STATUS,
        },
        "timing": {
            "elapsed_seconds": 2.5,
            "user_seconds": 2.0,
            "system_seconds": 0.25,
            "maximum_resident_kb": 1024,
        },
        "valid": True,
        "status": "PASS",
    }
    run_result_sha = _sealed(run_result, run_result_value)
    provenance_value["auditor_run"] = {
        "plan_path": run_plan.relative_to(workspace).as_posix(),
        "plan_sha256": run_plan_sha,
        "result_path": run_result.relative_to(workspace).as_posix(),
        "result_sha256": run_result_sha,
    }
    provenance = runtime / "provenance.json"
    provenance_sha = _sealed(provenance, provenance_value)

    epoch = 1_700_000_000_000_000_000
    for path, offset in (
        (run_plan, 0), (stderr, 1), (proc_cmdline, 2), (raw, 3),
        (stdout, 3), (time_file, 3), (run_result, 4),
    ):
        os.utime(path, ns=(epoch + offset, epoch + offset))
    args = SimpleNamespace(
        workspace=workspace,
        raw=raw,
        raw_sha256=raw_sha,
        asset_bindings=manifest,
        asset_bindings_sha256=manifest_sha,
        provenance=provenance,
        provenance_sha256=provenance_sha,
        output_directory=output,
        publish=publish,
        quiet=True,
    )
    return args, raw_value


def _rewrite_raw(args, raw):
    args.raw.chmod(0o600)
    args.raw_sha256 = _sealed(args.raw, raw)
    return args.raw_sha256


def _rewrite_run_document(args, kind, value, *, canonical=True):
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    relative = provenance["auditor_run"][f"{kind}_path"]
    path = args.workspace / relative
    path.chmod(0o600)
    raw = _canonical(value) if canonical else json.dumps(
        value, indent=2, sort_keys=True).encode("utf-8") + b"\n"
    digest = _sealed(path, raw)
    provenance["auditor_run"][f"{kind}_sha256"] = digest
    args.provenance.chmod(0o600)
    args.provenance_sha256 = _sealed(args.provenance, provenance)
    return path, digest


def _rewrite_run_output(args, identifier, raw):
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    result_path = args.workspace / provenance["auditor_run"]["result_path"]
    result = json.loads(result_path.read_text(encoding="utf-8"))
    output_path = args.workspace / result["outputs"][identifier]["path"]
    output_path.chmod(0o600)
    digest = _sealed(output_path, raw)
    result["outputs"][identifier]["sha256"] = digest
    result["outputs"][identifier]["size_bytes"] = len(raw)
    _rewrite_run_document(args, "result", result)
    return output_path


def test_full_synthetic_raw_self_proves_and_publishes_ordered_dag(
        tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch, publish=True)
    bundle = MODULE.assemble(args)
    assert tuple(bundle.documents) == (
        "plan", "result", "certificate", "support_manifest")
    assert bundle.documents["result"]["plan"] == bundle.documents["plan"]
    assert (bundle.documents["result"]["plan_file_sha256"] ==
            bundle.sha256["plan"])
    assert (bundle.documents["certificate"]["exact_result_sha256"] ==
            bundle.sha256["result"])
    assert (bundle.documents["support_manifest"]["bindings"]
            ["certificate_sha256"] == bundle.sha256["certificate"])
    provenance = bundle.documents["plan"]["provenance"]
    assert provenance["baseline_backup"]["sha256"] == (
        MODULE.BASELINE_BACKUP_SHA256)
    assert tuple(provenance["execution_authorities"]) == MODULE.AUTHORITY_IDS
    assert provenance["raw_auditor"]["source_sha256"] == (
        MODULE.RAW_AUDITOR_SOURCE_SHA256)
    assert provenance["raw_auditor_run"]["plan_sha256"]
    assert provenance["raw_auditor_run"]["result_sha256"]
    for key, path in bundle.published_paths.items():
        assert path.read_bytes() == bundle.raw_bytes[key]
        assert path.read_bytes().endswith(b"\n")
        assert b"\n " not in path.read_bytes()
        assert path.stat().st_mode & 0o777 == 0o400
        assert path.stat().st_nlink == 1


def test_provenance_requires_exact_auditor_run_binding(
        tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    provenance["auditor_run"]["untrusted_note"] = "ignored"
    args.provenance.chmod(0o600)
    args.provenance_sha256 = _sealed(args.provenance, provenance)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match="AUDITOR_RUN_BINDING_KEYS"):
        MODULE.assemble(args)


def test_auditor_run_plan_must_be_canonical(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    plan_path = args.workspace / provenance["auditor_run"]["plan_path"]
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    _rewrite_run_document(args, "plan", plan, canonical=False)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match="AUDITOR_RUN_PLAN_NOT_CANONICAL"):
        MODULE.assemble(args)


def test_auditor_run_result_must_be_canonical(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    result_path = args.workspace / provenance["auditor_run"]["result_path"]
    result = json.loads(result_path.read_text(encoding="utf-8"))
    _rewrite_run_document(args, "result", result, canonical=False)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match="AUDITOR_RUN_RESULT_NOT_CANONICAL"):
        MODULE.assemble(args)


@pytest.mark.parametrize(("mutation", "status"), (
    ("offline", "AUDITOR_RUN_PLAN_OFFLINE"),
    ("auditor", "AUDITOR_RUN_AUDITOR_BINDING"),
    ("input_set", "AUDITOR_RUN_INPUTS_KEYS"),
    ("argument_order", "AUDITOR_RUN_ARGUMENT_BINDING"),
    ("wrapper", "AUDITOR_RUN_WRAPPER_BINDING"),
    ("expected_raw", "AUDITOR_RUN_EXPECTED_RAW_BINDING"),
    ("output_alias", "AUDITOR_RUN_OUTPUT_PATH_ALIAS"),
))
def test_auditor_run_plan_mutations_fail_closed(
        tmp_path, monkeypatch, mutation, status):
    args, _ = _fixture(tmp_path, monkeypatch)
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    path = args.workspace / provenance["auditor_run"]["plan_path"]
    plan = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "offline":
        plan["offline_only"] = False
    elif mutation == "auditor":
        plan["auditor"]["binary_sha256"] = "f" * 64
    elif mutation == "input_set":
        del plan["inputs"]["scene"]
    elif mutation == "argument_order":
        plan["arguments"][0], plan["arguments"][2] = (
            plan["arguments"][2], plan["arguments"][0])
    elif mutation == "wrapper":
        plan["wrapper"]["time_format"] += "\n"
    elif mutation == "expected_raw":
        plan["expected_raw"]["file_sha256"] = "f" * 64
    elif mutation == "output_alias":
        plan["outputs"]["stdout"] = plan["outputs"]["stderr"]
    _rewrite_run_document(args, "plan", plan)
    with pytest.raises(MODULE.MiddleBundleAssemblerError, match=status):
        MODULE.assemble(args)
    assert list(args.output_directory.iterdir()) == []


@pytest.mark.parametrize(("mutation", "status"), (
    ("exit", "AUDITOR_RUN_EXECUTION_BINDING"),
    ("device", "AUDITOR_RUN_EXECUTABLE_DEV"),
    ("plan", "AUDITOR_RUN_RESULT_PLAN_BINDING"),
    ("output_size", "AUDITOR_RUN_RESULT_OUTPUT_SIZE:raw"),
    ("raw_validation", "AUDITOR_RUN_RAW_VALIDATION_BINDING"),
    ("raw_bool", "AUDITOR_RUN_RAW_VALIDATION_TYPE:collision_free"),
    ("timing", "AUDITOR_RUN_TIME_BINDING"),
    ("status", "AUDITOR_RUN_RESULT_STATUS"),
))
def test_auditor_run_result_mutations_fail_closed(
        tmp_path, monkeypatch, mutation, status):
    args, _ = _fixture(tmp_path, monkeypatch)
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    path = args.workspace / provenance["auditor_run"]["result_path"]
    result = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "exit":
        result["execution"]["exit_code"] = 1
    elif mutation == "device":
        result["execution"]["executable_dev"] = 0
    elif mutation == "plan":
        result["plan"]["sha256"] = "f" * 64
    elif mutation == "output_size":
        result["outputs"]["raw"]["size_bytes"] += 1
    elif mutation == "raw_validation":
        result["raw_validation"]["sample_count"] += 1
    elif mutation == "raw_bool":
        result["raw_validation"]["collision_free"] = 1
    elif mutation == "timing":
        result["timing"]["elapsed_seconds"] += 1.0
    elif mutation == "status":
        result["valid"] = False
    _rewrite_run_document(args, "result", result)
    with pytest.raises(MODULE.MiddleBundleAssemblerError, match=status):
        MODULE.assemble(args)
    assert list(args.output_directory.iterdir()) == []


def test_proc_cmdline_must_match_every_planned_argv_item(
        tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    result_path = args.workspace / provenance["auditor_run"]["result_path"]
    result = json.loads(result_path.read_text(encoding="utf-8"))
    proc_path = args.workspace / result["execution"]["proc_cmdline_path"]
    proc_raw = proc_path.read_bytes().replace(b"--scene\0", b"--wrong\0", 1)
    proc_path.chmod(0o600)
    proc_sha = _sealed(proc_path, proc_raw)
    argv = [part.decode("utf-8") for part in proc_raw[:-1].split(b"\0")]
    result["execution"]["proc_cmdline_file_sha256"] = proc_sha
    result["execution"]["proc_cmdline_normalized_sha256"] = hashlib.sha256(
        ("\n".join(argv) + "\n").encode("utf-8")).hexdigest()
    _rewrite_run_document(args, "result", result)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match="AUDITOR_RUN_PROC_CMDLINE_ARGV"):
        MODULE.assemble(args)


def test_proc_cmdline_normalized_hash_is_independent_binding(
        tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    result_path = args.workspace / provenance["auditor_run"]["result_path"]
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["execution"]["proc_cmdline_normalized_sha256"] = "f" * 64
    _rewrite_run_document(args, "result", result)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match="AUDITOR_RUN_PROC_NORMALIZED_BINDING"):
        MODULE.assemble(args)


@pytest.mark.parametrize(("identifier", "raw", "status"), (
    ("stdout", b"CANDIDATE3_MIDDLE_EXACT_AUDIT=FAIL\n",
     "AUDITOR_RUN_STDOUT_PASS"),
    ("stderr", b"[FATAL] synthetic failure\n",
     "AUDITOR_RUN_STDERR_FAILURE_TOKEN"),
    ("time", b"elapsed_seconds=nan\n", "AUDITOR_RUN_TIME_FORMAT"),
))
def test_auditor_run_output_content_fails_closed(
        tmp_path, monkeypatch, identifier, raw, status):
    args, _ = _fixture(tmp_path, monkeypatch)
    _rewrite_run_output(args, identifier, raw)
    with pytest.raises(MODULE.MiddleBundleAssemblerError, match=status):
        MODULE.assemble(args)


def test_auditor_run_output_mode_is_sealed(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    provenance = json.loads(args.provenance.read_text(encoding="utf-8"))
    result_path = args.workspace / provenance["auditor_run"]["result_path"]
    result = json.loads(result_path.read_text(encoding="utf-8"))
    stdout = args.workspace / result["outputs"]["stdout"]["path"]
    stdout.chmod(0o600)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match="AUDITOR_RUN_RESULT_OUTPUT:stdout_MODE"):
        MODULE.assemble(args)


def test_cube_boundaries_use_phase4_and_phase5_accumulated_endpoints(
        tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    bundle = MODULE.assemble(args)
    support = bundle.documents["support_manifest"]["support_contract"]
    cube_wrapper = json.loads(
        (args.workspace / "runtime/cube.trajectory.json").read_text())
    messages = cube_wrapper["payload"]["messages"]
    for phase, asset_id, message_range in zip(
            bundle.documents["plan"]["phases"], MODULE.PHASE_ASSET_IDS,
            MODULE.PHASE_RANGES):
        wrapper = json.loads((
            args.workspace / f"runtime/{asset_id}.trajectory.json").read_text())
        start, end = message_range
        assert phase["commands"] == wrapper["payload"]["messages"][start:end]
    contact = support["boundaries"]["CUBE_CONTACT"]["robot_positions"]
    release = support["boundaries"]["CUBE_RELEASE"]["robot_positions"]
    expected_contact_arm = messages[1]["points"][-1]["positions"]
    expected_contact_gripper = messages[2]["points"][-1]["positions"]
    expected_release_arm = messages[4]["points"][-1]["positions"]
    for name, value in zip(MODULE.LEFT_ARM_JOINTS, expected_contact_arm):
        assert contact[name] == value
    for name, value in zip(MODULE.LEFT_GRIPPER_JOINTS,
                           expected_contact_gripper):
        assert contact[name] == release[name] == value
    for name, value in zip(MODULE.LEFT_ARM_JOINTS, expected_release_arm):
        assert release[name] == value
    assert release["left_dex1_finger_1_joint"] != (
        messages[5]["points"][-1]["positions"][0])


def test_missing_measured_retreat_value_fails_without_outputs(
        tmp_path, monkeypatch):
    args, raw = _fixture(tmp_path, monkeypatch)
    del raw["cube_release_retreat"]["minimum_time_step_ns"]
    monkeypatch.setattr(
        MODULE, "EXPECTED_RAW_FILE_SHA256", _rewrite_raw(args, raw))
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match="RAW_RETREAT_REQUIRED:minimum_time_step_ns"):
        MODULE.assemble(args)
    assert list(args.output_directory.glob("candidate3_middle*.json")) == []
    assert list(args.output_directory.glob(".candidate3_middle_828_stage_*")) == []


@pytest.mark.parametrize("field", (
    "gate_reached", "gate_ordinal", "last_below_gate_ordinal",
))
def test_required_retreat_gate_fields_fail_closed(
        tmp_path, monkeypatch, field):
    args, raw = _fixture(tmp_path, monkeypatch)
    del raw["cube_release_retreat"][field]
    monkeypatch.setattr(
        MODULE, "EXPECTED_RAW_FILE_SHA256", _rewrite_raw(args, raw))
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match=f"RAW_RETREAT_REQUIRED:{field}"):
        MODULE.assemble(args)
    assert list(args.output_directory.iterdir()) == []


@pytest.mark.parametrize(("field", "value", "status"), (
    ("message_index", 5, "RAW_RETREAT_MESSAGE_INDEX"),
    ("start_point_index", 1, "RAW_RETREAT_START"),
    ("start_subdivision", 1, "RAW_RETREAT_START"),
    ("gate_reached", False, "RAW_RETREAT_GATE_FLAGS"),
    ("gate_never_regressed", False, "RAW_RETREAT_GATE_FLAGS"),
    ("gate_ordinal", 2, "RAW_RETREAT_SHORTEST_PREFIX"),
    ("last_below_gate_ordinal", 1, "RAW_RETREAT_SHORTEST_PREFIX"),
    ("window_sample_count", 2, "RAW_RETREAT_WINDOW_GATE_BINDING"),
    ("gate_subdivision", 0, "RAW_RETREAT_GATE_SUBDIVISION"),
    ("minimum_path_parameter_step", 0.0, "RAW_RETREAT_PATH_STEP"),
    ("post_gate_minimum_exact_distance_m", 0.010,
     "RAW_RETREAT_POST_GATE_EXACT_CLEARANCE"),
    ("post_gate_minimum_guard_physical_lower_bound_m", 0.010,
     "RAW_RETREAT_POST_GATE_GUARD_CLEARANCE"),
    ("maximum_exact_penetration_m", 0.0001,
     "RAW_RETREAT_EXACT_PENETRATION"),
    ("final_tcp_projection_m", 0.0, "RAW_RETREAT_FINAL_TCP_PROJECTION"),
    ("observed_pair_semantics", "nearest_distance",
     "RAW_RETREAT_PAIR_SEMANTICS"),
    ("observed_exact_pairs", [], "RAW_RETREAT_EXACT_PAIRS_EMPTY"),
    ("observed_guard_pairs", [], "RAW_RETREAT_GUARD_PAIRS_EMPTY"),
))
def test_retreat_semantic_mutations_fail_closed(
        tmp_path, monkeypatch, field, value, status):
    args, raw = _fixture(tmp_path, monkeypatch)
    raw["cube_release_retreat"][field] = value
    monkeypatch.setattr(
        MODULE, "EXPECTED_RAW_FILE_SHA256", _rewrite_raw(args, raw))
    with pytest.raises(MODULE.MiddleBundleAssemblerError, match=status):
        MODULE.assemble(args)
    assert list(args.output_directory.iterdir()) == []


@pytest.mark.parametrize(("field", "value", "status"), (
    ("authorized_sample_count", 2, "RAW_RETREAT_CONTACT_SAMPLE_BINDING"),
    ("maximum_observed_penetration_m", 0.001,
     "RAW_RETREAT_CONTACT_PENETRATION_BINDING"),
    ("observed_pair_semantics", "nearest_distance",
     "RAW_RETREAT_CONTACT_SEMANTICS"),
    ("guard_nearest_pairs", [], "RAW_RETREAT_CONTACT_GUARD_PAIRS_EMPTY"),
))
def test_retreat_contact_window_mutations_fail_closed(
        tmp_path, monkeypatch, field, value, status):
    args, raw = _fixture(tmp_path, monkeypatch)
    raw["contact_window_measurements"]["cube_release_retreat"][field] = value
    monkeypatch.setattr(
        MODULE, "EXPECTED_RAW_FILE_SHA256", _rewrite_raw(args, raw))
    with pytest.raises(MODULE.MiddleBundleAssemblerError, match=status):
        MODULE.assemble(args)
    assert list(args.output_directory.iterdir()) == []


@pytest.mark.parametrize(("field", "value", "status"), (
    ("minimum_environment_clearance_is_lower_bound", False,
     "RAW_ENVIRONMENT_CLEARANCE_LOWER_BOUND"),
    ("right_handle_nonpad_clearance_required_m", 1.0,
     "RAW_RIGHT_HANDLE_NONPAD_CLEARANCE_REQUIRED"),
    ("left_handle_clearance_required_m", 1.0,
     "RAW_LEFT_HANDLE_CLEARANCE_REQUIRED"),
    ("park_static_clearance_required_m", 1.0,
     "RAW_PARK_STATIC_CLEARANCE_REQUIRED"),
    ("cube_guard_exact_refinement_samples", -1,
     "RAW_CUBE_GUARD_REFINEMENT_SAMPLES"),
))
def test_root_contract_mutations_fail_closed(
        tmp_path, monkeypatch, field, value, status):
    args, raw = _fixture(tmp_path, monkeypatch)
    raw[field] = value
    monkeypatch.setattr(
        MODULE, "EXPECTED_RAW_FILE_SHA256", _rewrite_raw(args, raw))
    with pytest.raises(MODULE.MiddleBundleAssemblerError, match=status):
        MODULE.assemble(args)
    assert list(args.output_directory.iterdir()) == []


def test_raw_root_rejects_unknown_key(tmp_path, monkeypatch):
    args, raw = _fixture(tmp_path, monkeypatch)
    raw["unexpected_claim"] = True
    monkeypatch.setattr(
        MODULE, "EXPECTED_RAW_FILE_SHA256", _rewrite_raw(args, raw))
    with pytest.raises(MODULE.MiddleBundleAssemblerError, match="RAW_KEYS"):
        MODULE.assemble(args)


def test_frozen_raw_hash_rejects_legal_mutation_with_recomputed_cli_hash(
        tmp_path, monkeypatch):
    args, raw = _fixture(tmp_path, monkeypatch)
    raw["final_target"]["translation_error_m"] = 0.002
    _rewrite_raw(args, raw)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError,
            match="RAW_COMBINED_FROZEN_SHA256"):
        MODULE.assemble(args)
    assert list(args.output_directory.iterdir()) == []


def test_raw_must_remain_nonfinal_but_complete(tmp_path, monkeypatch):
    args, raw = _fixture(tmp_path, monkeypatch)
    raw["valid"] = True
    raw["status"] = "PASS"
    monkeypatch.setattr(
        MODULE, "EXPECTED_RAW_FILE_SHA256", _rewrite_raw(args, raw))
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError, match="RAW_PROMOTION_STATUS"):
        MODULE.assemble(args)


def test_publish_is_no_replace(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch, publish=True)
    MODULE.assemble(args)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError, match="PUBLISH_TARGET_EXISTS"):
        MODULE.assemble(args)


def test_partial_link_failure_rolls_back_every_target(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch, publish=True)
    real_link = MODULE.os.link
    calls = 0

    def fail_third_link(source, target, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 3:
            raise OSError("injected link failure")
        return real_link(source, target, **kwargs)

    monkeypatch.setattr(MODULE.os, "link", fail_third_link)
    with pytest.raises(
            MODULE.MiddleBundleAssemblerError, match="PUBLISH_NO_REPLACE"):
        MODULE.assemble(args)
    assert list(args.output_directory.glob("candidate3_middle*.json")) == []
