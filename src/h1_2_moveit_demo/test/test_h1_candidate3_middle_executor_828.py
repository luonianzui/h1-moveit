"""ROS-free tests for the real sealed 828 middle executor."""

import ast
import hashlib
import importlib.util
import json
import os
import py_compile
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

from h1_2_vision import candidate3_middle_execution_contract_828 as CONTRACT


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = (WORKSPACE / "src/h1_2_moveit_demo/scripts/"
          "h1_candidate3_middle_executor_828.py")
ADMISSION_SOURCE = (
    WORKSPACE / "src/h1_2_vision/h1_2_vision/candidate3_middle_admission.py")
CONTRACT_SOURCE = (
    WORKSPACE /
    "src/h1_2_vision/h1_2_vision/candidate3_middle_execution_contract_828.py")
SPEC = importlib.util.spec_from_file_location(
    "h1_candidate3_middle_executor_828_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _sealed_json(path, value):
    path.write_text(
        json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    path.chmod(0o400)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _sealed_bytes(path, value, mode=0o400):
    path.write_bytes(value)
    path.chmod(mode)
    return hashlib.sha256(value).hexdigest()


def _positions(topic):
    if topic == "/torso_controller/joint_trajectory":
        return [MODULE.E0_TORSO_RAD]
    if topic == "/right_arm_controller/joint_trajectory":
        return list(MODULE.E0_RIGHT_ARM_RAD)
    if topic == "/left_arm_controller/joint_trajectory":
        return [0.0, 0.30, 0.0, 0.60, 0.0, 0.0, 0.0]
    if topic == "/left_gripper_controller/joint_trajectory":
        return [-0.019, -0.019]
    raise AssertionError(topic)


def _fixture(tmp_path, monkeypatch=None, bundle_tag="primary"):
    tmp_path.mkdir(parents=True, exist_ok=True)
    phases = []
    asset_messages = []
    synchronization_groups = []
    for index, (name, topics, support_values) in enumerate(zip(
            MODULE.PHASE_NAMES, MODULE.PHASE_COMMAND_TOPICS,
            MODULE.SUPPORT_EXPECTATIONS)):
        commands = [{
            "topic": topic,
            "joint_names": list(MODULE.CONTROLLER_JOINTS[topic]),
            "points": [{
                "positions": _positions(topic),
                "time_from_start_ns": 0,
            }],
        } for topic in topics]
        range_start = len(asset_messages)
        asset_messages.extend(commands)
        range_end = len(asset_messages)
        if index in MODULE.SYNCHRONIZED_PHASES:
            synchronization_groups.append({
                "message_indices": list(range(range_start, range_end)),
                "shared_time_from_start_ns": [0],
            })
        phases.append({
            "name": name,
            "commanded": bool(topics),
            "asset": None,
            "message_range": [range_start, range_end],
            "commands": commands,
            "support": dict(zip(
                ("door_pre", "door_post", "cube_pre", "cube_post"),
                support_values)),
            "entry_state_sha256": "0" * 64,
            "exit_state_sha256": "1" * 64,
        })
    asset_payload = {
        "schema": MODULE.ASSET_SCHEMA,
        "asset_id": "shared",
        "messages": asset_messages,
        "synchronization_groups": synchronization_groups,
    }
    payload_sha = MODULE._canonical_sha256(asset_payload)
    asset_path = tmp_path / "asset.json"
    asset_sha = _sealed_json(asset_path, {
        "payload": asset_payload,
        "sha256": payload_sha,
    })
    asset_binding = {
        "path": asset_path.name,
        "file_sha256": asset_sha,
        "payload_sha256": payload_sha,
        "schema": asset_payload["schema"],
    }
    for phase in phases:
        phase["asset"] = dict(asset_binding)
    hooks = []
    for index, expected in enumerate(MODULE.HOOK_EXPECTATIONS):
        kind, phase_index, action, pre_state, post_state = expected
        hooks.append({
            "id": f"hook-{index}",
            "boundary": {
                "kind": kind,
                "phase_index": phase_index,
                "phase_name": MODULE.PHASE_NAMES[phase_index],
            },
            "action": action,
            "pre_state": pre_state,
            "post_state": post_state,
            "require_verified": True,
        })
    scene_path = tmp_path / "scene.cdr"
    scene_sha = _sealed_bytes(scene_path, b"scene")
    support_contract = {
        "schema": MODULE.SUPPORT_CONTRACT_SCHEMA,
        "bundle_tag": bundle_tag,
    }
    support_contract_sha = MODULE._canonical_sha256(support_contract)
    scene_binding = {
        "scene_path": scene_path.name,
        "scene_cdr_sha256": scene_sha,
        "scene_name": "fresh_candidate3_scene",
        "raw_scene_hash": "b" * 64,
        "planning_frame": "world",
        "world": dict(CONTRACT.WORLD_VALUE),
        "required_object_ids": [
            "table", "incubator", "incubator_handle_exact", "green_cube",
        ],
        "removed_visual_mask_ids": [],
        "unknown_visual_mask_ids": [],
    }
    raw_robot_state = {
        "label": "raw_scene_q0",
        "torso_yaw_rad": 0.0,
        "right_arm_rad": list(CONTRACT.Q0_RIGHT_ARM_RAD),
        "right_fingers_m": dict(CONTRACT.OPEN_RIGHT_FINGERS),
        "left_arm_rad": list(CONTRACT.LAUNCH_LEFT_ARM_RAD),
        "left_fingers_m": dict(CONTRACT.OPEN_LEFT_FINGERS),
    }
    projected_e0 = {
        "label": "canonical_e0_q90_held",
        "torso_yaw_rad": MODULE.E0_TORSO_RAD,
        "right_arm_rad": list(MODULE.E0_RIGHT_ARM_RAD),
        "right_fingers_m": dict(zip(
            MODULE.RIGHT_GRIPPER_JOINTS, MODULE.E0_RIGHT_GRIPPER)),
        "left_arm_rad": list(CONTRACT.PARK_LEFT_ARM_RAD),
        "left_fingers_m": dict(CONTRACT.OPEN_LEFT_FINGERS),
    }
    entry_projection = {
        "schema": CONTRACT.ENTRY_PROJECTION_SCHEMA,
        "method": "prefix_certified_endpoint_projection",
        "source_scene_sha256": scene_sha,
        "raw_robot_state": raw_robot_state,
        "source_prefix": dict(CONTRACT.RIGHT_PREFIX_BINDING),
        "projected_e0": projected_e0,
        "projected_door_angle_rad": 0.0,
        "projected_handle_angle_rad": 3.141592653589793 / 2.0,
        "max_abs_joint_error_rad": 0.0,
        "valid": True,
    }
    assets = {
        "shared": {
            "path": asset_path.name,
            "file_sha256": asset_sha,
            "payload_sha256": payload_sha,
            "schema": asset_payload["schema"],
        },
    }
    terminal_seam = {
        "entry_e0": {
            "torso_yaw_rad": MODULE.E0_TORSO_RAD,
            "right_arm_rad": list(MODULE.E0_RIGHT_ARM_RAD),
            "right_fingers_m": dict(zip(
                MODULE.RIGHT_GRIPPER_JOINTS, MODULE.E0_RIGHT_GRIPPER)),
            "max_abs_error": 0.0,
        },
        "return_e0": {
            "torso_yaw_rad": MODULE.E0_TORSO_RAD,
            "right_arm_rad": list(MODULE.E0_RIGHT_ARM_RAD),
            "right_fingers_m": dict(zip(
                MODULE.RIGHT_GRIPPER_JOINTS, MODULE.E0_RIGHT_GRIPPER)),
            "max_abs_error": 0.0,
        },
        "left_park_open": {
            "left_arm_rad": list(CONTRACT.PARK_LEFT_ARM_RAD),
            "left_fingers_m": dict(CONTRACT.OPEN_LEFT_FINGERS),
            "max_abs_error": 0.0,
        },
        "door_state": {
            "entry_angle_rad": 0.0,
            "return_angle_rad": 0.0,
            "handle_angle_rad": 3.141592653589793 / 2.0,
        },
        "support_state": {
            "before_entry": "detached",
            "after_entry": "attached",
            "before_return": "attached",
            "after_return": "detached",
        },
        "release_owner": MODULE.RELEASE_OWNER,
        "forbidden_middle_phases": ["right_release", "right_retreat"],
        "valid": True,
    }
    plan_value = {
        "schema": MODULE.PLAN_SCHEMA,
        "worker_version": "h1_candidate3_middle_worker/828-e0-v1",
        "topology": "right_door_open_left_cube_right_door_close_828_e0",
        "offline_only": True,
        "controller_commands_issued": 0,
        "scene_binding": scene_binding,
        "entry_projection": entry_projection,
        "assets": assets,
        "phase_names": list(MODULE.PHASE_NAMES),
        "phases": phases,
        "hooks": hooks,
        "support_contract_sha256": support_contract_sha,
        "release_owner": MODULE.RELEASE_OWNER,
        "terminal_seam": terminal_seam,
        "final_state": {"owner": MODULE.RELEASE_OWNER},
        "required_thresholds": {"torso_max_deg": 45.0},
        "timing": {"maximum_phase_seconds": 120.0},
        "provenance": {"bundle_tag": bundle_tag},
    }
    plan_path = tmp_path / "plan.json"
    plan_sha = _sealed_json(plan_path, plan_value)
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
    metrics = {
        "sample_count": 11,
        "subdivision_count": 11,
        "minimum_clearances_m": dict(clearances),
        "maximum_abs_torso_yaw_rad": abs(MODULE.E0_TORSO_RAD),
        "cube_bump": {
            "maximum_tcp_translation_m": 0.0,
            "maximum_tcp_rotation_rad": 0.0,
            "translation_limit_m": 0.006,
            "rotation_limit_rad": 0.013962634015954637,
        },
        "cube_release_retreat": {
            "phase_name": "cube_retreat",
            "message_index": 0,
            "start_point_index": 0,
            "start_subdivision": 0,
            "gate_point_index": 0,
            "gate_subdivision": 0,
            "window_sample_count": 1,
            "minimum_exact_distance_m": 0.0,
            "maximum_exact_distance_m": 0.01,
            "final_exact_distance_m": 0.01,
            "minimum_time_step_ns": 1,
            "minimum_path_parameter_step": 0.001,
            "minimum_tcp_separation_projection_step_m": -0.001,
            "maximum_exact_penetration_m": 0.0,
            "post_gate_minimum_exact_distance_m": 0.001,
            "post_gate_minimum_guard_physical_lower_bound_m": 0.010001,
            "observed_exact_pairs": [],
            "observed_guard_pairs": [],
            "gate_never_regressed": True,
        },
        "final_target": {
            "translation_error_m": 0.0,
            "rotation_error_rad": 0.0,
            "translation_limit_m": 0.003,
            "rotation_limit_rad": 0.03490658503988659,
        },
        "all_thresholds_passed": True,
    }
    right_window = "right_handle_pad_contact"
    cube_window = "left_cube_grasp_carry"
    retreat_window = "left_cube_release_retreat"
    phase_results = []
    for index, (name, phase) in enumerate(zip(MODULE.PHASE_NAMES, phases)):
        window_ids = [right_window]
        if index in (4, 5, 6):
            window_ids.append(cube_window)
        if index == 7:
            window_ids.append(retreat_window)
        phase_clearances = dict(clearances)
        if index not in (4, 5, 6, 7):
            phase_clearances["cube"] = 0.010001
        phase_results.append({
            "index": index,
            "name": name,
            "asset_id": "shared",
            "message_range": {
                "start_message_index": phase["message_range"][0],
                "end_message_index_exclusive": phase["message_range"][1],
            },
            "topics": [command["topic"] for command in phase["commands"]],
            "samples": 1,
            "subdivisions": 1,
            "minimum_clearances_m": phase_clearances,
            "maximum_abs_torso_yaw_rad": abs(MODULE.E0_TORSO_RAD),
            "contact_window_ids": window_ids,
            "endpoint_sha256": phase["exit_state_sha256"],
            "valid": True,
            "status": "PASS",
        })

    def phase_binding(index):
        phase = phase_results[index]
        return {
            "phase_name": phase["name"],
            "asset_id": phase["asset_id"],
            "message_range": dict(phase["message_range"]),
        }

    contact_authorizations = {
        "schema": CONTRACT.CONTACT_AUTHORIZATION_SCHEMA,
        "windows": [
            {
                "id": right_window,
                "phase_bindings": [
                    phase_binding(index)
                    for index in range(len(MODULE.PHASE_NAMES))
                ],
                "object_ids": ["incubator_handle_exact"],
                "allowed_links": sorted(CONTRACT.RIGHT_HANDLE_PADS),
                "penetration_policy": "AUTHORIZED_PAD_CONTACT",
                "maximum_observed_penetration_m": 0.0,
                "observed_pairs": [
                    ["incubator_handle_exact", link]
                    for link in sorted(CONTRACT.RIGHT_HANDLE_PADS)
                ],
                "authorized_sample_count": 1,
                "valid": True,
            },
            {
                "id": cube_window,
                "phase_bindings": [
                    phase_binding(index) for index in (4, 5, 6)
                ],
                "object_ids": list(CONTRACT.CUBE_OBJECT_IDS),
                "allowed_links": sorted(CONTRACT.LEFT_CUBE_TOUCH_LINKS),
                "penetration_policy": "AUTHORIZED_GRASP_CONTACT",
                "maximum_observed_penetration_m": 0.001,
                "observed_pairs": [],
                "authorized_sample_count": 1,
                "valid": True,
            },
            {
                "id": retreat_window,
                "phase_bindings": [phase_binding(7)],
                "object_ids": list(CONTRACT.CUBE_OBJECT_IDS),
                "allowed_links": sorted(CONTRACT.LEFT_CUBE_TOUCH_LINKS),
                "penetration_policy": "NONPENETRATING",
                "maximum_observed_penetration_m": 0.0,
                "observed_pairs": [],
                "authorized_sample_count": 1,
                "valid": True,
            },
        ],
        "forbidden_left_handle": {
            "object_id": "incubator_handle_exact",
            "link_prefix": "left_",
            "contact_count": 0,
            "minimum_clearance_m": 0.05,
            "required_minimum_clearance_m": 0.05,
            "valid": True,
        },
        "unauthorized_contacts": {"count": 0, "pairs": []},
        "valid": True,
    }
    result_value = {
        "schema": MODULE.RESULT_SCHEMA,
        "auditor_version": MODULE.AUDITOR_VERSION,
        "audit_method": "exact_discrete_contact_path_replay",
        "offline_only": True,
        "controller_commands_issued": 0,
        "scene_binding": scene_binding,
        "entry_projection": entry_projection,
        "plan": plan_value,
        "plan_file_sha256": plan_sha,
        "support_contract_sha256": support_contract_sha,
        "assets": assets,
        "phase_results": phase_results,
        "contact_authorizations": contact_authorizations,
        "metrics": metrics,
        "terminal_seam": terminal_seam,
        "valid": True,
        "status": "PASS",
    }
    result_path = tmp_path / "result.json"
    result_sha = _sealed_json(result_path, result_value)
    certificate_value = {
        "schema": MODULE.CERTIFICATE_SCHEMA,
        "auditor_version": MODULE.AUDITOR_VERSION,
        "offline_only": True,
        "controller_commands_issued": 0,
        "plan_schema": MODULE.PLAN_SCHEMA,
        "plan_sha256": plan_sha,
        "exact_result_schema": MODULE.RESULT_SCHEMA,
        "exact_result_sha256": result_sha,
        "support_contract_sha256": support_contract_sha,
        "scene_binding_sha256": MODULE._canonical_sha256(scene_binding),
        "entry_projection_sha256": MODULE._canonical_sha256(entry_projection),
        "asset_bindings_sha256": MODULE._canonical_sha256(assets),
        "terminal_seam_sha256": MODULE._canonical_sha256(terminal_seam),
        "metrics": metrics,
        "valid": True,
        "status": "PASS",
    }
    certificate_path = tmp_path / "certificate.json"
    certificate_sha = _sealed_json(certificate_path, certificate_value)
    support_manifest = tmp_path / "support_manifest.json"
    support_manifest_value = {
        "schema": MODULE.SUPPORT_MANIFEST_SCHEMA,
        "controller_execution_authorized": True,
        "live_execution_passed": False,
        "bindings": {
            "plan_sha256": plan_sha,
            "plan_schema": MODULE.PLAN_SCHEMA,
            "certificate_sha256": certificate_sha,
            "certificate_schema": MODULE.CERTIFICATE_SCHEMA,
            "exact_result_sha256": result_sha,
            "exact_result_schema": MODULE.RESULT_SCHEMA,
            "auditor_version": MODULE.AUDITOR_VERSION,
            "support_contract_sha256": support_contract_sha,
        },
        "support_contract": support_contract,
    }
    support_manifest_sha = _sealed_json(
        support_manifest, support_manifest_value)
    orchestrator = tmp_path / "orchestrator"
    orchestrator_sha = _sealed_bytes(
        orchestrator,
        b"#!/usr/bin/python3\n"
        b"print('H1_SUPPORT_ORCHESTRATOR_828=PASS operation=validate "
        b"execution=none')\n",
        0o500)

    args = SimpleNamespace(
        workspace=tmp_path,
        scene=scene_path,
        scene_sha256=scene_sha,
        plan=plan_path,
        plan_sha256=plan_sha,
        certificate=certificate_path,
        certificate_sha256=certificate_sha,
        result=result_path,
        result_sha256=result_sha,
        middle_admission=ADMISSION_SOURCE,
        middle_admission_sha256=hashlib.sha256(
            ADMISSION_SOURCE.read_bytes()).hexdigest(),
        middle_contract=CONTRACT_SOURCE,
        middle_contract_sha256=hashlib.sha256(
            CONTRACT_SOURCE.read_bytes()).hexdigest(),
        support_orchestrator=orchestrator,
        support_orchestrator_sha256=orchestrator_sha,
        support_manifest=support_manifest,
        support_manifest_sha256=support_manifest_sha,
        support_certificate=certificate_path,
        support_certificate_sha256=certificate_sha,
        support_timeout=10.0,
    )
    return args, plan_value


def _reseal_dag(args, plan):
    """Rebuild all downstream DAG file bindings after a plan mutation."""
    args.plan.chmod(0o600)
    args.plan_sha256 = _sealed_json(args.plan, plan)

    result = json.loads(args.result.read_text())
    result["plan"] = plan
    result["plan_file_sha256"] = args.plan_sha256
    for name in ("scene_binding", "entry_projection", "assets",
                 "terminal_seam", "support_contract_sha256"):
        result[name] = plan[name]
    args.result.chmod(0o600)
    args.result_sha256 = _sealed_json(args.result, result)

    certificate = json.loads(args.certificate.read_text())
    certificate.update({
        "plan_sha256": args.plan_sha256,
        "exact_result_sha256": args.result_sha256,
        "support_contract_sha256": plan["support_contract_sha256"],
        "scene_binding_sha256": MODULE._canonical_sha256(
            result["scene_binding"]),
        "entry_projection_sha256": MODULE._canonical_sha256(
            result["entry_projection"]),
        "asset_bindings_sha256": MODULE._canonical_sha256(result["assets"]),
        "terminal_seam_sha256": MODULE._canonical_sha256(
            result["terminal_seam"]),
        "metrics": result["metrics"],
    })
    args.certificate.chmod(0o600)
    args.certificate_sha256 = _sealed_json(args.certificate, certificate)
    args.support_certificate_sha256 = args.certificate_sha256

    manifest = json.loads(args.support_manifest.read_text())
    manifest["bindings"].update({
        "plan_sha256": args.plan_sha256,
        "certificate_sha256": args.certificate_sha256,
        "exact_result_sha256": args.result_sha256,
        "support_contract_sha256": plan["support_contract_sha256"],
    })
    args.support_manifest.chmod(0o600)
    args.support_manifest_sha256 = _sealed_json(args.support_manifest, manifest)


def test_valid_bundle_has_exact_eleven_phases_and_no_right_gripper(
        tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    bundle = MODULE.validate_bundle(args)
    result = json.loads(args.result.read_text())
    assert "certificate" not in result
    assert result["plan_file_sha256"] == args.plan_sha256
    assert bundle.support_certificate.resolve() == bundle.certificate.path.resolve()
    assert bundle.support_certificate_sha256 == bundle.certificate.sha256
    assert tuple(phase.name for phase in bundle.phases) == MODULE.PHASE_NAMES
    assert all(command.topic != "/right_gripper_controller/joint_trajectory"
               for phase in bundle.phases for command in phase.commands)
    assert tuple(command.topic for command in bundle.phases[5].commands) == (
        "/left_arm_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
    )


def test_handle_window_does_not_exempt_cube_clearance(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    result = json.loads(args.result.read_text())
    phase = result["phase_results"][0]
    assert phase["contact_window_ids"] == ["right_handle_pad_contact"]
    phase["minimum_clearances_m"]["cube"] = 0.01
    with pytest.raises(CONTRACT.MiddleExecutionContract828Error,
                       match="RESULT_PHASE_CLEARANCES:0:cube"):
        CONTRACT.validate_result(result)


def test_cube_window_does_not_exempt_handle_clearance(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    result = json.loads(args.result.read_text())
    phase = result["phase_results"][4]
    phase["contact_window_ids"] = ["left_cube_grasp_carry"]
    phase["minimum_clearances_m"]["handle"] = 0.01
    with pytest.raises(CONTRACT.MiddleExecutionContract828Error,
                       match="RESULT_PHASE_CLEARANCES:4:handle"):
        CONTRACT.validate_result(result)


def test_contact_binding_rejects_cross_asset_reference(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    result = json.loads(args.result.read_text())
    binding = result["contact_authorizations"]["windows"][0][
        "phase_bindings"][0]
    binding["asset_id"] = "another-valid-asset"
    with pytest.raises(CONTRACT.MiddleExecutionContract828Error,
                       match="RESULT_PHASE_CONTACT_WINDOW_ASSET:0"):
        CONTRACT.validate_result(result)


def test_contact_binding_rejects_missing_phase_binding(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    result = json.loads(args.result.read_text())
    del result["contact_authorizations"]["windows"][0][
        "phase_bindings"][0]
    with pytest.raises(CONTRACT.MiddleExecutionContract828Error,
                       match="RESULT_PHASE_CONTACT_WINDOW_BINDING:0"):
        CONTRACT.validate_result(result)


def test_contact_binding_rejects_unreferenced_extra_phase(
        tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    result = json.loads(args.result.read_text())
    phase = result["phase_results"][4]
    phase["contact_window_ids"] = ["right_handle_pad_contact"]
    phase["minimum_clearances_m"]["cube"] = 0.010001
    with pytest.raises(CONTRACT.MiddleExecutionContract828Error,
                       match="RESULT_CONTACT_WINDOW_REFERENCE_SET:"
                             "left_cube_grasp_carry"):
        CONTRACT.validate_result(result)


def test_contact_binding_rejects_wrong_message_range(tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    result = json.loads(args.result.read_text())
    message_range = result["contact_authorizations"]["windows"][0][
        "phase_bindings"][1]["message_range"]
    message_range["end_message_index_exclusive"] += 1
    with pytest.raises(CONTRACT.MiddleExecutionContract828Error,
                       match="RESULT_PHASE_CONTACT_WINDOW_RANGE:1"):
        CONTRACT.validate_result(result)


@pytest.mark.parametrize("object_ids", [
    ["fake_cube_dynamic_exact", "green_cube_10mm_guard"],
    ["green_cube_10mm_guard", "green_cube_dynamic_exact"],
])
def test_cube_window_requires_exact_ordered_auditor_object_ids(
        tmp_path, monkeypatch, object_ids):
    args, _ = _fixture(tmp_path, monkeypatch)
    result = json.loads(args.result.read_text())
    result["contact_authorizations"]["windows"][1][
        "object_ids"] = object_ids
    with pytest.raises(CONTRACT.MiddleExecutionContract828Error,
                       match="RESULT_CONTACT_WINDOW_OBJECT_KIND:1"):
        CONTRACT.validate_result(result)


def test_swapped_individually_valid_support_bundle_fails_closed(
        tmp_path, monkeypatch):
    args_a, _ = _fixture(tmp_path / "a", monkeypatch, "bundle-a")
    args_b, _ = _fixture(tmp_path / "b", monkeypatch, "bundle-b")
    # Every file in B is valid with B, but B's final manifest may not be used
    # to authorize A's invocation.
    args_a.support_manifest = args_b.support_manifest
    args_a.support_manifest_sha256 = args_b.support_manifest_sha256
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="SUPPORT_BINDING_PLAN_SHA256"):
        MODULE.validate_bundle(args_a)


def test_distinct_support_certificate_path_fails_even_with_valid_json(
        tmp_path, monkeypatch):
    args, _ = _fixture(tmp_path, monkeypatch)
    duplicate = tmp_path / "support_certificate_copy.json"
    duplicate.write_bytes(args.certificate.read_bytes())
    duplicate.chmod(0o400)
    args.support_certificate = duplicate
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="SUPPORT_CERTIFICATE_PATH_IDENTITY"):
        MODULE.validate_bundle(args)


def test_wrong_support_boundary_fails_closed(tmp_path, monkeypatch):
    args, plan = _fixture(tmp_path, monkeypatch)
    plan["phases"][5]["support"]["cube_post"] = True
    _reseal_dag(args, plan)
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="PLAN_PHASE_SUPPORT:5"):
        MODULE.validate_bundle(args)


def test_right_gripper_command_injection_fails_closed(tmp_path, monkeypatch):
    args, plan = _fixture(tmp_path, monkeypatch)
    command = plan["phases"][4]["commands"][1]
    command["topic"] = "/right_gripper_controller/joint_trajectory"
    command["joint_names"] = list(MODULE.RIGHT_GRIPPER_JOINTS)
    command["points"][0]["positions"] = list(MODULE.E0_RIGHT_GRIPPER)
    _reseal_dag(args, plan)
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="PLAN_PHASE_COMMAND:4:1_TOPIC"):
        MODULE.validate_bundle(args)


def test_plan_command_may_not_diverge_from_sealed_asset(
        tmp_path, monkeypatch):
    args, plan = _fixture(tmp_path, monkeypatch)
    plan["phases"][3]["commands"][0]["points"][0]["positions"][0] = 0.01
    _reseal_dag(args, plan)
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="PLAN_PHASE_ASSET_COMMAND_IDENTITY:3"):
        MODULE.validate_bundle(args)


AUTHORITY_MODULE_NAMES = (
    "h1_2_vision",
    "h1_2_vision.candidate3_middle_execution_contract_828",
    "h1_2_vision.candidate3_middle_admission",
)


def _authority_args(tmp_path, contract_bytes, admission_bytes):
    contract = tmp_path / "contract.py"
    admission = tmp_path / "admission.py"
    contract_sha = _sealed_bytes(contract, contract_bytes, 0o500)
    admission_sha = _sealed_bytes(admission, admission_bytes, 0o500)
    return SimpleNamespace(
        middle_contract=contract,
        middle_contract_sha256=contract_sha,
        middle_admission=admission,
        middle_admission_sha256=admission_sha,
    )


def _restore_authority_modules(previous):
    for name in AUTHORITY_MODULE_NAMES:
        sys.modules.pop(name, None)
    for name, module in previous.items():
        if module is not None:
            sys.modules[name] = module


def test_semantic_authority_ignores_matching_timestamp_pyc(tmp_path):
    malicious = (
        b"VALUE = 'EVIL'\n"
        b"def validate_result(value):\n    return value\n"
        b"def validate_certificate(*args, **kwargs):\n    return None\n")
    verified = malicious.replace(b"'EVIL'", b"'GOOD'")
    admission = (
        b"from .candidate3_middle_execution_contract_828 import validate_result\n"
        b"def admit_result(path, digest):\n"
        b"    return validate_result((path, digest))\n")
    args = _authority_args(tmp_path, malicious, admission)
    timestamp = args.middle_contract.stat()
    pyc = Path(importlib.util.cache_from_source(str(args.middle_contract)))
    py_compile.compile(
        str(args.middle_contract), cfile=str(pyc), doraise=True)
    args.middle_contract.chmod(0o700)
    args.middle_contract_sha256 = _sealed_bytes(
        args.middle_contract, verified, 0o500)
    os.utime(
        args.middle_contract,
        ns=(timestamp.st_atime_ns, timestamp.st_mtime_ns))
    previous = {
        name: sys.modules.pop(name, None) for name in AUTHORITY_MODULE_NAMES
    }
    try:
        contract, loaded_admission = MODULE._load_semantic_authority(args)
        assert contract.VALUE == "GOOD"
        assert loaded_admission.validate_result is contract.validate_result
        assert contract.__loader__ is None
    finally:
        _restore_authority_modules(previous)


def test_semantic_authority_never_loads_package_init_pyc(
        tmp_path, monkeypatch):
    package_directory = tmp_path / "h1_2_vision"
    package_directory.mkdir()
    init_source = package_directory / "__init__.py"
    malicious = b"raise RuntimeError('INIT_PYC_EXECUTED')\n"
    verified = b"PACKAGE_VALUE = 'GOOD'\n"
    width = max(len(malicious), len(verified))
    malicious = malicious.rstrip(b"\n").ljust(width - 1) + b"\n"
    verified = verified.rstrip(b"\n").ljust(width - 1) + b"\n"
    init_source.write_bytes(malicious)
    timestamp = init_source.stat()
    pyc = Path(importlib.util.cache_from_source(str(init_source)))
    py_compile.compile(str(init_source), cfile=str(pyc), doraise=True)
    init_source.write_bytes(verified)
    os.utime(
        init_source, ns=(timestamp.st_atime_ns, timestamp.st_mtime_ns))
    monkeypatch.syspath_prepend(str(tmp_path))

    contract = (
        b"def validate_result(value):\n    return value\n"
        b"def validate_certificate(*args, **kwargs):\n    return None\n")
    admission = (
        b"from .candidate3_middle_execution_contract_828 import validate_result\n"
        b"def admit_result(path, digest):\n"
        b"    return validate_result((path, digest))\n")
    args = _authority_args(tmp_path, contract, admission)
    previous = {
        name: sys.modules.pop(name, None) for name in AUTHORITY_MODULE_NAMES
    }
    try:
        MODULE._load_semantic_authority(args)
        package = sys.modules["h1_2_vision"]
        assert package.__path__ == []
        assert package.__loader__ is None
        assert not hasattr(package, "PACKAGE_VALUE")
    finally:
        _restore_authority_modules(previous)


def test_semantic_authority_executes_captured_bytes_after_path_change(
        tmp_path, monkeypatch):
    verified = (
        b"VALUE = 'GOOD'\n"
        b"def validate_result(value):\n    return value\n"
        b"def validate_certificate(*args, **kwargs):\n    return None\n")
    replacement = verified.replace(b"'GOOD'", b"'EVIL'")
    admission = (
        b"from .candidate3_middle_execution_contract_828 import validate_result\n"
        b"def admit_result(path, digest):\n"
        b"    return validate_result((path, digest))\n")
    args = _authority_args(tmp_path, verified, admission)
    read_bytes = MODULE._read_sealed_bytes

    def read_then_replace(path, expected_sha256, mode):
        captured = read_bytes(path, expected_sha256, mode)
        if path == args.middle_contract:
            path.chmod(0o700)
            path.write_bytes(replacement)
            path.chmod(0o500)
        return captured

    monkeypatch.setattr(MODULE, "_read_sealed_bytes", read_then_replace)
    previous = {
        name: sys.modules.pop(name, None) for name in AUTHORITY_MODULE_NAMES
    }
    try:
        contract, _ = MODULE._load_semantic_authority(args)
        assert args.middle_contract.read_bytes() == replacement
        assert contract.VALUE == "GOOD"
    finally:
        _restore_authority_modules(previous)


def test_runtime_torso_watchdog_latches_violation():
    feedback = MODULE._JointFeedback()

    class Message:
        name = ["torso_joint"]
        position = [MODULE.HARD_MAX_TORSO_RAD + 1e-6]

    feedback.update(Message())
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="RUNTIME_TORSO_LIMIT"):
        feedback.require_torso_safe()


def test_joint_feedback_requires_every_requested_joint_to_be_fresh():
    feedback = MODULE._JointFeedback()

    class Initial:
        name = ["torso_joint", "left_shoulder_pitch_joint"]
        position = [0.0, 0.0]

    feedback.update(Initial())
    gate = __import__("time").monotonic_ns()

    class TorsoOnly:
        name = ["torso_joint"]
        position = [0.0]

    feedback.update(TorsoOnly())
    assert feedback.snapshot(
        ["torso_joint", "left_shoulder_pitch_joint"], gate) is None

    class LeftOnly:
        name = ["left_shoulder_pitch_joint"]
        position = [0.0]

    feedback.update(LeftOnly())
    assert feedback.snapshot(
        ["torso_joint", "left_shoulder_pitch_joint"], gate) is not None


def test_joint_feedback_watermark_requires_a_new_complete_joint_set(
        monkeypatch):
    receipt_ns = [100]
    monkeypatch.setattr(
        MODULE.time, "monotonic_ns", lambda: receipt_ns[0])
    feedback = MODULE._JointFeedback()
    names = ("torso_joint", "left_shoulder_pitch_joint")
    feedback.update(SimpleNamespace(name=list(names), position=[0.0, 0.0]))
    first = feedback.snapshot_with_receipt(names, 0)
    assert first is not None
    assert first[1] == 100

    receipt_ns[0] = 101
    feedback.update(SimpleNamespace(name=[names[0]], position=[0.0]))
    assert feedback.snapshot_with_receipt(names, first[1]) is None

    receipt_ns[0] = 102
    feedback.update(SimpleNamespace(name=[names[1]], position=[0.0]))
    second = feedback.snapshot_with_receipt(names, first[1])
    assert second is not None
    assert second[1] == 102

    receipt_ns[0] = 103
    feedback.update(SimpleNamespace(name=[names[0]], position=[0.0]))
    assert feedback.snapshot_with_receipt(names, second[1]) is None


def _endpoint_gate_harness(
        monkeypatch, samples, *, sim_step_ns=100_000_000,
        wall_step_s=0.01, publisher_mode="valid", arm_right_grasp=False):
    names = ("left_elbow_joint", "left_dex1_finger_1_joint")
    expected = {
        "left_elbow_joint": 1.0,
        "left_dex1_finger_1_joint": -0.019,
    }
    runtime = SimpleNamespace(
        sim_ns=1_000_000_000,
        wall_s=0.0,
        receipt_ns=100,
        samples=list(samples),
        spins=0,
        publisher_checks=0,
    )
    monkeypatch.setattr(
        MODULE.time, "monotonic", lambda: runtime.wall_s)
    monkeypatch.setattr(
        MODULE.time, "monotonic_ns", lambda: runtime.receipt_ns)

    feedback = MODULE._JointFeedback()
    initial = dict(expected)
    if arm_right_grasp:
        initial.update(zip(
            MODULE.RIGHT_GRIPPER_JOINTS, MODULE.E0_RIGHT_GRIPPER))
    feedback.update(SimpleNamespace(
        name=list(initial), position=list(initial.values())))
    if arm_right_grasp:
        feedback.arm_right_grasp_watchdog()

    class Clock:
        def now(self):
            return SimpleNamespace(nanoseconds=runtime.sim_ns)

    class Node:
        def __init__(self):
            self.clock = Clock()

        def get_clock(self):
            return self.clock

        def get_name(self):
            return "endpoint_gate_test"

        def get_publishers_info_by_topic(self, _topic):
            runtime.publisher_checks += 1
            if publisher_mode == "missing":
                return []
            owner = ("foreign" if publisher_mode == "foreign"
                     else self.get_name())
            return [SimpleNamespace(node_name=owner)]

    class Executor:
        def spin_once(self, *, timeout_sec):
            assert timeout_sec == pytest.approx(0.02)
            runtime.spins += 1
            runtime.wall_s += wall_step_s
            runtime.sim_ns += sim_step_ns
            if not runtime.samples:
                return
            values = runtime.samples.pop(0)
            if values is None:
                return
            runtime.receipt_ns += 1
            feedback.update(SimpleNamespace(
                name=list(values), position=list(values.values())))

    runtime.names = names
    runtime.expected = expected
    runtime.command_indices = {
        "left_elbow_joint": 1,
        "left_dex1_finger_1_joint": 2,
    }
    runtime.feedback = feedback
    runtime.node = Node()
    runtime.executor = Executor()
    return runtime


def _run_endpoint_gate(runtime, *, wall_deadline=10.0):
    MODULE._wait_endpoint_convergence(
        runtime.executor, runtime.node, runtime.feedback,
        runtime.names, runtime.expected, runtime.command_indices,
        "cube_carry_place", runtime.sim_ns, wall_deadline,
        runtime.sim_ns, runtime.wall_s)


def _endpoint_sample(runtime, elbow_error, finger_error=0.0):
    return {
        "left_elbow_joint": (
            runtime.expected["left_elbow_joint"] + elbow_error),
        "left_dex1_finger_1_joint": (
            runtime.expected["left_dex1_finger_1_joint"] + finger_error),
    }


def test_endpoint_gate_rejects_pre_target_cached_feedback(monkeypatch):
    runtime = _endpoint_gate_harness(monkeypatch, [None] * 8)
    with pytest.raises(
            MODULE.MiddleExecutionError,
            match=r"reason=SIM_SETTLE_TIMEOUT:.*fresh_samples=0$"):
        _run_endpoint_gate(runtime)
    assert runtime.spins == 6


def test_endpoint_gate_accepts_only_three_consecutive_fresh_samples(
        monkeypatch):
    runtime = _endpoint_gate_harness(monkeypatch, [])
    runtime.samples.extend([
        _endpoint_sample(runtime, 0.009),
        _endpoint_sample(runtime, 0.005),
        _endpoint_sample(runtime, 0.0035),
        _endpoint_sample(runtime, 0.0030),
        _endpoint_sample(runtime, 0.0020),
    ])
    _run_endpoint_gate(runtime)
    assert runtime.spins == 5
    assert runtime.publisher_checks == 6 * len(
        MODULE.CANONICAL_CONTROLLER_TOPICS)


def test_endpoint_gate_does_not_recount_one_fresh_in_band_sample(monkeypatch):
    runtime = _endpoint_gate_harness(monkeypatch, [])
    runtime.samples.extend([
        _endpoint_sample(runtime, 0.003),
        None, None, None, None, None,
    ])
    with pytest.raises(
            MODULE.MiddleExecutionError,
            match=r"consecutive=1:required=3:fresh_samples=1$"):
        _run_endpoint_gate(runtime)


def test_endpoint_gate_permanent_bias_reports_worst_joint_and_command(
        monkeypatch):
    runtime = _endpoint_gate_harness(monkeypatch, [])
    runtime.samples.extend(
        _endpoint_sample(runtime, 0.005) for _ in range(8))
    with pytest.raises(MODULE.MiddleExecutionError) as raised:
        _run_endpoint_gate(runtime)
    message = str(raised.value)
    assert "PHASE_ENDPOINT:cube_carry_place" in message
    assert "reason=SIM_SETTLE_TIMEOUT" in message
    assert "command_index=1" in message
    assert "worst_joint=left_elbow_joint" in message
    assert "measured=" in message
    assert "expected=" in message
    assert "abs_error=" in message
    assert "tolerance=" in message


def test_endpoint_gate_keeps_armed_right_grasp_watchdog_active(monkeypatch):
    runtime = _endpoint_gate_harness(
        monkeypatch, [], arm_right_grasp=True)
    sample = _endpoint_sample(runtime, 0.0)
    sample[MODULE.RIGHT_GRIPPER_JOINTS[0]] = (
        MODULE.E0_RIGHT_GRIPPER[0] +
        MODULE.HARD_GRIPPER_TOLERANCE_M + 1e-6)
    runtime.samples.append(sample)
    with pytest.raises(
            MODULE.MiddleExecutionError,
            match="RUNTIME_RIGHT_GRASP_VIOLATION"):
        _run_endpoint_gate(runtime)


def test_endpoint_gate_fails_closed_on_sim_clock_stall(monkeypatch):
    runtime = _endpoint_gate_harness(
        monkeypatch, [None] * 4, sim_step_ns=0, wall_step_s=0.6)
    with pytest.raises(
            MODULE.MiddleExecutionError,
            match="PHASE_SIM_CLOCK_STALL:cube_carry_place:ENDPOINT_SETTLE"):
        _run_endpoint_gate(runtime)


@pytest.mark.parametrize(
    ("publisher_mode", "failure"),
    (("missing", "PUBLISHER_COUNT"), ("foreign", "PUBLISHER_OWNER")),
)
def test_endpoint_gate_fails_closed_on_controller_publisher_change(
        monkeypatch, publisher_mode, failure):
    runtime = _endpoint_gate_harness(
        monkeypatch, [], publisher_mode=publisher_mode)
    with pytest.raises(MODULE.MiddleExecutionError, match=failure):
        _run_endpoint_gate(runtime)
    assert runtime.spins == 0


def test_endpoint_gate_fails_closed_on_wall_deadline(monkeypatch):
    runtime = _endpoint_gate_harness(
        monkeypatch, [None] * 4, sim_step_ns=0, wall_step_s=0.1)
    with pytest.raises(
            MODULE.MiddleExecutionError,
            match=r"reason=WALL_TIMEOUT:.*fresh_samples=0$"):
        _run_endpoint_gate(runtime, wall_deadline=0.05)


def test_right_grasp_watchdog_latches_single_message_excursion():
    feedback = MODULE._JointFeedback()
    feedback.arm_right_grasp_watchdog()

    class Message:
        name = [MODULE.RIGHT_GRIPPER_JOINTS[0]]
        position = [MODULE.E0_RIGHT_GRIPPER[0] +
                    MODULE.HARD_GRIPPER_TOLERANCE_M + 1e-6]

    feedback.update(Message())
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="RUNTIME_RIGHT_GRASP_VIOLATION"):
        feedback.require_torso_safe()


def test_right_grasp_watchdog_rejects_missing_finger_feedback():
    feedback = MODULE._JointFeedback()

    class FirstFingerOnly:
        name = [MODULE.RIGHT_GRIPPER_JOINTS[0]]
        position = [MODULE.E0_RIGHT_GRIPPER[0]]

    feedback.update(FirstFingerOnly())
    feedback.arm_right_grasp_watchdog()
    with pytest.raises(
            MODULE.MiddleExecutionError,
            match="RUNTIME_RIGHT_GRASP_FEEDBACK_MISSING:"
                  "right_dex1_finger_2_joint"):
        feedback.require_torso_safe()


def test_right_grasp_watchdog_rejects_stale_per_finger_feedback(monkeypatch):
    now_ns = [10_000_000_000]
    monkeypatch.setattr(
        MODULE.time, "monotonic_ns", lambda: now_ns[0])
    feedback = MODULE._JointFeedback()

    class Message:
        name = list(MODULE.RIGHT_GRIPPER_JOINTS)
        position = list(MODULE.E0_RIGHT_GRIPPER)

    feedback.update(Message())
    now_ns[0] += int(
        MODULE.HARD_RIGHT_GRASP_FEEDBACK_MAX_AGE_S * 1_000_000_000) + 1

    class FirstFingerOnly:
        name = [MODULE.RIGHT_GRIPPER_JOINTS[0]]
        position = [MODULE.E0_RIGHT_GRIPPER[0]]

    feedback.update(FirstFingerOnly())
    feedback.arm_right_grasp_watchdog()
    with pytest.raises(
            MODULE.MiddleExecutionError,
            match="RUNTIME_RIGHT_GRASP_FEEDBACK_STALE:"
                  "right_dex1_finger_2_joint"):
        feedback.require_torso_safe()


def test_right_grasp_watchdog_accepts_fresh_per_finger_feedback(monkeypatch):
    now_ns = [20_000_000_000]
    monkeypatch.setattr(
        MODULE.time, "monotonic_ns", lambda: now_ns[0])
    feedback = MODULE._JointFeedback()

    class Message:
        name = list(MODULE.RIGHT_GRIPPER_JOINTS)
        position = list(MODULE.E0_RIGHT_GRIPPER)

    feedback.update(Message())
    feedback.arm_right_grasp_watchdog()
    now_ns[0] += int(
        MODULE.HARD_RIGHT_GRASP_FEEDBACK_MAX_AGE_S * 1_000_000_000)
    feedback.require_torso_safe()


def test_e0_gate_is_full_nineteen_axis_middle_seam():
    expected = MODULE._e0_expected()
    assert len(expected) == 19
    assert expected["left_shoulder_roll_joint"] == 0.316
    assert expected["left_dex1_finger_1_joint"] == -0.019
    assert expected["right_dex1_finger_1_joint"] == 0.0108


def test_support_validate_timeout_kills_entire_process_group(monkeypatch):
    events = []

    class Process:
        pid = 4242
        returncode = None

        def communicate(self, timeout):
            raise MODULE.subprocess.TimeoutExpired("support", timeout)

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            self.returncode = -9
            return self.returncode

    def popen(*_args, **kwargs):
        assert kwargs["start_new_session"] is True
        return Process()

    monkeypatch.setattr(MODULE.subprocess, "Popen", popen)
    monkeypatch.setattr(
        MODULE.os, "killpg", lambda pid, signal: events.append((pid, signal)))
    args = SimpleNamespace(
        support_orchestrator=Path("/support"),
        support_manifest=Path("/manifest"),
        support_manifest_sha256="1" * 64,
        support_certificate=Path("/certificate"),
        support_certificate_sha256="2" * 64,
        workspace=Path("/workspace"),
        support_timeout=3.0,
    )
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="SUPPORT_VALIDATE_TIMEOUT"):
        MODULE._validate_support_contract_offline(args)
    assert events == [(4242, 9)]


def test_support_hook_has_separate_total_budget_and_preserves_cli_timeout(
        monkeypatch):
    kills = []
    commands = []
    watchdog = []

    class Process:
        pid = 4343
        returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            self.returncode = -9
            return self.returncode

    def popen(command, **kwargs):
        assert kwargs["start_new_session"] is True
        commands.append(command)
        return Process()

    monkeypatch.setattr(MODULE.subprocess, "Popen", popen)
    monkeypatch.setattr(
        MODULE.os, "killpg", lambda pid, signal: kills.append((pid, signal)))
    monkeypatch.setattr(
        MODULE, "_require_unique_controller_publishers",
        lambda _node, status: watchdog.append(("publishers", status)))
    executor = SimpleNamespace(
        spin_once=lambda timeout_sec: watchdog.append(
            ("spin", timeout_sec)))
    monotonic = iter((100.0, 399.0, 400.0))
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: next(monotonic))
    bundle = SimpleNamespace(
        support_orchestrator=Path("/support"),
        support_manifest=Path("/manifest"),
        support_manifest_sha256="1" * 64,
        support_certificate=Path("/certificate"),
        support_certificate_sha256="2" * 64,
        workspace=Path("/workspace"),
    )
    feedback = SimpleNamespace(
        require_torso_safe=lambda: watchdog.append(("torso", "checked")))
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="SUPPORT_TIMEOUT:door_attach"):
        MODULE._run_support_hook(
            SimpleNamespace(), executor, feedback, bundle,
            "door_attach", 17.25)
    assert kills == [(4343, 9)]
    assert watchdog == [
        ("publishers", "SUPPORT_PRE:door_attach"),
        ("spin", 0.02),
        ("torso", "checked"),
        ("publishers", "SUPPORT_ACTIVE:door_attach"),
    ]
    assert commands == [[
        "/support", "door-attach",
        "--manifest", "/manifest",
        "--manifest-sha256", "1" * 64,
        "--certificate", "/certificate",
        "--certificate-sha256", "2" * 64,
        "--workspace", "/workspace",
        "--timeout", "17.25",
    ]]
    assert MODULE.SUPPORT_PROCESS_WALL_TIMEOUT_S == 300.0
    assert (MODULE.SUPPORT_PROCESS_WALL_TIMEOUT_S >
            MODULE.HARD_MAX_SUPPORT_TIMEOUT_S)


@pytest.mark.parametrize("timeout", [2.99, 30.01])
def test_support_timeout_outside_orchestrator_contract_is_rejected(
        monkeypatch, timeout):
    reached = []
    monkeypatch.setattr(
        MODULE, "validate_bundle",
        lambda _args: reached.append(True))
    digest = "0" * 64
    arguments = [
        "--workspace", "/workspace", "--scene", "/scene",
        "--scene-sha256", digest, "--plan", "/plan",
        "--plan-sha256", digest, "--certificate", "/certificate",
        "--certificate-sha256", digest, "--result", "/result",
        "--result-sha256", digest,
        "--middle-admission", "/admission",
        "--middle-admission-sha256", digest,
        "--middle-contract", "/contract",
        "--middle-contract-sha256", digest,
        "--support-orchestrator", "/orchestrator",
        "--support-orchestrator-sha256", digest,
        "--support-manifest", "/support-manifest",
        "--support-manifest-sha256", digest,
        "--support-certificate", "/support-certificate",
        "--support-certificate-sha256", digest,
        "--support-timeout", str(timeout),
    ]
    assert MODULE.main(arguments) == 1
    assert reached == []


def test_source_publishes_only_canonical_topics_and_zero_derivatives():
    source = SOURCE.read_text(encoding="utf-8")
    for topic in MODULE.CANONICAL_CONTROLLER_TOPICS:
        assert topic in source
    assert "/h1_2/right_door_support" not in source
    assert "/h1_2/left_pose_grasp" not in source
    assert "point.velocities = [0.0]" in source
    assert "point.accelerations = [0.0]" in source
    assert "point.effort = []" in source
    assert source.index("bundle = validate_bundle(args)") < source.rindex(
        "_execute_ros(")
    assert "start_new_session=True" in source
    assert "os.killpg(process.pid, 9)" in source
    assert "PHASE_SIM_CLOCK_STALL" in source
    assert "feedback.latest(names)" in source
    assert "HARD_ENDPOINT_TOLERANCE_RAD = 0.004" in source
    assert "HARD_GRIPPER_TOLERANCE_M = 0.001" in source
    assert "HARD_ENDPOINT_SETTLE_SIM_NS = 500_000_000" in source
    assert "HARD_ENDPOINT_CONSECUTIVE_SAMPLES = 3" in source
    tree = ast.parse(source)
    endpoint_gate = next(
        node for node in tree.body
        if (isinstance(node, ast.FunctionDef) and
            node.name == "_wait_endpoint_convergence"))
    endpoint_source = ast.get_source_segment(source, endpoint_gate)
    assert ".publish(" not in endpoint_source


def test_controller_and_recorder_gate_requires_two_subscriptions():
    class Publisher:
        def __init__(self, count):
            self.count = count

        def get_subscription_count(self):
            return self.count

    assert MODULE.REQUIRED_CONTROLLER_AND_RECORDER_SUBSCRIPTIONS == 2
    assert not MODULE._controller_and_recorder_ready({
        "controller-only": Publisher(1), "ready": Publisher(2)})
    assert MODULE._controller_and_recorder_ready({
        "left": Publisher(2), "right": Publisher(2)})


def test_source_all_spin_paths_use_one_explicit_custom_context_executor():
    source = SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    spin_calls = [
        node for node in ast.walk(tree)
        if (isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "spin_once")
    ]
    assert len(spin_calls) == 7
    assert all(
        isinstance(call.func.value, ast.Name)
        and call.func.value.id == "executor"
        for call in spin_calls)
    assert "rclpy.spin_once" not in source
    assert source.count("SingleThreadedExecutor(context=context)") == 1
    assert source.count("executor.add_node(node)") == 1
    assert source.count("executor.remove_node(node)") == 1
    assert source.count("executor.shutdown()") == 1
    wait_function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_wait_feedback")
    assert wait_function.args.args[0].arg == "executor"
    support_function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_run_support_hook")
    assert support_function.args.args[1].arg == "executor"
    execute_function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_execute_ros")
    hold_function = next(
        node for node in ast.walk(execute_function)
        if isinstance(node, ast.FunctionDef) and node.name == "hold_current")
    hold_source = ast.get_source_segment(source, hold_function)
    assert "_wait_feedback(\n                executor" in hold_source
    assert "executor.spin_once(timeout_sec=0.02)" in hold_source


def _install_executor_aware_ros_mock(
        monkeypatch, *, fail_first_spin=False, recorder_ready_at_spin=0):
    runtime = SimpleNamespace(
        state=MODULE._e0_expected(),
        clock_ns=0,
        contexts=[],
        executors=[],
        nodes=[],
        publications=[],
        subscription_samples=[],
        controller_only_publication_counts=[],
        publication_subscription_counts=[],
        default_spin_calls=0,
        fail_first_spin=fail_first_spin,
        recorder_ready_at_spin=recorder_ready_at_spin,
    )

    class Context:
        def __init__(self):
            self.initialized = False
            self.shutdown_called = False
            runtime.contexts.append(self)

    class JointState:
        pass

    class _Clock:
        ros_time_is_active = True

        def now(self):
            runtime.clock_ns += 1_000_000_000_000
            return SimpleNamespace(nanoseconds=runtime.clock_ns)

    class _Publisher:
        def __init__(self, node, topic):
            self.node = node
            self.topic = topic
            self.destroyed = False

        def get_subscription_count(self):
            owner = next(
                executor for executor in runtime.executors
                if executor.node is self.node)
            count = (2 if owner.spin_calls >= runtime.recorder_ready_at_spin
                     else 1)
            runtime.subscription_samples.append(
                (owner.spin_calls, self.topic, count))
            return count

        def publish(self, message):
            assert not self.destroyed
            owner = next(
                executor for executor in runtime.executors
                if executor.node is self.node)
            count = (2 if owner.spin_calls >= runtime.recorder_ready_at_spin
                     else 1)
            runtime.publication_subscription_counts.append(count)
            runtime.publications.append((owner, self.topic, message))
            point = message.points[-1]
            runtime.state.update(zip(message.joint_names, point.positions))

    class Node:
        def __init__(self, name, *, parameter_overrides,
                     automatically_declare_parameters_from_overrides,
                     context):
            assert parameter_overrides
            assert automatically_declare_parameters_from_overrides is True
            assert context.initialized
            self.context = context
            self.name = name
            self.clock = _Clock()
            self.feedback_callback = None
            self.publishers = {}
            self.destroyed = False
            runtime.nodes.append(self)

        def get_name(self):
            return self.name

        def get_clock(self):
            return self.clock

        def create_subscription(self, _message_type, topic, callback, depth):
            assert topic == "/joint_states"
            assert depth == 10
            self.feedback_callback = callback
            return SimpleNamespace(callback=callback)

        def destroy_subscription(self, subscription):
            assert subscription.callback is self.feedback_callback
            self.feedback_callback = None

        def create_publisher(self, _message_type, topic, _qos):
            publisher = _Publisher(self, topic)
            self.publishers[topic] = publisher
            return publisher

        def destroy_publisher(self, publisher):
            assert self.publishers[publisher.topic] is publisher
            publisher.destroyed = True
            del self.publishers[publisher.topic]

        def get_publishers_info_by_topic(self, topic):
            if topic not in self.publishers:
                return []
            return [SimpleNamespace(node_name=self.name)]

        def destroy_node(self):
            self.destroyed = True

    class SingleThreadedExecutor:
        def __init__(self, *, context):
            assert context.initialized
            self.context = context
            self.node = None
            self.added_node = None
            self.removed_node = None
            self.spin_calls = 0
            self.shutdown_called = False
            runtime.executors.append(self)

        def add_node(self, node):
            assert self.node is None
            assert node.context is self.context
            self.node = node
            self.added_node = node
            return True

        def remove_node(self, node):
            assert self.node is node
            self.removed_node = node
            self.node = None

        def spin_once(self, *, timeout_sec):
            assert timeout_sec == pytest.approx(0.02)
            assert self.context.initialized
            assert self.node is not None
            self.spin_calls += 1
            if self.spin_calls < runtime.recorder_ready_at_spin:
                runtime.controller_only_publication_counts.append(
                    len(runtime.publications))
            if runtime.fail_first_spin:
                runtime.fail_first_spin = False
                raise MODULE.MiddleExecutionError("FAKE_PRIMARY_FAILURE")
            if self.node.feedback_callback is not None:
                message = JointState()
                message.name = list(runtime.state)
                message.position = [
                    runtime.state[name] for name in message.name]
                self.node.feedback_callback(message)

        def shutdown(self):
            assert self.node is None
            self.shutdown_called = True
            return True

    class Parameter:
        class Type:
            BOOL = "bool"

        def __init__(self, name, parameter_type, value):
            self.name = name
            self.parameter_type = parameter_type
            self.value = value

    class QoSProfile:
        def __init__(self, **kwargs):
            self.settings = kwargs

    class ReliabilityPolicy:
        RELIABLE = "reliable"

    class DurabilityPolicy:
        VOLATILE = "volatile"

    class SignalHandlerOptions:
        NO = "no"

    class _Duration:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    class JointTrajectoryPoint:
        def __init__(self):
            self.positions = []
            self.velocities = []
            self.accelerations = []
            self.effort = []
            self.time_from_start = _Duration()

    class JointTrajectory:
        def __init__(self):
            self.joint_names = []
            self.points = []

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.__path__ = []

    def init(*, context, signal_handler_options):
        assert signal_handler_options == SignalHandlerOptions.NO
        assert not context.initialized
        context.initialized = True

    def shutdown(*, context):
        assert context.initialized
        context.initialized = False
        context.shutdown_called = True

    def forbidden_default_spin(*_args, **_kwargs):
        runtime.default_spin_calls += 1
        raise AssertionError("default/global rclpy.spin_once was used")

    rclpy_module.init = init
    rclpy_module.shutdown = shutdown
    rclpy_module.spin_once = forbidden_default_spin
    modules = {
        "rclpy": rclpy_module,
        "rclpy.context": types.ModuleType("rclpy.context"),
        "rclpy.executors": types.ModuleType("rclpy.executors"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.parameter": types.ModuleType("rclpy.parameter"),
        "rclpy.qos": types.ModuleType("rclpy.qos"),
        "rclpy.signals": types.ModuleType("rclpy.signals"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "trajectory_msgs": types.ModuleType("trajectory_msgs"),
        "trajectory_msgs.msg": types.ModuleType("trajectory_msgs.msg"),
    }
    modules["rclpy.context"].Context = Context
    modules["rclpy.executors"].SingleThreadedExecutor = SingleThreadedExecutor
    modules["rclpy.node"].Node = Node
    modules["rclpy.parameter"].Parameter = Parameter
    modules["rclpy.qos"].QoSProfile = QoSProfile
    modules["rclpy.qos"].ReliabilityPolicy = ReliabilityPolicy
    modules["rclpy.qos"].DurabilityPolicy = DurabilityPolicy
    modules["rclpy.signals"].SignalHandlerOptions = SignalHandlerOptions
    modules["sensor_msgs"].__path__ = []
    modules["sensor_msgs"].msg = modules["sensor_msgs.msg"]
    modules["sensor_msgs.msg"].JointState = JointState
    modules["trajectory_msgs"].__path__ = []
    modules["trajectory_msgs"].msg = modules["trajectory_msgs.msg"]
    modules["trajectory_msgs.msg"].JointTrajectory = JointTrajectory
    modules["trajectory_msgs.msg"].JointTrajectoryPoint = (
        JointTrajectoryPoint)
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    return runtime


def _serialized_trajectory_publications(runtime):
    return [
        {
            "topic": topic,
            "joint_names": list(message.joint_names),
            "points": [
                {
                    "positions": list(point.positions),
                    "velocities": list(point.velocities),
                    "accelerations": list(point.accelerations),
                    "effort": list(point.effort),
                    "time_from_start": [
                        point.time_from_start.sec,
                        point.time_from_start.nanosec,
                    ],
                }
                for point in message.points
            ],
        }
        for _owner, topic, message in runtime.publications
    ]


def _trajectory_sequence_sha256(payloads):
    encoded = json.dumps(
        payloads, sort_keys=True, separators=(",", ":"),
        allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _expected_middle_trajectory_publications(bundle, speed_scale):
    payloads = []
    for phase in bundle.phases:
        for command in phase.commands:
            points = []
            for sealed_point in command.points:
                scaled_ns = int(round(
                    sealed_point.time_from_start_ns / speed_scale))
                scheduled_ns = int(
                    MODULE.HARD_MIN_LEAD_TIME_S * 1e9) + scaled_ns
                sec, nanosec = MODULE._duration_parts(scheduled_ns)
                points.append({
                    "positions": list(sealed_point.positions),
                    "velocities": [0.0] * len(command.joint_names),
                    "accelerations": [0.0] * len(command.joint_names),
                    "effort": [],
                    "time_from_start": [sec, nanosec],
                })
            payloads.append({
                "topic": command.topic,
                "joint_names": list(command.joint_names),
                "points": points,
            })
    return payloads


def _full_e0_middle_command_bundle():
    state = MODULE._e0_expected()
    phases = []
    for index, (name, topics) in enumerate(zip(
            MODULE.PHASE_NAMES, MODULE.PHASE_COMMAND_TOPICS)):
        commands = tuple(
            MODULE.Command(
                topic,
                MODULE.CONTROLLER_JOINTS[topic],
                (MODULE.Point(tuple(
                    state[joint]
                    for joint in MODULE.CONTROLLER_JOINTS[topic]), 0),),
            )
            for topic in topics
        )
        phases.append(MODULE.Phase(index, name, commands))
    return SimpleNamespace(phases=tuple(phases))


def test_controller_only_never_publishes_and_recorder_join_preserves_sequence(
        monkeypatch):
    bundle = _full_e0_middle_command_bundle()
    expected = _expected_middle_trajectory_publications(bundle, 0.5)

    def support_hook(_node, _executor, _feedback, _bundle, _action, _timeout):
        return None

    def endpoint_gate(
            _executor, _node, feedback, names, expected_end,
            _command_indices, _phase_name, _sim_target, _wall_deadline,
            _last_clock, _last_clock_wall):
        measured = feedback.latest(names)
        assert measured is not None
        assert measured == pytest.approx(expected_end)

    monkeypatch.setattr(MODULE, "_run_support_hook", support_hook)
    monkeypatch.setattr(MODULE, "_wait_endpoint_convergence", endpoint_gate)

    immediate = _install_executor_aware_ros_mock(
        monkeypatch, recorder_ready_at_spin=0)
    MODULE._execute_ros(bundle, 0.5, 1.0, 1.0, 10.0)
    immediate_payloads = _serialized_trajectory_publications(immediate)

    delayed = _install_executor_aware_ros_mock(
        monkeypatch, recorder_ready_at_spin=4)
    MODULE._execute_ros(bundle, 0.5, 1.0, 1.0, 10.0)
    delayed_payloads = _serialized_trajectory_publications(delayed)

    assert delayed.controller_only_publication_counts == [0, 0, 0]
    assert any(count == 1 for _spin, _topic, count
               in delayed.subscription_samples)
    assert any(count == 2 for _spin, _topic, count
               in delayed.subscription_samples)
    assert delayed.publication_subscription_counts
    assert set(delayed.publication_subscription_counts) == {2}

    assert delayed_payloads[0] == expected[0]
    assert immediate_payloads == expected
    assert delayed_payloads == expected
    expected_topics = [
        command.topic
        for phase in bundle.phases
        for command in phase.commands
    ]
    assert [payload["topic"] for payload in delayed_payloads] == expected_topics
    assert len(delayed_payloads) == sum(
        len(phase.commands) for phase in bundle.phases)
    assert (_trajectory_sequence_sha256(delayed_payloads) ==
            _trajectory_sequence_sha256(immediate_payloads) ==
            _trajectory_sequence_sha256(expected))


def _empty_middle_bundle():
    return SimpleNamespace(phases=tuple(
        MODULE.Phase(index, name, ())
        for index, name in enumerate(MODULE.PHASE_NAMES)))


def _phase5_two_command_bundle():
    middle = list(MODULE.E0_LEFT_ARM_RAD)
    middle[MODULE.LEFT_ARM_JOINTS.index("left_elbow_joint")] += 0.01
    command0 = MODULE.Command(
        "/left_arm_controller/joint_trajectory",
        MODULE.LEFT_ARM_JOINTS,
        (
            MODULE.Point(tuple(MODULE.E0_LEFT_ARM_RAD), 0),
            MODULE.Point(tuple(middle), 100_000_000),
        ),
    )
    command1 = MODULE.Command(
        "/left_arm_controller/joint_trajectory",
        MODULE.LEFT_ARM_JOINTS,
        (
            MODULE.Point(tuple(middle), 0),
            MODULE.Point(tuple(MODULE.E0_LEFT_ARM_RAD), 100_000_000),
        ),
    )
    phases = []
    for index, name in enumerate(MODULE.PHASE_NAMES):
        commands = (command0, command1) if index == 5 else ()
        phases.append(MODULE.Phase(index, name, commands))
    return SimpleNamespace(phases=tuple(phases)), tuple(middle)


def test_execute_ros_uses_one_custom_executor_for_full_middle_lifecycle(
        monkeypatch):
    runtime = _install_executor_aware_ros_mock(monkeypatch)
    support_calls = []

    def support_hook(node, executor, _feedback, _bundle, action, timeout):
        assert executor.node is node
        support_calls.append((executor, action, timeout))

    monkeypatch.setattr(MODULE, "_run_support_hook", support_hook)
    MODULE._execute_ros(_empty_middle_bundle(), 0.5, 0.1, 1.0, 10.0)

    assert runtime.default_spin_calls == 0
    assert len(runtime.contexts) == 1
    assert len(runtime.executors) == 1
    assert len(runtime.nodes) == 1
    context = runtime.contexts[0]
    executor = runtime.executors[0]
    node = runtime.nodes[0]
    assert executor.context is context
    assert executor.added_node is node
    assert executor.removed_node is node
    assert executor.node is None
    assert executor.spin_calls > 0
    assert executor.shutdown_called
    assert node.context is context
    assert node.destroyed
    assert context.shutdown_called
    assert not context.initialized
    assert [(action, timeout) for _executor, action, timeout
            in support_calls] == [
        ("door_attach", 10.0),
        ("cube_attach", 10.0),
        ("cube_detach", 10.0),
        ("door_detach", 10.0),
    ]
    assert all(owner is executor for owner, _action, _timeout
               in support_calls)


def test_execute_ros_phase5_preserves_command_indices_and_detach_order(
        monkeypatch):
    runtime = _install_executor_aware_ros_mock(monkeypatch)
    bundle, middle = _phase5_two_command_bundle()
    events = []

    def endpoint_gate(
            executor, node, feedback, names, expected, command_indices,
            phase_name, _sim_target, _wall_deadline, _last_clock,
            _last_clock_wall):
        assert executor.node is node
        assert phase_name == "cube_carry_place"
        elbow_index = command_indices["left_elbow_joint"]
        measured = feedback.latest(names)
        assert measured is not None
        assert measured["left_elbow_joint"] == pytest.approx(
            expected["left_elbow_joint"])
        events.append(("endpoint", elbow_index, len(runtime.publications)))

    def support_hook(
            node, executor, _feedback, _bundle, action, _timeout):
        assert executor.node is node
        events.append(("support", action, len(runtime.publications)))

    monkeypatch.setattr(
        MODULE, "_wait_endpoint_convergence", endpoint_gate)
    monkeypatch.setattr(MODULE, "_run_support_hook", support_hook)
    MODULE._execute_ros(bundle, 0.5, 0.1, 1.0, 10.0)

    assert len(runtime.publications) == 2
    elbow_position = MODULE.LEFT_ARM_JOINTS.index("left_elbow_joint")
    assert [topic for _owner, topic, _message in runtime.publications] == [
        "/left_arm_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
    ]
    assert runtime.publications[0][2].points[-1].positions[
        elbow_position] == pytest.approx(middle[elbow_position])
    assert runtime.publications[1][2].points[-1].positions[
        elbow_position] == pytest.approx(
            MODULE.E0_LEFT_ARM_RAD[elbow_position])
    endpoint_events = [event for event in events if event[0] == "endpoint"]
    assert endpoint_events == [
        ("endpoint", 0, 1),
        ("endpoint", 1, 2),
    ]
    cube_detach_index = events.index(("support", "cube_detach", 2))
    assert events.index(endpoint_events[0]) < events.index(endpoint_events[1])
    assert events.index(endpoint_events[1]) < cube_detach_index


def test_middle_failure_hold_uses_same_custom_executor(
        monkeypatch, capsys):
    runtime = _install_executor_aware_ros_mock(
        monkeypatch, fail_first_spin=True)
    tick = [0.0]

    def monotonic():
        tick[0] += 0.05
        return tick[0]

    monkeypatch.setattr(MODULE.time, "monotonic", monotonic)
    with pytest.raises(MODULE.MiddleExecutionError,
                       match="FAKE_PRIMARY_FAILURE"):
        MODULE._execute_ros(
            _empty_middle_bundle(), 0.5, 0.1, 1.0, 10.0)

    output = capsys.readouterr()
    assert "CANDIDATE3_MIDDLE_828_FAILURE_HOLD=PASS" in output.out
    assert output.err == ""
    assert runtime.default_spin_calls == 0
    assert len(runtime.executors) == 1
    executor = runtime.executors[0]
    assert executor.spin_calls >= 2
    assert len(runtime.publications) == len(MODULE.CANONICAL_CONTROLLER_TOPICS)
    assert all(owner is executor for owner, _topic, _message
               in runtime.publications)
    assert executor.shutdown_called
    assert executor.removed_node is executor.added_node
    assert executor.added_node.destroyed
    assert runtime.contexts[0].shutdown_called
