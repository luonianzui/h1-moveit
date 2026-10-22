"""ROS-free tests for the deterministic 831 C2/return plan builder."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = WORKSPACE / "runtime_assets/build_candidate3_return_c2_831.py"
SOURCE_PLAN = (
    WORKSPACE / "runtime_assets/candidate3_middle_e0_828/"
    "candidate3_middle_plan_828.json")
PROVENANCE = (
    WORKSPACE / "runtime_assets/candidate3_middle_e0_828/"
    "scene_model_provenance_828.json")
CHECKED_PLAN = (
    WORKSPACE / "runtime_assets/candidate3_return_c2_831/"
    "candidate3_return_plan_831.json")

SPEC = importlib.util.spec_from_file_location(
    "build_candidate3_return_c2_831_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture(scope="module")
def source_plan():
    return _load(SOURCE_PLAN)


@pytest.fixture(scope="module")
def return_plan():
    return MODULE.build_return_plan()


def _phases_by_name(plan):
    return {phase["name"]: phase for phase in plan["phases"]}


def test_checked_asset_is_exact_deterministic_builder_output(return_plan):
    assert CHECKED_PLAN.read_bytes() == MODULE._canonical(return_plan)
    assert _sha256(CHECKED_PLAN) == (
        "24767afddc9101094a419d9ab00c04db352e344b1aae90b287d86ca4773ebb21")
    assert _sha256(SOURCE_PLAN) == MODULE.EXPECTED_SOURCE_SHA256
    assert return_plan["source_binding"]["middle_plan"] == {
        "path": MODULE.SOURCE_RELATIVE_PATH.as_posix(),
        "file_sha256": MODULE.EXPECTED_SOURCE_SHA256,
        "schema": "h1_candidate3_middle_plan/828-e0-v1",
    }


def test_builder_reads_but_does_not_modify_sealed_inputs(return_plan):
    before = {
        path: (path.stat().st_mode & 0o777, _sha256(path))
        for path in (SOURCE_PLAN, PROVENANCE)
    }
    assert MODULE.build_return_plan() == return_plan
    after = {
        path: (path.stat().st_mode & 0o777, _sha256(path))
        for path in (SOURCE_PLAN, PROVENANCE)
    }
    assert before == after
    assert before[SOURCE_PLAN][0] == 0o400
    assert before[PROVENANCE][0] == 0o400


def test_c2_return_names_are_semantically_explicit(return_plan):
    expected = [spec["name"] for spec in MODULE.PHASE_SPECS]
    assert return_plan["schema"] == "h1_candidate3_return_plan/831-c2-v1"
    assert return_plan["cycle_id"] == "C2"
    assert return_plan["operation"] == "return"
    assert return_plan["stage_namespace"] == "C2/return"
    assert return_plan["phase_names"] == expected
    assert [phase["name"] for phase in return_plan["phases"]] == expected
    assert all(name.startswith("c2_return_") for name in expected)
    assert "cube_contact" not in expected
    assert "cube_release" not in expected
    assert all(phase["stage"].startswith("C2/return/")
               for phase in return_plan["phases"])


def test_every_derived_trajectory_is_an_exact_pointwise_transform(
        source_plan, return_plan):
    source_phases = _phases_by_name(source_plan)
    reverse_count = 0
    for phase in return_plan["phases"]:
        assert len(phase["commands"]) == len(phase["command_lineage"])
        for command, lineage in zip(
                phase["commands"], phase["command_lineage"]):
            source_command = source_phases[
                lineage["source_phase"]]["commands"][
                    lineage["source_command_index"]]
            assert lineage["source_command_sha256"] == (
                MODULE._canonical_sha256(source_command))
            assert lineage["derived_command_sha256"] == (
                MODULE._canonical_sha256(command))
            if lineage["transform"] == MODULE.TRANSFORM_COPY:
                assert command == source_command
                continue
            assert lineage["transform"] == MODULE.TRANSFORM_REVERSE
            reverse_count += 1
            assert command["topic"] == source_command["topic"]
            assert command["joint_names"] == source_command["joint_names"]
            source_points = source_command["points"]
            target_points = command["points"]
            assert len(target_points) == len(source_points)
            first_time = source_points[0]["time_from_start_ns"]
            last_time = source_points[-1]["time_from_start_ns"]
            for target, source in zip(target_points, reversed(source_points)):
                assert target["positions"] == source["positions"]
                assert {
                    key: value for key, value in target.items()
                    if key != "time_from_start_ns"
                } == {
                    key: value for key, value in source.items()
                    if key != "time_from_start_ns"
                }
                assert target["time_from_start_ns"] == (
                    first_time + last_time - source["time_from_start_ns"])
    assert reverse_count == 5


def test_left_hand_order_and_two_segment_carry_are_exact(return_plan):
    phases = _phases_by_name(return_plan)
    left_order = [
        "c2_return_left_ready",
        "c2_return_enter_inside",
        "c2_return_grasp_inside",
        "c2_return_carry_to_origin",
        "c2_return_release_at_origin",
        "c2_return_retreat_from_origin",
        "c2_return_left_park",
    ]
    indices = [return_plan["phase_names"].index(name) for name in left_order]
    assert indices == sorted(indices)
    assert [row["source_phase"] for row in
            phases["c2_return_enter_inside"]["command_lineage"]] == [
                "cube_retreat"]
    assert [(row["source_phase"], row["source_command_index"])
            for row in phases[
                "c2_return_carry_to_origin"]["command_lineage"]] == [
                    ("cube_carry_place", 1),
                    ("cube_carry_place", 0),
                ]
    assert [row["source_phase"] for row in
            phases["c2_return_retreat_from_origin"]["command_lineage"]] == [
                "cube_contact"]
    assert phases["c2_return_grasp_inside"]["commands"][0][
        "points"][-1]["positions"] == [-0.014942, -0.014942]
    assert phases["c2_return_release_at_origin"]["commands"][0][
        "points"][-1]["positions"] == [-0.019, -0.019]


def test_all_phase_and_command_endpoints_are_continuous(return_plan):
    joint_names = {
        topic: value["joint_names"]
        for topic, value in return_plan["initial_controller_state"].items()
    }
    positions = {
        topic: list(value["positions"])
        for topic, value in return_plan["initial_controller_state"].items()
    }
    cursor = 0
    for phase, continuity in zip(
            return_plan["phases"],
            return_plan["timing"]["first_point_continuity"]):
        state = MODULE._state_document(joint_names, positions)
        assert phase["entry_state_sha256"] == MODULE._canonical_sha256(state)
        assert phase["message_range"][0] == cursor
        measured = 0.0
        for command in phase["commands"]:
            topic = command["topic"]
            assert command["joint_names"] == joint_names[topic]
            measured = max(
                measured,
                MODULE._maximum_difference(
                    positions[topic], command["points"][0]["positions"]))
            assert measured <= MODULE.CONTINUITY_LIMIT_RAD_OR_M
            positions[topic] = list(command["points"][-1]["positions"])
            cursor += 1
        assert continuity["phase"] == phase["name"]
        assert continuity["maximum_first_point_error"] == measured
        state = MODULE._state_document(joint_names, positions)
        assert phase["exit_state_sha256"] == MODULE._canonical_sha256(state)
        assert phase["message_range"][1] == cursor
    assert cursor == 16
    terminal = return_plan["terminal_contract"]
    assert terminal["final_controller_state"] == (
        MODULE._state_document(joint_names, positions))
    assert terminal["final_controller_state_sha256"] == (
        MODULE._canonical_sha256(terminal["final_controller_state"]))


def test_message_accounting_is_16_30_and_60(return_plan):
    middle_messages = sum(
        len(phase["commands"]) for phase in return_plan["phases"])
    assert middle_messages == 16
    assert return_plan["message_contract"] == {
        "return_middle": 16,
        "right_handle_per_cycle": 14,
        "return_cycle_total": 30,
        "two_cycle_total": 60,
    }
    assert 16 + 14 == 30
    assert 30 + 30 == 60


def test_cube_support_attaches_inside_and_detaches_at_origin(
        return_plan):
    provenance = _load(PROVENANCE)
    poses = return_plan["cube_pose_contract"]
    assert poses["inside"]["pose"] == provenance["cube_poses"]["release"]
    assert poses["origin"]["pose"] == provenance["cube_poses"]["contact"]
    assert poses["inside"]["pose"]["position"] == [0.611, -0.03, 0.995]
    assert poses["origin"]["pose"]["position"] == [0.334, 0.377, 0.83]
    hooks = {hook["semantic_event"]: hook for hook in return_plan["hooks"]}
    attach = hooks["C2_RETURN_CUBE_ATTACH_INSIDE"]
    detach = hooks["C2_RETURN_CUBE_DETACH_ORIGIN"]
    assert attach["action"] == "cube_attach"
    assert attach["location"] == "inside"
    assert attach["boundary"] == {
        "kind": "after_phase",
        "phase_index": 5,
        "phase_name": "c2_return_grasp_inside",
    }
    assert detach["action"] == "cube_detach"
    assert detach["location"] == "origin"
    assert detach["boundary"] == {
        "kind": "after_phase",
        "phase_index": 6,
        "phase_name": "c2_return_carry_to_origin",
    }
    assert poses["attach_event"] == attach["semantic_event"]
    assert poses["detach_event"] == detach["semantic_event"]
    assert poses["final_pose"] == "origin"


def test_partial_execution_is_forbidden_and_tampering_fails_closed(
        tmp_path, return_plan):
    assert return_plan["path_contract"] == {
        "candidate3_only": True,
        "contains_partial": False,
        "partial_execution_allowed": False,
        "all_segments_required": True,
        "inverse_transform": "reverse_points_mirror_time",
    }
    assert return_plan["authorization"] == {
        "status": "PLAN_ONLY",
        "execution_authorized": False,
        "requires_fresh_runtime_audit": True,
    }
    tampered = tmp_path / SOURCE_PLAN.name
    shutil.copyfile(SOURCE_PLAN, tampered)
    value = _load(tampered)
    value["offline_only"] = False
    tampered.write_text(json.dumps(value) + "\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="INPUT_SHA256"):
        MODULE.build_return_plan(tampered, WORKSPACE)

