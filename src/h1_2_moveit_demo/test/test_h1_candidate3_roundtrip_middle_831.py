"""Contracts for the 831 rack-to-origin second-cycle middle."""

import importlib.util
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = (
    WORKSPACE /
    "src/h1_2_moveit_demo/scripts/"
    "h1_candidate3_middle_executor_fast_831_v3.py"
)
PLAN = (
    WORKSPACE /
    "runtime_assets/candidate3_middle_e0_828/"
    "candidate3_middle_plan_828.json"
)
SPEC = importlib.util.spec_from_file_location(
    "h1_candidate3_roundtrip_middle_831_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _source_bundle():
    value = json.loads(PLAN.read_text(encoding="utf-8"))
    phases = []
    for index, (phase, expected_topics) in enumerate(zip(
            value["phases"], MODULE.PHASE_COMMAND_TOPICS)):
        commands = tuple(
            MODULE._validate_command(
                command, topic, f"TEST_SOURCE:{index}:{command_index}")
            for command_index, (command, topic) in enumerate(
                zip(phase["commands"], expected_topics))
        )
        phases.append(MODULE.Phase(index, phase["name"], commands))
    return SimpleNamespace(phases=tuple(phases))


def _assert_exact_reverse(actual, source):
    assert actual.topic == source.topic
    assert actual.joint_names == source.joint_names
    assert tuple(point.positions for point in actual.points) == tuple(
        point.positions for point in reversed(source.points))
    start_ns = source.points[0].time_from_start_ns
    duration_ns = source.points[-1].time_from_start_ns
    assert tuple(point.time_from_start_ns for point in actual.points) == tuple(
        start_ns + duration_ns - point.time_from_start_ns
        for point in reversed(source.points))


def test_return_middle_is_the_explicit_sixteen_message_topology():
    source = _source_bundle().phases
    result = MODULE.build_return_phases(SimpleNamespace(phases=source))
    assert tuple(phase.name for phase in result) == MODULE.RETURN_PHASE_NAMES
    assert len(result) == 12
    assert sum(len(phase.commands) for phase in result) == 16
    assert all(
        command.topic != "/right_gripper_controller/joint_trajectory"
        for phase in result for command in phase.commands)


def test_cube_return_is_pointwise_reverse_with_explicit_grasp_semantics():
    source = _source_bundle().phases
    result = MODULE.build_return_phases(SimpleNamespace(phases=source))

    _assert_exact_reverse(result[4].commands[0], source[7].commands[0])
    _assert_exact_reverse(result[5].commands[0], source[6].commands[0])
    _assert_exact_reverse(result[6].commands[0], source[5].commands[1])
    _assert_exact_reverse(result[6].commands[1], source[5].commands[0])
    _assert_exact_reverse(result[8].commands[0], source[4].commands[0])

    # The left hand arrives open, closes at the old rack-release endpoint,
    # carries the attached cube back, then opens at the old origin-contact.
    close = result[5].commands[0]
    release = result[7].commands[0]
    assert close.points[0].positions == source[6].commands[0].points[-1].positions
    assert close.points[-1].positions == source[6].commands[0].points[0].positions
    assert release.points[0].positions == source[6].commands[0].points[0].positions
    assert release.points[-1].positions == source[6].commands[0].points[-1].positions


def test_return_profile_preserves_the_loaded_microsegment_timing_gate():
    phases = MODULE.build_return_phases(_source_bundle())
    assert MODULE._fast_divisor(
        phases[6].commands[0], phases[6].name, 0) == 1.0
    assert MODULE._fast_divisor(
        phases[6].commands[1], phases[6].name, 1) == 7.0
    for phase in phases:
        for command_index, command in enumerate(phase.commands):
            times = MODULE._fast_times_ns(command, phase.name, command_index)
            velocities = MODULE._pchip_command_velocities(command, times)
            MODULE._validate_fast_cubic_profile(
                command, times, velocities, phase.name)


def test_fast_return_carry_has_only_targeted_endpoint_gate_alignment():
    phases = MODULE.build_return_phases(_source_bundle())
    return_carry = phases[6]
    final_command_indices = {
        name: 1 for name in return_carry.commands[1].joint_names}
    first_command_indices = {
        name: 0 for name in return_carry.commands[0].joint_names}

    assert MODULE._fast_divisor(
        return_carry.commands[1], return_carry.name, 1) == 7.0
    assert MODULE._endpoint_settle_sim_ns(
        return_carry.name, final_command_indices) == 2_000_000_000
    assert MODULE._endpoint_settle_sim_ns(
        return_carry.name, first_command_indices
    ) == MODULE.HARD_ENDPOINT_SETTLE_SIM_NS
    assert MODULE._endpoint_settle_sim_ns(
        phases[5].name, final_command_indices
    ) == MODULE.HARD_ENDPOINT_SETTLE_SIM_NS

    joint = return_carry.commands[1].joint_names[0]
    assert MODULE._endpoint_tolerance(
        joint, return_carry.name, final_command_indices) == 0.003
    assert MODULE._endpoint_tolerance(
        joint, return_carry.name, first_command_indices) == 0.004
    assert MODULE._endpoint_tolerance(
        joint, phases[5].name, final_command_indices) == 0.004


def test_fast_return_carry_restores_only_the_final_twelve_point_intervals():
    return_carry = MODULE.build_return_phases(_source_bundle())[6]
    command = return_carry.commands[1]
    times = MODULE._fast_times_ns(command, return_carry.name, 1)
    tail_start = len(command.points) - MODULE.RETURN_CARRY_ORIGIN_TAIL_POINT_COUNT
    generic_fast = tuple(
        int(round(point.time_from_start_ns / MODULE.FAST_TIME_DIVISOR))
        for point in command.points)

    assert times[:tail_start + 1] == generic_fast[:tail_start + 1]
    assert tuple(
        times[index + 1] - times[index]
        for index in range(tail_start, len(times) - 1)
    ) == tuple(
        command.points[index + 1].time_from_start_ns -
        command.points[index].time_from_start_ns
        for index in range(tail_start, len(command.points) - 1)
    )
    expected_added_ns = (
        command.points[-1].time_from_start_ns -
        command.points[tail_start].time_from_start_ns -
        (generic_fast[-1] - generic_fast[tail_start]))
    assert times[-1] - generic_fast[-1] == expected_added_ns
    assert 542_000_000 < expected_added_ns < 543_000_000


def test_second_cycle_and_full_roundtrip_message_counts_are_unambiguous():
    middle_messages = sum(
        len(phase.commands)
        for phase in MODULE.build_return_phases(_source_bundle()))
    right_prefix_and_suffix_messages = 14
    assert middle_messages == 16
    assert middle_messages + right_prefix_and_suffix_messages == 30
    assert 2 * (middle_messages + right_prefix_and_suffix_messages) == 60


def _sealed_document(tmp_path, name, schema):
    path = tmp_path / name
    raw = (json.dumps({"schema": schema}, sort_keys=True) + "\n").encode()
    path.write_bytes(raw)
    path.chmod(0o400)
    return MODULE.SealedDocument(
        path, hashlib.sha256(raw).hexdigest(), {"schema": schema})


def _authorization_value(bundle, return_plan):
    return {
        "schema": MODULE.RETURN_AUTHORIZATION_SCHEMA,
        "run_id": "f" * 64,
        "authorization_scope": "candidate3_roundtrip_831_cycle2_return",
        "bindings": {
            "return_plan": {
                "path": str(return_plan.path),
                "sha256": return_plan.sha256,
                "schema": MODULE.RETURN_PLAN_SCHEMA,
            },
            "middle_plan": {
                "path": str(bundle.plan.path),
                "sha256": bundle.plan.sha256,
                "schema": MODULE.PLAN_SCHEMA,
            },
            "middle_certificate": {
                "path": str(bundle.certificate.path),
                "sha256": bundle.certificate.sha256,
                "schema": MODULE.CERTIFICATE_SCHEMA,
            },
            "middle_exact_result": {
                "path": str(bundle.result.path),
                "sha256": bundle.result.sha256,
                "schema": MODULE.RESULT_SCHEMA,
            },
        },
        "execution_contract": {
            "exact_inverse": True,
            "inverse_transform": "reverse_points_mirror_time",
            "all_segments_required": True,
            "partial_execution_allowed": False,
            "requires_live_final_validation": True,
            "controller_message_count": 60,
            "left_hand_handle_contact_forbidden": True,
            "hard_maximum_abs_torso_deg": 44.5,
            "terminal_cube_pose": "origin",
            "terminal_door_state": "locked",
        },
        "controller_execution_authorized": True,
        "live_execution_passed": False,
        "controller_commands_issued": 0,
        "valid": True,
        "status": "READY",
    }


def test_return_execution_requires_separate_exactly_bound_authorization(
        tmp_path):
    bundle = SimpleNamespace(
        plan=_sealed_document(tmp_path, "middle-plan.json", MODULE.PLAN_SCHEMA),
        certificate=_sealed_document(
            tmp_path, "middle-certificate.json", MODULE.CERTIFICATE_SCHEMA),
        result=_sealed_document(
            tmp_path, "middle-result.json", MODULE.RESULT_SCHEMA),
    )
    return_plan = _sealed_document(
        tmp_path, "return-plan.json", MODULE.RETURN_PLAN_SCHEMA)
    value = _authorization_value(bundle, return_plan)
    path = tmp_path / "return-authorization.json"
    raw = (json.dumps(value, sort_keys=True) + "\n").encode()
    path.write_bytes(raw)
    path.chmod(0o400)
    args = SimpleNamespace(
        return_authorization=path,
        return_authorization_sha256=hashlib.sha256(raw).hexdigest())
    document = MODULE._validate_return_authorization(
        args, bundle, return_plan)
    assert document.value == value


def test_return_authorization_cannot_relax_left_handle_rule(tmp_path):
    bundle = SimpleNamespace(
        plan=_sealed_document(tmp_path, "middle-plan.json", MODULE.PLAN_SCHEMA),
        certificate=_sealed_document(
            tmp_path, "middle-certificate.json", MODULE.CERTIFICATE_SCHEMA),
        result=_sealed_document(
            tmp_path, "middle-result.json", MODULE.RESULT_SCHEMA),
    )
    return_plan = _sealed_document(
        tmp_path, "return-plan.json", MODULE.RETURN_PLAN_SCHEMA)
    value = _authorization_value(bundle, return_plan)
    value["execution_contract"]["left_hand_handle_contact_forbidden"] = False
    path = tmp_path / "bad-return-authorization.json"
    raw = (json.dumps(value, sort_keys=True) + "\n").encode()
    path.write_bytes(raw)
    path.chmod(0o400)
    args = SimpleNamespace(
        return_authorization=path,
        return_authorization_sha256=hashlib.sha256(raw).hexdigest())
    with pytest.raises(
            MODULE.MiddleExecutionError,
            match="RETURN_AUTHORIZATION_EXECUTION_CONTRACT_VALUE"):
        MODULE._validate_return_authorization(args, bundle, return_plan)
