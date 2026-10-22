"""Offline contract tests for the unified 828 Gazebo support adapter."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = WORKSPACE / "src/h1_2_moveit_demo/scripts/h1_support_orchestrator_828.py"
SPEC = importlib.util.spec_from_file_location("h1_support_orchestrator_828_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _canonical(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8")


def _digest(value):
    return hashlib.sha256(value).hexdigest()


def _pose(x, y, z, yaw):
    return {
        "position": [x, y, z],
        "orientation": [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
    }


def _support_contract(
        snapshot_relative_path="runtime_assets/h1_ros_snapshot_8251.py",
        snapshot_sha="b" * 64):
    left_contact = dict(zip(MODULE.LEFT_JOINTS, (
        -0.1, 0.2, 0.3, 0.8, -0.2, 0.1, 0.0, -0.01472, -0.01472)))
    left_release = dict(left_contact)
    left_release["left_shoulder_pitch_joint"] = -0.2
    return {
        "schema": MODULE.CONTRACT_SCHEMA,
        "phases": list(MODULE.EXPECTED_PHASES),
        "world": {
            "world_name": "h1_world",
            "table_x": 0.600,
            "incubator_x": 0.590,
            "incubator_y": 0.0,
            "incubator_z": 0.815,
            "cube_spawn_x": 0.334,
            "cube_spawn_y": 0.377,
            "cube_spawn_z": 0.830,
            "cube_spawn_yaw": math.pi / 2.0,
        },
        "hooks": [
            {"operation": "door-attach", "boundary": "E0_ENTRY",
             "phase_index": 0, "edge": "entry", "door_before": "detached",
             "door_after": "attached", "cube_before": "detached", "cube_after": "detached"},
            {"operation": "cube-attach", "boundary": "CUBE_CONTACT",
             "phase_index": 4, "edge": "exit", "door_before": "attached",
             "door_after": "attached", "cube_before": "detached", "cube_after": "attached"},
            {"operation": "cube-detach", "boundary": "CUBE_RELEASE",
             "phase_index": 5, "edge": "exit", "door_before": "attached",
             "door_after": "attached", "cube_before": "attached", "cube_after": "detached"},
            {"operation": "door-detach", "boundary": "E0_RETURN",
             "phase_index": 10, "edge": "exit", "door_before": "attached",
             "door_after": "detached", "cube_before": "detached", "cube_after": "detached"},
        ],
        "boundaries": {
            "E0_ENTRY": {"robot_positions": dict(MODULE.CANONICAL_MIDDLE_SEAM)},
            "E0_RETURN": {"robot_positions": dict(MODULE.CANONICAL_MIDDLE_SEAM)},
            "CUBE_CONTACT": {
                "robot_positions": left_contact,
                "cube_pose": _pose(0.334, 0.377, 0.830, math.pi / 2.0),
            },
            "CUBE_RELEASE": {
                "robot_positions": left_release,
                "cube_pose": _pose(0.611, -0.030, 0.995, math.pi / 2.0),
            },
            "FINAL_ROBOT": {
                "robot_positions": dict(MODULE.CANONICAL_FINAL_ROBOT),
            },
        },
        "plugin": {
            "binary_sha256": "a" * 64,
            "door": {
                "parent_scope": "h1_2::right_wrist_yaw_link",
                "child_scope": "incubator::door_link",
                "state_topic": MODULE.DOOR_STATE_TOPIC,
                "attach_topic": MODULE.DOOR_ATTACH_TOPIC,
                "detach_topic": MODULE.DOOR_DETACH_TOPIC,
                "max_force": 120.0,
                "max_torque": 8.0,
                "gravity_compensation": False,
            },
            "cube": {
                "parent_scope": "h1_2::left_wrist_yaw_link",
                "child_model": "green_cube",
                "state_topic": MODULE.CUBE_STATE_TOPIC,
                "attach_topic": MODULE.CUBE_ATTACH_TOPIC,
                "detach_topic": MODULE.CUBE_DETACH_TOPIC,
                "initially_attached": False,
            },
        },
        "tools": {
            "ros_snapshot": {
                "workspace_relative_path": snapshot_relative_path,
                "sha256": snapshot_sha,
            },
        },
    }


def _values(**contract_kwargs):
    contract = _support_contract(**contract_kwargs)
    plan_sha = "1" * 64
    certificate = {
        "schema": "h1_candidate3_middle_certificate/test-v1",
        "auditor_version": "h1_candidate3_middle_exact_auditor/test-v1",
        "valid": True,
        "status": "PASS",
        "plan_schema": "h1_candidate3_middle_plan/test-v1",
        "plan_sha256": plan_sha,
        "exact_result_sha256": "2" * 64,
        "exact_result_schema": "h1_candidate3_middle_exact_result/test-v1",
        "support_contract_sha256": _digest(_canonical(contract)),
    }
    certificate_raw = _canonical(certificate) + b"\n"
    certificate_sha = _digest(certificate_raw)
    manifest = {
        "schema": MODULE.MANIFEST_SCHEMA,
        "controller_execution_authorized": True,
        "live_execution_passed": False,
        "bindings": {
            "plan_sha256": plan_sha,
            "plan_schema": "h1_candidate3_middle_plan/test-v1",
            "certificate_sha256": certificate_sha,
            "exact_result_sha256": "2" * 64,
            "certificate_schema": "h1_candidate3_middle_certificate/test-v1",
            "exact_result_schema": "h1_candidate3_middle_exact_result/test-v1",
            "auditor_version": "h1_candidate3_middle_exact_auditor/test-v1",
            "support_contract_sha256": _digest(_canonical(contract)),
        },
        "support_contract": contract,
    }
    return manifest, certificate, certificate_raw


def _sealed(tmp_path, name, raw):
    path = tmp_path / name
    path.write_bytes(raw)
    path.chmod(0o400)
    return path, _digest(raw)


def _load_valid(tmp_path, **contract_kwargs):
    manifest, certificate, certificate_raw = _values(**contract_kwargs)
    certificate_path, certificate_sha = _sealed(
        tmp_path, "certificate.json", certificate_raw)
    manifest_raw = _canonical(manifest) + b"\n"
    manifest_path, manifest_sha = _sealed(tmp_path, "manifest.json", manifest_raw)
    loaded_manifest = MODULE.load_sealed_json(manifest_path, manifest_sha)
    loaded_certificate = MODULE.load_sealed_json(certificate_path, certificate_sha)
    return (
        MODULE.validate_manifest_and_certificate(loaded_manifest, loaded_certificate),
        manifest_path, manifest_sha, certificate_path, certificate_sha,
    )


def test_valid_contract_binds_e0_phases_and_certificate(tmp_path):
    contract, *_ = _load_valid(tmp_path)
    assert (contract.boundaries["E0_ENTRY"]["robot_positions"] ==
            MODULE.CANONICAL_MIDDLE_SEAM)
    assert (contract.boundaries["E0_RETURN"]["robot_positions"] ==
            MODULE.CANONICAL_MIDDLE_SEAM)
    assert (contract.boundaries["FINAL_ROBOT"]["robot_positions"] ==
            MODULE.CANONICAL_FINAL_ROBOT)
    assert tuple(contract.value["phases"]) == MODULE.EXPECTED_PHASES
    assert contract.plan_sha256 == "1" * 64
    assert contract.exact_result_sha256 == "2" * 64
    assert contract.snapshot_relative_path == "runtime_assets/h1_ros_snapshot_8251.py"
    assert contract.snapshot_sha256 == "b" * 64


def test_validate_cli_is_offline_and_prints_exact_marker(tmp_path, monkeypatch, capsys):
    _, snapshot_sha = _snapshot_executable(tmp_path)
    _, manifest, manifest_sha, certificate, certificate_sha = _load_valid(
        tmp_path, snapshot_sha=snapshot_sha)

    def forbidden(*_args, **_kwargs):
        raise AssertionError("validate may not construct a Gazebo runtime")

    monkeypatch.setattr(MODULE, "GazeboRuntime", forbidden)
    assert MODULE.main([
        "validate", "--manifest", str(manifest),
        "--manifest-sha256", manifest_sha,
        "--certificate", str(certificate),
        "--certificate-sha256", certificate_sha,
        "--workspace", str(tmp_path),
    ]) == 0
    assert capsys.readouterr().out.strip() == (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=validate execution=none")


@pytest.mark.parametrize("mutation,status", [
    (lambda manifest, _certificate:
     manifest.update(controller_execution_authorized=False),
     "MANIFEST_CONTROLLER_NOT_AUTHORIZED"),
    (lambda manifest, _certificate: manifest.update(live_execution_passed=True),
     "MANIFEST_LIVE_STATUS_NOT_PRERUN"),
    (lambda manifest, _certificate: manifest.update(execution_ready=True),
     "MANIFEST_PREMATURE_EXECUTION_READY"),
    (lambda manifest, _certificate: manifest["support_contract"]["phases"].reverse(),
     "CONTRACT_PHASES"),
    (lambda manifest, _certificate: manifest["support_contract"]["world"].update(table_x=0.5),
     "CONTRACT_WORLD_TABLE_X"),
    (lambda manifest, _certificate: manifest["support_contract"]["tools"]
     ["ros_snapshot"].update(workspace_relative_path="runtime_assets/../snapshot.py"),
     "CONTRACT_SNAPSHOT_PATH_ALIAS"),
    (lambda manifest, _certificate: manifest["support_contract"]["tools"]
     ["ros_snapshot"].update(sha256="not-a-digest"),
     "CONTRACT_SNAPSHOT_SHA256"),
    (lambda manifest, _certificate: manifest["support_contract"]["boundaries"]
     ["E0_ENTRY"]["robot_positions"].update(right_dex1_finger_2_joint=0.01235),
     "E0_ENTRY_CANONICAL_E0_right_dex1_finger_2_joint"),
    (lambda manifest, _certificate: manifest["support_contract"]["boundaries"]
     ["E0_RETURN"]["robot_positions"].update(left_shoulder_roll_joint=0.249986),
     "E0_RETURN_CANONICAL_E0_left_shoulder_roll_joint"),
    (lambda _manifest, certificate: certificate.update(valid=False),
     "CERTIFICATE_NOT_PASS"),
    (lambda _manifest, certificate:
     certificate.update(exact_result_sha256="3" * 64),
     "CERTIFICATE_RESULT_BINDING"),
])
def test_contract_mutations_fail_closed(tmp_path, mutation, status):
    manifest, certificate, _ = _values()
    mutation(manifest, certificate)
    certificate["support_contract_sha256"] = _digest(
        _canonical(manifest["support_contract"]))
    certificate_raw = _canonical(certificate) + b"\n"
    certificate_sha = _digest(certificate_raw)
    manifest["bindings"]["certificate_sha256"] = certificate_sha
    manifest_raw = _canonical(manifest) + b"\n"
    manifest_path, manifest_sha = _sealed(tmp_path, "manifest.json", manifest_raw)
    certificate_path, _ = _sealed(tmp_path, "certificate.json", certificate_raw)
    with pytest.raises(MODULE.SupportError, match=status):
        MODULE.validate_manifest_and_certificate(
            MODULE.load_sealed_json(manifest_path, manifest_sha),
            MODULE.load_sealed_json(certificate_path, certificate_sha))


def test_sealed_input_rejects_writable_mode_and_duplicate_keys(tmp_path):
    path = tmp_path / "bad.json"
    raw = b'{"schema":"x","schema":"y"}\n'
    path.write_bytes(raw)
    path.chmod(0o600)
    with pytest.raises(MODULE.SupportError, match="SEALED_MODE"):
        MODULE.load_sealed_json(path, _digest(raw))
    path.chmod(0o400)
    with pytest.raises(MODULE.SupportError, match="JSON_DUPLICATE_KEY"):
        MODULE.load_sealed_json(path, _digest(raw))


def _snapshot_executable(
        tmp_path, relative="runtime_assets/h1_ros_snapshot_8251.py"):
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = b"#!/usr/bin/python3\nprint('snapshot')\n"
    path.write_bytes(raw)
    path.chmod(0o500)
    return path, _digest(raw)


def test_bound_snapshot_executable_accepts_only_exact_sealed_file(tmp_path):
    path, digest = _snapshot_executable(tmp_path)
    assert MODULE.load_bound_snapshot_executable(
        tmp_path, "runtime_assets/h1_ros_snapshot_8251.py", digest) == path


def test_bound_snapshot_executable_rejects_wrong_mode(tmp_path):
    _, digest = _snapshot_executable(tmp_path)
    path = tmp_path / "runtime_assets/h1_ros_snapshot_8251.py"
    path.chmod(0o700)
    with pytest.raises(MODULE.SupportError, match="SNAPSHOT_MODE"):
        MODULE.load_bound_snapshot_executable(
            tmp_path, "runtime_assets/h1_ros_snapshot_8251.py", digest)


def test_bound_snapshot_executable_rejects_alias(tmp_path):
    target, digest = _snapshot_executable(tmp_path, "runtime_assets/real.py")
    alias = tmp_path / "runtime_assets/alias.py"
    alias.symlink_to(target.name)
    with pytest.raises(MODULE.SupportError, match="SNAPSHOT_PATH_ALIAS"):
        MODULE.load_bound_snapshot_executable(
            tmp_path, "runtime_assets/alias.py", digest)


def test_bound_snapshot_executable_rejects_post_binding_tamper(tmp_path):
    path, original_digest = _snapshot_executable(tmp_path)
    path.chmod(0o700)
    path.write_bytes(b"#!/usr/bin/python3\nprint('tampered')\n")
    path.chmod(0o500)
    with pytest.raises(MODULE.SupportError, match="SNAPSHOT_SHA256"):
        MODULE.load_bound_snapshot_executable(
            tmp_path, "runtime_assets/h1_ros_snapshot_8251.py", original_digest)


def test_bound_snapshot_executable_rejects_hardlink(tmp_path):
    path, digest = _snapshot_executable(tmp_path)
    (tmp_path / "runtime_assets/second.py").hardlink_to(path)
    with pytest.raises(MODULE.SupportError, match="SNAPSHOT_IDENTITY"):
        MODULE.load_bound_snapshot_executable(
            tmp_path, "runtime_assets/h1_ros_snapshot_8251.py", digest)


def test_bound_snapshot_executable_rejects_oversize(tmp_path):
    path, _ = _snapshot_executable(tmp_path)
    path.chmod(0o700)
    with path.open("wb") as stream:
        stream.truncate(MODULE.MAX_BOUND_SNAPSHOT_BYTES + 1)
    path.chmod(0o500)
    with pytest.raises(MODULE.SupportError, match="SNAPSHOT_SIZE"):
        MODULE.load_bound_snapshot_executable(
            tmp_path, "runtime_assets/h1_ros_snapshot_8251.py", "0" * 64)


def test_validate_cli_rejects_tampered_bound_snapshot(
        tmp_path, monkeypatch, capsys):
    path, snapshot_sha = _snapshot_executable(tmp_path)
    _, manifest, manifest_sha, certificate, certificate_sha = _load_valid(
        tmp_path, snapshot_sha=snapshot_sha)
    path.chmod(0o700)
    path.write_bytes(b"#!/usr/bin/python3\nprint('tampered')\n")
    path.chmod(0o500)
    monkeypatch.setattr(
        MODULE, "GazeboRuntime",
        lambda *_args, **_kwargs: pytest.fail("validate constructed runtime"))
    assert MODULE.main([
        "validate", "--manifest", str(manifest),
        "--manifest-sha256", manifest_sha,
        "--certificate", str(certificate),
        "--certificate-sha256", certificate_sha,
        "--workspace", str(tmp_path),
    ]) == 1
    assert "H1_SUPPORT_ORCHESTRATOR_828=FAIL:SNAPSHOT_SHA256" in (
        capsys.readouterr().err)


def test_preload_rejects_duplicate_trajectory_publisher_before_plugin():
    runtime = object.__new__(MODULE.GazeboRuntime)
    plugin_calls = []
    topic_calls = []

    def run(command, timeout=None):
        del timeout
        topic_calls.append(command)
        return ("Type: trajectory_msgs/msg/JointTrajectory\n"
                "Publisher count: 2\nSubscription count: 1\n")

    runtime._run = run
    runtime._preload_one = lambda *_args: plugin_calls.append(True)
    with pytest.raises(
            MODULE.SupportError,
            match=r"PRELOAD_TRAJECTORY_TOPIC:/torso_controller/joint_trajectory"):
        runtime.preload()
    assert len(topic_calls) == 1
    assert plugin_calls == []


def test_trajectory_topic_unknown_retries_then_requires_exact_safe_counts(
        monkeypatch):
    runtime = object.__new__(MODULE.GazeboRuntime)
    calls = []
    sleeps = []

    def run(command, timeout=None):
        del timeout
        calls.append(command)
        if command[3] == MODULE.TRAJECTORY_TOPICS[0] and len(calls) <= 2:
            raise MODULE.SupportError(
                "COMMAND_EXIT:ros2:1:Unknown topic "
                f"'{MODULE.TRAJECTORY_TOPICS[0]}'")
        return ("Type: trajectory_msgs/msg/JointTrajectory\n"
                "Publisher count: 0\nSubscription count: 1\n")

    runtime._run = run
    monkeypatch.setattr(MODULE.time, "sleep", sleeps.append)
    runtime._require_trajectory_topics_quiescent("PRELOAD")
    assert [command[3] for command in calls] == [
        MODULE.TRAJECTORY_TOPICS[0], MODULE.TRAJECTORY_TOPICS[0],
        *MODULE.TRAJECTORY_TOPICS,
    ]
    assert all(command[-2:] == (
        "--spin-time", str(MODULE.TRAJECTORY_TOPIC_DISCOVERY_SPIN_SECONDS))
               for command in calls)
    assert sleeps == [
        MODULE.TRAJECTORY_TOPIC_DISCOVERY_RETRY_DELAY_SECONDS,
        MODULE.TRAJECTORY_TOPIC_DISCOVERY_RETRY_DELAY_SECONDS,
    ]


def test_trajectory_topic_incomplete_counts_retry_then_pass(monkeypatch):
    runtime = object.__new__(MODULE.GazeboRuntime)
    calls = []
    sleeps = []

    def run(command, timeout=None):
        del timeout
        calls.append(command)
        if command[3] == MODULE.TRAJECTORY_TOPICS[0] and len(calls) == 1:
            return ("Type: trajectory_msgs/msg/JointTrajectory\n"
                    "Publisher count: 0\n")
        return ("Type: trajectory_msgs/msg/JointTrajectory\n"
                "Publisher count: 0\nSubscription count: 1\n")

    runtime._run = run
    monkeypatch.setattr(MODULE.time, "sleep", sleeps.append)
    runtime._require_trajectory_topics_quiescent("FINAL")
    assert [command[3] for command in calls[:2]] == [
        MODULE.TRAJECTORY_TOPICS[0], MODULE.TRAJECTORY_TOPICS[0]]
    assert sleeps == [MODULE.TRAJECTORY_TOPIC_DISCOVERY_RETRY_DELAY_SECONDS]


@pytest.mark.parametrize("unsafe_output", [
    "Publisher count: 1\n",
    "Subscription count: 0\n",
    "Publisher count: 0\nSubscription count: 0\n",
    "Publisher count: 0\nPublisher count: 3\n",
])
def test_trajectory_topic_observed_unsafe_count_fails_without_retry(
        unsafe_output, monkeypatch):
    runtime = object.__new__(MODULE.GazeboRuntime)
    calls = []
    sleeps = []
    runtime._run = lambda command, timeout=None: (
        calls.append(command) or unsafe_output)
    monkeypatch.setattr(MODULE.time, "sleep", sleeps.append)
    with pytest.raises(
            MODULE.SupportError,
            match=r"PRELOAD_TRAJECTORY_TOPIC:/torso_controller/joint_trajectory"):
        runtime._require_trajectory_topics_quiescent("PRELOAD")
    assert len(calls) == 1
    assert sleeps == []


def test_trajectory_topic_unknown_exhaustion_is_bounded(monkeypatch):
    runtime = object.__new__(MODULE.GazeboRuntime)
    calls = []
    sleeps = []

    def run(command, timeout=None):
        del timeout
        calls.append(command)
        raise MODULE.SupportError("COMMAND_EXIT:ros2:1:Unknown topic")

    runtime._run = run
    monkeypatch.setattr(MODULE.time, "sleep", sleeps.append)
    with pytest.raises(
            MODULE.SupportError,
            match=(r"PRELOAD_TRAJECTORY_TOPIC_DISCOVERY:"
                   r"/torso_controller/joint_trajectory:UNKNOWN_TOPIC")):
        runtime._require_trajectory_topics_quiescent("PRELOAD")
    assert len(calls) == MODULE.TRAJECTORY_TOPIC_DISCOVERY_ATTEMPTS
    assert len(sleeps) == MODULE.TRAJECTORY_TOPIC_DISCOVERY_ATTEMPTS - 1


def test_trajectory_topic_non_discovery_command_failure_is_not_retried(
        monkeypatch):
    runtime = object.__new__(MODULE.GazeboRuntime)
    calls = []
    sleeps = []

    def run(command, timeout=None):
        del timeout
        calls.append(command)
        raise MODULE.SupportError("COMMAND_EXIT:ros2:2:RMW failure")

    runtime._run = run
    monkeypatch.setattr(MODULE.time, "sleep", sleeps.append)
    with pytest.raises(MODULE.SupportError, match="RMW failure"):
        runtime._require_trajectory_topics_quiescent("PRELOAD")
    assert len(calls) == 1
    assert sleeps == []


def test_preload_checks_all_five_trajectory_topics_before_plugins():
    runtime = object.__new__(MODULE.GazeboRuntime)
    events = []

    def run(command, timeout=None):
        del timeout
        events.append(("topic", command[3]))
        return ("Type: trajectory_msgs/msg/JointTrajectory\n"
                "Publisher count: 0\nSubscription count: 1\n")

    runtime._run = run
    runtime._require_empty_topics = lambda: None
    runtime._require_add_service = lambda: None
    runtime._static_model_pose_gate = lambda *_args: None
    runtime._mechanism_triplet_gate = lambda *_args: None
    runtime._cube_stability_gate = lambda *_args: None
    runtime._entity = lambda name: {"id": 1 if name == "incubator" else 2}
    runtime._preload_one = lambda *_args: events.append(("plugin", _args[1]))
    runtime._require_resident_plugin = lambda *_args: None
    runtime._support_state = lambda _topic: "detached"
    runtime._create_ownership = lambda: None
    runtime.preload()
    assert events[:5] == [("topic", topic) for topic in MODULE.TRAJECTORY_TOPICS]
    assert events[5:] == [("plugin", "incubator"), ("plugin", "green_cube")]


def test_entity_uses_fresh_full_scene_pose_and_requires_one_positive_id():
    runtime = object.__new__(MODULE.GazeboRuntime)
    calls = []
    payload = {
        "pose": [
            {"name": "table", "id": 0},
            {"name": "table", "id": 17,
             "position": {"x": 0.600}, "orientation": {"w": 1.0}},
        ],
    }

    def run(command, timeout=None, *, separate_stderr=False):
        del timeout
        calls.append((command, separate_stderr))
        return json.dumps(payload)

    runtime._run = run
    assert runtime._entity("table")["id"] == 17
    assert MODULE.POSE_TOPIC == "/world/h1_world/pose/info"
    assert calls == [((
        "gz", "topic", "-e", "--json-output", "-t",
        "/world/h1_world/pose/info", "-n", "1",
    ), True)]


def _full_pose_frame(entity_id=17, x=0.600):
    return {
        "pose": [
            {"name": "ground_plane", "id": 1,
             "orientation": {"w": 1.0}},
            {"name": "table", "id": entity_id,
             "position": {"x": x}, "orientation": {"w": 1.0}},
            {"name": "incubator", "id": 21,
             "position": {"x": 0.590, "z": 0.815},
             "orientation": {"w": 1.0}},
            {"name": "green_cube", "id": 22,
             "position": {"x": 0.334, "y": 0.377, "z": 0.830},
             "orientation": {"z": math.sqrt(0.5), "w": math.sqrt(0.5)}},
        ],
    }


def test_entity_returns_last_of_two_full_frames():
    runtime = object.__new__(MODULE.GazeboRuntime)
    first = _full_pose_frame(x=0.599)
    last = _full_pose_frame(x=0.600)
    runtime._run = lambda _command, timeout=None, **_kwargs: (
        json.dumps(first, separators=(",", ":")) + "\n" +
        json.dumps(last, separators=(",", ":")) + "\n")
    entity = runtime._entity("table")
    assert entity["id"] == 17
    assert entity["position"]["x"] == 0.600


def test_entity_rejects_identity_change_between_frames():
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime._run = lambda _command, timeout=None, **_kwargs: "\n".join((
        json.dumps(_full_pose_frame(entity_id=17)),
        json.dumps(_full_pose_frame(entity_id=18)),
    ))
    with pytest.raises(
            MODULE.SupportError, match="ENTITY_IDENTITY:table:FRAME:1"):
        runtime._entity("table")


@pytest.mark.parametrize("frames", [
    ({"pose": [{"name": "incubator", "id": 21}]}, _full_pose_frame()),
    (_full_pose_frame(), {"pose": [{"name": "incubator", "id": 21}]}),
    ({"pose": [{"name": "table", "id": 17},
               {"name": "table", "id": 18}]}, _full_pose_frame()),
    (_full_pose_frame(), {"pose": [{"name": "table", "id": 17},
                                   {"name": "table", "id": 18}]}),
])
def test_entity_rejects_missing_or_duplicate_entity_in_any_frame(frames):
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime._run = lambda _command, timeout=None, **_kwargs: "\n".join(
        json.dumps(frame) for frame in frames)
    with pytest.raises(MODULE.SupportError, match="ENTITY_UNIQUE:table:FRAME"):
        runtime._entity("table")


@pytest.mark.parametrize("output", [
    "",
    " \n\t ",
    json.dumps(_full_pose_frame()) + "\n{\"pose\":[",
    json.dumps(_full_pose_frame()) + "\ntrailing non-json diagnostics",
    json.dumps(_full_pose_frame()) + json.dumps(_full_pose_frame()),
    json.dumps(_full_pose_frame()) + "\n[]",
    json.dumps(_full_pose_frame()) + "\n" + json.dumps({"pose": [None]}),
])
def test_entity_rejects_invalid_json_stream(output):
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime._run = lambda _command, timeout=None, **_kwargs: output
    with pytest.raises(MODULE.SupportError, match="ENTITY_JSON:table"):
        runtime._entity("table")


def test_entity_rejects_duplicate_json_key_and_nonfinite_constant():
    runtime = object.__new__(MODULE.GazeboRuntime)
    for output in (
        '{"pose":[],"pose":[{"name":"table","id":17}]}',
        '{"pose":[{"name":"table","id":17,"position":{"x":NaN}}]}',
    ):
        runtime._run = (
            lambda _command, timeout=None, value=output, **_kwargs: value)
        with pytest.raises(MODULE.SupportError, match="ENTITY_JSON:table"):
            runtime._entity("table")


@pytest.mark.parametrize("output,status", [
    ('{"pose":[{"name":"table","id":17,'
     '"position":{"x":1e999},"orientation":{"w":1}}]}',
     "POSE_FINITE:position"),
    ('{"pose":[{"name":"table","id":17,'
     '"position":{"x":0},"orientation":{"w":"inf"}}]}',
     "POSE_FINITE:orientation"),
])
def test_entity_rejects_overflow_and_string_pose_values(output, status):
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime._run = lambda _command, timeout=None, **_kwargs: output
    with pytest.raises(MODULE.SupportError, match=status):
        runtime._entity("table")


def test_pose_error_rejects_nonfinite_quaternion_before_dot_clamp():
    actual = {
        "position": [0.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, math.inf],
    }
    expected = {
        "position": [0.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    with pytest.raises(MODULE.SupportError, match="POSE_QUATERNION"):
        MODULE.GazeboRuntime._pose_error(actual, expected)


def test_latch_and_support_state_use_latest_complete_frame():
    runtime = object.__new__(MODULE.GazeboRuntime)
    outputs = {
        MODULE.LATCH_TOPIC: (
            "\x1b[1;33mWarning [transport]: delayed frame\x1b[0m\n"
            'data: "locked"\ndata: "unlocked"\n'),
        MODULE.DOOR_STATE_TOPIC: (
            '[INFO] [transport]: two callbacks\n'
            'data: "detached"\ndata: "attached"\n'),
    }
    runtime._run = lambda command, timeout=None: outputs[command[4]]
    assert runtime._latch_state() == "unlocked"
    assert runtime._support_state(MODULE.DOOR_STATE_TOPIC) == "attached"


@pytest.mark.parametrize("output", [
    "",
    "only arbitrary stdout garbage\n",
    'data: "unknown"\n',
    'data: "locked\n',
    'data:\n',
])
def test_state_frame_parser_rejects_no_unknown_or_incomplete_frame(output):
    with pytest.raises(MODULE.SupportError):
        MODULE._last_state_message(
            output, ("locked", "unlocked"), "LATCH_STATE")


def test_listener_drains_burst_without_second_fd_readiness():
    read_fd, write_fd = MODULE.os.pipe()
    stream = MODULE.os.fdopen(read_fd, "rb", buffering=0)

    class Process:
        stdout = stream

        @staticmethod
        def poll():
            return None

    selector = MODULE.selectors.DefaultSelector()
    buffer = bytearray()
    try:
        MODULE.os.set_blocking(stream.fileno(), False)
        selector.register(stream, MODULE.selectors.EVENT_READ)
        MODULE.os.write(write_fd, b'data: "detached"\ndata: "detached"\n')
        deadline = MODULE.time.monotonic() + 1.0
        MODULE._wait_listener_state(
            Process(), selector, buffer, "detached",
            ("detached", "attached"), deadline, "TEST_LISTENER")
        MODULE.os.write(
            write_fd, b'data: "detached"\ndata: "attached"\n')
        MODULE._wait_listener_state(
            Process(), selector, buffer, "attached",
            ("detached", "attached"), deadline, "TEST_LISTENER")
    finally:
        selector.close()
        stream.close()
        MODULE.os.close(write_fd)


def test_snapshot_requests_structured_stdout_only():
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime._bound_snapshot_tool = lambda: Path("/sealed/snapshot.py")
    calls = []

    def run(command, timeout=None, *, separate_stderr=False):
        calls.append((command, timeout, separate_stderr))
        return "torso_joint 0.0\nright_elbow_joint 0.6\n"

    runtime._run = run
    assert runtime._snapshot("robot") == {
        "torso_joint": (0.0,), "right_elbow_joint": (0.6,),
    }
    assert calls == [(('/sealed/snapshot.py', 'robot', '--timeout', '10'),
                      12.0, True)]


def test_run_separates_successful_diagnostics_from_structured_stdout(
        monkeypatch):
    stdout_read, stdout_write = MODULE.os.pipe()
    stderr_read, stderr_write = MODULE.os.pipe()
    MODULE.os.write(stdout_write, b"torso_joint 0.0\n")
    MODULE.os.write(stderr_write, b"[WARN] delayed discovery\n")
    MODULE.os.close(stdout_write)
    MODULE.os.close(stderr_write)

    class Process:
        pid = 6161
        returncode = 0
        stdout = MODULE.os.fdopen(stdout_read, "rb", buffering=0)
        stderr = MODULE.os.fdopen(stderr_read, "rb", buffering=0)

        @staticmethod
        def poll():
            return 0

        @staticmethod
        def wait(timeout):
            assert timeout <= 3.0
            return 0

    def popen(*_args, **kwargs):
        assert kwargs["stderr"] is MODULE.subprocess.PIPE
        assert kwargs["text"] is False
        assert kwargs["bufsize"] == 0
        return Process()

    monkeypatch.setattr(MODULE.subprocess, "Popen", popen)
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime.timeout_seconds = 3.0
    assert runtime._run(
        ("snapshot",), separate_stderr=True) == "torso_joint 0.0\n"


def test_run_kills_process_group_as_soon_as_stream_exceeds_limit(
        monkeypatch):
    read_fd, write_fd = MODULE.os.pipe()
    MODULE.os.write(write_fd, b"12345")
    MODULE.os.close(write_fd)
    events = []

    class Process:
        pid = 7171
        # The leader has exited, but a descendant still owns this pipe.
        returncode = 0
        stdout = MODULE.os.fdopen(read_fd, "rb", buffering=0)
        stderr = None

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            pytest.fail(f"exited leader should not be reaped again: {timeout}")

    monkeypatch.setattr(MODULE, "MAX_COMMAND_OUTPUT_BYTES", 4)
    monkeypatch.setattr(
        MODULE.subprocess, "Popen", lambda *_args, **_kwargs: Process())
    monkeypatch.setattr(
        MODULE.os, "killpg", lambda pid, signal: events.append((pid, signal)))
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime.timeout_seconds = 3.0
    with pytest.raises(
            MODULE.SupportError, match="COMMAND_OUTPUT_LIMIT:gz:stdout"):
        runtime._run(("gz", "topic", "-l"))
    assert events == [(7171, 9)]


def test_run_accepts_exact_stream_limit_and_reaps_process(monkeypatch):
    read_fd, write_fd = MODULE.os.pipe()
    MODULE.os.write(write_fd, b"1234")
    MODULE.os.close(write_fd)
    waits = []

    class Process:
        pid = 8181
        returncode = None
        stdout = MODULE.os.fdopen(read_fd, "rb", buffering=0)
        stderr = None

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            waits.append(timeout)
            self.returncode = 0
            return self.returncode

    monkeypatch.setattr(MODULE, "MAX_COMMAND_OUTPUT_BYTES", 4)
    monkeypatch.setattr(
        MODULE.subprocess, "Popen", lambda *_args, **_kwargs: Process())
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime.timeout_seconds = 3.0
    assert runtime._run(("bounded",)) == "1234"
    assert len(waits) == 1 and 0.0 < waits[0] <= 3.0


@pytest.mark.parametrize("payload,status", [
    ({"pose": []}, "ENTITY_UNIQUE:table"),
    ({"pose": [{"name": "table", "id": 11},
               {"name": "table", "id": 12}]}, "ENTITY_UNIQUE:table"),
    ({}, "ENTITY_JSON:table"),
    ("not-json", "ENTITY_JSON:table"),
])
def test_entity_rejects_missing_duplicate_or_malformed_full_pose(payload, status):
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime._run = lambda _command, timeout=None, **_kwargs: (
        payload if isinstance(payload, str) else json.dumps(payload))
    with pytest.raises(MODULE.SupportError, match=status):
        runtime._entity("table")


def test_e0_runtime_boundary_gate_checks_left_park():
    runtime = object.__new__(MODULE.GazeboRuntime)

    class Contract:
        boundaries = {
            "E0_ENTRY": {
                "robot_positions": dict(MODULE.CANONICAL_MIDDLE_SEAM),
            },
        }

    rows = {
        joint: (value,) for joint, value in MODULE.CANONICAL_MIDDLE_SEAM.items()
    }
    rows["left_shoulder_roll_joint"] = (
        MODULE.CANONICAL_FINAL_ROBOT["left_shoulder_roll_joint"],)
    runtime.contract = Contract()
    runtime._snapshot = lambda mode: rows if mode == "robot" else {}
    with pytest.raises(
            MODULE.SupportError,
            match="E0_ENTRY_JOINT_ENDPOINT:left_shoulder_roll_joint"):
        runtime._boundary_gate("E0_ENTRY")
    rows["left_shoulder_roll_joint"] = (
        MODULE.CANONICAL_MIDDLE_SEAM["left_shoulder_roll_joint"],)
    rows["left_dex1_finger_1_joint"] = (-0.0175,)
    with pytest.raises(
            MODULE.SupportError,
            match="E0_ENTRY_JOINT_ENDPOINT:left_dex1_finger_1_joint"):
        runtime._boundary_gate("E0_ENTRY")


def test_operation_dispatch_and_markers_are_exact():
    class Runtime:
        def __init__(self):
            self.calls = []

        def __getattr__(self, name):
            def call():
                self.calls.append(name)
            return call

    expected = [
        ("preload", "preload", "door=detached cube=detached"),
        ("door-attach", "door_attach", "boundary=E0_ENTRY"),
        ("cube-attach", "cube_attach", "boundary=CUBE_CONTACT"),
        ("cube-detach", "cube_detach", "boundary=CUBE_RELEASE"),
        ("door-detach", "door_detach", "boundary=E0_RETURN"),
        ("finalize", "finalize", "supports=detached robot=launch publishers=0"),
    ]
    for operation, method, suffix in expected:
        runtime = Runtime()
        marker = MODULE.execute_operation(operation, runtime)
        assert runtime.calls == [method]
        assert marker.startswith(
            f"H1_SUPPORT_ORCHESTRATOR_828=PASS operation={operation} ")
        assert suffix in marker


def test_roundtrip_operation_dispatch_has_explicit_cycles_and_boundaries():
    class Runtime:
        def __init__(self):
            self.calls = []

        def __getattr__(self, name):
            def call():
                self.calls.append(name)
            return call

    expected = [
        ("preload", "preload", "mode=roundtrip", "CUBE_CONTACT_ORIGIN"),
        ("door-attach", "door_attach", "cycle=1", "boundary=E0_ENTRY"),
        ("cube-attach", "cube_attach", "cycle=1", "CUBE_CONTACT_ORIGIN"),
        ("cube-detach", "cube_detach", "cycle=1", "CUBE_RELEASE_INSIDE"),
        ("door-detach", "door_detach", "cycle=1", "boundary=E0_RETURN"),
        ("cycle-checkpoint", "roundtrip_checkpoint", "cycle=1",
         "CUBE_RELEASE_INSIDE"),
        ("door-attach-return", "door_attach", "cycle=2", "boundary=E0_ENTRY"),
        ("cube-attach-return", "cube_attach_return", "cycle=2",
         "CUBE_RELEASE_INSIDE"),
        ("cube-detach-return", "cube_detach_return", "cycle=2",
         "CUBE_CONTACT_ORIGIN"),
        ("door-detach-return", "door_detach", "cycle=2",
         "boundary=E0_RETURN"),
        ("finalize", "finalize_roundtrip", "cycle=2", "CUBE_CONTACT_ORIGIN"),
    ]
    for operation, method, cycle, boundary in expected:
        runtime = Runtime()
        marker = MODULE.execute_operation(
            operation, runtime, roundtrip=True)
        assert runtime.calls == [method]
        assert marker.startswith(
            f"H1_SUPPORT_ORCHESTRATOR_828=PASS operation={operation} ")
        assert cycle in marker
        assert boundary in marker
    with pytest.raises(MODULE.SupportError, match="OPERATION"):
        MODULE.execute_operation(
            "cube-attach-return", Runtime(), roundtrip=False)


def test_roundtrip_runtime_swaps_inside_and_origin_cube_boundaries():
    runtime = object.__new__(MODULE.GazeboRuntime)

    class Contract:
        boundaries = {
            "CUBE_CONTACT": {
                "robot_positions": {}, "cube_pose": {"id": "origin"}},
            "CUBE_RELEASE": {
                "robot_positions": {}, "cube_pose": {"id": "inside"}},
        }

    runtime.contract = Contract()
    runtime._require_ownership = lambda: None
    runtime._require_resident_plugin = lambda *_args: None
    support = {MODULE.DOOR_STATE_TOPIC: "attached",
               MODULE.CUBE_STATE_TOPIC: "detached"}
    runtime._support_state = lambda topic: support[topic]
    calls = []
    runtime._boundary_gate = lambda name: calls.append(("boundary", name))
    runtime._left_contact_gate = lambda: calls.append(("contact", None))
    runtime._cube_pose_gate = lambda pose, label: calls.append(
        ("pose", pose["id"], label))
    runtime._cube_stability_gate = lambda pose, label: calls.append(
        ("stable", pose["id"], label))
    runtime._mechanism_gate = lambda *_args: calls.append(
        ("mechanism", _args[-1]))
    runtime._transition = lambda *args: calls.append(
        ("transition", args[1]))

    runtime.cube_attach_return()
    assert [row for row in calls if row[0] == "boundary"] == [
        ("boundary", "CUBE_RELEASE"), ("boundary", "CUBE_RELEASE")]
    assert any(row[:2] == ("pose", "inside") for row in calls)

    calls.clear()
    support[MODULE.CUBE_STATE_TOPIC] = "attached"
    runtime.cube_detach_return()
    assert [row for row in calls if row[0] == "boundary"] == [
        ("boundary", "CUBE_CONTACT"), ("boundary", "CUBE_CONTACT")]
    assert all(row[1] == "origin" for row in calls
               if row[0] in ("stable",))


def test_roundtrip_checkpoint_and_finalize_gate_inside_then_origin():
    runtime = object.__new__(MODULE.GazeboRuntime)

    class Contract:
        boundaries = {
            "CUBE_CONTACT": {"cube_pose": {"id": "origin"}},
            "CUBE_RELEASE": {"cube_pose": {"id": "inside"}},
        }

    runtime.contract = Contract()
    runtime._require_ownership = lambda: None
    runtime._require_resident_plugin = lambda *_args: None
    runtime._support_state = lambda _topic: "detached"
    calls = []
    runtime._mechanism_triplet_gate = lambda *_args: calls.append(
        ("mechanism", _args[-1]))
    runtime._boundary_gate = lambda name: calls.append(("boundary", name))
    runtime._cube_stability_gate = lambda pose, label: calls.append(
        ("stable", pose["id"], label))
    runtime._require_trajectory_topics_quiescent = lambda label: calls.append(
        ("quiescent", label))

    runtime.roundtrip_checkpoint()
    assert ("stable", "inside", "CHECKPOINT") in calls
    assert ("boundary", "FINAL_ROBOT") in calls
    assert ("quiescent", "CHECKPOINT") in calls

    calls.clear()
    runtime.finalize_roundtrip()
    assert ("stable", "origin", "FINAL_RETURN") in calls
    assert ("boundary", "FINAL_ROBOT") in calls
    assert ("quiescent", "FINAL") in calls


def test_source_has_only_support_mutations_and_no_direct_scene_control():
    source = SOURCE.read_text(encoding="utf-8")
    assert "shell=True" not in source
    assert "import rclpy" not in source
    assert "trajectory_msgs" not in source
    assert "/h1_2/incubator/simulation_direct_joint_trajectory" not in source
    assert "set_pose" not in source
    assert "cmd_vel" not in source
    assert "world_control" not in source
    assert "reset:" not in source
    assert "--ros-snapshot-tool" not in source
    for required in (
        MODULE.DOOR_ATTACH_TOPIC, MODULE.DOOR_DETACH_TOPIC,
        MODULE.CUBE_ATTACH_TOPIC, MODULE.CUBE_DETACH_TOPIC,
        "gz.msgs.Empty", "support_contract_sha256",
        "Publisher count: ([0-9]+)",
    ):
        assert required in source


def test_five_controller_topics_include_torso_and_both_hands():
    assert MODULE.TRAJECTORY_TOPICS == (
        "/torso_controller/joint_trajectory",
        "/right_arm_controller/joint_trajectory",
        "/right_gripper_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
        "/left_gripper_controller/joint_trajectory",
    )


def test_runtime_command_timeout_kills_entire_process_group(monkeypatch):
    events = []
    read_fd, write_fd = MODULE.os.pipe()

    class Process:
        pid = 6262
        returncode = None
        stdout = MODULE.os.fdopen(read_fd, "rb", buffering=0)
        stderr = None

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
    runtime = object.__new__(MODULE.GazeboRuntime)
    runtime.timeout_seconds = 0.01
    try:
        with pytest.raises(MODULE.SupportError, match="COMMAND_TIMEOUT:gz"):
            runtime._run(("gz", "topic", "-l"))
    finally:
        MODULE.os.close(write_fd)
    assert events == [(6262, 9)]


def test_listener_subprocesses_use_dedicated_process_groups():
    source = SOURCE.read_text(encoding="utf-8")
    assert source.count("start_new_session=True") >= 3
    assert "os.killpg(process.pid, 9)" in source
    assert "process.terminate()" not in source
