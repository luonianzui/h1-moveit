"""ROS-free tests for the fail-closed Candidate 3 live supervisor 829."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import py_compile
import signal
import sqlite3
import subprocess
import sys
import time
from types import ModuleType, SimpleNamespace

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = (WORKSPACE / "src/h1_2_moveit_demo/scripts/"
          "h1_candidate3_live_supervisor_829.py")
MODULE = ModuleType("h1_candidate3_live_supervisor_829_test")
MODULE.__file__ = str(SOURCE)
sys.modules[MODULE.__name__] = MODULE
exec(compile(SOURCE.read_bytes(), str(SOURCE), "exec"), MODULE.__dict__)


def _canonical(value):
    return (json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def _sealed(path, value=b"evidence\n", mode=0o400):
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = value if type(value) is bytes else _canonical(value)
    path.write_bytes(raw)
    path.chmod(mode)
    return hashlib.sha256(raw).hexdigest()


def _right_authorization_value():
    return {
        "schema": MODULE.RIGHT_EXECUTION_AUTHORIZATION_SCHEMA,
        "controller_execution_authorized": True,
        "execution_ready": False,
        "live_execution_passed": False,
        "live_result": None,
        "controller_commands_issued": 0,
        "offline_only": True,
        "bindings": {},
        "controller_topics": [],
        "stage_order": {"prefix": [], "suffix": []},
        "limits": {},
        "auditor_version": "synthetic",
        "execution_topology": "synthetic",
    }


def _command_fixture(tmp_path):
    parent = (tmp_path / "runs").resolve()
    parent.mkdir(mode=0o700)
    run_root = parent / "run_829"
    run_id = "8" * 64
    fake_ros2 = (tmp_path / "bin" / "ros2.py").resolve()
    fake_flow = (tmp_path / "bin" / "flow.py").resolve()
    fake_support = (tmp_path / "bin" / "support.py").resolve()
    for script in (fake_ros2, fake_flow, fake_support):
        _sealed(script, b"# synthetic script\n", 0o500)

    paths = MODULE.EvidencePaths.from_root(run_root)
    supervisor_copy = (tmp_path / "tools" /
                       "h1_candidate3_live_supervisor_829.py").resolve()
    certificate_copy = (tmp_path / "tools" /
                        "h1_candidate3_live_certificate_829.py").resolve()
    _sealed(supervisor_copy, SOURCE.read_bytes(), 0o500)
    certificate_source = SOURCE.with_name(
        "h1_candidate3_live_certificate_829.py")
    _sealed(certificate_copy, certificate_source.read_bytes(), 0o500)
    evidence_tools = {
        "supervisor_source": supervisor_copy,
        "certificate_source": certificate_copy,
    }
    source_fcl_root = WORKSPACE / MODULE.EXACT_FCL_AUTHORITY_RELATIVE
    fcl_root = (tmp_path / MODULE.EXACT_FCL_AUTHORITY_RELATIVE).resolve()
    manifest = json.loads(
        (source_fcl_root / "manifest.json").read_text(encoding="utf-8"))
    _sealed(fcl_root / "manifest.json",
            (source_fcl_root / "manifest.json").read_bytes(), 0o400)
    fcl_files = {}
    for identifier in MODULE.EXACT_FCL_FILE_IDS:
        manifest_id = MODULE.EXACT_FCL_MANIFEST_FILE_IDS[identifier]
        source_path = WORKSPACE / manifest["files"][manifest_id]["path"]
        path = fcl_root / source_path.name
        _sealed(path, source_path.read_bytes(),
                0o500 if identifier == "auditor_binary" else 0o400)
        fcl_files[identifier] = path
    fcl_argv = [
        str(fcl_files["auditor_binary"]),
        "--scene", str(fcl_files["scene_cdr"]),
        "--path", str(fcl_files["contact_path"]),
        "--telemetry-tsv", str(paths.exact_fcl_telemetry),
        "--urdf", str(fcl_files["urdf"]),
        "--srdf", str(fcl_files["srdf"]),
        "--handle-stl", str(fcl_files["original_handle_stl"]),
    ]
    supervisor_sha = hashlib.sha256(supervisor_copy.read_bytes()).hexdigest()
    launch_formal_files = {}
    for identifier in (
            "gazebo_launch_source", "gazebo_world_sdf",
            "incubator_model_sdf", "robot_gazebo_control_xacro",
            "ros2_controllers_config", "robot_joint_state_relay_source",
            "telemetry_bridge_config", "rosbag_qos_overrides"):
        path = (tmp_path / "formal" / f"{identifier}.evidence").resolve()
        if identifier == "telemetry_bridge_config":
            raw = (WORKSPACE / "src/h1_2_moveit_config/config/"
                   "candidate3_telemetry_bridge_829.yaml").read_bytes()
        elif identifier == "rosbag_qos_overrides":
            raw = (WORKSPACE / "runtime_assets/"
                   "candidate3_rosbag_qos_overrides_829.yaml").read_bytes()
        else:
            raw = f"{identifier}\n".encode("ascii")
        _sealed(path, raw, 0o400)
        launch_formal_files[identifier] = path
    right_authorization_path = (
        tmp_path / "formal" / "right_execution_authorization.evidence"
    ).resolve()
    right_authorization_sha = _sealed(
        right_authorization_path, _right_authorization_value(), 0o400)
    right_manifest_path = (
        tmp_path / "formal" / "right_manifest.evidence").resolve()
    right_manifest_sha = _sealed(right_manifest_path, {
        "offline_only": True,
        "sealed_bindings": {
            "controller_execution_authorization": {
                "path": str(right_authorization_path),
                "sha256": right_authorization_sha,
                "mode_octal": "0400",
                "nlink": 1,
                "schema": MODULE.RIGHT_EXECUTION_AUTHORIZATION_SCHEMA,
            },
        },
    }, 0o400)
    right_authorization_flags = [
        "--right-manifest", str(right_authorization_path),
        "--right-manifest-sha256", right_authorization_sha,
    ]
    processes = {
        "launch": [
            "/usr/bin/python3", str(supervisor_copy), "_launch-child",
            "--self-sha256", supervisor_sha, "--plan", str(paths.plan),
            "--workspace", str(WORKSPACE), "--ros2", str(fake_ros2),
            "--gazebo-launch-source",
            str(launch_formal_files["gazebo_launch_source"]),
            "--gazebo-launch-source-sha256",
            hashlib.sha256(launch_formal_files[
                "gazebo_launch_source"].read_bytes()).hexdigest(),
            "--gazebo-world-sdf", str(launch_formal_files["gazebo_world_sdf"]),
            "--gazebo-world-sdf-sha256",
            hashlib.sha256(launch_formal_files[
                "gazebo_world_sdf"].read_bytes()).hexdigest(),
            "--incubator-model-sdf",
            str(launch_formal_files["incubator_model_sdf"]),
            "--incubator-model-sdf-sha256",
            hashlib.sha256(launch_formal_files[
                "incubator_model_sdf"].read_bytes()).hexdigest(),
            "--robot-gazebo-control-xacro",
            str(launch_formal_files["robot_gazebo_control_xacro"]),
            "--robot-gazebo-control-xacro-sha256",
            hashlib.sha256(launch_formal_files[
                "robot_gazebo_control_xacro"].read_bytes()).hexdigest(),
            "--ros2-controllers-config",
            str(launch_formal_files["ros2_controllers_config"]),
            "--ros2-controllers-config-sha256",
            hashlib.sha256(launch_formal_files[
                "ros2_controllers_config"].read_bytes()).hexdigest(),
            "--robot-joint-state-relay-source",
            str(launch_formal_files["robot_joint_state_relay_source"]),
            "--robot-joint-state-relay-source-sha256",
            hashlib.sha256(launch_formal_files[
                "robot_joint_state_relay_source"].read_bytes()).hexdigest(),
            "--telemetry-bridge-config",
            str(launch_formal_files["telemetry_bridge_config"]),
            "--telemetry-bridge-config-sha256",
            hashlib.sha256(launch_formal_files[
                "telemetry_bridge_config"].read_bytes()).hexdigest(),
            "spawn_incubator:=true", "table_x:=0.600",
            "incubator_x:=0.590", "incubator_y:=0.00", "incubator_z:=0.815",
            "cube_x:=0.334", "bridge_support_states:=false",
            "robot_joint_state_relay_source:=" +
            str(launch_formal_files["robot_joint_state_relay_source"]),
            "robot_joint_state_relay_sha256:=" + hashlib.sha256(
                launch_formal_files[
                    "robot_joint_state_relay_source"].read_bytes()
            ).hexdigest(),
            "telemetry_bridge_config:=" +
            str(launch_formal_files["telemetry_bridge_config"]),
            "telemetry_bridge_config_sha256:=" + hashlib.sha256(
                launch_formal_files[
                    "telemetry_bridge_config"].read_bytes()
            ).hexdigest()],
        "recorder": [
            "/usr/bin/python3", str(fake_ros2), "bag", "record",
            MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG, "-s", "sqlite3",
            "-o", str(paths.bag_directory), MODULE.RECORDER_QOS_FLAG,
            str(launch_formal_files["rosbag_qos_overrides"]),
            MODULE.RECORDER_CACHE_FLAG, str(MODULE.RECORDER_CACHE_BYTES),
            MODULE.RECORDER_TOPICS_FLAG,
            *MODULE.RECORDER_TOPICS],
        "video": [
            "/usr/bin/python3", str(supervisor_copy), "_video-child",
            "--self-sha256", supervisor_sha, "--run-id", run_id,
            "--plan", str(paths.plan),
            "--output", str(paths.video), "--window-audit",
            str(paths.video_window_audit), "--xwininfo", "/usr/bin/xwininfo",
            "--ffmpeg", "/usr/bin/ffmpeg", "--display", ":0",
            "--crop", "1160:1530:0:80", "--framerate", "12",
            "--crf", "20"],
        "prefix": [
            "/usr/bin/python3", str(fake_flow), "prefix",
            "--right-time-scale", "1.5", *right_authorization_flags],
        "middle_suffix": [
            "/usr/bin/python3", str(fake_flow), "middle-and-suffix",
            "--right-time-scale", "1.5", "--middle-speed-scale", "0.5",
            *right_authorization_flags],
        "postflight": [
            "/usr/bin/python3", str(fake_support), "finalize"],
    }
    commands = {
        "schema": MODULE.COMMAND_SCHEMA,
        "run_id": run_id,
        "marker_topic": MODULE.MARKER_TOPIC,
        "processes": [{"role": role, "argv": processes[role]}
                      for role in MODULE.PROCESS_ROLES],
        "exact_fcl_replay": {
            "argv": fcl_argv,
            "files": {identifier: str(fcl_files[identifier])
                      for identifier in MODULE.EXACT_FCL_FILE_IDS},
        },
        "valid": True,
        "status": "READY",
    }
    commands_path = (tmp_path / "commands_829.json").resolve()
    commands_sha = _sealed(commands_path, commands)
    authorities = {}
    for identifier in MODULE.FORMAL_AUTHORITY_IDS:
        path = launch_formal_files.get(identifier)
        if identifier == "right_execution_authorization":
            path = right_authorization_path
        elif identifier == "right_manifest":
            path = right_manifest_path
        if path is None:
            path = (tmp_path / "formal" / f"{identifier}.evidence").resolve()
            _sealed(path, f"{identifier}\n".encode("ascii"),
                    0o500 if identifier.endswith("source") else 0o400)
        authorities[identifier] = path
    return SimpleNamespace(
        run_root=run_root, run_id=run_id, paths=paths, commands=commands,
        commands_path=commands_path, commands_sha=commands_sha,
        authorities=authorities, evidence_tools=evidence_tools,
        fcl_files=fcl_files, right_authorization_sha=right_authorization_sha,
        right_manifest_sha=right_manifest_sha)


def test_atomic_publication_is_0400_single_link_and_no_replace(tmp_path):
    output = (tmp_path / "artifact.json").resolve()
    digest = MODULE._atomic_publish_no_replace(output, b"{}\n")
    assert digest == hashlib.sha256(b"{}\n").hexdigest()
    metadata = output.stat()
    assert metadata.st_mode & 0o777 == 0o400
    assert metadata.st_nlink == 1
    with pytest.raises(MODULE.SupervisorError, match="PUBLISH_EXISTS"):
        MODULE._atomic_publish_no_replace(output, b"changed\n")
    assert output.read_bytes() == b"{}\n"


def test_prepare_seals_exact_ordered_authorities_and_plan(tmp_path):
    fixture = _command_fixture(tmp_path)
    result = MODULE.prepare(
        run_root=fixture.run_root, commands_path=fixture.commands_path,
        authorities=fixture.authorities,
        evidence_tools=fixture.evidence_tools, monotonic_ns=123456789)
    assert result["status"] == "READY"
    assert result["commands_sha256"] == fixture.commands_sha
    plan, plan_sha = MODULE._read_canonical_sealed(fixture.paths.plan, "TEST")
    formal, formal_sha = MODULE._read_canonical_sealed(
        fixture.paths.formal, "TEST")
    assert plan_sha == result["plan_sha256"]
    assert formal_sha == result["formal_authorities_sha256"]
    assert [item["id"] for item in formal["authorities"]] == list(
        MODULE.FORMAL_AUTHORITY_IDS)
    assert MODULE.FORMAL_AUTHORITY_IDS[-5:] == (
        "robot_gazebo_control_xacro", "ros2_controllers_config",
        "robot_joint_state_relay_source", "telemetry_bridge_config",
        "rosbag_qos_overrides",
    )
    assert plan["sealed_monotonic_ns"] == 123456789
    assert plan["input_paths"] == fixture.paths.input_paths()
    assert plan["expected_exact_fcl_file_sha256"] == {
        identifier: hashlib.sha256(
            fixture.fcl_files[identifier].read_bytes()).hexdigest()
        for identifier in MODULE.EXACT_FCL_FILE_IDS}
    for path in (fixture.paths.plan, fixture.paths.formal):
        assert path.stat().st_mode & 0o777 == 0o400
        assert path.stat().st_nlink == 1
    with pytest.raises(MODULE.SupervisorError, match="RUN_ROOT_EXISTS"):
        MODULE.prepare(
            run_root=fixture.run_root, commands_path=fixture.commands_path,
            authorities=fixture.authorities,
            evidence_tools=fixture.evidence_tools)


def test_commands_reject_implicit_interpreter_and_missing_bag_topic(tmp_path):
    fixture = _command_fixture(tmp_path)
    fixture.run_root.mkdir(mode=0o700)
    fixture.paths.logs.mkdir(mode=0o700)
    value = json.loads(fixture.commands_path.read_text())
    value["processes"][0]["argv"][0] = str(
        Path(value["processes"][0]["argv"][1]))
    broken = (tmp_path / "implicit.json").resolve()
    _sealed(broken, value)
    with pytest.raises(MODULE.SupervisorError,
                       match="EXPLICIT_SYSTEM_PYTHON"):
        MODULE.load_commands(broken, fixture.paths)
    value = json.loads(fixture.commands_path.read_text())
    recorder = value["processes"][1]["argv"]
    recorder.remove("/incubator/joint_states")
    broken = (tmp_path / "missing_topic.json").resolve()
    _sealed(broken, value)
    with pytest.raises(MODULE.SupervisorError,
                       match="RECORDER_TOPIC:/incubator/joint_states"):
        MODULE.load_commands(broken, fixture.paths)


@pytest.mark.parametrize("flag_count", (0, 2))
def test_commands_require_exactly_one_include_unpublished_flag(
        tmp_path, flag_count):
    fixture = _command_fixture(tmp_path)
    fixture.run_root.mkdir(mode=0o700)
    fixture.paths.logs.mkdir(mode=0o700)
    value = json.loads(fixture.commands_path.read_text())
    recorder = value["processes"][1]["argv"]
    recorder.remove(MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG)
    recorder[recorder.index("record") + 1:recorder.index("record") + 1] = (
        [MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG] * flag_count)
    broken = (tmp_path / f"include_unpublished_{flag_count}.json").resolve()
    _sealed(broken, value)
    with pytest.raises(
            MODULE.SupervisorError,
            match="RECORDER_INCLUDE_UNPUBLISHED_TOPICS"):
        MODULE.load_commands(broken, fixture.paths)


@pytest.mark.parametrize(("mutation", "failure"), (
    ("positional", "RECORDER_TOPICS_FLAG"),
    ("duplicate_flag", "RECORDER_TOPICS_FLAG"),
    ("before_include", "RECORDER_TOPICS_FLAG"),
    ("swapped", "RECORDER_TOPICS_EXACT_ORDER"),
    ("trailing", "RECORDER_TOPICS_EXACT_ORDER"),
))
def test_commands_require_topics_flag_and_exact_terminal_topic_vector(
        tmp_path, mutation, failure):
    fixture = _command_fixture(tmp_path)
    fixture.run_root.mkdir(mode=0o700)
    fixture.paths.logs.mkdir(mode=0o700)
    value = json.loads(fixture.commands_path.read_text())
    recorder = value["processes"][1]["argv"]
    topics_index = recorder.index(MODULE.RECORDER_TOPICS_FLAG)
    if mutation == "positional":
        recorder.pop(topics_index)
    elif mutation == "duplicate_flag":
        recorder.insert(topics_index, MODULE.RECORDER_TOPICS_FLAG)
    elif mutation == "before_include":
        recorder.pop(topics_index)
        recorder.insert(
            recorder.index(MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG),
            MODULE.RECORDER_TOPICS_FLAG)
    elif mutation == "swapped":
        recorder[topics_index + 1], recorder[topics_index + 2] = (
            recorder[topics_index + 2], recorder[topics_index + 1])
    else:
        recorder.append("unexpected-trailing-argument")
    broken = (tmp_path / f"recorder_topics_{mutation}.json").resolve()
    _sealed(broken, value)
    with pytest.raises(MODULE.SupervisorError, match=failure):
        MODULE.load_commands(broken, fixture.paths)


@pytest.mark.parametrize(("mutation", "failure"), (
    ("missing_qos", "RECORDER_QOS"),
    ("wrong_qos_mode", "RECORDER_QOS_MODE"),
    ("missing_cache", "RECORDER_CACHE"),
    ("wrong_cache", "RECORDER_QOS_CACHE_CONTRACT"),
    ("qos_after_topics", "RECORDER_QOS_CACHE_ORDER"),
))
def test_commands_require_sealed_qos_and_exact_cache_before_topics(
        tmp_path, mutation, failure):
    fixture = _command_fixture(tmp_path)
    fixture.run_root.mkdir(mode=0o700)
    fixture.paths.logs.mkdir(mode=0o700)
    value = json.loads(fixture.commands_path.read_text())
    recorder = value["processes"][1]["argv"]
    if mutation == "missing_qos":
        index = recorder.index(MODULE.RECORDER_QOS_FLAG)
        del recorder[index:index + 2]
    elif mutation == "wrong_qos_mode":
        wrong_qos = (tmp_path / "formal" / "wrong_qos.yaml").resolve()
        _sealed(wrong_qos, b"/clock: {}\n", 0o600)
        recorder[recorder.index(MODULE.RECORDER_QOS_FLAG) + 1] = str(
            wrong_qos)
    elif mutation == "missing_cache":
        index = recorder.index(MODULE.RECORDER_CACHE_FLAG)
        del recorder[index:index + 2]
    elif mutation == "wrong_cache":
        recorder[recorder.index(MODULE.RECORDER_CACHE_FLAG) + 1] = str(
            MODULE.RECORDER_CACHE_BYTES - 1)
    else:
        index = recorder.index(MODULE.RECORDER_QOS_FLAG)
        qos_pair = recorder[index:index + 2]
        del recorder[index:index + 2]
        recorder.extend(qos_pair)
    broken = (tmp_path / f"recorder_contract_{mutation}.json").resolve()
    _sealed(broken, value)
    with pytest.raises(MODULE.SupervisorError, match=failure):
        MODULE.load_commands(broken, fixture.paths)


@pytest.mark.parametrize("identifier", (
    "robot_gazebo_control_xacro", "ros2_controllers_config",
    "robot_joint_state_relay_source", "telemetry_bridge_config",
))
def test_prepare_rejects_new_launch_authority_cross_binding_tamper(
        tmp_path, identifier):
    fixture = _command_fixture(tmp_path)
    authorities = dict(fixture.authorities)
    replacement = (tmp_path / "formal" /
                   f"replacement_{identifier}.evidence").resolve()
    _sealed(replacement, f"replacement:{identifier}\n".encode("ascii"))
    authorities[identifier] = replacement
    with pytest.raises(
            MODULE.SupervisorError,
            match=f"LAUNCH_FORMAL_CROSS_BIND:{identifier}"):
        MODULE.prepare(
            run_root=fixture.run_root, commands_path=fixture.commands_path,
            authorities=authorities, evidence_tools=fixture.evidence_tools)


def test_prepare_rejects_recorder_qos_authority_cross_binding_tamper(tmp_path):
    fixture = _command_fixture(tmp_path)
    authorities = dict(fixture.authorities)
    replacement = (tmp_path / "formal" / "replacement_qos.yaml").resolve()
    _sealed(replacement, b"/clock: {}\n")
    authorities["rosbag_qos_overrides"] = replacement
    with pytest.raises(
            MODULE.SupervisorError,
            match="RECORDER_FORMAL_QOS_CROSS_BIND"):
        MODULE.prepare(
            run_root=fixture.run_root, commands_path=fixture.commands_path,
            authorities=authorities, evidence_tools=fixture.evidence_tools)


@pytest.mark.parametrize("assignment", (
    "robot_joint_state_relay_source", "robot_joint_state_relay_sha256",
    "telemetry_bridge_config", "telemetry_bridge_config_sha256",
))
def test_commands_reject_telemetry_launch_assignment_tamper(
        tmp_path, assignment):
    fixture = _command_fixture(tmp_path)
    fixture.run_root.mkdir(mode=0o700)
    fixture.paths.logs.mkdir(mode=0o700)
    value = json.loads(fixture.commands_path.read_text())
    launch = value["processes"][0]["argv"]
    prefix = assignment + ":="
    index = next(index for index, item in enumerate(launch)
                 if item.startswith(prefix))
    launch[index] = prefix + ("f" * 64)
    broken = (tmp_path / f"launch_assignment_{assignment}.json").resolve()
    _sealed(broken, value)
    with pytest.raises(MODULE.SupervisorError,
                       match="LAUNCH_ARGUMENT_CONTRACT"):
        MODULE.load_commands(broken, fixture.paths)


def test_commands_reject_video_helper_bound_to_another_absolute_plan(tmp_path):
    fixture = _command_fixture(tmp_path)
    fixture.run_root.mkdir(mode=0o700)
    fixture.paths.logs.mkdir(mode=0o700)
    value = json.loads(fixture.commands_path.read_text())
    video = value["processes"][2]["argv"]
    plan_index = video.index("--plan") + 1
    video[plan_index] = str((tmp_path / "another_plan.json").resolve())
    broken = (tmp_path / "wrong_video_plan.json").resolve()
    _sealed(broken, value)
    with pytest.raises(MODULE.SupervisorError,
                       match="VIDEO_HELPER_PLAN_BINDING"):
        MODULE.load_commands(broken, fixture.paths)


def test_commands_reject_offline_manifest_as_controller_authorization(
        tmp_path):
    fixture = _command_fixture(tmp_path)
    fixture.run_root.mkdir(mode=0o700)
    fixture.paths.logs.mkdir(mode=0o700)
    value = json.loads(fixture.commands_path.read_text())
    for role in ("prefix", "middle_suffix"):
        argv = next(item["argv"] for item in value["processes"]
                    if item["role"] == role)
        argv[argv.index("--right-manifest") + 1] = str(
            fixture.authorities["right_manifest"])
        argv[argv.index("--right-manifest-sha256") + 1] = (
            fixture.right_manifest_sha)
    broken = (tmp_path / "offline_manifest_as_authorization.json").resolve()
    _sealed(broken, value)
    with pytest.raises(
            MODULE.SupervisorError,
            match="FLOW_RIGHT_EXECUTION_AUTHORIZATION_KEYS"):
        MODULE.load_commands(broken, fixture.paths)


def test_real_proc_identity_requires_explicit_interpreter(tmp_path):
    argv = ["/usr/bin/python3", "-c", "import time; time.sleep(30)"]
    process = subprocess.Popen(argv)
    try:
        identity = MODULE._read_proc_identity(
            process.pid, argv, expected_ppid=os.getpid(), timeout_s=1.0)
        assert identity["argv"] == argv
        assert identity["ppid"] == os.getpid()
        assert identity["start_ticks"] > 0
    finally:
        process.terminate()
        process.wait(timeout=5)

    script = (tmp_path / "sleep_script").resolve()
    script.write_text("#!/usr/bin/python3\nimport time\ntime.sleep(30)\n")
    script.chmod(0o500)
    shebang_argv = [str(script), "argument"]
    process = subprocess.Popen(shebang_argv)
    try:
        with pytest.raises(MODULE.SupervisorError,
                           match="PROC_ARGV_NOT_EXACT"):
            MODULE._read_proc_identity(
                process.pid, shebang_argv, expected_ppid=os.getpid(),
                timeout_s=0.1)
    finally:
        process.terminate()
        process.wait(timeout=5)


class _LaunchReadyClock:
    def __init__(self):
        self.now = 0.0
        self.spin_timeouts = []
        self.sleep_timeouts = []

    def monotonic(self):
        return self.now

    def spin_once(self, node, timeout_sec):
        del node
        self.spin_timeouts.append(timeout_sec)
        self.now += timeout_sec

    def sleep(self, timeout_s):
        self.sleep_timeouts.append(timeout_s)
        self.now += timeout_s


def _launch_ready_adapter(clock, controller_states):
    class Node:
        @staticmethod
        def get_publishers_info_by_topic(topic):
            assert topic in ("/joint_states", "/clock")
            return [object()]

    adapter = object.__new__(MODULE.RclpyLiveAdapter)
    adapter._robot = (1, {})
    adapter._robot_history = [
        (1, 100_000_000, MODULE.ALL_ROBOT_JOINTS),
        (2, 110_000_000, MODULE.ALL_ROBOT_JOINTS),
    ]
    adapter._incubator = (1, {})
    adapter._latch = (1, "LOCKED")
    adapter._clocks = [(1, 100), (2, 200)]
    adapter._rclpy = clock
    adapter.node = Node()
    calls = []

    def states(timeout_s):
        calls.append(timeout_s)
        index = min(len(calls) - 1, len(controller_states) - 1)
        return list(controller_states[index])

    adapter._controller_states = states
    return adapter, calls


def test_launch_ready_retries_partial_snapshots_until_exact_six_active(
        monkeypatch):
    clock = _LaunchReadyClock()
    adapter, calls = _launch_ready_adapter(clock, (
        ("active", "active"),
        ("active", "active", "active", "active", "inactive", "active"),
        ("active",) * 6,
    ))
    monkeypatch.setattr(MODULE.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(MODULE.time, "sleep", clock.sleep)

    adapter.wait_launch_ready(1.0)

    assert len(calls) == 3
    assert clock.now == pytest.approx(0.1)
    assert clock.spin_timeouts == []
    assert clock.sleep_timeouts == pytest.approx([0.05, 0.05])


@pytest.mark.parametrize("states", (("active",) * 5, ("active",) * 7),
                         ids=("missing_controller", "extra_controller"))
def test_launch_ready_persistent_nonexact_snapshot_fails_at_hard_timeout(
        monkeypatch, states):
    clock = _LaunchReadyClock()
    adapter, calls = _launch_ready_adapter(clock, (states,))
    monkeypatch.setattr(MODULE.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(MODULE.time, "sleep", clock.sleep)

    with pytest.raises(MODULE.SupervisorError, match="^LAUNCH_CONTROLLERS$"):
        adapter.wait_launch_ready(0.16)

    assert clock.now == pytest.approx(0.16)
    assert len(calls) == 4
    assert clock.spin_timeouts == []
    assert clock.sleep_timeouts == pytest.approx([0.05, 0.05, 0.05, 0.01])


class _Publisher:
    def __init__(self):
        self.messages = []

    def wait_for_recorder(self, timeout_s):
        del timeout_s
        return {}

    def publish(self, payload):
        self.messages.append(payload)


def test_marker_sequence_is_strict_and_sqlite_timestamps_are_authoritative(
        tmp_path):
    specs = (
        SimpleNamespace(
            name="CAPTURE_START", process_role="supervisor",
            pattern=__import__("re").compile(r"START\Z")),
        SimpleNamespace(
            name="STAGE", process_role="prefix",
            pattern=__import__("re").compile(r"PASS [0-9]+\Z")),
        SimpleNamespace(
            name="CAPTURE_END", process_role="supervisor",
            pattern=__import__("re").compile(r"END\Z")),
    )
    publisher = _Publisher()
    sequencer = MODULE.MarkerSequencer("9" * 64, specs, publisher)
    sequencer.supervisor("CAPTURE_START", "START")
    sequencer.child_line("prefix", "ordinary diagnostic")
    sequencer.child_line("prefix", "PASS 12")
    sequencer.supervisor("CAPTURE_END", "END")
    sequencer.complete()
    assert publisher.messages == [
        MODULE._canonical_bytes(item).decode("ascii").rstrip("\n")
        for item in sequencer.payloads]

    bag = (tmp_path / "markers.db3").resolve()
    connection = sqlite3.connect(bag)
    connection.execute(
        "CREATE TABLE topics (id INTEGER PRIMARY KEY,name TEXT,type TEXT,"
        "serialization_format TEXT)")
    connection.execute(
        "CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,"
        "timestamp INTEGER,data BLOB)")
    connection.execute(
        "INSERT INTO topics VALUES (1,?,?,?)",
        (MODULE.MARKER_TOPIC, MODULE.MARKER_TOPIC_TYPE, "cdr"))
    for index, payload in enumerate(publisher.messages):
        connection.execute(
            "INSERT INTO messages VALUES (?,?,?,?)",
            (index + 1, 1, 1000 + index * 10,
             sqlite3.Binary(payload.encode("ascii"))))
    connection.commit()
    connection.close()
    bag.chmod(0o400)

    class Decoder:
        @staticmethod
        def decode(payload):
            return payload.decode("ascii")

    raw = MODULE.extract_marker_jsonl(
        bag_path=bag, marker_topic=MODULE.MARKER_TOPIC,
        expected_payloads=sequencer.payloads, decoder=Decoder(),
        marker_schema="marker/final")
    values = [json.loads(line) for line in raw.splitlines()]
    assert [value["bag_timestamp_ns"] for value in values] == [1000, 1010, 1020]
    assert [value["sequence"] for value in values] == [0, 1, 2]

    broken = MODULE.MarkerSequencer("9" * 64, specs, _Publisher())
    with pytest.raises(MODULE.SupervisorError, match="MARKER_OUT_OF_ORDER"):
        broken.child_line("prefix", "PASS 2")


def test_marker_sequence_uses_current_position_for_duplicate_patterns(
        monkeypatch):
    marker_pattern = __import__("re").compile(r"Q90 PASS\Z")
    specs = (
        SimpleNamespace(
            name="CAPTURE_START", process_role="supervisor",
            pattern=__import__("re").compile(r"START\Z")),
        SimpleNamespace(
            name="Q90_BEFORE_MIDDLE", process_role="middle_suffix",
            pattern=marker_pattern),
        SimpleNamespace(
            name="MIDDLE_PASS", process_role="middle_suffix",
            pattern=__import__("re").compile(r"MIDDLE PASS\Z")),
        SimpleNamespace(
            name="Q90_AFTER_MIDDLE", process_role="middle_suffix",
            pattern=marker_pattern),
        SimpleNamespace(
            name="CAPTURE_END", process_role="supervisor",
            pattern=__import__("re").compile(r"END\Z")),
    )
    monotonic_values = iter((100, 200, 300, 400, 500))
    monkeypatch.setattr(
        MODULE.time, "monotonic_ns", lambda: next(monotonic_values))
    publisher = _Publisher()
    sequencer = MODULE.MarkerSequencer("9" * 64, specs, publisher)

    sequencer.supervisor("CAPTURE_START", "START")
    sequencer.child_line("middle_suffix", "ordinary diagnostic")
    sequencer.child_line("middle_suffix", "Q90 PASS")
    assert [item["name"] for item in sequencer.payloads] == [
        "CAPTURE_START", "Q90_BEFORE_MIDDLE"]

    with pytest.raises(MODULE.SupervisorError, match="MARKER_OUT_OF_ORDER"):
        sequencer.child_line("middle_suffix", "Q90 PASS")
    assert len(sequencer.payloads) == 2
    assert len(publisher.messages) == 2

    sequencer.child_line("middle_suffix", "MIDDLE PASS")
    sequencer.child_line("middle_suffix", "Q90 PASS")
    sequencer.supervisor("CAPTURE_END", "END")
    sequencer.complete()

    assert [item["name"] for item in sequencer.payloads] == [
        "CAPTURE_START", "Q90_BEFORE_MIDDLE", "MIDDLE_PASS",
        "Q90_AFTER_MIDDLE", "CAPTURE_END"]
    assert [item["sequence"] for item in sequencer.payloads] == list(range(5))
    assert [item["monotonic_ns"] for item in sequencer.payloads] == [
        100, 200, 300, 400, 500]
    assert publisher.messages == [
        MODULE._canonical_bytes(item).decode("ascii").rstrip("\n")
        for item in sequencer.payloads]


def test_marker_sequence_rejects_future_unique_marker_without_skipping():
    specs = (
        SimpleNamespace(
            name="FIRST", process_role="middle_suffix",
            pattern=__import__("re").compile(r"FIRST\Z")),
        SimpleNamespace(
            name="SECOND", process_role="middle_suffix",
            pattern=__import__("re").compile(r"SECOND\Z")),
    )
    sequencer = MODULE.MarkerSequencer("9" * 64, specs, _Publisher())

    sequencer.child_line("middle_suffix", "ordinary diagnostic")
    with pytest.raises(MODULE.SupervisorError, match="MARKER_OUT_OF_ORDER"):
        sequencer.child_line("middle_suffix", "SECOND")
    assert sequencer.payloads == []


def _pose_frame(table_x=0.600, terminal_cube=False):
    root_half = math.sqrt(0.5)
    cube_position = ([0.611, -0.030, 0.995] if terminal_cube else
                     [0.334, 0.377, 0.830])
    cube_orientation = (
        [0.0, 0.0, 0.7071067811865475, 0.7071067811865476]
        if terminal_cube else [0.0, 0.0, root_half, root_half])
    values = {
        "h1_2": (1, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        "table": (2, [table_x, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        "incubator": (3, [0.590, 0.0, 0.815], [0.0, 0.0, 0.0, 1.0]),
        "green_cube": (4, cube_position, cube_orientation),
    }
    poses = []
    for name, (entity_id, position, orientation) in values.items():
        poses.append({
            "name": name, "id": entity_id,
            "position": dict(zip(("x", "y", "z"), position)),
            "orientation": dict(zip(("x", "y", "z", "w"), orientation)),
        })
    return {"pose": poses}


def test_workcell_pose_three_sample_pre_and_post_gates(tmp_path):
    del tmp_path
    calls = 0

    def runner(argv, **kwargs):
        nonlocal calls
        del kwargs
        assert argv == list(MODULE.POSE_COMMAND_ARGV)
        terminal_cube = calls >= 3
        calls += 1
        return SimpleNamespace(
            returncode=0,
            stdout=_canonical(_pose_frame(terminal_cube=terminal_cube)),
            stderr=b"")

    preflight = MODULE.sample_workcell_poses(
        phase="preflight", run_factory=runner)
    postflight = MODULE.sample_workcell_poses(
        phase="postflight", run_factory=runner)
    audit = MODULE.build_workcell_pose_audit(
        run_id="a" * 64, plan_sha256="d" * 64,
        preflight=preflight, postflight=postflight)
    assert audit["source_command_argv"] == list(MODULE.POSE_COMMAND_ARGV)
    assert len(audit["preflight_samples"]) == 3
    assert len(audit["postflight_samples"]) == 3
    assert all(tuple(sample["entities"]) == (
        "h1_2", "table", "incubator", "green_cube")
        for sample in audit["postflight_samples"])
    assert audit["terminal_cube_target"] == MODULE.TERMINAL_CUBE_TARGET
    assert len(audit["terminal_cube_sample_gates"]) == 3
    assert all(item["passed"] is True
               for item in audit["terminal_cube_sample_gates"])
    assert audit["terminal_cube_gate_passed"] is True
    postflight[2]["entities"]["table"]["position_m"][0] = 0.610
    with pytest.raises(MODULE.SupervisorError, match="POSE_STABILITY:table"):
        MODULE.validate_workcell_pose_samples(preflight, postflight)
    postflight[2]["entities"]["table"]["position_m"][0] = 0.600
    postflight[2]["entities"]["green_cube"]["position_m"][0] += 0.0031
    with pytest.raises(
            MODULE.SupervisorError,
            match="POSE_POSTFLIGHT_GATE:green_cube"):
        MODULE.validate_workcell_pose_samples(preflight, postflight)
    postflight[2]["entities"]["green_cube"]["position_m"][0] -= 0.0031
    angle = math.radians(93.0) / 2.0
    postflight[2]["entities"]["green_cube"]["orientation_xyzw"] = [
        0.0, 0.0, math.sin(angle), math.cos(angle)]
    with pytest.raises(
            MODULE.SupervisorError,
            match="POSE_POSTFLIGHT_GATE:green_cube"):
        MODULE.validate_workcell_pose_samples(preflight, postflight)


def test_postflight_is_fail_closed():
    live = {
        "captured_monotonic_ns": 123,
        "door_angle_rad": 0.001,
        "handle_angle_rad": -0.001,
        "torso_angle_rad": math.radians(44.25),
        "latch_locked": True,
        "robot_at_launch": True,
        "controller_count": 6,
        "trajectory_publisher_count": 0,
        "clock_advancing": True,
    }
    result = MODULE.build_postflight(
        run_id="b" * 64, live=live, supports_detached=True,
        unexpected_executor_process_count=0)
    assert result["status"] == "PASS"
    live["torso_angle_rad"] = math.radians(44.50001)
    with pytest.raises(MODULE.SupervisorError, match="POSTFLIGHT_NOT_PASS"):
        MODULE.build_postflight(
            run_id="b" * 64, live=live, supports_detached=True,
            unexpected_executor_process_count=0)


def test_video_probe_and_full_decode_are_both_required(tmp_path):
    video = (tmp_path / "capture.mp4").resolve()
    _sealed(video, b"synthetic mp4", 0o400)
    plan_sha = "d" * 64
    window = (tmp_path / "window.json").resolve()
    camera = (tmp_path / "camera.json").resolve()
    _sealed(window, {
        "schema": MODULE.VIDEO_WINDOW_SCHEMA, "run_id": "c" * 64,
        "sealed_plan_sha256": plan_sha,
        "capture_started_monotonic_ns": 100,
        "window_width": 2000, "window_height": 1690,
        "width": 1160, "height": 1530, "crop": "1160:1530:0:80",
        "valid": True, "status": "CAPTURE_STARTED",
    })
    _sealed(camera, {
        "schema": MODULE.GUI_CAMERA_SCHEMA, "run_id": "c" * 64,
        "sealed_plan_sha256": plan_sha, "captured_monotonic_ns": 50,
        "valid": True, "status": "PASS",
    })
    probe_value = {
        "streams": [{
            "codec_name": "h264", "width": 1160, "height": 1530,
            "pix_fmt": "yuv420p", "avg_frame_rate": "12/1",
            "duration": "2.0", "nb_read_frames": "24"}],
        "format": {"duration": "2.0"},
    }
    calls = []

    def runner(argv, **kwargs):
        del kwargs
        calls.append(argv)
        if "-count_frames" in argv:
            return SimpleNamespace(
                returncode=0, stdout=json.dumps(probe_value).encode(), stderr=b"")
        return SimpleNamespace(returncode=0, stdout=b"", stderr=b"")

    result = MODULE.audit_video(
        video_path=video, run_id="c" * 64,
        started_monotonic_ns=90, ended_monotonic_ns=200,
        plan_sha256=plan_sha, window_audit_path=window,
        gui_camera_audit_path=camera,
        ffprobe=Path("/usr/bin/ffprobe"), ffmpeg=Path("/usr/bin/ffmpeg"),
        run_factory=runner)
    assert result["ffprobe"]["frame_count"] == 24
    assert result["decode"]["stderr_sha256"] == MODULE.EMPTY_SHA256
    assert len(calls) == 2


def test_video_child_rejects_nonfixed_window_before_ffmpeg(
        tmp_path, monkeypatch, capsys):
    plan_path = (tmp_path / "plan.json").resolve()
    output_path = (tmp_path / "capture.mp4").resolve()
    window_audit_path = (tmp_path / "window.json").resolve()
    run_id = "4" * 64
    source_sha = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    child_sys_argv = [str(SOURCE), "_video-child", "synthetic"]
    helper_argv = [sys.executable, *child_sys_argv]
    _sealed(plan_path, {
        "run_id": run_id,
        "evidence_tools": {
            "supervisor_source": {
                "path": str(SOURCE), "sha256": source_sha,
                "size_bytes": SOURCE.stat().st_size, "mode": "0500",
            },
        },
        "expected_process_argv_sha256": {
            "video": MODULE._argv_sha256(helper_argv),
        },
    })
    args = SimpleNamespace(
        self_sha256=source_sha, run_id=run_id, plan=plan_path,
        output=output_path, window_audit=window_audit_path,
        xwininfo=Path("/usr/bin/xwininfo"),
        ffmpeg=Path("/usr/bin/ffmpeg"), display=":0",
        crop="1160:1530:0:80", framerate=12, crf=20)
    calls = []

    def run_mock(argv, **kwargs):
        del kwargs
        calls.append(argv)
        if "-tree" in argv:
            return SimpleNamespace(
                returncode=0,
                stdout=b'0x123 ("gz-sim-gui" "Gazebo GUI")\n',
                stderr=b"")
        return SimpleNamespace(
            returncode=0, stdout=b"Width: 1998\nHeight: 1690\n",
            stderr=b"")

    def forbidden_popen(*args, **kwargs):
        del args, kwargs
        raise AssertionError("ffmpeg must not start for a nonfixed window")

    monkeypatch.setattr(sys, "argv", child_sys_argv)
    monkeypatch.setattr(MODULE.subprocess, "run", run_mock)
    monkeypatch.setattr(MODULE.subprocess, "Popen", forbidden_popen)
    assert MODULE._video_child(args) == 1
    assert "VIDEO_CHILD_WINDOW_FIXED_DIMENSIONS" in capsys.readouterr().err
    assert len(calls) == 2
    assert not output_path.exists()
    assert not window_audit_path.exists()


def test_spawn_child_uses_planned_argv_and_direct_parent_with_mock(tmp_path):
    root = (tmp_path / "run").resolve()
    root.mkdir(mode=0o700)
    paths = MODULE.EvidencePaths.from_root(root)
    paths.logs.mkdir(mode=0o700)
    argv = ("/usr/bin/python3", "-c", "pass")
    captured = {}

    class FakeProcess:
        pid = 777
        stdout = None
        stdin = None

    def popen_factory(actual, **kwargs):
        captured["argv"] = actual
        captured["kwargs"] = kwargs
        return FakeProcess()

    def identity(pid, expected, **kwargs):
        assert pid == 777
        assert tuple(expected) == argv
        assert kwargs["expected_ppid"] == os.getpid()
        return {
            "pid": pid, "ppid": os.getpid(), "start_ticks": 99,
            "argv": list(argv), "cmdline_sha256": MODULE._argv_sha256(argv),
        }

    child = MODULE._spawn_child(
        "launch", argv, paths, foreground=False,
        popen_factory=popen_factory, proc_identity_reader=identity)
    assert captured["argv"] == list(argv)
    assert captured["kwargs"]["start_new_session"] is True
    assert captured["kwargs"]["umask"] == 0o077
    assert child.identity["ppid"] == os.getpid()
    child.stdout_staged.unlink()
    child.stderr_staged.unlink()


def test_spawn_child_forces_owner_only_creation_under_parent_umask_0002(
        tmp_path):
    root = (tmp_path / "run").resolve()
    root.mkdir(mode=0o700)
    paths = MODULE.EvidencePaths.from_root(root)
    paths.logs.mkdir(mode=0o700)
    output = (tmp_path / "child-output").resolve()
    argv = (
        "/usr/bin/python3", "-c",
        "from pathlib import Path; import time; "
        f"Path({str(output)!r}).write_bytes(b'evidence'); time.sleep(0.5)",
    )
    previous_umask = os.umask(0o002)
    try:
        child = MODULE._spawn_child(
            "launch", argv, paths, foreground=False)
        assert child.process.wait(timeout=5.0) == 0
    finally:
        os.umask(previous_umask)
    assert output.stat().st_mode & 0o777 == 0o600
    child.stdout_staged.unlink()
    child.stderr_staged.unlink()


@pytest.mark.parametrize(("raw", "expected"), (
    (b"ordinary rosbag diagnostic\n", {
        "transport_layer": None, "recorder": None}),
    (b"[WARN] [123.456] [rosbag2_recorder]: Number of messages lost "
     b"on the transport layer: 0\n", {
         "transport_layer": 0, "recorder": None}),
    (b"Number of messages lost on the transport layer: 0\n"
     b"[WARN] [123.456] [rosbag2_recorder]: Number of messages lost in "
     b"the recorder: 0\n", {
         "transport_layer": 0, "recorder": 0}),
), ids=("diagnostics_absent", "transport_zero", "both_zero"))
def test_recorder_loss_gate_accepts_absent_or_explicit_zero(
        tmp_path, raw, expected):
    stderr = (tmp_path / ".recorder.stderr.partial").resolve()
    stderr.write_bytes(raw)
    stderr.chmod(0o600)

    assert MODULE._validate_recorder_loss_stderr(stderr) == expected
    assert stderr.read_bytes() == raw
    assert stderr.stat().st_mode & 0o777 == 0o600


@pytest.mark.parametrize(("diagnostic", "status"), (
    ("Number of messages lost on the transport layer: 2",
     "RECORDER_MESSAGES_LOST:transport_layer:2"),
    ("Number of messages lost in the recorder: 1",
     "RECORDER_MESSAGES_LOST:recorder:1"),
), ids=("transport_loss", "recorder_loss"))
def test_recorder_loss_gate_rejects_every_nonzero_count(
        tmp_path, diagnostic, status):
    stderr = (tmp_path / ".recorder.stderr.partial").resolve()
    stderr.write_text(
        f"[WARN] [123.456] [rosbag2_recorder]: {diagnostic}\n",
        encoding="utf-8")
    stderr.chmod(0o600)

    with pytest.raises(MODULE.SupervisorError, match=f"^{status}$"):
        MODULE._validate_recorder_loss_stderr(stderr)


@pytest.mark.parametrize("diagnostic", (
    "Number of messages lost on the transport layer: zero",
    "Number of messages lost on the transport layer 0",
    "Number of messages lost on the transport layer: -1",
    "Number of messages lost on the transport layer: 00",
    "Number of messages lost on the transport layer: 0 trailing",
    "Number of messages lost somewhere else: 0",
    "number of messages lost on the transport layer: 0",
), ids=("word_count", "missing_colon", "negative", "leading_zero",
        "trailing_text", "unknown_metric", "wrong_case"))
def test_recorder_loss_gate_rejects_malformed_approximate_lines(
        tmp_path, diagnostic):
    stderr = (tmp_path / ".recorder.stderr.partial").resolve()
    stderr.write_text(f"rosbag2: {diagnostic}\n", encoding="utf-8")
    stderr.chmod(0o600)

    with pytest.raises(
            MODULE.SupervisorError,
            match=r"^RECORDER_LOSS_DIAGNOSTIC_MALFORMED:1$"):
        MODULE._validate_recorder_loss_stderr(stderr)


@pytest.mark.parametrize("diagnostic", (
    "Number of messages lost on the transport layer: 0",
    "Number of messages lost in the recorder: 0",
), ids=("transport", "recorder"))
def test_recorder_loss_gate_rejects_duplicate_diagnostics(
        tmp_path, diagnostic):
    stderr = (tmp_path / ".recorder.stderr.partial").resolve()
    stderr.write_text(f"{diagnostic}\n{diagnostic}\n", encoding="utf-8")
    stderr.chmod(0o600)

    with pytest.raises(
            MODULE.SupervisorError,
            match=r"^RECORDER_LOSS_DIAGNOSTIC_DUPLICATE:"):
        MODULE._validate_recorder_loss_stderr(stderr)


def test_seal_existing_still_rejects_group_writable_initial_mode(tmp_path):
    evidence = (tmp_path / "metadata.yaml").resolve()
    evidence.write_bytes(b"evidence\n")
    evidence.chmod(0o664)
    with pytest.raises(MODULE.SupervisorError,
                       match="REGRESSION_INITIAL_MODE"):
        MODULE._seal_existing(evidence, "REGRESSION")
    assert evidence.stat().st_mode & 0o777 == 0o664


@pytest.mark.parametrize(("name", "status"), (
    ("metadata.yaml", "ROSBAG_METADATA"),
    ("candidate3.mp4", "VIDEO"),
))
def test_owner_only_child_output_seals_without_inode_or_content_change(
        tmp_path, name, status):
    evidence = (tmp_path / name).resolve()
    raw = b"sealed child evidence\n"
    evidence.write_bytes(raw)
    evidence.chmod(0o600)
    before = evidence.stat()
    binding = MODULE._seal_existing(evidence, status)
    after = evidence.stat()
    assert (after.st_dev, after.st_ino, after.st_size) == (
        before.st_dev, before.st_ino, before.st_size)
    assert after.st_mode & 0o777 == 0o400
    assert binding["sha256"] == hashlib.sha256(raw).hexdigest()


def test_certificate_loader_executes_sealed_source_and_ignores_forged_pyc(
        tmp_path, monkeypatch):
    source = (tmp_path / "sealed" /
              "h1_candidate3_live_certificate_829.py").resolve()

    def certificate_source(sentinel):
        values = {
            "PLAN_SCHEMA": MODULE.PLAN_SCHEMA,
            "FORMAL_SCHEMA": MODULE.FORMAL_SCHEMA,
            "PROC_SCHEMA": MODULE.PROC_SCHEMA,
            "VIDEO_SCHEMA": MODULE.VIDEO_SCHEMA,
            "VIDEO_WINDOW_SCHEMA": MODULE.VIDEO_WINDOW_SCHEMA,
            "GUI_CAMERA_SCHEMA": MODULE.GUI_CAMERA_SCHEMA,
            "POSTFLIGHT_SCHEMA": MODULE.POSTFLIGHT_SCHEMA,
            "WORKCELL_POSE_SCHEMA": MODULE.WORKCELL_POSE_SCHEMA,
            "EXACT_FCL_REPLAY_SCHEMA": MODULE.EXACT_FCL_REPLAY_SCHEMA,
            "PROCESS_ROLES": MODULE.PROCESS_ROLES,
            "FORMAL_AUTHORITY_IDS": MODULE.FORMAL_AUTHORITY_IDS,
            "INPUT_IDS": MODULE.INPUT_IDS,
            "EVIDENCE_TOOL_IDS": MODULE.EVIDENCE_TOOL_IDS,
            "FCL_PREBOUND_FILE_IDS": MODULE.EXACT_FCL_FILE_IDS,
            "SOURCE_SENTINEL": sentinel,
        }
        return ("\n".join(
            f"{name} = {value!r}" for name, value in values.items()) +
            "\n").encode("utf-8")

    forged_source = certificate_source("FORGED_BYTES")
    sealed_source = certificate_source("SOURCE_BYTES")
    assert len(forged_source) == len(sealed_source)
    source.parent.mkdir(mode=0o700)
    source.write_bytes(forged_source)
    fixed_timestamp = 1_700_000_000
    os.utime(source, (fixed_timestamp, fixed_timestamp))
    cache_directory = source.parent / "__pycache__"
    cache_directory.mkdir(mode=0o700)
    cache_path = cache_directory / (
        f"{source.stem}.{sys.implementation.cache_tag}.pyc")
    py_compile.compile(
        str(source), cfile=str(cache_path), doraise=True,
        invalidation_mode=py_compile.PycInvalidationMode.TIMESTAMP)
    cache_path.chmod(0o600)
    cache_before = cache_path.read_bytes()
    cache_mtime_before = cache_path.stat().st_mtime_ns

    source.write_bytes(sealed_source)
    os.utime(source, (fixed_timestamp, fixed_timestamp))
    source.chmod(0o500)
    monkeypatch.setattr(MODULE, "_CERTIFICATE_MODULE", None)
    monkeypatch.setattr(MODULE, "_BOUND_CERTIFICATE_SOURCE", source)
    previous_module = sys.modules.pop(MODULE.CERTIFICATE_MODULE_NAME, None)
    try:
        certificate = MODULE._certificate_module()
        assert certificate.SOURCE_SENTINEL == "SOURCE_BYTES"
        assert certificate.__file__ == str(source)
        assert certificate.__package__ == ""
        assert certificate.__loader__ is None
        assert certificate.__cached__ is None
        assert certificate.__spec__.name == MODULE.CERTIFICATE_MODULE_NAME
        assert certificate.__spec__.origin == str(source)
        assert certificate.__spec__.loader is None
        assert tuple(cache_directory.iterdir()) == (cache_path,)
        assert cache_path.read_bytes() == cache_before
        assert cache_path.stat().st_mtime_ns == cache_mtime_before
        assert cache_path.stat().st_mode & 0o777 == 0o600
    finally:
        sys.modules.pop(MODULE.CERTIFICATE_MODULE_NAME, None)
        if previous_module is not None:
            sys.modules[MODULE.CERTIFICATE_MODULE_NAME] = previous_module


def _strict_grip_metric_fixture():
    rows = [
        {
            "status": "PASS", "valid": "true",
            "collision_free": "true", "geometric_only": "true",
            "force_or_wrench_claimed": "false",
            "contact_required": "false", "pad1_patch_valid": "false",
            "pad2_patch_valid": "false",
            "right_pad_grip_engaged": "false", "pad1_contacts": "0",
            "pad2_contacts": "0", "pad1_handle_distance_m": "0.0004",
            "pad2_handle_distance_m": "0.0005",
            "nonpad_handle_clearance_m": "0.0007",
            "adjacent_finger_handle_clearance_m": "0.00001",
            "left_handle_clearance_m": "0.051",
        },
        {
            "status": "PASS", "valid": "true",
            "collision_free": "true", "geometric_only": "true",
            "force_or_wrench_claimed": "false",
            "contact_required": "true", "pad1_patch_valid": "false",
            "pad2_patch_valid": "false",
            "right_pad_grip_engaged": "true", "pad1_contacts": "0",
            "pad2_contacts": "1", "pad1_handle_distance_m": "0.0005",
            "pad2_handle_distance_m": "0.0002",
            "nonpad_handle_clearance_m": "0.0007",
            "adjacent_finger_handle_clearance_m": "0.00001",
            "left_handle_clearance_m": "0.051",
        },
    ]
    summary = {
        "status": "PASS", "valid": "true", "collision_free": "true",
        "geometric_only": "true", "force_or_wrench_claimed": "false",
        "reason": "NONE", "evaluated": "2", "passed": "2",
        "contact_required": "1", "contact_required_passed": "1",
        "noncontact": "1", "noncontact_passed": "1",
        "pad1_patch_valid": "0", "pad2_patch_valid": "0",
        "right_pad_grip_engaged": "1",
        "max_contact_minimum_right_pad_distance_m": "0.0002",
        "min_nonpad_handle_clearance_m": "0.0007",
        "min_adjacent_finger_handle_clearance_m": "0.00001",
        "min_left_handle_clearance_m": "0.051",
    }
    telemetry = SimpleNamespace(rows=(
        {"contact_required": False}, {"contact_required": True}))
    return rows, summary, telemetry


def test_strict_fcl_metrics_use_actual_pad_contacts_not_patch_validity():
    certificate = MODULE._certificate_module()
    rows, summary, telemetry = _strict_grip_metric_fixture()
    metrics = MODULE._strict_exact_fcl_grip_metrics(
        rows=rows, summary=summary, telemetry=telemetry,
        certificate=certificate)
    assert metrics["required_count"] == 1
    assert metrics["grip_engaged_count"] == 1
    assert metrics["pad1_patch_count"] == 0
    assert metrics["pad2_patch_count"] == 0
    assert metrics["observed_pairs"] == [
        list(certificate.ALLOWED_HANDLE_CONTACT_PAIRS[1])]


@pytest.mark.parametrize(("row_index", "field", "value", "error"), (
    (1, "right_pad_grip_engaged", "false", "GRIP_DISTANCE_BINDING"),
    (0, "pad1_handle_distance_m", "0.0003", "GRIP_DISTANCE_BINDING"),
    (1, "adjacent_finger_handle_clearance_m", "-0.000000001",
     "CLEARANCE"),
    (1, "nonpad_handle_clearance_m", "0.000599999", "CLEARANCE"),
    (1, "left_handle_clearance_m", "0.049999", "CLEARANCE"),
    (0, "pad1_contacts", "1", "NONCONTACT_PAD_CONTACT"),
    (1, "force_or_wrench_claimed", "true", "NOT_PASS"),
))
def test_strict_fcl_metrics_reject_grip_envelope_tampering(
        row_index, field, value, error):
    certificate = MODULE._certificate_module()
    rows, summary, telemetry = _strict_grip_metric_fixture()
    rows[row_index][field] = value
    with pytest.raises(MODULE.SupervisorError, match=error):
        MODULE._strict_exact_fcl_grip_metrics(
            rows=rows, summary=summary, telemetry=telemetry,
            certificate=certificate)


@pytest.mark.parametrize(("field", "value", "error"), (
    ("right_pad_grip_engaged", "0", "SUMMARY_COUNT"),
    ("max_contact_minimum_right_pad_distance_m", "0.0001",
     "SUMMARY_METRIC"),
    ("force_or_wrench_claimed", "true", "SUMMARY_NOT_PASS"),
))
def test_strict_fcl_metrics_reject_summary_tampering(field, value, error):
    certificate = MODULE._certificate_module()
    rows, summary, telemetry = _strict_grip_metric_fixture()
    summary[field] = value
    with pytest.raises(MODULE.SupervisorError, match=error):
        MODULE._strict_exact_fcl_grip_metrics(
            rows=rows, summary=summary, telemetry=telemetry,
            certificate=certificate)


def test_default_exact_fcl_execution_passes_real_certificate_validator(
        tmp_path, monkeypatch):
    certificate_source = SOURCE.with_name(
        "h1_candidate3_live_certificate_829.py")
    monkeypatch.setattr(MODULE, "_CERTIFICATE_MODULE", None)
    monkeypatch.setattr(MODULE, "_BOUND_CERTIFICATE_SOURCE",
                        certificate_source)
    certificate = MODULE._certificate_module()
    assert tuple(certificate.FCL_PREBOUND_FILE_IDS) == \
        MODULE.EXACT_FCL_FILE_IDS

    root = (tmp_path / "run").resolve()
    root.mkdir(mode=0o700)
    paths = MODULE.EvidencePaths.from_root(root)
    _sealed(paths.bag, b"sealed synthetic bag bytes\n")
    _sealed(paths.markers, b"sealed synthetic markers\n")

    authority_root = WORKSPACE / MODULE.EXACT_FCL_AUTHORITY_RELATIVE
    manifest = json.loads(
        (authority_root / "manifest.json").read_text(encoding="utf-8"))
    exact_files = {
        identifier: WORKSPACE / manifest["files"][manifest_id]["path"]
        for identifier, manifest_id in
        MODULE.EXACT_FCL_MANIFEST_FILE_IDS.items()
    }
    exact_argv = (
        str(exact_files["auditor_binary"]),
        "--scene", str(exact_files["scene_cdr"]),
        "--path", str(exact_files["contact_path"]),
        "--telemetry-tsv", str(paths.exact_fcl_telemetry),
        "--urdf", str(exact_files["urdf"]),
        "--srdf", str(exact_files["srdf"]),
        "--handle-stl", str(exact_files["original_handle_stl"]),
    )
    bundle = MODULE.CommandBundle(
        run_id="e" * 64, marker_topic=MODULE.MARKER_TOPIC, processes={},
        exact_fcl_argv=exact_argv, exact_fcl_files=exact_files,
        raw_sha256="f" * 64)
    telemetry_raw = (
        certificate.REPLAY_TSV_HEADER + "\n"
        "r0\tfalse\t0\t0\t0\t[]\t[]\t[]\t[]\t[]\n"
        "r1\ttrue\t45\t90\t0.1\t[]\t[]\t[]\t[]\t[]\n"
    ).encode("ascii")
    rows = (
        {"id": "r0", "contact_required": False,
         "door_angle_rad": 0.0, "handle_angle_rad": 0.0,
         "positions": {"torso_joint": 0.0}},
        {"id": "r1", "contact_required": True,
         "door_angle_rad": math.pi / 4.0,
         "handle_angle_rad": math.pi / 2.0,
         "positions": {"torso_joint": 0.1}},
    )
    telemetry = certificate.ReplayTelemetry(
        raw=telemetry_raw, rows=rows, source_sample_count=2,
        maximum_source_gap_ns=10_000_000,
        maximum_revolute_step_rad=0.01,
        maximum_linear_step_m=0.0001,
        close_command_timestamp_ns=100,
        close_pass_timestamp_ns=200,
        release_command_timestamp_ns=300,
        release_pass_timestamp_ns=400,
        excluded_close_transition_rows=1,
        excluded_release_transition_rows=1)

    geometry_sha = {
        "scene": hashlib.sha256(
            exact_files["scene_cdr"].read_bytes()).hexdigest(),
        "path": hashlib.sha256(
            exact_files["contact_path"].read_bytes()).hexdigest(),
        "urdf": hashlib.sha256(
            exact_files["urdf"].read_bytes()).hexdigest(),
        "srdf": hashlib.sha256(
            exact_files["srdf"].read_bytes()).hexdigest(),
        "handle": hashlib.sha256(
            exact_files["original_handle_stl"].read_bytes()).hexdigest(),
    }
    telemetry_sha = hashlib.sha256(telemetry_raw).hexdigest()

    def machine_line(prefix, keys, values):
        return prefix + " " + " ".join(
            f"{key}={values[key]}" for key in keys) + "\n"

    output_lines = []
    for index, row in enumerate(rows):
        required = row["contact_required"]
        values = {
            "id": row["id"], "status": "PASS", "valid": "true",
            "contact_required": str(required).lower(),
            "door_angle_deg": certificate._decimal(math.degrees(
                row["door_angle_rad"])),
            "handle_angle_deg": certificate._decimal(math.degrees(
                row["handle_angle_rad"])),
            "torso_rad": certificate._decimal(
                row["positions"]["torso_joint"]),
            "collision_free": "true",
            "pad1_patch_valid": str(required).lower(),
            "pad2_patch_valid": str(required).lower(),
            "right_pad_grip_engaged": str(required).lower(),
            "pad1_contacts": "4" if required else "0",
            "pad2_contacts": "4" if required else "0",
            "pad1_handle_distance_m": "0.0002" if required else "0.0004",
            "pad2_handle_distance_m": "0.0005" if required else "0.0004",
            "nonpad_handle_clearance_m": "0.0007",
            "adjacent_finger_handle_clearance_m": "0.00001",
            "left_handle_clearance_m": "0.051",
            "patch_span_m": "0.007" if required else "0",
            "patch_station_m": "0.0225" if required else "0",
            "patch_radius_m": "0.044" if required else "0",
            "geometric_only": "true", "force_or_wrench_claimed": "false",
        }
        output_lines.append(machine_line(
            "RIGHT_HANDLE_LIVE_FCL_ROW_829", certificate.FCL_ROW_KEYS,
            values))
    summary = {
        "status": "PASS", "valid": "true", "evaluated": "2",
        "passed": "2", "contact_required": "1",
        "contact_required_passed": "1", "noncontact": "1",
        "noncontact_passed": "1", "pad1_patch_valid": "1",
        "pad2_patch_valid": "1", "right_pad_grip_engaged": "1",
        "max_torso_rad": "0.1",
        "max_contact_minimum_right_pad_distance_m": "0.0002",
        "min_nonpad_handle_clearance_m": "0.0007",
        "min_adjacent_finger_handle_clearance_m": "0.00001",
        "min_left_handle_clearance_m": "0.051",
        "min_patch_span_m": "0.007", "min_patch_station_m": "0.0225",
        "min_patch_radius_m": "0.044", "collision_free": "true",
        "scene_sha256": geometry_sha["scene"],
        "path_sha256": geometry_sha["path"],
        "telemetry_sha256": telemetry_sha,
        "urdf_sha256": geometry_sha["urdf"],
        "srdf_sha256": geometry_sha["srdf"],
        "handle_stl_sha256": geometry_sha["handle"],
        "reason": "NONE", "geometric_only": "true",
        "force_or_wrench_claimed": "false",
    }
    output_lines.append(machine_line(
        "RIGHT_HANDLE_LIVE_FCL_SUMMARY_829", certificate.FCL_SUMMARY_KEYS,
        summary))
    auditor_stdout = "".join(output_lines).encode("ascii")

    plan = {
        "run_id": bundle.run_id,
        "expected_exact_fcl_replay_argv_sha256":
            MODULE._argv_sha256(exact_argv),
        "expected_exact_fcl_file_sha256": {
            identifier: hashlib.sha256(path.read_bytes()).hexdigest()
            for identifier, path in exact_files.items()},
    }
    monkeypatch.setattr(
        certificate, "_load_markers", lambda *args, **kwargs: (None, []))
    monkeypatch.setattr(
        certificate, "_analyze_rosbag",
        lambda *args, **kwargs: ({}, telemetry))

    def runner(argv, **kwargs):
        del kwargs
        assert tuple(argv) == exact_argv
        return SimpleNamespace(
            returncode=0, stdout=auditor_stdout, stderr=b"")

    value = MODULE.execute_exact_fcl_replay(
        paths=paths, bundle=bundle, plan=plan, run_factory=runner)
    assert value["scope"] == certificate._exact_fcl_scope()
    assert value["replay"]["excluded_close_transition_rows"] == 1
    assert value["replay"]["right_grip_engaged_samples"] == 1
    assert value["replay"][
        "maximum_contact_minimum_right_pad_distance_m"] == 0.0002
    assert value["replay"][
        "minimum_adjacent_finger_handle_clearance_m"] == 0.00001
    assert value["replay"][
        "minimum_buffered_nonpad_handle_clearance_m"] == 0.0007
    assert value["native_physics_contacts"][
        "force_or_wrench_claimed"] is False
    assert value["auditor"]["path_base_source"]["sha256"] == \
        plan["expected_exact_fcl_file_sha256"]["auditor_path_base_source"]
    assert paths.exact_fcl_replay.stat().st_mode & 0o777 == 0o400
