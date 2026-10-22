"""Offline contract tests for the persistent fast-831 support daemon."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import socket
import sys
import tempfile

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = WORKSPACE / "src/h1_2_moveit_demo/scripts/h1_fast_support_daemon_831.py"
SPEC = importlib.util.spec_from_file_location(
    "h1_fast_support_daemon_831_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_exact_released_markers_and_sequence():
    state = MODULE.OperationState()
    expected = (
        "door-attach", "cube-attach", "cube-detach", "door-detach",
        "finalize", "shutdown",
    )
    assert MODULE.COMMANDS == expected
    assert MODULE.TRANSITION_COMMANDS == expected[:4]
    for command in expected:
        state.admit(command)
        assert MODULE.PASS_MARKERS[command].startswith(
            "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=")
        state.commit(command)
    assert state.finalized is True


@pytest.mark.parametrize("command", [
    "cube-attach", "cube-detach", "door-detach", "finalize",
])
def test_out_of_order_is_fail_closed(command):
    with pytest.raises(MODULE.DaemonError, match="PROTOCOL:"):
        MODULE.OperationState().admit(command)


def test_shutdown_is_always_available_for_failure_cleanup():
    state = MODULE.OperationState()
    state.admit("shutdown")
    state.commit("shutdown")
    assert state.finalized is False


def test_finalize_is_exactly_once():
    state = MODULE.OperationState()
    for command in (*MODULE.TRANSITION_COMMANDS, "finalize"):
        state.admit(command)
        state.commit(command)
    with pytest.raises(MODULE.DaemonError, match="FINALIZE_ALREADY_DONE"):
        state.admit("finalize")


def test_line_protocol_is_exact_and_bounded():
    for command in MODULE.COMMANDS:
        assert MODULE._parse_request((command + "\n").encode()) == command
    for raw in (b"", b"door-attach", b"door-attach extra\n",
                b"door-attach\nshutdown\n", b"\xff\n"):
        with pytest.raises(MODULE.DaemonError, match="PROTOCOL:"):
            MODULE._parse_request(raw)


def test_roundtrip_protocol_has_two_explicit_cycles_and_checkpoint():
    assert MODULE.ROUNDTRIP_PROTOCOL == (
        "door-attach", "cube-attach", "cube-detach", "door-detach",
        "cycle-checkpoint",
        "door-attach-return", "cube-attach-return",
        "cube-detach-return", "door-detach-return",
    )
    state = MODULE.OperationState(roundtrip=True)
    for command in (*MODULE.ROUNDTRIP_PROTOCOL, "finalize", "shutdown"):
        state.admit(command)
        assert command in MODULE.ROUNDTRIP_PASS_MARKERS
        state.commit(command)
    assert state.finalized is True
    assert "cycle=1" in MODULE.ROUNDTRIP_PASS_MARKERS["cube-detach"]
    assert "boundary=CUBE_RELEASE_INSIDE" in (
        MODULE.ROUNDTRIP_PASS_MARKERS["cycle-checkpoint"])
    assert "cycle=2" in MODULE.ROUNDTRIP_PASS_MARKERS["cube-attach-return"]
    assert "boundary=CUBE_CONTACT_ORIGIN" in (
        MODULE.ROUNDTRIP_PASS_MARKERS["finalize"])


def test_roundtrip_mode_and_cycle_order_fail_closed():
    with pytest.raises(MODULE.DaemonError, match="PROTOCOL:COMMAND"):
        MODULE.OperationState().admit("door-attach-return")

    state = MODULE.OperationState(roundtrip=True)
    with pytest.raises(MODULE.DaemonError, match="PROTOCOL:ORDER"):
        state.admit("door-attach-return")
    for command in MODULE.TRANSITION_COMMANDS:
        state.admit(command)
        state.commit(command)
    with pytest.raises(MODULE.DaemonError, match="PROTOCOL:ORDER"):
        state.admit("door-attach-return")
    state.admit("cycle-checkpoint")
    state.commit("cycle-checkpoint")
    with pytest.raises(MODULE.DaemonError, match="PROTOCOL:ORDER"):
        state.admit("cycle-checkpoint")


def test_roundtrip_line_protocol_is_mode_explicit():
    for command in MODULE.ROUNDTRIP_COMMANDS:
        assert MODULE._parse_request(
            (command + "\n").encode(), roundtrip=True) == command
    for command in (
            "cycle-checkpoint", "door-attach-return",
            "cube-attach-return", "cube-detach-return",
            "door-detach-return"):
        with pytest.raises(MODULE.DaemonError, match="PROTOCOL:COMMAND"):
            MODULE._parse_request((command + "\n").encode())


def test_ready_claim_frame_is_exact_run_bound_and_non_replayable():
    run_id = "a" * 64
    nonce = "b" * 64
    assert MODULE._parse_ready_claim(
        f"claim-ready {run_id} {nonce}\n".encode(), run_id) == nonce
    for raw in (
            f"claim-ready {'c' * 64} {nonce}\n".encode(),
            f"claim-ready {run_id} short\n".encode(),
            f"claim-ready {run_id} {nonce}\nextra\n".encode()):
        with pytest.raises(MODULE.DaemonError, match="READY_CLAIM"):
            MODULE._parse_ready_claim(raw, run_id)


def test_required_ready_claim_blocks_mutation_until_claimed():
    engine = object.__new__(MODULE.SupportEngine)
    engine.require_ready_claim = True
    engine.ready_claimed = False
    with pytest.raises(MODULE.DaemonError, match="READY_CLAIM_REQUIRED"):
        engine.dispatch("door-attach")


def test_ready_claim_reports_stable_path_socket_identity():
    with tempfile.TemporaryDirectory(prefix="h1_", dir="/tmp") as directory:
        path = Path(directory) / "support.sock"
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        replacement = None
        try:
            server.bind(str(path))
            server.listen(1)
            os.chmod(path, 0o600)
            expected = path.lstat()

            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                client.connect(str(path))
                accepted, _ = server.accept()
                accepted.close()
            finally:
                client.close()

            current = MODULE._verified_socket_path_metadata(path, expected)
            assert (current.st_dev, current.st_ino) == (
                expected.st_dev, expected.st_ino)
            # A listening socket fd has a kernel socket inode, not the
            # filesystem inode that the client sees at the bound pathname.
            descriptor = os.fstat(server.fileno())
            assert (descriptor.st_dev, descriptor.st_ino) != (
                expected.st_dev, expected.st_ino)

            path.unlink()
            replacement = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            replacement.bind(str(path))
            replacement.listen(1)
            os.chmod(path, 0o600)
            with pytest.raises(
                    MODULE.DaemonError, match="SOCKET_PATH_IDENTITY"):
                MODULE._verified_socket_path_metadata(path, expected)
        finally:
            if replacement is not None:
                replacement.close()
            server.close()


def test_roundtrip_each_mutation_topic_is_admitted_exactly_twice(monkeypatch):
    topic = "/h1_2/left_pose_grasp/attach"
    engine = object.__new__(MODULE.SupportEngine)
    engine._command_counts = {topic: 0}
    engine.timeout = 1.0
    calls = []

    class Listener:
        def begin_transition(self, before, after):
            calls.append(("begin", before, after))
            return 17

        def finish_transition(self, after, sequence, timeout):
            calls.append(("finish", after, sequence, timeout))

    monkeypatch.setattr(MODULE, "_process_start_ticks", lambda *_args: 23)

    def run(command, **kwargs):
        calls.append(("run", command[-1], kwargs["timeout"]))
        return MODULE.types.SimpleNamespace(
            returncode=0, stdout=b"", stderr=b"")

    monkeypatch.setattr(MODULE.subprocess, "run", run)
    listener = Listener()
    engine._transition(listener, topic, "detached", "attached", 1)
    engine._transition(listener, topic, "detached", "attached", 2)
    assert engine._command_counts[topic] == 2
    assert [row for row in calls if row[0] == "run"] == [
        ("run", topic, 1.0), ("run", topic, 1.0)]
    with pytest.raises(MODULE.DaemonError, match="COMMAND_OCCURRENCE_ORDER"):
        engine._transition(listener, topic, "detached", "attached", 2)
    with pytest.raises(MODULE.DaemonError, match="COMMAND_OCCURRENCE"):
        engine._transition(listener, topic, "detached", "attached", 3)


def test_roundtrip_checkpoint_and_finalize_gate_inside_then_origin():
    events = []

    class Listener:
        sequence = 31

        def enforce(self, expected):
            events.append(("enforce", expected))

        def wait_new_state(self, expected, timeout, sequence):
            events.append(("heartbeat", expected, timeout, sequence))

    class Telemetry:
        def require_trajectory_topics_quiescent(self, topics, timeout):
            events.append(("quiescent", topics, timeout))

    engine = object.__new__(MODULE.SupportEngine)
    engine.roundtrip = True
    engine.door = Listener()
    engine.cube = Listener()
    engine.timeout = 2.0
    engine.support = MODULE.types.SimpleNamespace(
        DOOR_CLOSED_TOLERANCE=0.01,
        TRAJECTORY_TOPICS=("topic-a", "topic-b"),
    )
    engine.runtime = MODULE.types.SimpleNamespace(
        contract=MODULE.types.SimpleNamespace(boundaries={
            "CUBE_CONTACT": {"cube_pose": {"id": "origin"}},
            "CUBE_RELEASE": {"cube_pose": {"id": "inside"}},
        }))
    engine.telemetry = Telemetry()
    engine._mechanism_triplet_gate = lambda *args: events.append(
        ("mechanism", args[-1]))
    engine._boundary_gate = lambda boundary: events.append(
        ("boundary", boundary))
    engine._cube_stability_gate = lambda pose, label: events.append(
        ("stable", pose["id"], label))
    engine._command_counts = {name: 1 for name in "abcd"}

    engine.roundtrip_checkpoint()
    assert ("stable", "inside", "CHECKPOINT") in events
    assert ("boundary", "FINAL_ROBOT") in events
    assert ("quiescent", ("topic-a", "topic-b"), 2.0) in events

    events.clear()
    engine._command_counts = {name: 2 for name in "abcd"}
    engine.finalize()
    assert ("stable", "origin", "FINAL_RETURN") in events
    assert ("boundary", "FINAL_ROBOT") in events
    assert ("quiescent", ("topic-a", "topic-b"), 2.0) in events


def test_return_boundary_waits_for_three_fresh_three_mrad_samples():
    samples = iter((0.0035, 0.0029, 0.0028, 0.0027))

    class Telemetry:
        sequence = 0

        def sequences(self):
            return (self.sequence, 0, 0)

        def next_robot(self, _after, _timeout):
            self.sequence += 1
            return MODULE.types.SimpleNamespace(
                sequence=self.sequence,
                positions={"left_elbow_joint": 1.0 + next(samples)})

    engine = object.__new__(MODULE.SupportEngine)
    engine.timeout = 1.0
    engine.support = MODULE.types.SimpleNamespace(
        ROBOT_ENDPOINT_TOLERANCE=0.003,
        RIGHT_FINGER_ENDPOINT_TOLERANCE=0.0015)
    engine.runtime = MODULE.types.SimpleNamespace(
        contract=MODULE.types.SimpleNamespace(boundaries={
            "CUBE_CONTACT": {
                "robot_positions": {"left_elbow_joint": 1.0}},
        }))
    engine.telemetry = Telemetry()

    engine._settled_boundary_gate("CUBE_CONTACT")
    assert engine.telemetry.sequence == 4


def test_cube_stability_restarts_instead_of_failing_first_transient(
        monkeypatch):
    poses = iter((0.0, 0.0067, 0.0029, 0.0020, 0.0015, 0.0010))
    calls = []

    class Runtime:
        def _entity(self, name):
            assert name == "green_cube"
            return name

        def _pose_from_entity(self, _entity):
            value = next(poses)
            calls.append(value)
            return value

        @staticmethod
        def _pose_error(actual, expected):
            return abs(actual - expected), 0.0

    monkeypatch.setattr(MODULE.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: 0.0)
    engine = object.__new__(MODULE.SupportEngine)
    engine.timeout = 5.0
    engine.runtime = Runtime()
    engine.support = MODULE.types.SimpleNamespace(
        CUBE_POSITION_TOLERANCE=0.003,
        CUBE_ORIENTATION_TOLERANCE=0.035)

    engine._cube_stability_gate(0.0, "RETURN")
    assert calls == [0.0, 0.0067, 0.0029, 0.0020, 0.0015, 0.0010]


def test_native_state_parser_rejects_unknown_or_malformed():
    parse = MODULE.NativeStateListener._parse_line
    assert parse('data: "attached"') == "attached"
    assert parse("data: detached") == "detached"
    assert parse("[INFO] native listener ready") is None
    with pytest.raises(MODULE.DaemonError):
        parse('data: "unknown"')
    with pytest.raises(MODULE.DaemonError):
        parse("unframed garbage")


def test_source_has_only_read_only_ros_api_and_declared_topics():
    source = SOURCE.read_text(encoding="utf-8")
    assert ".create_publisher(" not in source
    assert source.count(".create_subscription(") == 3
    for topic in (
            '"/joint_states"', '"/incubator/joint_states"',
            '"/incubator/latch/state"'):
        assert topic in source
    assert '"-m", "gz.msgs.Empty"' in source
    assert "set_pose" not in source.lower()


def test_builtin_self_test(capsys):
    assert MODULE.main(["self-test"]) == 0
    assert capsys.readouterr().out.strip() == (
        "H1_FAST_SUPPORT_DAEMON_831_SELF_TEST=PASS "
        "commands=6 ros_publishers=0 failure_latch=enabled")
