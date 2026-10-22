"""Ordering contracts for the complete two-cycle 831 round trip."""

import importlib.util
import sys
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = (
    WORKSPACE /
    "src/h1_2_moveit_demo/scripts/h1_candidate3_flow_fast_831_v3.py"
)
SPEC = importlib.util.spec_from_file_location(
    "h1_candidate3_roundtrip_flow_831_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _arguments():
    return [
        "roundtrip",
        "--workspace", str(WORKSPACE),
        "--right-executor", "/sealed/right-executor",
        "--right-executor-sha256", "1" * 64,
        "--right-manifest", "/sealed/right-manifest",
        "--right-manifest-sha256", "2" * 64,
        "--support-orchestrator", "/sealed/support",
        "--support-orchestrator-sha256", "3" * 64,
        "--support-manifest", "/sealed/support-manifest",
        "--support-manifest-sha256", "4" * 64,
        "--support-certificate", "/sealed/support-certificate",
        "--support-certificate-sha256", "5" * 64,
        "--scene", "/sealed/scene.cdr",
        "--scene-sha256", "6" * 64,
        "--plan", "/sealed/plan.json",
        "--plan-sha256", "7" * 64,
        "--certificate", "/sealed/support-certificate",
        "--certificate-sha256", "5" * 64,
        "--result", "/sealed/result.json",
        "--result-sha256", "8" * 64,
        "--middle-executor", "/sealed/middle-executor",
        "--middle-executor-sha256", "9" * 64,
        "--middle-admission", "/sealed/middle-admission",
        "--middle-admission-sha256", "a" * 64,
        "--middle-contract", "/sealed/middle-contract",
        "--middle-contract-sha256", "b" * 64,
        "--fast-support-daemon", "/sealed/support-daemon",
        "--fast-support-daemon-sha256", "c" * 64,
        "--return-plan", "/sealed/return-plan",
        "--return-plan-sha256", "d" * 64,
        "--return-authorization", "/sealed/return-authorization",
        "--return-authorization-sha256", "e" * 64,
        "--fast-support-daemon-socket", "/run/user/1000/roundtrip.sock",
    ]


def _preparation_arguments():
    result = _arguments()
    result[0] = "prepare-roundtrip"
    return result


def test_roundtrip_is_two_complete_right_hand_cycles_around_inverse_middle(
        monkeypatch, capsys):
    events = []
    monkeypatch.setattr(
        MODULE, "_run_support",
        lambda _args, operation, *, roundtrip=False:
        events.append(("support", operation, roundtrip)))
    monkeypatch.setattr(
        MODULE, "_wait_external_fast_support_socket",
        lambda path: events.append(("wait_socket", str(path))))
    monkeypatch.setattr(
        MODULE, "_fast_support_claim_ready",
        lambda _daemon, run_id:
        events.append(("claim", run_id)) or "CLAIM=PASS")
    monkeypatch.setattr(
        MODULE, "_run_right",
        lambda _args, direction: events.append(("right", direction)))
    monkeypatch.setattr(
        MODULE, "_check_right_e0",
        lambda _args, label: events.append(("e0", label)))
    monkeypatch.setattr(
        MODULE, "_run_middle",
        lambda _args, _daemon, direction="insert", *, roundtrip=False,
        execute=True:
        events.append(("middle", direction, roundtrip, execute)))
    monkeypatch.setattr(
        MODULE, "_check_right_launch",
        lambda _args: events.append(("launch",)))

    def support_request(_daemon, operation):
        events.append(("request", operation))
        return f"PASS:{operation}"

    monkeypatch.setattr(MODULE, "_fast_support_request", support_request)
    monkeypatch.setattr(
        MODULE, "_finish_external_fast_support_daemon",
        lambda _daemon: events.append(("finish_daemon",)))

    assert MODULE.main(_arguments()) == 0
    assert events == [
        ("middle", "return", True, False),
        ("support", "preload", True),
        ("wait_socket", "/run/user/1000/roundtrip.sock"),
        ("right", "prefix"),
        ("e0", "CYCLE1_BEFORE_MIDDLE"),
        ("middle", "insert", True, True),
        ("e0", "CYCLE1_AFTER_MIDDLE"),
        ("right", "suffix"),
        ("launch",),
        ("request", "cycle-checkpoint"),
        ("right", "prefix"),
        ("e0", "CYCLE2_BEFORE_MIDDLE"),
        ("middle", "return", True, True),
        ("e0", "CYCLE2_AFTER_MIDDLE"),
        ("right", "suffix"),
        ("launch",),
        ("request", "finalize"),
        ("finish_daemon",),
    ]
    output = capsys.readouterr().out
    assert "CANDIDATE3_ROUNDTRIP_831_CYCLE1=PASS" in output
    assert "CANDIDATE3_ROUNDTRIP_831=PASS cycles=2" in output


def test_return_middle_failure_prevents_second_close_and_finalization(
        monkeypatch):
    events = []
    middle_count = 0

    monkeypatch.setattr(MODULE, "_run_support", lambda *_, **__: None)
    monkeypatch.setattr(
        MODULE, "_wait_external_fast_support_socket", lambda *_: None)
    monkeypatch.setattr(
        MODULE, "_check_right_e0", lambda _args, label: events.append(label))
    monkeypatch.setattr(
        MODULE, "_run_right",
        lambda _args, direction: events.append(f"right:{direction}"))
    monkeypatch.setattr(MODULE, "_check_right_launch", lambda *_: None)

    def middle(
            _args, _daemon, direction="insert", *, roundtrip=False,
            execute=True):
        nonlocal middle_count
        assert roundtrip is True
        middle_count += 1
        events.append(f"middle:{direction}:{execute}")
        if direction == "return" and execute:
            raise MODULE.FlowError("RETURN_STOP_AND_HOLD")

    monkeypatch.setattr(MODULE, "_run_middle", middle)
    monkeypatch.setattr(
        MODULE, "_fast_support_request",
        lambda _daemon, operation: events.append(f"request:{operation}") or
        f"PASS:{operation}")
    monkeypatch.setattr(
        MODULE, "_finish_external_fast_support_daemon",
        lambda *_: events.append("unexpected_finish"))

    assert MODULE.main(_arguments()) == 1
    assert middle_count == 3
    assert events[-1] == "middle:return:True"
    assert "request:finalize" not in events
    assert "unexpected_finish" not in events


def test_bad_return_authority_fails_before_any_controller_motion(monkeypatch):
    events = []
    monkeypatch.setattr(
        MODULE, "_run_middle",
        lambda *_args, **kwargs: (
            (_ for _ in ()).throw(MODULE.FlowError("BAD_RETURN_AUTHORITY"))
            if kwargs.get("execute") is False else
            events.append("unexpected_live_middle")))
    monkeypatch.setattr(
        MODULE, "_run_support",
        lambda *_args, **_kwargs: events.append("unexpected_support"))
    monkeypatch.setattr(
        MODULE, "_run_right",
        lambda *_args, **_kwargs: events.append("unexpected_right"))

    assert MODULE.main(_arguments()) == 1
    assert events == []


def test_prepared_support_skips_preload_but_requires_ready_socket(
        monkeypatch, capsys):
    events = []
    monkeypatch.setattr(
        MODULE, "_run_support",
        lambda *_args, **_kwargs: events.append("unexpected_preload"))
    monkeypatch.setattr(
        MODULE, "_wait_external_fast_support_socket",
        lambda path: events.append(("wait_socket", str(path))))
    monkeypatch.setattr(
        MODULE, "_fast_support_claim_ready",
        lambda _daemon, run_id:
        events.append(("claim", run_id)) or "CLAIM=PASS")
    monkeypatch.setattr(
        MODULE, "_run_right",
        lambda _args, direction: events.append(("right", direction)))
    monkeypatch.setattr(MODULE, "_check_right_e0", lambda *_: None)
    monkeypatch.setattr(MODULE, "_check_right_launch", lambda *_: None)
    monkeypatch.setattr(MODULE, "_run_middle", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        MODULE, "_fast_support_request",
        lambda _daemon, operation: f"PASS:{operation}")
    monkeypatch.setattr(
        MODULE, "_finish_external_fast_support_daemon", lambda *_: None)

    run_id = "f" * 64
    assert MODULE.main([
        *_arguments(), "--authority-run-id", run_id,
        "--prepared-support"]) == 0
    assert "unexpected_preload" not in events
    assert events[0] == ("wait_socket", "/run/user/1000/roundtrip.sock")
    assert events[1] == ("claim", run_id)
    assert events[2] == ("right", "prefix")
    assert (
        "CANDIDATE3_ROUNDTRIP_831_PREPARED_SUPPORT=PASS "
        "preload=complete daemon=ready"
    ) in capsys.readouterr().out


def test_preparation_validates_return_before_support_mutation(
        monkeypatch, capsys):
    events = []
    monkeypatch.setattr(
        MODULE, "_run_middle",
        lambda _args, _daemon, direction="insert", *, roundtrip=False,
        execute=True:
        events.append(("middle", direction, roundtrip, execute)))
    monkeypatch.setattr(
        MODULE, "_run_support",
        lambda _args, operation, *, roundtrip=False:
        events.append(("support", operation, roundtrip)))
    monkeypatch.setattr(
        MODULE, "_wait_external_fast_support_socket",
        lambda path: events.append(("wait_socket", str(path))))
    monkeypatch.setattr(
        MODULE, "_run_right",
        lambda *_args, **_kwargs: events.append("unexpected_controller"))

    assert MODULE.main(_preparation_arguments()) == 0
    assert events == [
        ("middle", "return", True, False),
        ("support", "preload", True),
        ("wait_socket", "/run/user/1000/roundtrip.sock"),
    ]
    assert (
        "CANDIDATE3_ROUNDTRIP_831_PREPARATION=PASS "
        "return_authority=valid preload=complete daemon=ready "
        "controller_commands=0"
    ) in capsys.readouterr().out


def test_bad_preparation_authority_prevents_support_mutation(monkeypatch):
    events = []
    monkeypatch.setattr(
        MODULE, "_run_middle",
        lambda *_args, **_kwargs:
        (_ for _ in ()).throw(MODULE.FlowError("BAD_RETURN_AUTHORITY")))
    monkeypatch.setattr(
        MODULE, "_run_support",
        lambda *_args, **_kwargs: events.append("unexpected_support"))
    monkeypatch.setattr(
        MODULE, "_wait_external_fast_support_socket",
        lambda *_args: events.append("unexpected_socket"))

    assert MODULE.main(_preparation_arguments()) == 1
    assert events == []


def test_failed_ready_claim_prevents_first_controller_command(monkeypatch):
    events = []
    monkeypatch.setattr(MODULE, "_run_support", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(MODULE, "_run_middle", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        MODULE, "_wait_external_fast_support_socket", lambda *_args: None)
    monkeypatch.setattr(
        MODULE, "_fast_support_claim_ready",
        lambda *_args: (_ for _ in ()).throw(
            MODULE.FlowError("STALE_OR_FORGED_SOCKET")))
    monkeypatch.setattr(
        MODULE, "_run_right",
        lambda *_args, **_kwargs: events.append("unexpected_controller"))

    assert MODULE.main([
        *_arguments(), "--authority-run-id", "a" * 64,
        "--prepared-support"]) == 1
    assert events == []
