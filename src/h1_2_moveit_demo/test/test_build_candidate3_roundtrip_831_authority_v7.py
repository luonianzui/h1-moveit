"""Contracts for the bound-path ready-claim 831 authority v7."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = (
    WORKSPACE / "runtime_assets" /
    "build_candidate3_roundtrip_831_authority_v7.py")
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_authority_builder_831_v7_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
BUILDER = MODULE.BUILDER


def _roles(prepared):
    return {
        row["role"]: row["argv"]
        for row in prepared.commands["processes"]
    }


def _flag(argv, name):
    return argv[argv.index(name) + 1]


def test_v7_fixes_only_the_ready_claim_socket_identity():
    prepared = BUILDER.prepare_authority()
    daemon = prepared.output_files["h1_fast_support_daemon_831.py"].raw
    assert b"_verified_socket_path_metadata" in daemon
    assert b"socket_metadata = os.fstat(server.fileno())" not in daemon
    assert prepared.commands["prepared_support"][
        "ready_claim_protocol"] == (
            "h1_fast_support_daemon_831/claim-v2-bound-path")
    assert prepared.manifest["execution_contract"][
        "ready_claim_socket_identity"] == (
            "stable bound filesystem pathname device/inode")


def test_v7_preserves_v6_motion_and_under_three_minute_contract():
    prepared = BUILDER.prepare_authority()
    roles = _roles(prepared)
    assert _flag(roles["flow"], "--right-time-scale") == "1.0"
    assert _flag(roles["flow"], "--middle-speed-scale") == "1.0"
    assert roles["flow"].count("--prepared-support") == 1
    assert roles["support_daemon"].count("--require-ready-claim") == 1
    assert prepared.commands["timing_contract"] == {
        "metric": "flow_invocation_to_terminal_pass",
        "maximum_seconds": 180.0,
        "one_time_preparation_excluded": True,
        "trajectory_time_scales_unchanged": True,
    }
    assert prepared.manifest["execution_contract"][
        "controller_message_count"] == 60
    assert prepared.manifest["execution_contract"][
        "left_hand_handle_contact_forbidden"] is True
    assert prepared.manifest["execution_contract"][
        "hard_maximum_abs_torso_deg"] == 44.5
    assert BUILDER.sha(prepared.output_files[
        "h1_right_handle_executor_828.py"].raw) == (
            "93bdb04608f0e31ebc064c6f0f6659a30e97d2c879d20e0b171e77499c0335cc")
    assert BUILDER.sha(prepared.output_files[
        "h1_candidate3_middle_executor_fast_831_v3.py"].raw) == (
            "e88dd07c1de79585c1e2534f870e0422394d227744ac2d4fafa0fc9267ab4246")


def test_v7_binds_verified_v6_and_is_read_only():
    before = (
        BUILDER.NEW.exists() or BUILDER.NEW.is_symlink(),
        BUILDER.RUN_ROOT.exists() or BUILDER.RUN_ROOT.is_symlink(),
    )
    prepared = BUILDER.prepare_authority()
    after = (
        BUILDER.NEW.exists() or BUILDER.NEW.is_symlink(),
        BUILDER.RUN_ROOT.exists() or BUILDER.RUN_ROOT.is_symlink(),
    )
    assert after == before
    assert prepared.manifest["derived_from"] == {
        "path": BUILDER._relative(MODULE.V6_AUTHORITY),
        "manifest_sha256": MODULE.V6_MANIFEST_SHA256,
        "commands_sha256": MODULE.V6_COMMANDS_SHA256,
        "semantic_changes": [
            "report and verify the stable bound pathname socket identity",
            "reject pathname replacement from daemon READY through claim",
            "do not change any trajectory, timing scale, or controller message",
        ],
    }
    assert prepared.manifest["schema"] == (
        "h1_candidate3_roundtrip_authority/831-v7")
    assert prepared.commands["schema"] == (
        "h1_candidate3_roundtrip_commands/831-v7")


def test_v7_self_test_passes_without_writing():
    result = BUILDER.self_test()
    assert result["status"] == "PASS"
    assert result["writes_performed"] is False
    assert result["socket_identity"] == "bound-path"
    assert result["timed_maximum_seconds"] == 180.0
