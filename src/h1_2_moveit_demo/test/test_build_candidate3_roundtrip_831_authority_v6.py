"""Contracts for the prepared-support 831 round-trip v6 authority."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = (
    WORKSPACE / "runtime_assets" /
    "build_candidate3_roundtrip_831_authority_v6.py")
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_authority_builder_831_v6_test", SOURCE)
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
    index = argv.index(name)
    return argv[index + 1]


def test_v6_preparation_is_explicit_and_timed_flow_reuses_it():
    prepared = BUILDER.prepare_authority()
    assert prepared.commands["process_order"] == [
        "launch", "recorder", "support_daemon", "preparation", "flow"]
    roles = _roles(prepared)
    assert roles["preparation"][2] == "prepare-roundtrip"
    assert roles["flow"].count("--prepared-support") == 1
    assert _flag(roles["flow"], "--authority-run-id") == prepared.run_id
    assert _flag(roles["preparation"], "--authority-run-id") == prepared.run_id
    assert _flag(roles["support_daemon"], "--authority-run-id") == (
        prepared.run_id)
    assert roles["support_daemon"].count("--require-ready-claim") == 1
    assert "--prepared-support" not in roles["preparation"]
    assert "--execute" not in roles["preparation"]
    assert _flag(roles["preparation"], "--support-manifest-sha256") == (
            _flag(roles["flow"], "--support-manifest-sha256"))
    assert _flag(roles["preparation"], "--certificate-sha256") == (
            _flag(roles["flow"], "--certificate-sha256"))
    assert prepared.commands["prepared_support"]["required"] is True
    assert prepared.commands["prepared_support"][
        "controller_commands_issued_during_preparation"] == 0


def test_v6_changes_no_motion_scale_or_message_contract():
    prepared = BUILDER.prepare_authority()
    roles = _roles(prepared)
    assert _flag(roles["flow"], "--right-time-scale") == "1.0"
    assert _flag(roles["flow"], "--middle-speed-scale") == "1.0"
    assert _flag(roles["flow"], "--return-plan-sha256") == (
        BUILDER.RETURN_PLAN_SHA256)
    assert prepared.manifest["execution_contract"][
        "controller_message_count"] == 60
    assert prepared.manifest["execution_contract"][
        "left_hand_handle_contact_forbidden"] is True
    assert prepared.manifest["execution_contract"][
        "hard_maximum_abs_torso_deg"] == 44.5
    assert prepared.manifest["execution_contract"][
        "trajectory_time_scales_unchanged"] is True
    assert BUILDER.sha(prepared.output_files[
        "h1_right_handle_executor_828.py"].raw) == (
        "93bdb04608f0e31ebc064c6f0f6659a30e97d2c879d20e0b171e77499c0335cc")
    assert BUILDER.sha(prepared.output_files[
        "h1_candidate3_middle_executor_fast_831_v3.py"].raw) == (
        "e88dd07c1de79585c1e2534f870e0422394d227744ac2d4fafa0fc9267ab4246")
    assert b"claim-ready" in prepared.output_files[
        "h1_fast_support_daemon_831.py"].raw


def test_v6_binds_the_verified_v5_authority_and_is_read_only():
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
        "path": BUILDER._relative(MODULE.V5_AUTHORITY),
        "manifest_sha256": MODULE.V5_MANIFEST_SHA256,
        "commands_sha256": MODULE.V5_COMMANDS_SHA256,
        "semantic_changes": [
            "move validated one-time support preparation before timed flow",
            "require a one-shot run-id/nonce/SO_PEERCRED ready claim",
            "add a monotonic terminal flow-completion marker",
        ],
    }
    assert prepared.manifest["schema"] == (
        "h1_candidate3_roundtrip_authority/831-v6")
    assert prepared.commands["schema"] == (
        "h1_candidate3_roundtrip_commands/831-v6")


def test_v6_flow_source_has_fail_closed_ready_socket_gate():
    prepared = BUILDER.prepare_authority()
    source = prepared.output_files[
        "h1_candidate3_flow_fast_831_v3.py"].raw
    assert b"--prepared-support" in source
    assert b"if not args.prepared_support" in source
    assert b"_fast_support_claim_ready" in source
    assert source.index(b"_fast_support_claim_ready") < source.index(
        b"CANDIDATE3_ROUNDTRIP_831_MOTION_WINDOW=START")
    assert source.index(b"_wait_external_fast_support_socket") < source.index(
        b"CANDIDATE3_ROUNDTRIP_831_MOTION_WINDOW=START")
    assert BUILDER.RUN_ID_EXTRA_BINDING["timing_contract"] == {
        "metric": "flow_invocation_to_terminal_pass",
        "maximum_seconds": 180.0,
        "one_time_preparation_excluded": True,
    }


def test_v6_self_test_passes_without_writing():
    result = BUILDER.self_test()
    assert result["status"] == "PASS"
    assert result["writes_performed"] is False
    assert result["prepared_support_required"] is True
    assert result["timed_maximum_seconds"] == 180.0
