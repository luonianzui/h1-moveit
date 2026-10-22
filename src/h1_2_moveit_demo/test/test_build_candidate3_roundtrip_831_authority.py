"""Static/unit checks for the non-overwriting 831 round-trip builder."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest


WS = Path(
    "/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/"
    "h1_2_ros2_ws")
BUILDER = WS / "runtime_assets/build_candidate3_roundtrip_831_authority.py"


def _load_builder():
    spec = importlib.util.spec_from_file_location(
        "candidate3_roundtrip_authority_builder_831", BUILDER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def builder():
    return _load_builder()


@pytest.fixture(scope="module")
def prepared(builder):
    return builder.prepare_authority()


def _roles(prepared):
    return {
        row["role"]: row["argv"]
        for row in prepared.commands["processes"]
    }


def _flag(argv, name):
    positions = [index for index, value in enumerate(argv) if value == name]
    assert len(positions) == 1
    assert positions[0] + 1 < len(argv)
    return argv[positions[0] + 1]


def test_prepare_is_read_only_and_has_exact_roles(builder):
    before = (
        builder.NEW.exists() or builder.NEW.is_symlink(),
        builder.RUN_ROOT.exists() or builder.RUN_ROOT.is_symlink(),
    )
    prepared = builder.prepare_authority()
    after = (
        builder.NEW.exists() or builder.NEW.is_symlink(),
        builder.RUN_ROOT.exists() or builder.RUN_ROOT.is_symlink(),
    )
    assert after == before
    assert prepared.commands["process_order"] == [
        "launch", "recorder", "support_daemon", "flow"]
    assert [row["role"] for row in prepared.commands["processes"]] == [
        "launch", "recorder", "support_daemon", "flow"]


def test_roundtrip_argv_and_computed_hashes(builder, prepared):
    roles = _roles(prepared)
    daemon = roles["support_daemon"]
    flow = roles["flow"]
    assert daemon.count("--roundtrip") == 1
    assert flow[2] == "roundtrip"
    assert _flag(daemon, "--socket") == _flag(
        flow, "--fast-support-daemon-socket")
    assert _flag(flow, "--return-plan") == str(
        builder.NEW / "candidate3_return_plan_831.json")
    assert _flag(flow, "--return-authorization") == str(
        builder.NEW / "return_execution_authorization.json")
    for flag, name in (
        ("--middle-executor-sha256",
         "h1_candidate3_middle_executor_fast_831_v3.py"),
        ("--support-orchestrator-sha256", "h1_support_orchestrator_828.py"),
        ("--fast-support-daemon-sha256", "h1_fast_support_daemon_831.py"),
        ("--return-plan-sha256", "candidate3_return_plan_831.json"),
        ("--return-authorization-sha256",
         "return_execution_authorization.json"),
    ):
        assert _flag(flow, flag) == builder.sha(
            prepared.output_files[name].raw)
    assert _flag(flow, "--support-manifest-sha256") == builder.sha(
        builder.read_regular(
            builder.EXTERNAL_INPUTS["support_manifest"][0],
            allowed_modes=(0o400,)).raw)
    assert _flag(flow, "--plan-sha256") == builder.sha(
        builder.read_regular(
            builder.EXTERNAL_INPUTS["middle_plan"][0],
            allowed_modes=(0o400,)).raw)


def test_return_authorization_exact_v1_contract(builder, prepared):
    value = prepared.return_authorization
    assert set(value) == {
        "schema", "run_id", "authorization_scope", "bindings",
        "execution_contract", "controller_execution_authorized",
        "live_execution_passed", "controller_commands_issued",
        "valid", "status",
    }
    assert value["schema"] == (
        "h1_candidate3_return_execution_authorization/831-v1")
    assert value["controller_execution_authorized"] is True
    assert value["live_execution_passed"] is False
    assert value["controller_commands_issued"] == 0
    assert value["valid"] is True and value["status"] == "READY"
    bindings = value["bindings"]
    assert set(bindings) == {
        "return_plan", "middle_plan", "middle_certificate",
        "middle_exact_result",
    }
    for binding in bindings.values():
        assert set(binding) == {"path", "sha256", "schema"}
        assert Path(binding["path"]).is_absolute()
        assert len(binding["sha256"]) == 64
        assert binding["schema"].startswith("h1_")
    assert bindings["return_plan"]["sha256"] == builder.RETURN_PLAN_SHA256
    contract = value["execution_contract"]
    assert contract["exact_inverse"] is True
    assert contract["inverse_transform"] == "reverse_points_mirror_time"
    assert contract["requires_live_final_validation"] is True
    assert contract["controller_message_count"] == 60
    assert contract["left_hand_handle_contact_forbidden"] is True
    assert contract["hard_maximum_abs_torso_deg"] == 44.5


def test_output_modes_provenance_and_no_live_dependency(builder, prepared):
    for name in builder.SNAPSHOT_SOURCES:
        assert prepared.output_files[name].mode == 0o500
    assert prepared.output_files["candidate3_return_plan_831.json"].mode == 0o400
    for name in (
        "commands_831.json", "return_execution_authorization.json",
        "manifest.json",
    ):
        assert prepared.output_files[name].mode == 0o400
    assert not {
        "h1_candidate3_live_supervisor_829.py",
        "h1_candidate3_live_certificate_829.py",
    }.intersection(prepared.output_files)
    argv_text = "\0".join(
        value
        for row in prepared.commands["processes"]
        for value in row["argv"])
    assert "h1_candidate3_live_supervisor_829.py" not in argv_text
    assert "h1_candidate3_live_certificate_829.py" not in argv_text


def test_manifest_binds_base_outputs_and_execution_contract(builder, prepared):
    manifest = prepared.manifest
    assert manifest["schema"] == "h1_candidate3_roundtrip_authority/831-v1"
    assert manifest["base_authority"]["manifest_sha256"] == (
        builder.BASE_MANIFEST_SHA256)
    assert manifest["process_roles"] == [
        "launch", "recorder", "support_daemon", "flow"]
    assert manifest["execution_contract"]["exact_inverse_return"] is True
    assert manifest["execution_contract"]["controller_message_count"] == 60
    assert manifest["execution_contract"][
        "left_hand_handle_contact_forbidden"] is True
    assert manifest["execution_contract"][
        "hard_maximum_abs_torso_deg"] == 44.5
    assert manifest["execution_contract"][
        "requires_live_final_validation"] is True
    assert "manifest.json" not in manifest["files"]
    assert set(builder.IMMUTABLE_MODES).issubset(manifest["files"])


def test_snapshot_interfaces_expose_roundtrip_authority_flags(prepared):
    flow = prepared.output_files[
        "h1_candidate3_flow_fast_831_v3.py"].raw
    middle = prepared.output_files[
        "h1_candidate3_middle_executor_fast_831_v3.py"].raw
    daemon = prepared.output_files["h1_fast_support_daemon_831.py"].raw
    assert b"--return-plan" in flow
    assert b"--return-plan-sha256" in flow
    assert b"--return-authorization" in flow
    assert b"--return-authorization-sha256" in flow
    assert b"--return-plan" in middle
    assert b"--return-authorization" in middle
    assert b"--roundtrip" in daemon


def test_write_exclusive_refuses_overwrite(builder, tmp_path):
    root = tmp_path / "exclusive"
    os.mkdir(root, 0o700)
    builder.write_exclusive(root, "sealed.json", b"one\n", 0o400)
    with pytest.raises(FileExistsError):
        builder.write_exclusive(root, "sealed.json", b"two\n", 0o400)
    assert (root / "sealed.json").read_bytes() == b"one\n"
    assert (root / "sealed.json").stat().st_mode & 0o777 == 0o400


def test_read_regular_rejects_wrong_hash_symlink_and_hardlink(builder, tmp_path):
    original = tmp_path / "original"
    original.write_bytes(b"sealed\n")
    original.chmod(0o400)
    with pytest.raises(RuntimeError, match="INPUT_SHA256"):
        builder.read_regular(original, "0" * 64, allowed_modes=(0o400,))
    symlink = tmp_path / "alias"
    symlink.symlink_to(original)
    with pytest.raises(RuntimeError, match="INPUT_IDENTITY"):
        builder.read_regular(symlink, allowed_modes=(0o400,))
    hardlink = tmp_path / "hardlink"
    os.link(original, hardlink)
    with pytest.raises(RuntimeError, match="INPUT_IDENTITY"):
        builder.read_regular(original, allowed_modes=(0o400,))


def test_json_parser_rejects_duplicate_and_nonfinite(builder):
    with pytest.raises(RuntimeError, match="DUPLICATE_KEY"):
        builder.parse_json(b'{"a":1,"a":2}\n', "DUP")
    with pytest.raises(RuntimeError, match="NONFINITE"):
        builder.parse_json(b'{"a":NaN}\n', "NAN")


def test_rename_noreplace_preserves_both_directories(builder, tmp_path):
    source = tmp_path / "source"
    destination = tmp_path / "destination"
    os.mkdir(source, 0o700)
    os.mkdir(destination, 0o700)
    builder.write_exclusive(source, "source.txt", b"source\n", 0o400)
    builder.write_exclusive(
        destination, "destination.txt", b"destination\n", 0o400)
    with pytest.raises(FileExistsError):
        builder._rename_noreplace(source, destination)
    assert (source / "source.txt").read_bytes() == b"source\n"
    assert (destination / "destination.txt").read_bytes() == b"destination\n"


def test_scoped_temporary_cleanup(builder, monkeypatch, tmp_path):
    new = tmp_path / "candidate3_roundtrip_831_authority_v1"
    run_root = tmp_path / "candidate3_roundtrip_831_r1"
    monkeypatch.setattr(builder, "NEW", new)
    monkeypatch.setattr(builder, "RUN_ROOT", run_root)
    temporary = tmp_path / (
        ".candidate3_roundtrip_831_authority_v1.building-123-deadbeef")
    os.mkdir(temporary, 0o700)
    builder.write_exclusive(temporary, "partial", b"partial\n", 0o400)
    builder._safe_cleanup_temporary(temporary)
    assert not temporary.exists()
    assert not new.exists()
    assert not run_root.exists()


def test_self_test_report_is_read_only(builder):
    result = builder.self_test()
    encoded = json.dumps(result, sort_keys=True, allow_nan=False)
    assert json.loads(encoded)["status"] == "PASS"
    assert result["writes_performed"] is False
