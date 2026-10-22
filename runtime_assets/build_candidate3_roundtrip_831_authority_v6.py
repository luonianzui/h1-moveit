#!/usr/bin/python3
"""Build the non-overwriting 831 round-trip authority v6.

V6 keeps every v5 trajectory, timing scale, endpoint gate, and controller
message.  It moves the one-time Gazebo support-plugin preload into an explicit
preparation role.  The timed flow is admissible only after the external
support daemon has published its mode-0600 READY socket.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V5_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v5.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v6_base", V5_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V6_BASE_BUILDER_IMPORT")
V5 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V5
SPEC.loader.exec_module(V5)
BUILDER = V5.BUILDER


V5_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v5")
V5_MANIFEST_SHA256 = (
    "d5fa37a982d3eb2d73c175a63c22efa52483e93ed973ccb02b8a2763a70d84ef")
V5_COMMANDS_SHA256 = (
    "1ae170e78cee68f7b6f7ac7ec2ff29c1aaa91a9f35bba3809d806ad961d1c544")

BASE_MAKE_COMMANDS = BUILDER._make_commands
BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
BASE_SELF_TEST = BUILDER.self_test


def _make_commands(
    run_id: str,
    socket_path: Path,
    immutable: Mapping[str, Any],
    snapshots: Mapping[str, Any],
    external: Mapping[str, Any],
    return_plan: Any,
    return_authorization_sha256: str,
) -> dict[str, Any]:
    commands = BASE_MAKE_COMMANDS(
        run_id, socket_path, immutable, snapshots, external,
        return_plan, return_authorization_sha256)
    processes = list(commands["processes"])
    daemon = next(
        row["argv"] for row in processes
        if row["role"] == "support_daemon")
    daemon.extend((
        "--authority-run-id", run_id,
        "--require-ready-claim",
    ))
    flow = next(row["argv"] for row in processes if row["role"] == "flow")
    if "--prepared-support" in flow:
        raise RuntimeError("V6_PREPARED_SUPPORT_DUPLICATE")
    flow.extend(("--authority-run-id", run_id))
    preparation = list(flow)
    preparation[2] = "prepare-roundtrip"
    flow.append("--prepared-support")
    flow_index = next(
        index for index, row in enumerate(processes)
        if row["role"] == "flow")
    processes.insert(
        flow_index, {"role": "preparation", "argv": preparation})
    commands["processes"] = processes
    commands["process_order"] = list(BUILDER.PROCESS_ROLES)
    commands["prepared_support"] = {
        "required": True,
        "preparation_role": "preparation",
        "required_marker": (
            "CANDIDATE3_ROUNDTRIP_831_PREPARATION=PASS "
            "return_authority=valid preload=complete daemon=ready "
            "controller_commands=0"),
        "daemon_ready_edge": "mode-0600 Unix socket",
        "ready_claim_required": True,
        "ready_claim_protocol": "h1_fast_support_daemon_831/claim-v1",
        "claim_checks": [
            "nonce", "run_id", "SO_PEERCRED", "daemon_start_ticks",
            "socket_inode", "ownership_sha256", "fresh_initial_state",
        ],
        "controller_commands_issued_during_preparation": 0,
    }
    commands["timing_contract"] = {
        "metric": "flow_invocation_to_terminal_pass",
        "maximum_seconds": 180.0,
        "one_time_preparation_excluded": True,
        "trajectory_time_scales_unchanged": True,
    }
    commands["lifecycle"] = {
        "launch": {"mode": "long_running"},
        "recorder": {"mode": "long_running", "depends_on": ["launch"]},
        "support_daemon": {
            "mode": "start_async", "depends_on": ["launch_ready"]},
        "preparation": {
            "mode": "run_to_completion",
            "depends_on": ["fresh_launch", "support_daemon_started"],
            "failure_policy": "do_not_spawn_flow_and_stop_daemon",
        },
        "flow": {
            "mode": "run_to_completion",
            "depends_on": [
                "preparation_exact_pass", "daemon_exact_ready",
                "socket_mode_0600", "one_shot_ready_claim"],
        },
    }
    return commands


def prepare_authority() -> Any:
    v5_manifest = BUILDER.read_regular(
        V5_AUTHORITY / "manifest.json", V5_MANIFEST_SHA256,
        allowed_modes=(0o400,))
    v5_commands = BUILDER.read_regular(
        V5_AUTHORITY / "commands_831.json", V5_COMMANDS_SHA256,
        allowed_modes=(0o400,))
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract.update({
        "prepared_support_required": True,
        "preparation_controller_commands": 0,
        "timed_metric": "flow_invocation_to_terminal_pass",
        "timed_maximum_seconds": 180.0,
        "trajectory_time_scales_unchanged": True,
        "ready_claim_required_before_motion": True,
        "flow_completion_marker": (
            "CANDIDATE3_ROUNDTRIP_831_FLOW_COMPLETION=PASS"),
    })
    manifest["execution_contract"] = contract
    manifest["derived_from"] = {
        "path": BUILDER._relative(V5_AUTHORITY),
        "manifest_sha256": v5_manifest.sha256,
        "commands_sha256": v5_commands.sha256,
        "semantic_changes": [
            "move validated one-time support preparation before timed flow",
            "require a one-shot run-id/nonce/SO_PEERCRED ready claim",
            "add a monotonic terminal flow-completion marker",
        ],
    }
    output_files = dict(prepared.output_files)
    output_files["manifest.json"] = BUILDER.OutputBlob(
        BUILDER.pretty(manifest), 0o400, "generated")
    return BUILDER.PreparedAuthority(
        run_id=prepared.run_id,
        socket_path=prepared.socket_path,
        output_files=output_files,
        commands=prepared.commands,
        return_authorization=prepared.return_authorization,
        manifest=manifest,
    )


def self_test() -> Mapping[str, Any]:
    result = dict(BASE_SELF_TEST())
    prepared = prepare_authority()
    roles = {
        row["role"]: row["argv"]
        for row in prepared.commands["processes"]
    }
    if roles["flow"].count("--prepared-support") != 1:
        raise RuntimeError("V6_SELF_TEST_PREPARED_FLAG")
    if roles["preparation"][2] != "prepare-roundtrip":
        raise RuntimeError("V6_SELF_TEST_PREPARATION")
    if roles["support_daemon"].count("--require-ready-claim") != 1:
        raise RuntimeError("V6_SELF_TEST_READY_CLAIM")
    if prepared.commands["timing_contract"] != {
        "metric": "flow_invocation_to_terminal_pass",
        "maximum_seconds": 180.0,
        "one_time_preparation_excluded": True,
        "trajectory_time_scales_unchanged": True,
    }:
        raise RuntimeError("V6_SELF_TEST_TIMING_CONTRACT")
    result.update({
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v6",
        "process_roles": list(BUILDER.PROCESS_ROLES),
        "prepared_support_required": True,
        "timed_maximum_seconds": 180.0,
    })
    return result


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v6")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r14"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v6"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v6"
BUILDER.PROCESS_ROLES = (
    "launch", "recorder", "support_daemon", "preparation", "flow")
BUILDER.RUN_ID_EXTRA_BINDING = {
    "prepared_support": {
        "preparation_operation": "prepare-roundtrip",
        "required_marker": (
            "CANDIDATE3_ROUNDTRIP_831_PREPARATION=PASS "
            "return_authority=valid preload=complete daemon=ready "
            "controller_commands=0"),
        "ready_claim_protocol": "h1_fast_support_daemon_831/claim-v1",
    },
    "process_dependencies": {
        "preparation": ["fresh_launch", "support_daemon_started"],
        "flow": [
            "preparation_exact_pass", "daemon_exact_ready",
            "socket_mode_0600", "one_shot_ready_claim"],
    },
    "timing_contract": {
        "metric": "flow_invocation_to_terminal_pass",
        "maximum_seconds": 180.0,
        "one_time_preparation_excluded": True,
    },
}
BUILDER._make_commands = _make_commands
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
