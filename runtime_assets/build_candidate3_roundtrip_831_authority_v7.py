#!/usr/bin/python3
"""Build the non-overwriting 831 round-trip authority v7.

V7 retains the complete v6 prepared-support motion contract.  It corrects the
ready-claim identity proof so the daemon reports the filesystem socket created
by ``bind`` rather than the unrelated kernel socket inode returned by
``fstat(server.fileno())``.
"""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V6_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v6.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v7_base", V6_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V7_BASE_BUILDER_IMPORT")
V6 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V6
SPEC.loader.exec_module(V6)
BUILDER = V6.BUILDER


V6_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v6")
V6_MANIFEST_SHA256 = (
    "de63865ea708810708b960ac827ded942882c87f006eb3a40019adb4f525726e")
V6_COMMANDS_SHA256 = (
    "2964cc5feb54881f9c73b5d5b136342d0653c6b7766c483c50395a1ec8c143be")

BASE_MAKE_COMMANDS = BUILDER._make_commands
BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
BASE_SELF_TEST = BUILDER.self_test
V6_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)


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
    prepared = dict(commands["prepared_support"])
    prepared["ready_claim_protocol"] = (
        "h1_fast_support_daemon_831/claim-v2-bound-path")
    prepared["claim_checks"] = [
        "nonce", "run_id", "SO_PEERCRED", "daemon_start_ticks",
        "bound_path_socket_inode", "ownership_sha256",
        "fresh_initial_state",
    ]
    commands["prepared_support"] = prepared
    return commands


def prepare_authority() -> Any:
    v6_manifest = BUILDER.read_regular(
        V6_AUTHORITY / "manifest.json", V6_MANIFEST_SHA256,
        allowed_modes=(0o400,))
    v6_commands = BUILDER.read_regular(
        V6_AUTHORITY / "commands_831.json", V6_COMMANDS_SHA256,
        allowed_modes=(0o400,))
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["ready_claim_socket_identity"] = (
        "stable bound filesystem pathname device/inode")
    manifest["execution_contract"] = contract
    manifest["derived_from"] = {
        "path": BUILDER._relative(V6_AUTHORITY),
        "manifest_sha256": v6_manifest.sha256,
        "commands_sha256": v6_commands.sha256,
        "semantic_changes": [
            "report and verify the stable bound pathname socket identity",
            "reject pathname replacement from daemon READY through claim",
            "do not change any trajectory, timing scale, or controller message",
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
    daemon = prepared.output_files["h1_fast_support_daemon_831.py"].raw
    if b"_verified_socket_path_metadata" not in daemon:
        raise RuntimeError("V7_SELF_TEST_BOUND_PATH_HELPER")
    if b"socket_metadata = os.fstat(server.fileno())" in daemon:
        raise RuntimeError("V7_SELF_TEST_KERNEL_SOCKET_IDENTITY")
    if prepared.commands["prepared_support"]["ready_claim_protocol"] != (
            "h1_fast_support_daemon_831/claim-v2-bound-path"):
        raise RuntimeError("V7_SELF_TEST_CLAIM_PROTOCOL")
    result.update({
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v7",
        "socket_identity": "bound-path",
        "timed_maximum_seconds": 180.0,
    })
    return result


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v7")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r15"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v7"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v7"
BUILDER.RUN_ID_EXTRA_BINDING = V6_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING["ready_claim_socket_identity"] = {
    "protocol": "h1_fast_support_daemon_831/claim-v2-bound-path",
    "daemon_reports": "bind_path_lstat_device_inode",
    "client_requires": "same_path_identity_before_and_after_claim",
}
BUILDER._make_commands = _make_commands
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
