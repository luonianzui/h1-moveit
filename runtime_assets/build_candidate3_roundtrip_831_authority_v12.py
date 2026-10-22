#!/usr/bin/python3
"""Build the clean requested-angle, non-overwriting v9 replay as r20."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V11_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v11.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v12_base", V11_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V12_BASE_BUILDER_IMPORT")
V11 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V11
SPEC.loader.exec_module(V11)
BUILDER = V11.BUILDER


BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V11_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)


def prepare_authority() -> Any:
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["live_demonstration_instance"] = "r20"
    contract["clean_requested_angle_video"] = True
    manifest["execution_contract"] = contract
    derived = dict(manifest["derived_from"])
    derived["semantic_changes"] = [
        "allocate a fresh non-overwriting r20 replay identity",
        "retain v9 trajectories, timing, safety gates, and payloads exactly",
        "record the requested view after clearing GUI entity selection",
    ]
    manifest["derived_from"] = derived
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
    prepared = prepare_authority()
    exact = {
        "h1_right_handle_executor_828.py":
            V11.V9_RIGHT_EXECUTOR_SHA256,
        "controller_execution_authorization.json":
            V11.V9_CONTROLLER_AUTHORIZATION_SHA256,
    }
    for name, expected in exact.items():
        actual = hashlib.sha256(prepared.output_files[name].raw).hexdigest()
        if actual != expected:
            raise RuntimeError(f"V12_SELF_TEST_V9_DRIFT:{name}")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v12",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "semantic_authority_identical_to_v9": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v12")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r20"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v12"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v12"
BUILDER.RUN_ID_EXTRA_BINDING = V11_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("requested_angle_video_v11", None)
BUILDER.RUN_ID_EXTRA_BINDING["clean_requested_angle_video_v12"] = {
    "instance": "r20",
    "semantic_authority_manifest_sha256": V11.V9_MANIFEST_SHA256,
    "trajectory_and_timing_unchanged": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
