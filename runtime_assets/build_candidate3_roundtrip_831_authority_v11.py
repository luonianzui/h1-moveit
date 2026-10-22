#!/usr/bin/python3
"""Build a fresh non-overwriting v9-equivalent video replay as r19."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V10_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v10.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v11_base", V10_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V11_BASE_BUILDER_IMPORT")
V10 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V10
SPEC.loader.exec_module(V10)
BUILDER = V10.BUILDER


V9_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v9")
V9_MANIFEST_SHA256 = (
    "b9e49b32c60979eb5a97d8ed3d5dc325a5d73e10b1985a45fd99c61f8848cd29")
V9_COMMANDS_SHA256 = (
    "aef171adfacb8b1af212caa1c193e8fabf51e06d03f8fff4ceddf9ed0a22e8fb")
V9_RIGHT_EXECUTOR_SHA256 = (
    "7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8")
V9_CONTROLLER_AUTHORIZATION_SHA256 = (
    "6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c")

BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V10_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)


def prepare_authority() -> Any:
    v9_manifest = BUILDER.read_regular(
        V9_AUTHORITY / "manifest.json", V9_MANIFEST_SHA256,
        allowed_modes=(0o400,))
    v9_commands = BUILDER.read_regular(
        V9_AUTHORITY / "commands_831.json", V9_COMMANDS_SHA256,
        allowed_modes=(0o400,))
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract.update({
        "live_demonstration_instance": "r19",
        "semantic_authority_identical_to_v9": True,
        "trajectory_time_scales_unchanged_from_v9": True,
        "trajectory_geometry_unchanged_from_v9": True,
        "video_recording_is_non_authoritative": True,
    })
    manifest["execution_contract"] = contract
    manifest["derived_from"] = {
        "path": BUILDER._relative(V9_AUTHORITY),
        "manifest_sha256": v9_manifest.sha256,
        "commands_sha256": v9_commands.sha256,
        "semantic_changes": [
            "allocate a fresh non-overwriting r19 replay identity",
            "retain v9 trajectories, timing, safety gates, and payloads exactly",
            "record the visible Gazebo replay from the requested camera angle",
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
    prepared = prepare_authority()
    exact = {
        "h1_right_handle_executor_828.py": V9_RIGHT_EXECUTOR_SHA256,
        "controller_execution_authorization.json":
            V9_CONTROLLER_AUTHORIZATION_SHA256,
    }
    for name, expected in exact.items():
        actual = hashlib.sha256(prepared.output_files[name].raw).hexdigest()
        if actual != expected:
            raise RuntimeError(f"V11_SELF_TEST_V9_DRIFT:{name}")
    if prepared.commands["timing_contract"]["right_stage_time_scales"] != (
            V10.V9.RIGHT_STAGE_TIME_SCALES):
        raise RuntimeError("V11_SELF_TEST_TIMING_DRIFT")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v11",
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
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v11")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r19"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v11"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v11"
BUILDER.RUN_ID_EXTRA_BINDING = V10_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("live_demonstration_v10", None)
BUILDER.RUN_ID_EXTRA_BINDING["requested_angle_video_v11"] = {
    "instance": "r19",
    "semantic_authority_manifest_sha256": V9_MANIFEST_SHA256,
    "trajectory_and_timing_unchanged": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
