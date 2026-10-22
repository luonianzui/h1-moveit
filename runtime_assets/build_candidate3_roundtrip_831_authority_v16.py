#!/usr/bin/python3
"""Build the final fresh GPU-video v9/r17 replay as non-overwriting r24."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V15_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v15.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v16_base", V15_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V16_BASE_BUILDER_IMPORT")
V15 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V15
SPEC.loader.exec_module(V15)
BUILDER = V15.BUILDER


BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V15_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)
V15_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v15")


def prepare_authority() -> Any:
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["live_demonstration_instance"] = "r24"
    contract["fresh_launch_after_failed_preload"] = True
    manifest["execution_contract"] = contract
    derived = dict(manifest["derived_from"])
    derived["semantic_changes"] = [
        "allocate a fresh non-overwriting r24 replay identity",
        "retain v9 trajectories, timing, safety gates, and payloads exactly",
        "start from a fresh Gazebo world and restore the user's exact view",
        "retain the mandatory rosbag subscriber and GPU video encoding",
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
    payloads = V15.V14.V13_IDENTITY_INDEPENDENT_SHA256
    for name, expected in payloads.items():
        actual = hashlib.sha256(prepared.output_files[name].raw).hexdigest()
        if actual != expected:
            raise RuntimeError(f"V16_SELF_TEST_PAYLOAD_DRIFT:{name}")
    v15_commands = json.loads(
        (V15_AUTHORITY / "commands_831.json").read_text(encoding="utf-8"))
    if prepared.commands["timing_contract"] != v15_commands["timing_contract"]:
        raise RuntimeError("V16_SELF_TEST_TIMING_DRIFT")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v16",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "semantic_authority_identical_to_v9": True,
        "identity_independent_payloads_checked": len(payloads),
        "timing_contract_identical_to_v15": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v16")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r24"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v16"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v16"
BUILDER.RUN_ID_EXTRA_BINDING = V15_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("gpu_complete_video_v15", None)
BUILDER.RUN_ID_EXTRA_BINDING["fresh_gpu_complete_video_v16"] = {
    "instance": "r24",
    "semantic_authority_manifest_sha256":
        V15.V14.V13.V12.V11.V9_MANIFEST_SHA256,
    "trajectory_and_timing_unchanged": True,
    "required_rosbag_present": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
