#!/usr/bin/python3
"""Build the complete GPU-video v9/r17 replay as non-overwriting r23."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V14_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v14.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v15_base", V14_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V15_BASE_BUILDER_IMPORT")
V14 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V14
SPEC.loader.exec_module(V14)
BUILDER = V14.BUILDER


BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V14_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)
V14_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v14")


def prepare_authority() -> Any:
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["live_demonstration_instance"] = "r23"
    contract["required_rosbag_present"] = True
    contract["gpu_video_encoding"] = True
    manifest["execution_contract"] = contract
    derived = dict(manifest["derived_from"])
    derived["semantic_changes"] = [
        "allocate a fresh non-overwriting r23 replay identity",
        "retain v9 trajectories, timing, safety gates, and payloads exactly",
        "restore the exact camera pose set live by the user",
        "retain the mandatory controller-topic rosbag discovery subscriber",
        "use dedicated GPU video encoding to reduce CPU capture load",
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
    for name, expected in V14.V13_IDENTITY_INDEPENDENT_SHA256.items():
        actual = hashlib.sha256(prepared.output_files[name].raw).hexdigest()
        if actual != expected:
            raise RuntimeError(f"V15_SELF_TEST_PAYLOAD_DRIFT:{name}")
    v14_commands = json.loads(
        (V14_AUTHORITY / "commands_831.json").read_text(encoding="utf-8"))
    if prepared.commands["timing_contract"] != v14_commands["timing_contract"]:
        raise RuntimeError("V15_SELF_TEST_TIMING_DRIFT")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v15",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "semantic_authority_identical_to_v9": True,
        "identity_independent_payloads_checked": len(
            V14.V13_IDENTITY_INDEPENDENT_SHA256),
        "timing_contract_identical_to_v14": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v15")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r23"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v15"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v15"
BUILDER.RUN_ID_EXTRA_BINDING = V14_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("reduced_capture_load_video_v14", None)
BUILDER.RUN_ID_EXTRA_BINDING["gpu_complete_video_v15"] = {
    "instance": "r23",
    "semantic_authority_manifest_sha256": V14.V13.V12.V11.V9_MANIFEST_SHA256,
    "trajectory_and_timing_unchanged": True,
    "required_rosbag_present": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
