#!/usr/bin/python3
"""Build the second user-selected-view v9/r17 replay as non-overwriting r28."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V17_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v17.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v18_base", V17_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V18_BASE_BUILDER_IMPORT")
V17 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V17
SPEC.loader.exec_module(V17)
BUILDER = V17.BUILDER


BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V17_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)
V17_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v17")


def prepare_authority() -> Any:
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["live_demonstration_instance"] = "r28"
    contract["user_selected_second_view"] = True
    manifest["execution_contract"] = contract
    derived = dict(manifest["derived_from"])
    derived["semantic_changes"] = [
        "allocate a fresh non-overwriting r28 replay identity",
        "retain v9 trajectories, timing, safety gates, and payloads exactly",
        "restore the exact second camera view selected live by the user",
        "record the complete natural-speed replay with mandatory rosbag evidence",
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
    payloads = V17.V16.V15.V14.V13_IDENTITY_INDEPENDENT_SHA256
    for name, expected in payloads.items():
        actual = hashlib.sha256(prepared.output_files[name].raw).hexdigest()
        if actual != expected:
            raise RuntimeError(f"V18_SELF_TEST_PAYLOAD_DRIFT:{name}")
    v17_commands = json.loads(
        (V17_AUTHORITY / "commands_831.json").read_text(encoding="utf-8"))
    if prepared.commands["timing_contract"] != v17_commands["timing_contract"]:
        raise RuntimeError("V18_SELF_TEST_TIMING_DRIFT")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v18",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "semantic_authority_identical_to_v9": True,
        "identity_independent_payloads_checked": len(payloads),
        "timing_contract_identical_to_v17": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v18")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r28"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v18"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v18"
BUILDER.RUN_ID_EXTRA_BINDING = V17_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("user_front_view_video_v17", None)
BUILDER.RUN_ID_EXTRA_BINDING["user_second_view_video_v18"] = {
    "instance": "r28",
    "semantic_authority_manifest_sha256":
        V17.V16.V15.V14.V13.V12.V11.V9_MANIFEST_SHA256,
    "trajectory_and_timing_unchanged": True,
    "required_rosbag_present": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
