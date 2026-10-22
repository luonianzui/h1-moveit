#!/usr/bin/python3
"""Build the post-Mid360 v9/r17 regression replay as non-overwriting r29."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V18_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v18.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v19_base", V18_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V19_BASE_BUILDER_IMPORT")
V18 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V18
SPEC.loader.exec_module(V18)
BUILDER = V18.BUILDER


BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V18_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)
V18_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v18")


def prepare_authority() -> Any:
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["live_demonstration_instance"] = "r29"
    contract["post_mid360_shared_description_regression"] = True
    contract["user_selected_second_view"] = True
    manifest["execution_contract"] = contract
    derived = dict(manifest["derived_from"])
    derived["semantic_changes"] = [
        "allocate a fresh non-overwriting r29 replay identity",
        "retain v9 trajectories, timing, safety gates, and payloads exactly",
        "revalidate the complete round trip after Mid360 integration",
        "retain the second camera view selected live by the user for r28",
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
    payloads = V18.V17.V16.V15.V14.V13_IDENTITY_INDEPENDENT_SHA256
    for name, expected in payloads.items():
        actual = hashlib.sha256(prepared.output_files[name].raw).hexdigest()
        if actual != expected:
            raise RuntimeError(f"V19_SELF_TEST_PAYLOAD_DRIFT:{name}")
    v18_commands = json.loads(
        (V18_AUTHORITY / "commands_831.json").read_text(encoding="utf-8"))
    if prepared.commands["timing_contract"] != v18_commands["timing_contract"]:
        raise RuntimeError("V19_SELF_TEST_TIMING_DRIFT")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v19",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "semantic_authority_identical_to_v9": True,
        "post_mid360_shared_description_regression": True,
        "identity_independent_payloads_checked": len(payloads),
        "timing_contract_identical_to_v18": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v19")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r29"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v19"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v19"
BUILDER.RUN_ID_EXTRA_BINDING = V18_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("user_second_view_video_v18", None)
BUILDER.RUN_ID_EXTRA_BINDING["post_mid360_regression_replay_v19"] = {
    "instance": "r29",
    "semantic_authority_manifest_sha256":
        V18.V17.V16.V15.V14.V13.V12.V11.V9_MANIFEST_SHA256,
    "trajectory_and_timing_unchanged": True,
    "required_rosbag_present": True,
    "shared_robot_description_includes_mid360": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
