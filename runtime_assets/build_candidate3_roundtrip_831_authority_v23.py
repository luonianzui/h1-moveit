#!/usr/bin/python3
"""Build a fresh v23/r33 replay of the sealed v21 Mid360 scene."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V21_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v21.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v23_base", V21_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V23_BASE_BUILDER_IMPORT")
V21 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V21
SPEC.loader.exec_module(V21)
BUILDER = V21.BUILDER


BASE_PREPARE_AUTHORITY = V21.prepare_authority
V21_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)
V21_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v21")
V9_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v9")
V21_MANIFEST_SHA256 = (
    "363314e291863c2e3ed8230e2a3549fcf6c17d73f913126f937f34e787666edb")


def _sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def prepare_authority() -> Any:
    """Prepare a byte-sealed r33 identity without changing v21 payloads."""
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["live_demonstration_instance"] = "r33"
    contract["fresh_replay_derived_from_v21"] = True
    manifest["execution_contract"] = contract

    derived = dict(manifest["derived_from"])
    derived["semantic_changes"] = [
        "allocate a fresh non-overwriting r33 replay identity",
        "retain all 14 non-scene action, timing, and safety payloads exactly",
        "retain the exact v21 5 m Mid360 navigation enclosure and observers",
        "retain the proven 5 ms action physics and manipulation coordinates",
        "record the user-requested natural-speed replay with rosbag and GUI video",
    ]
    derived["fresh_replay_base_manifest_sha256"] = V21_MANIFEST_SHA256
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
    """Prove freshness and payload identity without writing any output."""
    prepared = prepare_authority()
    action_payloads = {
        name: digest
        for name, digest in (
            V21.V20.V19.V18.V17.V16.V15.V14.
            V13_IDENTITY_INDEPENDENT_SHA256.items())
        if name not in {"gazebo.launch.py", "h1_world.sdf"}
    }
    if len(action_payloads) != 14:
        raise RuntimeError("V23_SELF_TEST_ACTION_PAYLOAD_COUNT")
    for name, expected in action_payloads.items():
        raw = prepared.output_files[name].raw
        if _sha256(raw) != expected:
            raise RuntimeError(f"V23_SELF_TEST_ACTION_PAYLOAD_DRIFT:{name}")
        if raw != (V9_AUTHORITY / name).read_bytes():
            raise RuntimeError(f"V23_SELF_TEST_V9_ACTION_DRIFT:{name}")

    for name in (
            "gazebo.launch.py", "h1_world.sdf",
            "mid360_localization.yaml"):
        if (prepared.output_files[name].raw !=
                (V21_AUTHORITY / name).read_bytes()):
            raise RuntimeError(f"V23_SELF_TEST_V21_SCENE_DRIFT:{name}")

    v21_commands = json.loads(
        (V21_AUTHORITY / "commands_831.json").read_text(encoding="utf-8"))
    if prepared.commands["timing_contract"] != v21_commands["timing_contract"]:
        raise RuntimeError("V23_SELF_TEST_TIMING_DRIFT")
    if prepared.commands.get("authority_root") != str(BUILDER.NEW):
        raise RuntimeError("V23_SELF_TEST_AUTHORITY_ROOT")
    if prepared.commands.get("run_root") != str(BUILDER.RUN_ROOT):
        raise RuntimeError("V23_SELF_TEST_RUN_ROOT")
    if prepared.manifest["execution_contract"].get(
            "live_demonstration_instance") != "r33":
        raise RuntimeError("V23_SELF_TEST_INSTANCE")
    encoded = BUILDER.canonical({
        "commands": prepared.commands,
        "manifest": prepared.manifest,
    })
    for stale_run in (b"candidate3_roundtrip_831_r31",
                      b"candidate3_roundtrip_831_r32"):
        if stale_run in encoded:
            raise RuntimeError("V23_SELF_TEST_STALE_RUN_PATH")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v23",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "socket_path": str(prepared.socket_path),
        "semantic_action_authority_identical_to_v9": True,
        "combined_scene_payloads_identical_to_v21": True,
        "identity_independent_action_payloads_checked": len(action_payloads),
        "timing_contract_identical_to_v21": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v23")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r33"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v23"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v23"
BUILDER.RUN_ID_EXTRA_BINDING = V21_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("combined_mid360_map_replay_v21", None)
BUILDER.RUN_ID_EXTRA_BINDING["fresh_combined_mid360_map_replay_v23"] = {
    "instance": "r33",
    "base_combined_manifest_sha256": V21_MANIFEST_SHA256,
    "action_payload_count_unchanged": 14,
    "action_trajectory_and_timing_unchanged": True,
    "combined_scene_payloads_unchanged": True,
    "action_physics_unchanged": True,
    "required_rosbag_present": True,
    "required_gui_video_present": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
