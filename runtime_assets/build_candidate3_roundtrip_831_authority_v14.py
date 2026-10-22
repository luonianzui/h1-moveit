#!/usr/bin/python3
"""Build the reduced-capture-load v9/r17 replay as non-overwriting r22."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Mapping


HERE = Path(__file__).resolve().parent
V13_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v13.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v14_base", V13_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V14_BASE_BUILDER_IMPORT")
V13 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V13
SPEC.loader.exec_module(V13)
BUILDER = V13.BUILDER


BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V13_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)
V13_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v13")

V13_IDENTITY_INDEPENDENT_SHA256 = {
    "candidate3_return_plan_831.json": "24767afddc9101094a419d9ab00c04db352e344b1aae90b287d86ca4773ebb21",
    "candidate3_rosbag_qos_overrides_829.yaml": "da086156e2c7599f05e8c3ccc9066af8e4c6acc8210451de4ecc8b20bc0a6f1c",
    "candidate3_telemetry_bridge_829.yaml": "615231d3fc7d810f38165cbc679e4044263d4000cace8b56cc0ed35705ec9b0d",
    "controller_execution_authorization.json": "6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c",
    "gazebo.launch.py": "4f2a3da2f03716a8905738d1c10c6b41600a44fc6a5b725fed961419bb514bf5",
    "h1_2.gazebo.ros2_control.xacro": "3c865dcf19fec32403aac51365702afdf8232111840fcffb8ee80eb2923f1535",
    "h1_candidate3_flow_fast_831_v3.py": "358391076710527b6ec2ab360db6a18d6f7b03da74fb1ef4595c30fdf3f7fd2d",
    "h1_candidate3_middle_executor_fast_831_v3.py": "e88dd07c1de79585c1e2534f870e0422394d227744ac2d4fafa0fc9267ab4246",
    "h1_fast_support_daemon_831.py": "ba325e6406957bed1c36e1eb5eca99adb321ddeb019eb65a4243ea223adcc015",
    "h1_right_handle_executor_828.py": "7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8",
    "h1_robot_joint_state_relay_829.py": "5055b923daaf322cfcf7a7dbb738359c5e4d702010be539c4ce75b8f5b8f193c",
    "h1_support_orchestrator_828.py": "76b016bc26e57fb6f2f8e1cf5a51b72a0a302c64ae2b47c51238a89dacd605ac",
    "h1_world.sdf": "4b61050f468df5f81a6805d410dd63daeb8b7c9d0aa9143b00b90282c38eb96c",
    "incubator_model.sdf": "44ff23fc7077f0c232e2e69ff9d73aba336140d6a7951968c9c4241a97e73db9",
    "right_handle_manifest.json": "d4260f136a2a699683516b4c55d74c7546beea59f4f7c5c3d925583d307ff6e0",
    "ros2_controllers.yaml": "612722d2143e39ef467219c67b9fb4bbfba9cb90a6bf047701308b4ff6279795",
}


def prepare_authority() -> Any:
    prepared = BASE_PREPARE_AUTHORITY()
    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["live_demonstration_instance"] = "r22"
    contract["reduced_capture_load"] = True
    manifest["execution_contract"] = contract
    derived = dict(manifest["derived_from"])
    derived["semantic_changes"] = [
        "allocate a fresh non-overwriting r22 replay identity",
        "retain v9 trajectories, timing, safety gates, and payloads exactly",
        "restore the exact camera pose set live by the user",
        "record without a concurrent rosbag to reduce capture load",
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
    for name, expected in V13_IDENTITY_INDEPENDENT_SHA256.items():
        actual = hashlib.sha256(prepared.output_files[name].raw).hexdigest()
        if actual != expected:
            raise RuntimeError(f"V14_SELF_TEST_PAYLOAD_DRIFT:{name}")
        source = V13_AUTHORITY / name
        if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"V14_SELF_TEST_V13_SOURCE_DRIFT:{name}")
    v13_commands = json.loads(
        (V13_AUTHORITY / "commands_831.json").read_text(encoding="utf-8"))
    if prepared.commands["timing_contract"] != v13_commands["timing_contract"]:
        raise RuntimeError("V14_SELF_TEST_TIMING_DRIFT")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v14",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "semantic_authority_identical_to_v9": True,
        "identity_independent_payloads_checked": len(
            V13_IDENTITY_INDEPENDENT_SHA256),
        "timing_contract_identical_to_v13": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v14")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r22"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v14"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v14"
BUILDER.RUN_ID_EXTRA_BINDING = V13_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("user_set_camera_video_v13", None)
BUILDER.RUN_ID_EXTRA_BINDING["reduced_capture_load_video_v14"] = {
    "instance": "r22",
    "semantic_authority_manifest_sha256": V13.V12.V11.V9_MANIFEST_SHA256,
    "trajectory_and_timing_unchanged": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
