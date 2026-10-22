#!/usr/bin/python3
"""Exclusively seal candidate3 live authority v10 and empty run root r9.

This builder intentionally does not call the production supervisor's prepare
operation and never starts ROS, Gazebo, a recorder, or a controller process.
Every target file is created with O_CREAT|O_EXCL and every target directory is
created with mkdir's no-replace semantics.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import secrets
import stat
import sys
import time
from typing import Any, Iterable, Mapping, Sequence


WORKSPACE = Path(
    "/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
AUTHORITY = WORKSPACE / "runtime_assets/candidate3_live_829_authority_v10"
RUN_ROOT = WORKSPACE / "runtime_assets/candidate3_live_829_final_r9"
OLD_AUTHORITY = WORKSPACE / "runtime_assets/candidate3_live_829_authority_v8"
OLD_RUN_ROOT = WORKSPACE / "runtime_assets/candidate3_live_829_final_r7"
FCL_ROOT = (
    WORKSPACE / "runtime_assets/right_handle_8263_natural/"
    "variable_torso_v4_829/live_fcl_authority_v4_31j")


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def pretty_bytes(value: Any) -> bytes:
    return (json.dumps(
        value, sort_keys=True, indent=2, ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def checked_source(path: Path) -> bytes:
    metadata = path.lstat()
    if (path.is_symlink() or not stat.S_ISREG(metadata.st_mode) or
            metadata.st_nlink != 1 or path.resolve(strict=True) != path):
        raise RuntimeError(f"SOURCE_IDENTITY:{path}")
    raw = path.read_bytes()
    after = path.lstat()
    if ((metadata.st_dev, metadata.st_ino, metadata.st_size,
         metadata.st_mtime_ns) !=
            (after.st_dev, after.st_ino, after.st_size,
             after.st_mtime_ns)):
        raise RuntimeError(f"SOURCE_CHANGED_DURING_READ:{path}")
    return raw


def exclusive_write(path: Path, raw: bytes, mode: int) -> str:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags, mode)
    try:
        view = memoryview(raw)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise RuntimeError(f"WRITE_SHORT:{path}")
            view = view[written:]
        os.fsync(descriptor)
        os.fchmod(descriptor, mode)
    finally:
        os.close(descriptor)
    metadata = path.lstat()
    if (path.is_symlink() or not stat.S_ISREG(metadata.st_mode) or
            metadata.st_nlink != 1 or
            stat.S_IMODE(metadata.st_mode) != mode or
            metadata.st_size != len(raw)):
        raise RuntimeError(f"WRITE_IDENTITY:{path}")
    return sha256_bytes(raw)


def binding(path: Path) -> dict[str, Any]:
    metadata = path.lstat()
    mode = stat.S_IMODE(metadata.st_mode)
    if (path.is_symlink() or not stat.S_ISREG(metadata.st_mode) or
            metadata.st_nlink != 1 or mode not in (0o400, 0o500) or
            path.resolve(strict=True) != path):
        raise RuntimeError(f"BINDING_IDENTITY:{path}")
    return {
        "path": str(path),
        "sha256": sha256_file(path),
        "size_bytes": metadata.st_size,
        "mode": f"{mode:04o}",
    }


def argv_sha256(argv: Sequence[str]) -> str:
    return hashlib.sha256(b"".join(
        item.encode("utf-8") + b"\0" for item in argv)).hexdigest()


def flag_value(argv: Sequence[str], flag: str) -> str:
    indices = [index for index, item in enumerate(argv) if item == flag]
    if len(indices) != 1 or indices[0] + 1 >= len(argv):
        raise RuntimeError(f"FLAG:{flag}")
    return argv[indices[0] + 1]


def set_flag_value(argv: list[str], flag: str, value: str) -> None:
    index = argv.index(flag)
    argv[index + 1] = value


def replace_tree(value: Any, replacements: Iterable[tuple[str, str]]) -> Any:
    if isinstance(value, str):
        for old, new in replacements:
            value = value.replace(old, new)
        return value
    if isinstance(value, list):
        return [replace_tree(item, replacements) for item in value]
    if isinstance(value, dict):
        return {
            key: replace_tree(item, replacements)
            for key, item in value.items()
        }
    return value


def refresh_path_sha(
    argv: list[str], path_flag: str, sha_flag: str,
) -> None:
    path = Path(flag_value(argv, path_flag))
    if not path.is_absolute():
        raise RuntimeError(f"NONABSOLUTE:{path_flag}")
    set_flag_value(argv, sha_flag, sha256_file(path))


def import_sealed_supervisor(path: Path) -> Any:
    sys.dont_write_bytecode = True
    name = "candidate3_live_829_authority_v9_supervisor"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError("SUPERVISOR_IMPORT_SPEC")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main() -> None:
    if WORKSPACE.resolve(strict=True) != WORKSPACE:
        raise RuntimeError("WORKSPACE_IDENTITY")
    for target in (AUTHORITY, RUN_ROOT):
        if target.exists() or target.is_symlink():
            raise FileExistsError(target)

    source_to_target: tuple[tuple[str, Path, str, int], ...] = (
        ("supervisor_source",
         WORKSPACE / "src/h1_2_moveit_demo/scripts/"
         "h1_candidate3_live_supervisor_829.py",
         "h1_candidate3_live_supervisor_829.py", 0o500),
        ("certificate_source",
         WORKSPACE / "src/h1_2_moveit_demo/scripts/"
         "h1_candidate3_live_certificate_829.py",
         "h1_candidate3_live_certificate_829.py", 0o500),
        ("gazebo_launch_source",
         WORKSPACE / "src/h1_2_moveit_config/launch/gazebo.launch.py",
         "gazebo.launch.py", 0o400),
        ("gazebo_world_sdf",
         WORKSPACE / "src/h1_2_description/worlds/h1_world.sdf",
         "h1_world.sdf", 0o400),
        ("incubator_model_sdf",
         WORKSPACE / "src/h1_2_description/models/incubator/model.sdf",
         "incubator_model.sdf", 0o400),
        ("robot_gazebo_control_xacro",
         WORKSPACE / "src/h1_2_description/urdf/"
         "h1_2.gazebo.ros2_control.xacro",
         "h1_2.gazebo.ros2_control.xacro", 0o400),
        ("ros2_controllers_config",
         WORKSPACE / "src/h1_2_moveit_config/config/ros2_controllers.yaml",
         "ros2_controllers.yaml", 0o400),
        ("robot_joint_state_relay_source",
         WORKSPACE / "src/h1_2_moveit_demo/scripts/"
         "h1_robot_joint_state_relay_829.py",
         "h1_robot_joint_state_relay_829.py", 0o400),
        ("telemetry_bridge_config",
         WORKSPACE / "src/h1_2_moveit_config/config/"
         "candidate3_telemetry_bridge_829.yaml",
         "candidate3_telemetry_bridge_829.yaml", 0o400),
        ("rosbag_qos_overrides",
         WORKSPACE / "runtime_assets/"
         "candidate3_rosbag_qos_overrides_829.yaml",
         "candidate3_rosbag_qos_overrides_829.yaml", 0o400),
    )
    source_raw = {
        identifier: checked_source(source)
        for identifier, source, _, _ in source_to_target
    }
    source_hashes = {
        identifier: sha256_bytes(source_raw[identifier])
        for identifier, _, _, _ in source_to_target
    }

    old_commands = json.loads(
        checked_source(OLD_AUTHORITY / "commands_829.json").decode("utf-8"))
    old_run_id = old_commands["run_id"]
    run_id = secrets.token_hex(32)
    if len(run_id) != 64 or run_id == old_run_id:
        raise RuntimeError("RUN_ID")
    replacements = (
        (str(OLD_AUTHORITY), str(AUTHORITY)),
        (str(OLD_RUN_ROOT), str(RUN_ROOT)),
        ("live_fcl_authority_v3_31j", "live_fcl_authority_v4_31j"),
        (old_run_id, run_id),
    )
    commands = replace_tree(old_commands, replacements)
    commands["run_id"] = run_id

    rows = commands["processes"]
    if [row["role"] for row in rows] != [
            "launch", "recorder", "video", "prefix", "middle_suffix",
            "postflight"]:
        raise RuntimeError("PROCESS_ORDER")
    processes = {row["role"]: row["argv"] for row in rows}

    launch = processes["launch"]
    set_flag_value(launch, "--self-sha256",
                   source_hashes["supervisor_source"])
    # Rebind every pre-existing launch input from current source bytes.  No
    # digest is inherited from the v8 command template.
    set_flag_value(launch, "--gazebo-launch-source-sha256",
                   source_hashes["gazebo_launch_source"])
    set_flag_value(launch, "--gazebo-world-sdf-sha256",
                   source_hashes["gazebo_world_sdf"])
    set_flag_value(launch, "--incubator-model-sdf-sha256",
                   source_hashes["incubator_model_sdf"])
    launch_insert = launch.index("spawn_incubator:=true")
    launch[launch_insert:launch_insert] = [
        "--robot-gazebo-control-xacro",
        str(AUTHORITY / "h1_2.gazebo.ros2_control.xacro"),
        "--robot-gazebo-control-xacro-sha256",
        source_hashes["robot_gazebo_control_xacro"],
        "--ros2-controllers-config",
        str(AUTHORITY / "ros2_controllers.yaml"),
        "--ros2-controllers-config-sha256",
        source_hashes["ros2_controllers_config"],
        "--robot-joint-state-relay-source",
        str(AUTHORITY / "h1_robot_joint_state_relay_829.py"),
        "--robot-joint-state-relay-source-sha256",
        source_hashes["robot_joint_state_relay_source"],
        "--telemetry-bridge-config",
        str(AUTHORITY / "candidate3_telemetry_bridge_829.yaml"),
        "--telemetry-bridge-config-sha256",
        source_hashes["telemetry_bridge_config"],
    ]
    launch.extend([
        "robot_joint_state_relay_source:=" +
        str(AUTHORITY / "h1_robot_joint_state_relay_829.py"),
        "robot_joint_state_relay_sha256:=" +
        source_hashes["robot_joint_state_relay_source"],
        "telemetry_bridge_config:=" +
        str(AUTHORITY / "candidate3_telemetry_bridge_829.yaml"),
        "telemetry_bridge_config_sha256:=" +
        source_hashes["telemetry_bridge_config"],
    ])

    recorder = [
        "/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "bag", "record",
        "--include-unpublished-topics", "-s", "sqlite3", "-o",
        str(RUN_ROOT / "rosbag_829"),
        "--qos-profile-overrides-path",
        str(AUTHORITY / "candidate3_rosbag_qos_overrides_829.yaml"),
        "--max-cache-size", "1073741824", "--topics",
        "/clock", "/joint_states", "/incubator/joint_states",
        "/torso_controller/joint_trajectory",
        "/right_arm_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
        "/left_gripper_controller/joint_trajectory",
        "/right_gripper_controller/joint_trajectory",
        "/h1_candidate3/live_829/markers",
    ]
    processes["recorder"][:] = recorder

    video = processes["video"]
    set_flag_value(video, "--self-sha256",
                   source_hashes["supervisor_source"])
    set_flag_value(video, "--run-id", run_id)

    for role, pairs in {
        "prefix": (
            ("--right-executor", "--right-executor-sha256"),
            ("--right-manifest", "--right-manifest-sha256"),
            ("--support-orchestrator", "--support-orchestrator-sha256"),
            ("--support-manifest", "--support-manifest-sha256"),
            ("--support-certificate", "--support-certificate-sha256"),
        ),
        "middle_suffix": (
            ("--right-executor", "--right-executor-sha256"),
            ("--right-manifest", "--right-manifest-sha256"),
            ("--support-orchestrator", "--support-orchestrator-sha256"),
            ("--support-manifest", "--support-manifest-sha256"),
            ("--support-certificate", "--support-certificate-sha256"),
            ("--scene", "--scene-sha256"),
            ("--plan", "--plan-sha256"),
            ("--certificate", "--certificate-sha256"),
            ("--result", "--result-sha256"),
            ("--middle-executor", "--middle-executor-sha256"),
            ("--middle-admission", "--middle-admission-sha256"),
            ("--middle-contract", "--middle-contract-sha256"),
        ),
        "postflight": (
            ("--manifest", "--manifest-sha256"),
            ("--certificate", "--certificate-sha256"),
        ),
    }.items():
        for path_flag, sha_flag in pairs:
            refresh_path_sha(processes[role], path_flag, sha_flag)

    fcl_manifest = json.loads(
        checked_source(FCL_ROOT / "manifest.json").decode("utf-8"))
    if (fcl_manifest.get("schema") !=
            "h1_moveit/right_handle_live_fcl_authority/829-v4-31j" or
            fcl_manifest.get("valid") is not True or
            fcl_manifest.get("sealed") is not True):
        raise RuntimeError("FCL_MANIFEST_HEADER")
    fcl_files = {
        identifier: str(FCL_ROOT / Path(entry["path"]).name)
        for identifier, entry in fcl_manifest["files"].items()
    }
    commands["exact_fcl_replay"] = {
        "argv": [
            fcl_files["auditor_binary"],
            "--scene", fcl_files["scene_cdr"],
            "--path", fcl_files["contact_path"],
            "--telemetry-tsv", str(RUN_ROOT / "exact_fcl_telemetry_829.tsv"),
            "--urdf", fcl_files["urdf"],
            "--srdf", fcl_files["srdf"],
            "--handle-stl", fcl_files["original_handle_stl"],
        ],
        "files": fcl_files,
    }
    commands_raw = canonical_bytes(commands)

    previous_umask = os.umask(0o077)
    try:
        os.mkdir(AUTHORITY, 0o700)
        copied: dict[str, dict[str, Any]] = {}
        target_names: dict[str, str] = {}
        for identifier, source, target_name, mode in source_to_target:
            target = AUTHORITY / target_name
            digest = exclusive_write(
                target, source_raw[identifier], mode)
            if (digest != source_hashes[identifier] or
                    sha256_file(source) != source_hashes[identifier]):
                raise RuntimeError(f"SOURCE_COPY_MISMATCH:{identifier}")
            copied[identifier] = binding(target)
            target_names[identifier] = target_name
        commands_sha = exclusive_write(
            AUTHORITY / "commands_829.json", commands_raw, 0o400)
        copied["commands"] = binding(AUTHORITY / "commands_829.json")

        manifest = {
            "schema": "h1_candidate3_live_production_authority/829-v10",
            "run_id": run_id,
            "run_root": str(RUN_ROOT),
            "controller_commands_issued": 0,
            "exact_fcl_authority": {
                "path": str((FCL_ROOT / "manifest.json").relative_to(
                    WORKSPACE)),
                "sha256": sha256_file(FCL_ROOT / "manifest.json"),
                "file_count": 9,
            },
            "execution_contract": {
                "baseline": "826.3",
                "table_x_m": 0.600,
                "incubator_x_m": 0.590,
                "right_handle_only": True,
                "left_hand_handle_contact_forbidden": True,
                "planned_maximum_abs_torso_deg": 44.25,
                "hard_maximum_abs_torso_deg": 44.5,
                "right_time_scale": 2.0,
                "middle_speed_scale": 0.5,
                "terminal_cube_target_position_m": [0.611, -0.03, 0.995],
                "terminal_cube_target_orientation_xyzw": [
                    0.0, 0.0, 0.7071067811865475,
                    0.7071067811865476],
                "terminal_cube_translation_tolerance_m": 0.003,
                "terminal_cube_rotation_tolerance_deg": 2.0,
                "public_joint_state_exact_gap_ns": 10_000_000,
                "raw_clock_exact_gap_ns": 1_000_000,
                "incubator_joint_state_exact_gap_ns": 1_000_000,
            },
            "files": {
                identifier: {
                    "path": str((AUTHORITY / target_names[identifier]).relative_to(
                        WORKSPACE)),
                    "sha256": copied[identifier]["sha256"],
                    "mode": copied[identifier]["mode"],
                    "nlink": 1,
                }
                for identifier in target_names
            } | {
                "commands": {
                    "path": str((AUTHORITY / "commands_829.json").relative_to(
                        WORKSPACE)),
                    "sha256": commands_sha,
                    "mode": "0400",
                    "nlink": 1,
                }
            },
            "supersedes": {
                "path": "runtime_assets/candidate3_live_829_authority_v8",
                "reason": (
                    "v10 preserves the sealed 826.3 motion and workcell "
                    "contract while binding the v4 31-joint live FCL "
                    "authority, lossless recorder QoS/cache contract, native "
                    "robot telemetry relay and bridge configuration, exact "
                    "10 ms public robot states, and exact 1 ms raw clock and "
                    "incubator evidence gaps"),
            },
            "sealed": True,
            "valid": True,
            "status": "READY",
        }
        manifest_sha = exclusive_write(
            AUTHORITY / "manifest.json", pretty_bytes(manifest), 0o400)
        os.chmod(AUTHORITY, 0o555)

        os.mkdir(RUN_ROOT, 0o700)
        os.mkdir(RUN_ROOT / "logs", 0o700)

        supervisor = import_sealed_supervisor(
            AUTHORITY / "h1_candidate3_live_supervisor_829.py")
        evidence_paths = supervisor.EvidencePaths.from_root(RUN_ROOT)
        bundle = supervisor.load_commands(
            AUTHORITY / "commands_829.json", evidence_paths)
        if bundle.run_id != run_id or bundle.raw_sha256 != commands_sha:
            raise RuntimeError("COMMANDS_READBACK")

        old_formal = json.loads(checked_source(
            OLD_RUN_ROOT / "formal_authorities_829.json").decode("utf-8"))
        first_twelve = old_formal["authorities"][:12]
        expected_first_twelve = list(supervisor.FORMAL_AUTHORITY_IDS[:12])
        if [row["id"] for row in first_twelve] != expected_first_twelve:
            raise RuntimeError("FORMAL_PREFIX_ORDER")
        authority_paths: dict[str, Path] = {
            row["id"]: Path(row["path"]) for row in first_twelve
        }
        authority_paths.update({
            "gazebo_launch_source": AUTHORITY / "gazebo.launch.py",
            "gazebo_world_sdf": AUTHORITY / "h1_world.sdf",
            "incubator_model_sdf": AUTHORITY / "incubator_model.sdf",
            "robot_gazebo_control_xacro":
                AUTHORITY / "h1_2.gazebo.ros2_control.xacro",
            "ros2_controllers_config":
                AUTHORITY / "ros2_controllers.yaml",
            "robot_joint_state_relay_source":
                AUTHORITY / "h1_robot_joint_state_relay_829.py",
            "telemetry_bridge_config":
                AUTHORITY / "candidate3_telemetry_bridge_829.yaml",
            "rosbag_qos_overrides":
                AUTHORITY / "candidate3_rosbag_qos_overrides_829.yaml",
        })
        if tuple(authority_paths) != supervisor.FORMAL_AUTHORITY_IDS:
            raise RuntimeError("FORMAL_AUTHORITY_ORDER")

        # Dry-run every cross-binding performed by prepare(), without calling
        # it or publishing formal/plan evidence into r9.
        authority_bindings = {
            identifier: binding(path)
            for identifier, path in authority_paths.items()
        }
        right_authorization = authority_bindings[
            "right_execution_authorization"]
        for role in ("prefix", "middle_suffix"):
            argv = bundle.processes[role]
            if (flag_value(argv, "--right-manifest") !=
                    right_authorization["path"] or
                    flag_value(argv, "--right-manifest-sha256") !=
                    right_authorization["sha256"]):
                raise RuntimeError(
                    f"PREPARE_RIGHT_AUTHORIZATION:{role}")
        right_manifest = json.loads(Path(
            authority_bindings["right_manifest"]["path"]
        ).read_text(encoding="utf-8"))
        manifest_authorization = right_manifest[
            "sealed_bindings"]["controller_execution_authorization"]
        raw_manifest_auth_path = manifest_authorization["path"]
        if not Path(raw_manifest_auth_path).is_absolute():
            path_matches = right_authorization["path"].endswith(
                "/" + raw_manifest_auth_path)
        else:
            path_matches = raw_manifest_auth_path == right_authorization["path"]
        if (not path_matches or
                manifest_authorization["sha256"] !=
                right_authorization["sha256"] or
                manifest_authorization["mode_octal"] != "0400" or
                manifest_authorization["nlink"] != 1 or
                manifest_authorization["schema"] !=
                supervisor.RIGHT_EXECUTION_AUTHORIZATION_SCHEMA):
            raise RuntimeError("PREPARE_RIGHT_MANIFEST_CROSS_BIND")
        launch_cross_bindings = (
            ("gazebo_launch_source", "--gazebo-launch-source",
             "--gazebo-launch-source-sha256"),
            ("gazebo_world_sdf", "--gazebo-world-sdf",
             "--gazebo-world-sdf-sha256"),
            ("incubator_model_sdf", "--incubator-model-sdf",
             "--incubator-model-sdf-sha256"),
            ("robot_gazebo_control_xacro", "--robot-gazebo-control-xacro",
             "--robot-gazebo-control-xacro-sha256"),
            ("ros2_controllers_config", "--ros2-controllers-config",
             "--ros2-controllers-config-sha256"),
            ("robot_joint_state_relay_source",
             "--robot-joint-state-relay-source",
             "--robot-joint-state-relay-source-sha256"),
            ("telemetry_bridge_config", "--telemetry-bridge-config",
             "--telemetry-bridge-config-sha256"),
        )
        launch_argv = bundle.processes["launch"]
        for identifier, path_flag, sha_flag in launch_cross_bindings:
            expected = authority_bindings[identifier]
            if (flag_value(launch_argv, path_flag) != expected["path"] or
                    flag_value(launch_argv, sha_flag) != expected["sha256"] or
                    expected["mode"] != "0400"):
                raise RuntimeError(f"PREPARE_LAUNCH_CROSS_BIND:{identifier}")
        recorder_qos = authority_bindings["rosbag_qos_overrides"]
        if (flag_value(bundle.processes["recorder"],
                       "--qos-profile-overrides-path") !=
                recorder_qos["path"] or recorder_qos["mode"] != "0400"):
            raise RuntimeError("PREPARE_RECORDER_QOS_CROSS_BIND")
        if (Path(bundle.processes["video"][1]) !=
                AUTHORITY / "h1_candidate3_live_supervisor_829.py" or
                flag_value(bundle.processes["video"], "--self-sha256") !=
                source_hashes["supervisor_source"]):
            raise RuntimeError("PREPARE_VIDEO_EVIDENCE_CROSS_BIND")
        # Manifest, mode, link, and content audit after the authority is sealed.
        authority_metadata = AUTHORITY.lstat()
        if (AUTHORITY.is_symlink() or
                stat.S_IMODE(authority_metadata.st_mode) != 0o555):
            raise RuntimeError("AUTHORITY_DIRECTORY_MODE")
        loaded_manifest = json.loads(
            (AUTHORITY / "manifest.json").read_text(encoding="utf-8"))
        if loaded_manifest != manifest:
            raise RuntimeError("MANIFEST_READBACK")
        for identifier, entry in manifest["files"].items():
            path = WORKSPACE / entry["path"]
            actual = binding(path)
            if (actual["sha256"] != entry["sha256"] or
                    actual["mode"] != entry["mode"] or
                    path.lstat().st_nlink != entry["nlink"]):
                raise RuntimeError(f"MANIFEST_FILE:{identifier}")
        if binding(AUTHORITY / "manifest.json")["sha256"] != manifest_sha:
            raise RuntimeError("FINAL_DIGEST_READBACK")
        if set(RUN_ROOT.iterdir()) != {RUN_ROOT / "logs"}:
            raise RuntimeError("RUN_ROOT_CONTENTS")
        if any((RUN_ROOT / name).exists() for name in (
                "formal_authorities_829.json", "live_plan_829.json")):
            raise RuntimeError("RUN_ROOT_PREPARE_OUTPUT_FORBIDDEN")

        # Build the exact command that the root agent will use to invoke the
        # sealed production prepare operation.  Validate all rows and modes
        # now, but deliberately do not call prepare here.
        prepare_command = [
            "/usr/bin/python3",
            str(AUTHORITY / "h1_candidate3_live_supervisor_829.py"),
            "prepare", "--run-root", str(RUN_ROOT),
            "--commands", str(AUTHORITY / "commands_829.json"),
        ]
        for identifier in supervisor.FORMAL_AUTHORITY_IDS:
            actual = binding(authority_paths[identifier])
            if actual["path"] != str(authority_paths[identifier]):
                raise RuntimeError(f"PREPARE_AUTHORITY:{identifier}")
            prepare_command.extend([
                "--authority", f"{identifier}={authority_paths[identifier]}"])
        for identifier, path in (
            ("supervisor_source",
             AUTHORITY / "h1_candidate3_live_supervisor_829.py"),
            ("certificate_source",
             AUTHORITY / "h1_candidate3_live_certificate_829.py"),
        ):
            if binding(path)["mode"] != "0500":
                raise RuntimeError(f"PREPARE_EVIDENCE_TOOL:{identifier}")
            prepare_command.extend([
                "--evidence-tool", f"{identifier}={path}"])
        parsed_authorities = supervisor._authority_arguments([
            prepare_command[index + 1]
            for index, item in enumerate(prepare_command)
            if item == "--authority"
        ])
        parsed_tools = supervisor._evidence_tool_arguments([
            prepare_command[index + 1]
            for index, item in enumerate(prepare_command)
            if item == "--evidence-tool"
        ])
        if (tuple(parsed_authorities) != supervisor.FORMAL_AUTHORITY_IDS or
                tuple(parsed_tools) != supervisor.EVIDENCE_TOOL_IDS):
            raise RuntimeError("PREPARE_DRY_PARSE")
        result = {
            "status": "READY",
            "valid": True,
            "controller_commands_issued": 0,
            "run_id": run_id,
            "authority": str(AUTHORITY),
            "run_root": str(RUN_ROOT),
            "authority_manifest_sha256": manifest_sha,
            "commands_sha256": commands_sha,
            "authority_files": {
                path.name: sha256_file(path)
                for path in sorted(AUTHORITY.iterdir())
                if path.is_file()
            },
            "run_files": {},
            "prepare_command_argv": prepare_command,
            "load_commands_self_check": "PASS",
            "prepare_dry_validation": "PASS",
        }
        print(json.dumps(result, sort_keys=True, indent=2))
    finally:
        os.umask(previous_umask)


if __name__ == "__main__":
    main()
