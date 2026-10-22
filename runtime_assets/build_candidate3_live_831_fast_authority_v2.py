#!/usr/bin/python3
"""Build the non-overwriting Candidate-3 fast-831 authority v14."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
from typing import Any, Mapping, Sequence


WS = Path("/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
BASE = WS / "runtime_assets/candidate3_live_829_authority_v18"
NEW = WS / "runtime_assets/candidate3_live_831_fast_authority_v14"
RUN_ROOT = WS / "runtime_assets/candidate3_live_831_fast_r14"
RIGHT_ROOT = WS / "runtime_assets/right_handle_8263_natural/variable_torso_v4_829"
OLD_RIGHT_AUTH = RIGHT_ROOT / "controller_execution_authorization.json"
OLD_RIGHT_MANIFEST = RIGHT_ROOT / "manifest.json"
FAST_SCRIPTS = WS / "src/h1_2_moveit_demo/scripts"
FAST_PACKAGE = WS / "install/h1_candidate3_fast_831/share/h1_candidate3_fast_831"
OLD_FCL_AUTHORITY = RIGHT_ROOT / "live_fcl_authority_v6_31j"
NEW_FCL_AUTHORITY = RIGHT_ROOT / "live_fcl_authority_v8_31j"
NEW_FCL_MANIFEST_SHA256 = (
    "b73f57d3d7d5a02ead65e3fdbdd6b6d0c71ec136a8334c1f41b3bd6de85b4d67")

BASE_MANIFEST_SHA256 = "0679eb7e9169db5fdc309284eedf8dffd04f79fd1cff0e977306f1024f1ae320"
BASE_COMMANDS_SHA256 = "976c0a694da226ad62522c99a9b66c304f76e1bab58572c5b2a6e0a6145f5b80"
OLD_RIGHT_AUTH_SHA256 = "9892913662edb89fd3ccf7be4d68adc527b7058121c2a1b57ddcc8a2e08f52d2"
OLD_RIGHT_MANIFEST_SHA256 = "b2fcc6e09e48990d20c0ee4d6939faad4f20e08dadaea5638c7e14aec9fa9b52"

UNCHANGED = (
    "candidate3_rosbag_qos_overrides_829.yaml",
    "candidate3_telemetry_bridge_829.yaml",
    "h1_2.gazebo.ros2_control.xacro",
    "incubator_model.sdf",
    "ros2_controllers.yaml",
)
FAST_SOURCE_MAP = {
    "h1_candidate3_live_supervisor_829.py":
        FAST_SCRIPTS / "h1_candidate3_live_supervisor_fast_831.py",
    "h1_candidate3_live_certificate_829.py":
        FAST_SCRIPTS / "h1_candidate3_live_certificate_fast_831_v3.py",
    "h1_candidate3_flow_828.py":
        FAST_SCRIPTS / "h1_candidate3_flow_fast_831_v3.py",
    "h1_right_handle_executor_828.py":
        FAST_SCRIPTS / "h1_right_handle_executor_fast_831.py",
    "h1_candidate3_middle_executor_828.py":
        FAST_SCRIPTS / "h1_candidate3_middle_executor_fast_831_v3.py",
    "h1_robot_joint_state_relay_829.py":
        FAST_SCRIPTS / "h1_robot_joint_state_relay_fast_831.py",
    "h1_fast_support_daemon_831.py":
        FAST_SCRIPTS / "h1_fast_support_daemon_831.py",
    "h1_support_orchestrator_828.py":
        FAST_SCRIPTS / "h1_support_orchestrator_828.py",
    "h1_candidate3_fast90_certificate_831.py":
        FAST_SCRIPTS / "h1_candidate3_fast90_certificate_831.py",
}


def sha(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def read_regular(path: Path, expected_sha: str | None = None) -> bytes:
    before = path.lstat()
    if (path.is_symlink() or not stat.S_ISREG(before.st_mode) or
            before.st_nlink != 1 or path.resolve(strict=True) != path):
        raise RuntimeError(f"INPUT_IDENTITY:{path}")
    raw = path.read_bytes()
    after = path.lstat()
    signature = lambda item: (
        item.st_dev, item.st_ino, item.st_size, item.st_mtime_ns,
        item.st_ctime_ns)
    if signature(before) != signature(after) or len(raw) != after.st_size:
        raise RuntimeError(f"INPUT_CHANGED:{path}")
    if expected_sha is not None and sha(raw) != expected_sha:
        raise RuntimeError(f"INPUT_SHA256:{path}")
    return raw


def parse(raw: bytes, label: str) -> dict[str, Any]:
    def unique(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise RuntimeError(f"DUPLICATE_KEY:{label}:{key}")
            result[key] = value
        return result
    value = json.loads(
        raw.decode("utf-8"), object_pairs_hook=unique,
        parse_constant=lambda token: (_ for _ in ()).throw(
            RuntimeError(f"NONFINITE:{label}:{token}")))
    if type(value) is not dict:
        raise RuntimeError(f"JSON_ROOT:{label}")
    return value


def canonical(value: Any) -> bytes:
    return (json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def pretty(value: Any) -> bytes:
    return (json.dumps(
        value, sort_keys=True, indent=2, ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def replace_tree(value: Any, replacements: Sequence[tuple[str, str]]) -> Any:
    if isinstance(value, str):
        for old, new in replacements:
            value = value.replace(old, new)
        return value
    if isinstance(value, list):
        return [replace_tree(item, replacements) for item in value]
    if isinstance(value, dict):
        return {key: replace_tree(item, replacements)
                for key, item in value.items()}
    return value


def set_flag(argv: list[str], flag: str, value: str) -> None:
    indices = [index for index, item in enumerate(argv) if item == flag]
    if len(indices) != 1 or indices[0] + 1 >= len(argv):
        raise RuntimeError(f"FLAG:{flag}")
    argv[indices[0] + 1] = value


def get_flag(argv: Sequence[str], flag: str) -> str:
    indices = [index for index, item in enumerate(argv) if item == flag]
    if len(indices) != 1 or indices[0] + 1 >= len(argv):
        raise RuntimeError(f"FLAG:{flag}")
    return argv[indices[0] + 1]


def write_exclusive(root: Path, name: str, raw: bytes, mode: int) -> None:
    path = root / name
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL |
        getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0), mode)
    try:
        view = memoryview(raw)
        while view:
            count = os.write(descriptor, view)
            if count <= 0:
                raise RuntimeError(f"SHORT_WRITE:{path}")
            view = view[count:]
        os.fchmod(descriptor, mode)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    if read_regular(path) != raw or stat.S_IMODE(path.stat().st_mode) != mode:
        raise RuntimeError(f"WRITE_READBACK:{path}")


def binding(root: Path, name: str) -> Mapping[str, Any]:
    path = root / name
    raw = read_regular(path)
    mode = stat.S_IMODE(path.stat().st_mode)
    return {
        "path": str(path.relative_to(WS)), "sha256": sha(raw),
        "mode": f"{mode:04o}", "nlink": path.stat().st_nlink,
        "size_bytes": len(raw),
    }


def main() -> None:
    if NEW.exists() or NEW.is_symlink() or RUN_ROOT.exists() or RUN_ROOT.is_symlink():
        raise FileExistsError("FAST_AUTHORITY_OR_RUN_ROOT_EXISTS")
    read_regular(BASE / "manifest.json", BASE_MANIFEST_SHA256)
    base_commands = parse(read_regular(
        BASE / "commands_829.json", BASE_COMMANDS_SHA256), "BASE_COMMANDS")
    old_auth = parse(read_regular(
        OLD_RIGHT_AUTH, OLD_RIGHT_AUTH_SHA256), "OLD_RIGHT_AUTH")
    old_right_manifest = parse(read_regular(
        OLD_RIGHT_MANIFEST, OLD_RIGHT_MANIFEST_SHA256), "OLD_RIGHT_MANIFEST")
    read_regular(NEW_FCL_AUTHORITY / "manifest.json",
                 NEW_FCL_MANIFEST_SHA256)

    source_bytes = {name: read_regular(path)
                    for name, path in FAST_SOURCE_MAP.items()}
    launch_raw = read_regular(FAST_PACKAGE / "launch/gazebo.launch.py")
    world_raw = read_regular(FAST_PACKAGE / "worlds/h1_world.sdf")
    if (launch_raw != read_regular(
            WS / "src/h1_candidate3_fast_831/launch/gazebo.launch.py") or
            world_raw != read_regular(
                WS / "src/h1_candidate3_fast_831/worlds/h1_world.sdf")):
        raise RuntimeError("FAST_PACKAGE_INSTALL_MISMATCH")

    old_auth["limits"]["point_dt_s"] = 0.004
    auth_raw = pretty(old_auth)
    auth_rel = str((NEW / "controller_execution_authorization.json").relative_to(WS))
    auth_binding = old_right_manifest["sealed_bindings"][
        "controller_execution_authorization"]
    auth_binding["path"] = auth_rel
    auth_binding["sha256"] = sha(auth_raw)
    right_manifest_raw = pretty(old_right_manifest)

    identity_seed = b"\0".join((
        b"candidate3-fast-831-v14", launch_raw, world_raw,
        *(source_bytes[name] for name in sorted(source_bytes)),
        auth_raw, bytes.fromhex(NEW_FCL_MANIFEST_SHA256),
    ))
    run_id = sha(identity_seed)
    socket_path = Path(
        f"/run/user/{os.geteuid()}/h1fs831_{run_id[:32]}.sock")
    if len(os.fsencode(socket_path)) > 100:
        raise RuntimeError("FAST_SUPPORT_SOCKET_PATH_TOO_LONG")
    replacements = (
        (base_commands["run_id"], run_id),
        (str(BASE), str(NEW)),
        (str(WS / "runtime_assets/candidate3_live_829_final_r17"),
         str(RUN_ROOT)),
        (str(FAST_SCRIPTS / "h1_candidate3_flow_828.py"),
         str(NEW / "h1_candidate3_flow_828.py")),
        (str(FAST_SCRIPTS / "h1_right_handle_executor_828.py"),
         str(NEW / "h1_right_handle_executor_828.py")),
        (str(FAST_SCRIPTS / "h1_candidate3_middle_executor_828.py"),
         str(NEW / "h1_candidate3_middle_executor_828.py")),
        (str(OLD_RIGHT_AUTH),
         str(NEW / "controller_execution_authorization.json")),
        (str(OLD_FCL_AUTHORITY), str(NEW_FCL_AUTHORITY)),
        (str(FAST_SCRIPTS / "h1_support_orchestrator_828.py"),
         str(NEW / "h1_support_orchestrator_828.py")),
    )
    commands = replace_tree(base_commands, replacements)
    commands["run_id"] = run_id
    processes = {row["role"]: row["argv"] for row in commands["processes"]}
    supervisor_sha = sha(source_bytes["h1_candidate3_live_supervisor_829.py"])
    flow_sha = sha(source_bytes["h1_candidate3_flow_828.py"])
    right_sha = sha(source_bytes["h1_right_handle_executor_828.py"])
    middle_sha = sha(source_bytes["h1_candidate3_middle_executor_828.py"])
    relay_sha = sha(source_bytes["h1_robot_joint_state_relay_829.py"])
    daemon_sha = sha(source_bytes["h1_fast_support_daemon_831.py"])
    support_sha = sha(source_bytes["h1_support_orchestrator_828.py"])
    for role in ("launch", "video"):
        processes[role][1] = str(NEW / "h1_candidate3_live_supervisor_829.py")
        set_flag(processes[role], "--self-sha256", supervisor_sha)
    processes["prefix"][1] = str(NEW / "h1_candidate3_flow_828.py")
    processes["middle_suffix"][1] = str(NEW / "h1_candidate3_flow_828.py")
    for role in ("prefix", "middle_suffix"):
        argv = processes[role]
        set_flag(argv, "--right-executor",
                 str(NEW / "h1_right_handle_executor_828.py"))
        set_flag(argv, "--right-executor-sha256", right_sha)
        set_flag(argv, "--right-manifest",
                 str(NEW / "controller_execution_authorization.json"))
        set_flag(argv, "--right-manifest-sha256", sha(auth_raw))
        set_flag(argv, "--right-time-scale", "1.0")
        set_flag(argv, "--support-orchestrator-sha256", support_sha)
    set_flag(processes["middle_suffix"], "--middle-executor",
             str(NEW / "h1_candidate3_middle_executor_828.py"))
    set_flag(processes["middle_suffix"], "--middle-executor-sha256", middle_sha)
    set_flag(processes["middle_suffix"], "--middle-speed-scale", "1.0")
    processes["middle_suffix"].extend((
        "--fast-support-daemon",
        str(NEW / "h1_fast_support_daemon_831.py"),
        "--fast-support-daemon-sha256", daemon_sha,
    ))
    for role in ("prefix", "middle_suffix"):
        processes[role].extend((
            "--fast-support-daemon-socket", str(socket_path),
        ))
    daemon_argv = [
        "/usr/bin/python3",
        str(NEW / "h1_fast_support_daemon_831.py"),
        "serve",
        "--socket", str(socket_path),
        "--owner-pid", "0",
        "--owner-start-ticks", "0",
        "--support-orchestrator",
        str(NEW / "h1_support_orchestrator_828.py"),
        "--support-orchestrator-sha256", support_sha,
        "--manifest",
        get_flag(processes["middle_suffix"], "--support-manifest"),
        "--manifest-sha256",
        get_flag(processes["middle_suffix"], "--support-manifest-sha256"),
        "--certificate",
        get_flag(processes["middle_suffix"], "--support-certificate"),
        "--certificate-sha256",
        get_flag(
            processes["middle_suffix"], "--support-certificate-sha256"),
        "--workspace", str(WS),
        "--timeout",
        get_flag(processes["middle_suffix"], "--support-timeout"),
        "--startup-timeout", "120.0",
        "--request-timeout", "5.0",
        "--telemetry-max-age", "1.0",
    ]
    commands["processes"].insert(3, {
        "role": "support_daemon", "argv": daemon_argv,
    })
    processes["support_daemon"] = daemon_argv
    launch = processes["launch"]
    set_flag(launch, "--gazebo-launch-source", str(NEW / "gazebo.launch.py"))
    set_flag(launch, "--gazebo-launch-source-sha256", sha(launch_raw))
    set_flag(launch, "--gazebo-world-sdf", str(NEW / "h1_world.sdf"))
    set_flag(launch, "--gazebo-world-sdf-sha256", sha(world_raw))
    set_flag(launch, "--robot-joint-state-relay-source",
             str(NEW / "h1_robot_joint_state_relay_829.py"))
    set_flag(launch, "--robot-joint-state-relay-source-sha256", relay_sha)
    for index, item in enumerate(launch):
        if item.startswith("robot_joint_state_relay_source:="):
            launch[index] = "robot_joint_state_relay_source:=" + str(
                NEW / "h1_robot_joint_state_relay_829.py")
        elif item.startswith("robot_joint_state_relay_sha256:="):
            launch[index] = "robot_joint_state_relay_sha256:=" + relay_sha

    schedule = {
        "schema": "h1_candidate3_fast_timing_schedule/831-v9",
        "baseline": "826.3",
        "physics_step_ns": 5_000_000,
        "public_robot_joint_period_ns": 10_000_000,
        "target_real_time_factor": 1.0,
        "right_dense_point_dt_s": 0.004,
        "right_time_scale": 1.0,
        "middle_non_left_gripper_time_divisor": 7.0,
        "middle_loaded_place_microsegment_time_divisor": 1.0,
        "middle_speed_scale": 1.0,
        "left_gripper_time_divisor": 1.0,
        "maximum_wall_motion_window_ns": 90_000_000_000,
        "slow_mode_enabled": False,
        "persistent_support_daemon": True,
        "support_daemon_prewarmed_before_motion": True,
        "lifecycle_guardian": True,
        "nested_control_pdeath": True,
        "final_plugin_proof": (
            "pinned_server_and_binary_plus_double_startup_topology_"
            "four_unique_transitions_and_final_fresh_heartbeats"),
        "gazebo_setproctitle_nul_tail_cleanup": True,
        "right_contact_marker_settle_sim_ns": 100_000_000,
        "geometry": {
            "table_x_m": 0.600, "incubator_x_m": 0.590,
            "contact_path_sha256":
                "92d0f131324db1755ecdf2a1e03b102b450fbd783a647625e9baf9dfb0a0fb98",
            "middle_plan_sha256":
                "cb20f8303cb2e75a5fe45025ea4a119fa5ba81054c8cda55442d320052d1634e",
            "left_hand_handle_contact_forbidden": True,
            "right_hand_only_handle_operation": True,
            "planned_maximum_abs_torso_deg": 44.25,
            "hard_maximum_abs_torso_deg": 44.5,
        },
        "expected_controller_message_count": 30,
        "run_id": run_id,
        "valid": True,
        "status": "READY",
    }
    schedule_raw = canonical(schedule)

    parent = NEW.parent
    temporary = parent / f".{NEW.name}.building-{os.getpid()}"
    if temporary.exists() or temporary.is_symlink():
        raise FileExistsError("FAST_AUTHORITY_TEMP_EXISTS")
    temporary.mkdir(mode=0o700)
    try:
        for name in UNCHANGED:
            write_exclusive(temporary, name, read_regular(BASE / name), 0o400)
        write_exclusive(temporary, "gazebo.launch.py", launch_raw, 0o400)
        write_exclusive(temporary, "h1_world.sdf", world_raw, 0o400)
        for name, raw in source_bytes.items():
            mode = 0o400 if name == "h1_robot_joint_state_relay_829.py" else 0o500
            write_exclusive(temporary, name, raw, mode)
        write_exclusive(temporary, "controller_execution_authorization.json",
                        auth_raw, 0o400)
        write_exclusive(temporary, "right_handle_manifest.json",
                        right_manifest_raw, 0o400)
        write_exclusive(temporary, "fast_90_schedule_831.json",
                        schedule_raw, 0o400)
        commands_raw = canonical(commands)
        write_exclusive(temporary, "commands_829.json", commands_raw, 0o400)

        file_names = sorted(entry.name for entry in os.scandir(temporary))
        files = {name: binding(temporary, name) for name in file_names}
        for value in files.values():
            value["path"] = value["path"].replace(temporary.name, NEW.name, 1)
        manifest = {
            "schema": "h1_candidate3_live_fast_authority/831-v14",
            "run_id": run_id,
            "run_root": str(RUN_ROOT),
            "execution_contract": {
                "baseline": "826.3", "slow_mode_enabled": False,
                "right_time_scale": 1.0, "middle_speed_scale": 1.0,
                "right_dense_point_dt_s": 0.004,
                "middle_non_left_gripper_time_divisor": 7.0,
                "middle_loaded_place_microsegment_time_divisor": 1.0,
                "physics_step_ns": 5_000_000,
                "public_joint_state_exact_gap_ns": 10_000_000,
                "maximum_wall_motion_window_ns": 90_000_000_000,
                "persistent_support_daemon": True,
                "support_daemon_prewarmed_before_motion": True,
                "lifecycle_guardian": True,
                "nested_control_pdeath": True,
                "final_plugin_proof": (
                    "pinned_server_and_binary_plus_double_startup_topology_"
                    "four_unique_transitions_and_final_fresh_heartbeats"),
                "gazebo_setproctitle_nul_tail_cleanup": True,
                "right_contact_marker_settle_sim_ns": 100_000_000,
                "left_hand_handle_contact_forbidden": True,
                "right_handle_only": True,
                "table_x_m": 0.600, "incubator_x_m": 0.590,
                "planned_maximum_abs_torso_deg": 44.25,
                "hard_maximum_abs_torso_deg": 44.5,
            },
            "exact_fcl_authority": {
                "path": (
                    "runtime_assets/right_handle_8263_natural/"
                    "variable_torso_v4_829/live_fcl_authority_v8_31j/manifest.json"),
                "sha256": NEW_FCL_MANIFEST_SHA256,
                "file_count": 9,
            },
            "formal_authority_ids": [
                "right_execution_authorization", "right_manifest",
                "middle_plan", "middle_exact_result", "middle_certificate",
                "middle_support_manifest", "right_executor_source",
                "middle_executor_source", "flow_source",
                "fast_support_daemon_source", "support_source",
                "middle_admission", "middle_execution_contract",
                "gazebo_launch_source", "gazebo_world_sdf",
                "incubator_model_sdf", "robot_gazebo_control_xacro",
                "ros2_controllers_config", "robot_joint_state_relay_source",
                "telemetry_bridge_config", "rosbag_qos_overrides",
            ],
            "files": files,
            "supersedes": {
                "path": "runtime_assets/candidate3_live_831_fast_authority_v13",
                "reason": (
                    "v14 is a fresh non-overwriting live-demonstration "
                    "identity.  It retains v13's robot motion, 826.3 posture, "
                    "geometry, timings, commands, thresholds, evidence "
                    "verifiers and lifecycle proofs byte-for-byte unchanged; "
                    "only authority, run-root and run-id bindings advance"),
            },
            "controller_commands_issued": 0,
            "sealed": True, "valid": True, "status": "READY",
        }
        write_exclusive(temporary, "manifest.json", pretty(manifest), 0o400)
        os.chmod(temporary, 0o555)
        os.rename(temporary, NEW)
    except BaseException:
        if temporary.exists() and not temporary.is_symlink():
            shutil.rmtree(temporary)
        raise

    if stat.S_IMODE(NEW.stat().st_mode) != 0o555:
        raise RuntimeError("AUTHORITY_MODE")
    print(canonical({
        "authority": str(NEW), "run_root": str(RUN_ROOT),
        "run_id": run_id,
        "commands_sha256": sha(read_regular(NEW / "commands_829.json")),
        "manifest_sha256": sha(read_regular(NEW / "manifest.json")),
        "valid": True, "status": "PASS",
    }).decode("ascii"), end="")


if __name__ == "__main__":
    main()
