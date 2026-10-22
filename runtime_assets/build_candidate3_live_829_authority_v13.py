#!/usr/bin/python3
"""Create sealed authority v13 while leaving its r12 run root absent."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import secrets
import stat
import sys
from typing import Any, Sequence


WS = Path("/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
BASE = WS / "runtime_assets/candidate3_live_829_authority_v12"
NEW = WS / "runtime_assets/candidate3_live_829_authority_v13"
RUN_ROOT = WS / "runtime_assets/candidate3_live_829_final_r12"
OLD_FORMAL = WS / "runtime_assets/candidate3_live_829_final_r7/formal_authorities_829.json"


def lexists(path: Path) -> bool:
    return os.path.lexists(os.fspath(path))


def sha_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def canonical(value: Any) -> bytes:
    return (json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def pretty(value: Any) -> bytes:
    return (json.dumps(
        value, sort_keys=True, indent=2, ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def read_regular(path: Path, expected_mode: int | None = None) -> bytes:
    metadata = path.lstat()
    mode = stat.S_IMODE(metadata.st_mode)
    if (path.is_symlink() or not stat.S_ISREG(metadata.st_mode) or
            metadata.st_nlink != 1 or path.resolve(strict=True) != path or
            (expected_mode is not None and mode != expected_mode)):
        raise RuntimeError(f"SOURCE_IDENTITY:{path}")
    raw = path.read_bytes()
    after = path.lstat()
    if ((metadata.st_dev, metadata.st_ino, metadata.st_size,
         metadata.st_mtime_ns) !=
            (after.st_dev, after.st_ino, after.st_size,
             after.st_mtime_ns)):
        raise RuntimeError(f"SOURCE_CHANGED:{path}")
    return raw


def exclusive_write(path: Path, raw: bytes, mode: int) -> str:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags, mode)
    try:
        view = memoryview(raw)
        while view:
            count = os.write(descriptor, view)
            if count <= 0:
                raise RuntimeError(f"SHORT_WRITE:{path}")
            view = view[count:]
        os.fsync(descriptor)
        os.fchmod(descriptor, mode)
    finally:
        os.close(descriptor)
    metadata = path.lstat()
    if (path.is_symlink() or not stat.S_ISREG(metadata.st_mode) or
            metadata.st_nlink != 1 or
            stat.S_IMODE(metadata.st_mode) != mode or
            metadata.st_size != len(raw) or sha(path) != sha_bytes(raw)):
        raise RuntimeError(f"WRITE_IDENTITY:{path}")
    return sha_bytes(raw)


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


def binding(path: Path) -> dict[str, Any]:
    metadata = path.lstat()
    mode = stat.S_IMODE(metadata.st_mode)
    if (path.is_symlink() or not stat.S_ISREG(metadata.st_mode) or
            metadata.st_nlink != 1 or mode not in (0o400, 0o500) or
            path.resolve(strict=True) != path):
        raise RuntimeError(f"BINDING:{path}")
    return {"path": str(path), "sha256": sha(path),
            "size_bytes": metadata.st_size, "mode": f"{mode:04o}"}


def flag(argv: Sequence[str], name: str) -> str:
    positions = [i for i, item in enumerate(argv) if item == name]
    if len(positions) != 1 or positions[0] + 1 >= len(argv):
        raise RuntimeError(f"FLAG:{name}")
    return argv[positions[0] + 1]


def import_supervisor(path: Path) -> Any:
    sys.dont_write_bytecode = True
    name = "candidate3_live_829_authority_v13_supervisor"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError("IMPORT_SPEC")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main() -> None:
    if WS.resolve(strict=True) != WS:
        raise RuntimeError("WORKSPACE")
    if lexists(NEW) or lexists(RUN_ROOT):
        raise FileExistsError("V13_OR_R12_EXISTS")
    if (stat.S_IMODE(BASE.lstat().st_mode) != 0o555 or BASE.is_symlink() or
            sha(BASE / "manifest.json") !=
            "d5a67a17b02ded2b277ff44af4e30444d938a5ec934070faa58a6d1deb276332" or
            sha(BASE / "commands_829.json") !=
            "547331535448d43b05e0083e106757d2d92fa462c5415c73b1077b3b92d6875a"):
        raise RuntimeError("V12_BASELINE")

    v10_manifest = json.loads(read_regular(
        BASE / "manifest.json", 0o400).decode("utf-8"))
    v10_commands = json.loads(read_regular(
        BASE / "commands_829.json", 0o400).decode("utf-8"))
    old_run_id = v10_commands["run_id"]
    new_run_id = secrets.token_hex(32)
    if len(new_run_id) != 64 or new_run_id == old_run_id:
        raise RuntimeError("RUN_ID")
    replacements = (
        (str(BASE), str(NEW)),
        (str(WS / "runtime_assets/candidate3_live_829_final_r11"),
         str(RUN_ROOT)),
        (old_run_id, new_run_id),
    )
    commands = replace_tree(v10_commands, replacements)
    commands["run_id"] = new_run_id
    commands_raw = canonical(commands)

    copy_names = (
        "candidate3_rosbag_qos_overrides_829.yaml",
        "candidate3_telemetry_bridge_829.yaml",
        "gazebo.launch.py",
        "h1_2.gazebo.ros2_control.xacro",
        "h1_candidate3_live_certificate_829.py",
        "h1_candidate3_live_supervisor_829.py",
        "h1_robot_joint_state_relay_829.py",
        "h1_world.sdf",
        "incubator_model.sdf",
        "ros2_controllers.yaml",
    )
    source_bytes: dict[str, bytes] = {}
    source_modes: dict[str, int] = {}
    for name in copy_names:
        source = BASE / name
        source_modes[name] = stat.S_IMODE(source.lstat().st_mode)
        source_bytes[name] = read_regular(source, source_modes[name])

    previous_umask = os.umask(0o077)
    try:
        os.mkdir(NEW, 0o700)
        for name in copy_names:
            digest = exclusive_write(
                NEW / name, source_bytes[name], source_modes[name])
            if digest != sha(BASE / name):
                raise RuntimeError(f"BYTE_COPY:{name}")
        commands_sha = exclusive_write(
            NEW / "commands_829.json", commands_raw, 0o400)

        manifest = replace_tree(v10_manifest, replacements)
        manifest["schema"] = "h1_candidate3_live_production_authority/829-v13"
        manifest["run_id"] = new_run_id
        manifest["run_root"] = str(RUN_ROOT)
        # Rebuild every relative file path from the actual v13 destination.
        # This is intentionally not a string substitution: v11 proved that
        # absolute-only replacement can miss relative manifest paths.
        for identifier, entry in manifest["files"].items():
            name = Path(entry["path"]).name
            destination = NEW / name
            actual = binding(destination)
            entry.update({
                "path": str(destination.relative_to(WS)),
                "sha256": actual["sha256"],
                "mode": actual["mode"],
                "nlink": destination.lstat().st_nlink,
            })
        manifest["supersedes"] = {
            "path": "runtime_assets/candidate3_live_829_authority_v12",
            "reason": (
                "v13 preserves every v12 execution and evidence payload after "
                "the r11 formal startup gate failed before actions, while "
                "binding a fresh absent r12 run root and self-contained v13 "
                "authority paths"),
        }
        manifest_sha = exclusive_write(
            NEW / "manifest.json", pretty(manifest), 0o400)
        os.chmod(NEW, 0o555)

        if lexists(RUN_ROOT):
            raise RuntimeError("R12_CREATED")
        supervisor = import_supervisor(
            NEW / "h1_candidate3_live_supervisor_829.py")
        paths = supervisor.EvidencePaths.from_root(RUN_ROOT)
        bundle = supervisor.load_commands(NEW / "commands_829.json", paths)
        if bundle.run_id != new_run_id or bundle.raw_sha256 != commands_sha:
            raise RuntimeError("LOAD_COMMANDS")

        prior_formal = json.loads(read_regular(
            OLD_FORMAL, 0o400).decode("utf-8"))
        first_twelve = prior_formal["authorities"][:12]
        if ([row["id"] for row in first_twelve] !=
                list(supervisor.FORMAL_AUTHORITY_IDS[:12])):
            raise RuntimeError("AUTHORITY_PREFIX")
        authority_paths = {row["id"]: Path(row["path"])
                           for row in first_twelve}
        authority_paths.update({
            "gazebo_launch_source": NEW / "gazebo.launch.py",
            "gazebo_world_sdf": NEW / "h1_world.sdf",
            "incubator_model_sdf": NEW / "incubator_model.sdf",
            "robot_gazebo_control_xacro":
                NEW / "h1_2.gazebo.ros2_control.xacro",
            "ros2_controllers_config": NEW / "ros2_controllers.yaml",
            "robot_joint_state_relay_source":
                NEW / "h1_robot_joint_state_relay_829.py",
            "telemetry_bridge_config":
                NEW / "candidate3_telemetry_bridge_829.yaml",
            "rosbag_qos_overrides":
                NEW / "candidate3_rosbag_qos_overrides_829.yaml",
        })
        if tuple(authority_paths) != supervisor.FORMAL_AUTHORITY_IDS:
            raise RuntimeError("AUTHORITY_ORDER")
        authority_bindings = {key: binding(value)
                              for key, value in authority_paths.items()}

        right = authority_bindings["right_execution_authorization"]
        for role in ("prefix", "middle_suffix"):
            argv = bundle.processes[role]
            if (flag(argv, "--right-manifest") != right["path"] or
                    flag(argv, "--right-manifest-sha256") != right["sha256"]):
                raise RuntimeError(f"RIGHT_AUTH:{role}")
        launch_pairs = (
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
        launch = bundle.processes["launch"]
        for identifier, path_flag, sha_flag in launch_pairs:
            expected = authority_bindings[identifier]
            if (flag(launch, path_flag) != expected["path"] or
                    flag(launch, sha_flag) != expected["sha256"] or
                    expected["mode"] != "0400"):
                raise RuntimeError(f"LAUNCH_BIND:{identifier}")
        qos = authority_bindings["rosbag_qos_overrides"]
        if (flag(bundle.processes["recorder"],
                 "--qos-profile-overrides-path") != qos["path"] or
                qos["mode"] != "0400"):
            raise RuntimeError("RECORDER_QOS")
        if (Path(bundle.processes["video"][1]) !=
                NEW / "h1_candidate3_live_supervisor_829.py" or
                flag(bundle.processes["video"], "--self-sha256") !=
                sha(NEW / "h1_candidate3_live_supervisor_829.py")):
            raise RuntimeError("VIDEO_BIND")

        prepare_argv = [
            "/usr/bin/python3",
            str(NEW / "h1_candidate3_live_supervisor_829.py"),
            "prepare", "--run-root", str(RUN_ROOT),
            "--commands", str(NEW / "commands_829.json"),
        ]
        authority_args = []
        for identifier in supervisor.FORMAL_AUTHORITY_IDS:
            value = f"{identifier}={authority_paths[identifier]}"
            authority_args.append(value)
            prepare_argv.extend(["--authority", value])
        tool_args = [
            "supervisor_source=" +
            str(NEW / "h1_candidate3_live_supervisor_829.py"),
            "certificate_source=" +
            str(NEW / "h1_candidate3_live_certificate_829.py"),
        ]
        for value in tool_args:
            prepare_argv.extend(["--evidence-tool", value])
        if (tuple(supervisor._authority_arguments(authority_args)) !=
                supervisor.FORMAL_AUTHORITY_IDS or
                tuple(supervisor._evidence_tool_arguments(tool_args)) !=
                supervisor.EVIDENCE_TOOL_IDS):
            raise RuntimeError("PREPARE_DRY_PARSE")
        if lexists(RUN_ROOT):
            raise RuntimeError("R12_CREATED_DURING_VALIDATION")

        loaded_manifest = json.loads(
            (NEW / "manifest.json").read_text(encoding="utf-8"))
        if loaded_manifest != manifest:
            raise RuntimeError("MANIFEST_READBACK")
        for identifier, entry in manifest["files"].items():
            path = WS / entry["path"]
            actual = binding(path)
            if (actual["sha256"] != entry["sha256"] or
                    actual["mode"] != entry["mode"] or
                    path.lstat().st_nlink != entry["nlink"]):
                raise RuntimeError(f"MANIFEST_FILE:{identifier}")
        if (stat.S_IMODE(NEW.lstat().st_mode) != 0o555 or
                sha(NEW / "manifest.json") != manifest_sha):
            raise RuntimeError("FINAL_SEAL")
        for name in copy_names:
            if (NEW / name).read_bytes() != (BASE / name).read_bytes():
                raise RuntimeError(f"NONSELF_BYTE_DIFF:{name}")
        if lexists(RUN_ROOT):
            raise RuntimeError("R12_FINAL_LEXISTS")

        for identifier, entry in manifest["files"].items():
            entry_path = WS / entry["path"]
            if entry_path.parent != NEW or not entry_path.is_file():
                raise RuntimeError(f"MANIFEST_NOT_SELF_CONTAINED:{identifier}")

        print(json.dumps({
            "status": "READY", "valid": True,
            "controller_commands_issued": 0,
            "run_id": new_run_id,
            "authority": str(NEW),
            "bound_absent_run_root": str(RUN_ROOT),
            "manifest_sha256": manifest_sha,
            "commands_sha256": commands_sha,
            "load_commands": "PASS",
            "prepare_equivalent_dry_validation": "PASS",
            "r12_lexists": False,
            "prepare_argv": prepare_argv,
            "files": {path.name: sha(path)
                      for path in sorted(NEW.iterdir()) if path.is_file()},
        }, sort_keys=True, indent=2))
    finally:
        os.umask(previous_umask)


if __name__ == "__main__":
    main()
