#!/usr/bin/python3
"""Create sealed Candidate 3 authority v14 for a fresh absent r13 root."""

from __future__ import annotations

from dataclasses import dataclass
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
BASE = WS / "runtime_assets/candidate3_live_829_authority_v13"
NEW = WS / "runtime_assets/candidate3_live_829_authority_v14"
OLD_RUN = WS / "runtime_assets/candidate3_live_829_final_r12"
RUN_ROOT = WS / "runtime_assets/candidate3_live_829_final_r13"
PRIOR_FORMAL = OLD_RUN / "formal_authorities_829.json"
OLD_FCL = (
    WS / "runtime_assets/right_handle_8263_natural/variable_torso_v4_829/"
    "live_fcl_authority_v4_31j")
NEW_FCL = (
    WS / "runtime_assets/right_handle_8263_natural/variable_torso_v4_829/"
    "live_fcl_authority_v5_31j")

BASE_MANIFEST_SHA256 = (
    "55f06aeeb9a3e8c0b7879060010fbd44e68db2a037ea7cc91145650b55bab866")
BASE_COMMANDS_SHA256 = (
    "748a38ef42583885a16ca88fe9d134741782a0ee8c58bb853457727bc1270c82")
NEW_FCL_MANIFEST_SHA256 = (
    "7cc8c38ff54964986583354452f26963ae483bc9418ec8dd06cc10f5a6ab4c4e")

LIVE_SOURCES = {
    "h1_candidate3_live_certificate_829.py":
        WS / "src/h1_2_moveit_demo/scripts/h1_candidate3_live_certificate_829.py",
    "h1_candidate3_live_supervisor_829.py":
        WS / "src/h1_2_moveit_demo/scripts/h1_candidate3_live_supervisor_829.py",
}
BASE_COPY_NAMES = (
    "candidate3_rosbag_qos_overrides_829.yaml",
    "candidate3_telemetry_bridge_829.yaml",
    "gazebo.launch.py",
    "h1_2.gazebo.ros2_control.xacro",
    "h1_robot_joint_state_relay_829.py",
    "h1_world.sdf",
    "incubator_model.sdf",
    "ros2_controllers.yaml",
)


@dataclass(frozen=True)
class Snapshot:
    path: Path
    raw: bytes
    sha256: str
    mode: int
    signature: tuple[int, ...]


def lexists(path: Path) -> bool:
    return os.path.lexists(os.fspath(path))


def sha_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def signature(metadata: os.stat_result) -> tuple[int, ...]:
    return (
        metadata.st_dev, metadata.st_ino, metadata.st_mode,
        metadata.st_nlink, metadata.st_uid, metadata.st_gid,
        metadata.st_size, metadata.st_mtime_ns, metadata.st_ctime_ns,
    )


def snapshot_regular(path: Path, expected_mode: int | None = None) -> Snapshot:
    if not path.is_absolute() or path.resolve(strict=True) != path:
        raise RuntimeError(f"SOURCE_ABSOLUTE_IDENTITY:{path}")
    before = path.lstat()
    mode = stat.S_IMODE(before.st_mode)
    if (path.is_symlink() or not stat.S_ISREG(before.st_mode) or
            before.st_nlink != 1 or
            (expected_mode is not None and mode != expected_mode)):
        raise RuntimeError(f"SOURCE_IDENTITY:{path}")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        opened = os.fstat(descriptor)
        if (not stat.S_ISREG(opened.st_mode) or opened.st_nlink != 1 or
                (opened.st_dev, opened.st_ino) !=
                (before.st_dev, before.st_ino)):
            raise RuntimeError(f"SOURCE_OPEN_IDENTITY:{path}")
        blocks = []
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            blocks.append(block)
        raw = b"".join(blocks)
        after_fd = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    after_path = path.lstat()
    if (signature(before) != signature(opened) or
            signature(opened) != signature(after_fd) or
            signature(after_fd) != signature(after_path) or
            len(raw) != after_fd.st_size):
        raise RuntimeError(f"SOURCE_TOCTOU:{path}")
    return Snapshot(path, raw, sha_bytes(raw), mode, signature(after_path))


def revalidate(source: Snapshot, expected_mode: int | None = None) -> None:
    current = snapshot_regular(source.path, expected_mode)
    if (current.signature != source.signature or
            current.sha256 != source.sha256 or current.raw != source.raw):
        raise RuntimeError(f"SOURCE_CHANGED:{source.path}")


def exclusive_write(path: Path, raw: bytes, mode: int) -> Snapshot:
    if path.parent != NEW or not path.is_absolute() or lexists(path):
        raise RuntimeError(f"OUTPUT_PRECONDITION:{path}")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags, mode)
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
    output = snapshot_regular(path, mode)
    if output.raw != raw or output.sha256 != sha_bytes(raw):
        raise RuntimeError(f"OUTPUT_READBACK:{path}")
    return output


def reject_constant(value: str) -> None:
    raise ValueError(f"NONFINITE_JSON:{value}")


def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"DUPLICATE_JSON_KEY:{key}")
        result[key] = value
    return result


def parse_json(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=unique_object,
            parse_constant=reject_constant)
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as error:
        raise RuntimeError(f"JSON:{label}:{error}") from error
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


def binding(path: Path) -> dict[str, Any]:
    item = snapshot_regular(path)
    if item.mode not in (0o400, 0o500):
        raise RuntimeError(f"BINDING_MODE:{path}")
    return {
        "path": str(path), "sha256": item.sha256,
        "size_bytes": len(item.raw), "mode": f"{item.mode:04o}",
    }


def flag(argv: Sequence[str], name: str) -> str:
    positions = [index for index, value in enumerate(argv) if value == name]
    if len(positions) != 1 or positions[0] + 1 >= len(argv):
        raise RuntimeError(f"FLAG:{name}")
    return argv[positions[0] + 1]


def import_supervisor(path: Path) -> Any:
    sys.dont_write_bytecode = True
    name = "candidate3_live_829_authority_v14_supervisor"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError("SUPERVISOR_IMPORT_SPEC")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def main() -> None:
    if WS.resolve(strict=True) != WS:
        raise RuntimeError("WORKSPACE_IDENTITY")
    if (BASE.is_symlink() or stat.S_IMODE(BASE.lstat().st_mode) != 0o555 or
            BASE.resolve(strict=True) != BASE):
        raise RuntimeError("BASE_AUTHORITY_IDENTITY")
    if lexists(NEW) or lexists(RUN_ROOT):
        raise FileExistsError("V14_OR_R13_EXISTS")

    base_manifest_snapshot = snapshot_regular(BASE / "manifest.json", 0o400)
    base_commands_snapshot = snapshot_regular(BASE / "commands_829.json", 0o400)
    if base_manifest_snapshot.sha256 != BASE_MANIFEST_SHA256:
        raise RuntimeError("BASE_MANIFEST_SHA256")
    if base_commands_snapshot.sha256 != BASE_COMMANDS_SHA256:
        raise RuntimeError("BASE_COMMANDS_SHA256")
    fcl_manifest = snapshot_regular(NEW_FCL / "manifest.json", 0o400)
    if fcl_manifest.sha256 != NEW_FCL_MANIFEST_SHA256:
        raise RuntimeError("NEW_FCL_MANIFEST_SHA256")

    base_manifest = parse_json(base_manifest_snapshot.raw, "BASE_MANIFEST")
    base_commands = parse_json(base_commands_snapshot.raw, "BASE_COMMANDS")
    if (base_manifest.get("schema") !=
            "h1_candidate3_live_production_authority/829-v13" or
            base_commands.get("run_id") != base_manifest.get("run_id") or
            base_manifest.get("run_root") != str(OLD_RUN)):
        raise RuntimeError("BASE_SEMANTICS")

    base_files: dict[str, Snapshot] = {}
    for name in BASE_COPY_NAMES:
        mode = stat.S_IMODE((BASE / name).lstat().st_mode)
        if mode not in (0o400, 0o500):
            raise RuntimeError(f"BASE_COPY_MODE:{name}")
        base_files[name] = snapshot_regular(BASE / name, mode)
    live_files = {
        name: snapshot_regular(path, 0o500)
        for name, path in LIVE_SOURCES.items()
    }
    supervisor_sha = live_files[
        "h1_candidate3_live_supervisor_829.py"].sha256
    certificate_sha = live_files[
        "h1_candidate3_live_certificate_829.py"].sha256
    old_supervisor_sha = base_manifest["files"]["supervisor_source"]["sha256"]
    old_certificate_sha = base_manifest["files"]["certificate_source"]["sha256"]

    old_run_id = base_commands["run_id"]
    new_run_id = secrets.token_hex(32)
    if len(new_run_id) != 64 or new_run_id == old_run_id:
        raise RuntimeError("RUN_ID")
    replacements = (
        (str(BASE), str(NEW)),
        (str(OLD_RUN), str(RUN_ROOT)),
        (str(OLD_FCL), str(NEW_FCL)),
        (old_run_id, new_run_id),
        (old_supervisor_sha, supervisor_sha),
        (old_certificate_sha, certificate_sha),
    )
    commands = replace_tree(base_commands, replacements)
    commands["run_id"] = new_run_id
    commands_raw = canonical(commands)
    for forbidden in (str(BASE), str(OLD_RUN), str(OLD_FCL), old_run_id,
                      old_supervisor_sha, old_certificate_sha):
        if forbidden.encode("utf-8") in commands_raw:
            raise RuntimeError(f"STALE_COMMAND_BINDING:{forbidden}")

    # Revalidate all mutable inputs immediately before claiming NEW.
    revalidate(base_manifest_snapshot, 0o400)
    revalidate(base_commands_snapshot, 0o400)
    revalidate(fcl_manifest, 0o400)
    for item in base_files.values():
        revalidate(item, item.mode)
    for item in live_files.values():
        revalidate(item, 0o500)

    previous_umask = os.umask(0o077)
    try:
        os.mkdir(NEW, 0o700)
        outputs: dict[str, Snapshot] = {}
        for name, source in base_files.items():
            outputs[name] = exclusive_write(NEW / name, source.raw, source.mode)
        for name, source in live_files.items():
            outputs[name] = exclusive_write(NEW / name, source.raw, 0o500)
        outputs["commands_829.json"] = exclusive_write(
            NEW / "commands_829.json", commands_raw, 0o400)

        manifest = replace_tree(base_manifest, replacements)
        manifest["schema"] = "h1_candidate3_live_production_authority/829-v14"
        manifest["run_id"] = new_run_id
        manifest["run_root"] = str(RUN_ROOT)
        manifest["supersedes"] = {
            "path": "runtime_assets/candidate3_live_829_authority_v13",
            "reason": (
                "v14 preserves the successful v13/r12 controller trajectory "
                "and 826.3 posture while binding the corrected scheduled-time "
                "certificate, right-pad geometric grip contract, immutable "
                "31-joint FCL authority v5, and a fresh absent r13 root"),
        }
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
        manifest_output = exclusive_write(
            NEW / "manifest.json", pretty(manifest), 0o400)
        fsync_directory(NEW)
        os.chmod(NEW, 0o555)
        fsync_directory(NEW)

        if lexists(RUN_ROOT):
            raise RuntimeError("R13_CREATED_DURING_AUTHORITY_BUILD")
        supervisor = import_supervisor(
            NEW / "h1_candidate3_live_supervisor_829.py")
        paths = supervisor.EvidencePaths.from_root(RUN_ROOT)
        bundle = supervisor.load_commands(NEW / "commands_829.json", paths)
        if bundle.run_id != new_run_id or bundle.raw_sha256 != sha_bytes(commands_raw):
            raise RuntimeError("LOAD_COMMANDS")
        if (flag(bundle.processes["prefix"], "--right-time-scale") != "2.0" or
                flag(bundle.processes["middle_suffix"],
                     "--right-time-scale") != "2.0" or
                flag(bundle.processes["middle_suffix"],
                     "--middle-speed-scale") != "0.5"):
            raise RuntimeError("EXECUTION_TIME_SCALES")

        prior = parse_json(
            snapshot_regular(PRIOR_FORMAL, 0o400).raw, "PRIOR_FORMAL")
        first_twelve = prior["authorities"][:12]
        if [row["id"] for row in first_twelve] != list(
                supervisor.FORMAL_AUTHORITY_IDS[:12]):
            raise RuntimeError("FORMAL_AUTHORITY_PREFIX")
        authority_paths = {
            row["id"]: Path(row["path"]) for row in first_twelve}
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
            raise RuntimeError("FORMAL_AUTHORITY_ORDER")
        authority_bindings = {
            identifier: binding(path)
            for identifier, path in authority_paths.items()}

        right = authority_bindings["right_execution_authorization"]
        for role in ("prefix", "middle_suffix"):
            argv = bundle.processes[role]
            if (flag(argv, "--right-manifest") != right["path"] or
                    flag(argv, "--right-manifest-sha256") != right["sha256"]):
                raise RuntimeError(f"RIGHT_AUTHORIZATION:{role}")
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
                raise RuntimeError(f"LAUNCH_BINDING:{identifier}")
        qos = authority_bindings["rosbag_qos_overrides"]
        if (flag(bundle.processes["recorder"],
                 "--qos-profile-overrides-path") != qos["path"] or
                qos["mode"] != "0400"):
            raise RuntimeError("RECORDER_QOS")
        if (Path(bundle.processes["video"][1]) !=
                NEW / "h1_candidate3_live_supervisor_829.py" or
                flag(bundle.processes["video"], "--self-sha256") !=
                supervisor_sha):
            raise RuntimeError("VIDEO_SELF_BINDING")

        authority_args = [
            f"{identifier}={authority_paths[identifier]}"
            for identifier in supervisor.FORMAL_AUTHORITY_IDS]
        tool_args = [
            "supervisor_source=" +
            str(NEW / "h1_candidate3_live_supervisor_829.py"),
            "certificate_source=" +
            str(NEW / "h1_candidate3_live_certificate_829.py"),
        ]
        if (tuple(supervisor._authority_arguments(authority_args)) !=
                supervisor.FORMAL_AUTHORITY_IDS or
                tuple(supervisor._evidence_tool_arguments(tool_args)) !=
                supervisor.EVIDENCE_TOOL_IDS):
            raise RuntimeError("PREPARE_ARGUMENTS")
        prepare_argv = [
            "/usr/bin/python3",
            str(NEW / "h1_candidate3_live_supervisor_829.py"),
            "prepare", "--run-root", str(RUN_ROOT),
            "--commands", str(NEW / "commands_829.json"),
        ]
        for value in authority_args:
            prepare_argv.extend(["--authority", value])
        for value in tool_args:
            prepare_argv.extend(["--evidence-tool", value])

        loaded_manifest = parse_json(
            snapshot_regular(NEW / "manifest.json", 0o400).raw,
            "V14_MANIFEST_READBACK")
        if loaded_manifest != manifest:
            raise RuntimeError("MANIFEST_READBACK")
        expected_names = set(BASE_COPY_NAMES) | set(LIVE_SOURCES) | {
            "commands_829.json", "manifest.json"}
        if {entry.name for entry in os.scandir(NEW)} != expected_names:
            raise RuntimeError("FINAL_FILE_SET")
        if stat.S_IMODE(NEW.lstat().st_mode) != 0o555:
            raise RuntimeError("FINAL_DIRECTORY_MODE")
        for identifier, entry in manifest["files"].items():
            path = WS / entry["path"]
            actual = binding(path)
            if (path.parent != NEW or actual["sha256"] != entry["sha256"] or
                    actual["mode"] != entry["mode"] or
                    path.lstat().st_nlink != 1):
                raise RuntimeError(f"FINAL_MANIFEST_FILE:{identifier}")
        if lexists(RUN_ROOT):
            raise RuntimeError("R13_FINAL_LEXISTS")

        revalidate(base_manifest_snapshot, 0o400)
        revalidate(base_commands_snapshot, 0o400)
        revalidate(fcl_manifest, 0o400)
        for item in base_files.values():
            revalidate(item, item.mode)
        for item in live_files.values():
            revalidate(item, 0o500)

        print(json.dumps({
            "status": "READY", "valid": True,
            "controller_commands_issued": 0,
            "run_id": new_run_id,
            "authority": str(NEW),
            "bound_absent_run_root": str(RUN_ROOT),
            "manifest_sha256": manifest_output.sha256,
            "commands_sha256": sha_bytes(commands_raw),
            "supervisor_sha256": supervisor_sha,
            "certificate_sha256": certificate_sha,
            "fcl_authority_manifest_sha256": fcl_manifest.sha256,
            "load_commands": "PASS",
            "prepare_equivalent_dry_validation": "PASS",
            "r13_lexists": False,
            "prepare_argv": prepare_argv,
        }, sort_keys=True, indent=2))
    finally:
        os.umask(previous_umask)


if __name__ == "__main__":
    main()
