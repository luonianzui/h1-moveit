#!/usr/bin/python3
"""Build the non-overwriting Candidate-3 831 round-trip authority v3.

The default action is the one-shot build.  ``--self-test`` performs the full
read-only input and in-memory document preparation without creating either
the authority directory or the run root.  This builder never starts ROS,
Gazebo, a controller, or a live certificate/supervisor.
"""

from __future__ import annotations

import argparse
import ctypes
from dataclasses import dataclass
import errno
import hashlib
import hmac
import json
import os
from pathlib import Path
import shutil
import stat
from typing import Any, Callable, Mapping, Sequence


WS = Path(
    "/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/"
    "h1_2_ros2_ws")
RUNTIME_ASSETS = WS / "runtime_assets"
BASE = RUNTIME_ASSETS / "candidate3_live_831_fast_authority_v14"
NEW = RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v3"
RUN_ROOT = RUNTIME_ASSETS / "candidate3_roundtrip_831_r5"
SCRIPTS = WS / "src/h1_2_moveit_demo/scripts"
MIDDLE_ROOT = RUNTIME_ASSETS / "candidate3_middle_e0_828"
RETURN_SOURCE_ROOT = RUNTIME_ASSETS / "candidate3_return_c2_831"
VISION = WS / "src/h1_2_vision/h1_2_vision"
INSTALLED_FAST_PACKAGE = (
    WS / "install/h1_candidate3_fast_831/share/h1_candidate3_fast_831")

BASE_MANIFEST_SHA256 = (
    "0344f0725198ed7773ac45feb45dbf29e75fc4e26514b97ef9a6de28615546cd")
BASE_COMMANDS_SHA256 = (
    "80b37527f2c47ee9d5463cfd6afe4142b84af9f5aaea51f85cc550517c5e8b88")
RETURN_PLAN_SHA256 = (
    "24767afddc9101094a419d9ab00c04db352e344b1aae90b287d86ca4773ebb21")
RETURN_PLAN_SCHEMA = "h1_candidate3_return_plan/831-c2-v1"
RETURN_AUTH_SCHEMA = (
    "h1_candidate3_return_execution_authorization/831-v1")
AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v3"
COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v3"
PROCESS_ROLES = ("launch", "recorder", "support_daemon", "flow")
MAX_INPUT_BYTES = 64 * 1024 * 1024
AT_FDCWD = -100
RENAME_NOREPLACE = 1

# These are copied byte-for-byte from the immutable v14 authority.  The v14
# manifest is itself pinned above and every binding is checked before use.
IMMUTABLE_MODES: Mapping[str, int] = {
    "candidate3_rosbag_qos_overrides_829.yaml": 0o400,
    "candidate3_telemetry_bridge_829.yaml": 0o400,
    "controller_execution_authorization.json": 0o400,
    "gazebo.launch.py": 0o400,
    "h1_2.gazebo.ros2_control.xacro": 0o400,
    "h1_right_handle_executor_828.py": 0o500,
    "h1_robot_joint_state_relay_829.py": 0o400,
    "h1_world.sdf": 0o400,
    "incubator_model.sdf": 0o400,
    "right_handle_manifest.json": 0o400,
    "ros2_controllers.yaml": 0o400,
}

# Later non-overwriting derived builders may opt into a deterministic byte
# transform after the pinned base input has been fully validated.  The default
# remains empty, so v3 preparation is byte-for-byte unchanged.
IMMUTABLE_BYTE_TRANSFORMS: Mapping[str, Callable[[bytes], bytes]] = {}

# Optional derived-authority semantics which do not live in an asset file
# (for example a process dependency DAG) must participate in the run id.
# Keeping the default empty preserves the original v3 seed byte-for-byte.
RUN_ID_EXTRA_BINDING: Mapping[str, Any] = {}

# These four files are deliberately unpinned mutable inputs: invoking this
# builder snapshots their then-current, securely-read bytes.  The resulting
# hashes are embedded in both commands and manifest, and all destinations are
# owner read/execute only.
SNAPSHOT_SOURCES: Mapping[str, Path] = {
    "h1_candidate3_flow_fast_831_v3.py":
        SCRIPTS / "h1_candidate3_flow_fast_831_v3.py",
    "h1_candidate3_middle_executor_fast_831_v3.py":
        SCRIPTS / "h1_candidate3_middle_executor_fast_831_v3.py",
    "h1_support_orchestrator_828.py":
        SCRIPTS / "h1_support_orchestrator_828.py",
    "h1_fast_support_daemon_831.py":
        SCRIPTS / "h1_fast_support_daemon_831.py",
}

# The old middle/support evidence remains external and immutable.  Hashes are
# recomputed during every preparation, after comparison with these known-good
# identities, and the computed values (not these literals) populate argv.
EXTERNAL_INPUTS: Mapping[str, tuple[Path, str, int]] = {
    "support_manifest": (
        MIDDLE_ROOT / "candidate3_middle_support_manifest_828.json",
        "a93ec01822bf6edb86531d281ba43e655c4a0e9f694c2058faa372a0e5bde1eb",
        0o400),
    "middle_certificate": (
        MIDDLE_ROOT / "candidate3_middle_certificate_828.json",
        "58a683347c95be35c7c9c39e814c7be23dfab7311a037a0a0e6d7bb5f62aaade",
        0o400),
    "scene": (
        MIDDLE_ROOT / "scene.cdr",
        "dacf67c055f5ba1b8d46c30b6be4aafdd80a6d453faee069b2d22e5a0e791568",
        0o400),
    "middle_plan": (
        MIDDLE_ROOT / "candidate3_middle_plan_828.json",
        "cb20f8303cb2e75a5fe45025ea4a119fa5ba81054c8cda55442d320052d1634e",
        0o400),
    "middle_exact_result": (
        MIDDLE_ROOT / "candidate3_middle_exact_result_828.json",
        "3e49dd9a34b206da4699545cff5656fb0db3fb38796ffd9a9435ebf55048e1fb",
        0o400),
    "middle_admission": (
        VISION / "candidate3_middle_admission.py",
        "3292d0cb1da48cd4083770cd930d5f6cc8aa5e7686d8f9ee60918300a18de352",
        0o500),
    "middle_contract": (
        VISION / "candidate3_middle_execution_contract_828.py",
        "681df3fce9b12c23388fed7cd4a62cccaba87353ef44a1e82234ec8d1864392a",
        0o500),
}


@dataclass(frozen=True)
class InputBlob:
    path: Path
    raw: bytes
    mode: int
    sha256: str


@dataclass(frozen=True)
class OutputBlob:
    raw: bytes
    mode: int
    provenance: str


@dataclass(frozen=True)
class PreparedAuthority:
    run_id: str
    socket_path: Path
    output_files: Mapping[str, OutputBlob]
    commands: Mapping[str, Any]
    return_authorization: Mapping[str, Any]
    manifest: Mapping[str, Any]


def sha(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _identity(metadata: os.stat_result) -> tuple[int, ...]:
    return (
        metadata.st_dev, metadata.st_ino, metadata.st_mode,
        metadata.st_uid, metadata.st_gid, metadata.st_nlink,
        metadata.st_size, metadata.st_mtime_ns, metadata.st_ctime_ns,
    )


def _valid_sha256(value: Any) -> bool:
    return (
        type(value) is str and len(value) == 64 and
        all(character in "0123456789abcdef" for character in value))


def read_regular(
    path: Path,
    expected_sha: str | None = None,
    *,
    allowed_modes: Sequence[int] | None = None,
    require_owner: bool = True,
) -> InputBlob:
    """Read one canonical, single-link file through a race-checked fd."""
    if not path.is_absolute():
        raise RuntimeError(f"INPUT_NOT_ABSOLUTE:{path}")
    if expected_sha is not None and not _valid_sha256(expected_sha):
        raise RuntimeError(f"EXPECTED_SHA256:{path}")
    try:
        before_path = path.lstat()
    except OSError as error:
        raise RuntimeError(f"INPUT_LSTAT:{path}:{error}") from error
    if (
        path.is_symlink() or not stat.S_ISREG(before_path.st_mode) or
        before_path.st_nlink != 1 or before_path.st_size <= 0 or
        before_path.st_size > MAX_INPUT_BYTES or
        path.resolve(strict=True) != path
    ):
        raise RuntimeError(f"INPUT_IDENTITY:{path}")
    if require_owner and before_path.st_uid != os.geteuid():
        raise RuntimeError(f"INPUT_OWNER:{path}")
    mode = stat.S_IMODE(before_path.st_mode)
    if mode & 0o022:
        raise RuntimeError(f"INPUT_WRITABLE_BY_OTHERS:{path}")
    if allowed_modes is not None and mode not in allowed_modes:
        raise RuntimeError(f"INPUT_MODE:{path}:{mode:04o}")

    flags = (
        os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) |
        getattr(os, "O_NOFOLLOW", 0))
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise RuntimeError(f"INPUT_OPEN:{path}:{error}") from error
    try:
        before_fd = os.fstat(descriptor)
        if _identity(before_fd) != _identity(before_path):
            raise RuntimeError(f"INPUT_RACE_BEFORE:{path}")
        remaining = before_fd.st_size
        chunks: list[bytes] = []
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            if not block:
                raise RuntimeError(f"INPUT_SHORT_READ:{path}")
            chunks.append(block)
            remaining -= len(block)
        if os.read(descriptor, 1):
            raise RuntimeError(f"INPUT_GREW:{path}")
        after_fd = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    try:
        after_path = path.lstat()
    except OSError as error:
        raise RuntimeError(f"INPUT_RELSTAT:{path}:{error}") from error
    if (
        _identity(before_fd) != _identity(after_fd) or
        _identity(before_fd) != _identity(after_path)
    ):
        raise RuntimeError(f"INPUT_CHANGED:{path}")
    raw = b"".join(chunks)
    digest = sha(raw)
    if len(raw) != before_fd.st_size:
        raise RuntimeError(f"INPUT_SIZE:{path}")
    if expected_sha is not None and not hmac.compare_digest(
            digest, expected_sha):
        raise RuntimeError(f"INPUT_SHA256:{path}")
    return InputBlob(path=path, raw=raw, mode=mode, sha256=digest)


def parse_json(raw: bytes, label: str) -> dict[str, Any]:
    def unique(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
        value: dict[str, Any] = {}
        for key, item in pairs:
            if key in value:
                raise RuntimeError(f"DUPLICATE_KEY:{label}:{key}")
            value[key] = item
        return value

    try:
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=unique,
            parse_constant=lambda token: (_ for _ in ()).throw(
                RuntimeError(f"NONFINITE:{label}:{token}")))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"JSON_PARSE:{label}:{error}") from error
    if type(value) is not dict:
        raise RuntimeError(f"JSON_ROOT:{label}")
    return value


def canonical(value: Any) -> bytes:
    return (
        json.dumps(
            value, sort_keys=True, separators=(",", ":"),
            ensure_ascii=True, allow_nan=False).encode("utf-8") + b"\n")


def pretty(value: Any) -> bytes:
    return (
        json.dumps(
            value, sort_keys=True, indent=2,
            ensure_ascii=True, allow_nan=False).encode("utf-8") + b"\n")


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(WS))
    except ValueError as error:
        raise RuntimeError(f"PATH_OUTSIDE_WORKSPACE:{path}") from error


def _output_binding(name: str, blob: OutputBlob) -> dict[str, Any]:
    return {
        "path": _relative(NEW / name),
        "sha256": sha(blob.raw),
        "mode": f"{blob.mode:04o}",
        "nlink": 1,
        "size_bytes": len(blob.raw),
    }


def _external_binding(blob: InputBlob) -> dict[str, Any]:
    return {
        "path": _relative(blob.path),
        "sha256": blob.sha256,
        "mode": f"{blob.mode:04o}",
        "nlink": 1,
        "size_bytes": len(blob.raw),
    }


def _flag(argv: Sequence[str], name: str) -> str:
    indices = [index for index, value in enumerate(argv) if value == name]
    if len(indices) != 1 or indices[0] + 1 >= len(argv):
        raise RuntimeError(f"COMMAND_FLAG:{name}")
    return argv[indices[0] + 1]


def _validate_base_binding(
    manifest: Mapping[str, Any], name: str, mode: int,
) -> InputBlob:
    files = manifest.get("files")
    if type(files) is not dict or type(files.get(name)) is not dict:
        raise RuntimeError(f"BASE_BINDING_MISSING:{name}")
    row = files[name]
    expected_path = _relative(BASE / name)
    if (
        row.get("path") != expected_path or
        not _valid_sha256(row.get("sha256")) or
        row.get("mode") != f"{mode:04o}" or
        row.get("nlink") != 1 or
        type(row.get("size_bytes")) is not int or
        row["size_bytes"] <= 0
    ):
        raise RuntimeError(f"BASE_BINDING_INVALID:{name}")
    blob = read_regular(
        BASE / name, row["sha256"], allowed_modes=(mode,))
    if len(blob.raw) != row["size_bytes"]:
        raise RuntimeError(f"BASE_BINDING_SIZE:{name}")
    return blob


def _validate_return_plan(value: Mapping[str, Any]) -> None:
    authorization = value.get("authorization")
    path_contract = value.get("path_contract")
    messages = value.get("message_contract")
    terminal = value.get("terminal_contract")
    if (
        value.get("schema") != RETURN_PLAN_SCHEMA or
        value.get("cycle_id") != "C2" or
        value.get("operation") != "return" or
        type(authorization) is not dict or
        authorization.get("execution_authorized") is not False or
        authorization.get("status") != "PLAN_ONLY" or
        type(path_contract) is not dict or
        path_contract.get("inverse_transform") !=
            "reverse_points_mirror_time" or
        path_contract.get("all_segments_required") is not True or
        path_contract.get("contains_partial") is not False or
        path_contract.get("partial_execution_allowed") is not False or
        type(messages) is not dict or
        messages.get("return_middle") != 16 or
        messages.get("return_cycle_total") != 30 or
        messages.get("two_cycle_total") != 60 or
        type(terminal) is not dict or
        terminal.get("cube_pose") != "origin"
    ):
        raise RuntimeError("RETURN_PLAN_CONTRACT")


def _make_return_authorization(
    run_id: str,
    return_plan: InputBlob,
    external: Mapping[str, InputBlob],
) -> dict[str, Any]:
    return {
        "schema": RETURN_AUTH_SCHEMA,
        "run_id": run_id,
        "authorization_scope": "candidate3_roundtrip_831_cycle2_return",
        "bindings": {
            "return_plan": {
                "path": str(NEW / "candidate3_return_plan_831.json"),
                "sha256": return_plan.sha256,
                "schema": RETURN_PLAN_SCHEMA,
            },
            "middle_plan": {
                "path": str(external["middle_plan"].path),
                "sha256": external["middle_plan"].sha256,
                "schema": "h1_candidate3_middle_plan/828-e0-v1",
            },
            "middle_certificate": {
                "path": str(external["middle_certificate"].path),
                "sha256": external["middle_certificate"].sha256,
                "schema": "h1_candidate3_middle_certificate/828-e0-v1",
            },
            "middle_exact_result": {
                "path": str(external["middle_exact_result"].path),
                "sha256": external["middle_exact_result"].sha256,
                "schema": "h1_candidate3_middle_exact_result/828-e0-v1",
            },
        },
        "execution_contract": {
            "exact_inverse": True,
            "inverse_transform": "reverse_points_mirror_time",
            "all_segments_required": True,
            "partial_execution_allowed": False,
            "requires_live_final_validation": True,
            "controller_message_count": 60,
            "left_hand_handle_contact_forbidden": True,
            "hard_maximum_abs_torso_deg": 44.5,
            "terminal_cube_pose": "origin",
            "terminal_door_state": "locked",
        },
        "controller_execution_authorized": True,
        "live_execution_passed": False,
        "controller_commands_issued": 0,
        "valid": True,
        "status": "READY",
    }


def _make_commands(
    run_id: str,
    socket_path: Path,
    immutable: Mapping[str, InputBlob],
    snapshots: Mapping[str, InputBlob],
    external: Mapping[str, InputBlob],
    return_plan: InputBlob,
    return_authorization_sha256: str,
) -> dict[str, Any]:
    launch = [
        "/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "launch", "-n",
        str(NEW / "gazebo.launch.py"),
        "spawn_incubator:=true",
        "table_x:=0.600",
        "cube_x:=0.334",
        "incubator_x:=0.590",
        "incubator_y:=0.00",
        "incubator_z:=0.815",
        "bridge_support_states:=false",
        "robot_joint_state_relay_source:=" + str(
            NEW / "h1_robot_joint_state_relay_829.py"),
        "robot_joint_state_relay_sha256:=" +
            immutable["h1_robot_joint_state_relay_829.py"].sha256,
        "telemetry_bridge_config:=" + str(
            NEW / "candidate3_telemetry_bridge_829.yaml"),
        "telemetry_bridge_config_sha256:=" +
            immutable["candidate3_telemetry_bridge_829.yaml"].sha256,
    ]
    recorder = [
        "/usr/bin/python3", "/opt/ros/jazzy/bin/ros2", "bag", "record",
        "--include-unpublished-topics", "-s", "sqlite3",
        "-o", str(RUN_ROOT / "rosbag_831"),
        "--qos-profile-overrides-path",
        str(NEW / "candidate3_rosbag_qos_overrides_829.yaml"),
        "--max-cache-size", "1073741824", "--topics",
        "/clock", "/joint_states", "/incubator/joint_states",
        "/torso_controller/joint_trajectory",
        "/right_arm_controller/joint_trajectory",
        "/left_arm_controller/joint_trajectory",
        "/left_gripper_controller/joint_trajectory",
        "/right_gripper_controller/joint_trajectory",
        "/h1_candidate3/live_829/markers",
        "/incubator/handle/contacts",
    ]
    support_daemon = [
        "/usr/bin/python3",
        str(NEW / "h1_fast_support_daemon_831.py"),
        "serve", "--roundtrip",
        "--socket", str(socket_path),
        "--owner-pid", "0",
        "--owner-start-ticks", "0",
        "--support-orchestrator",
        str(NEW / "h1_support_orchestrator_828.py"),
        "--support-orchestrator-sha256",
        snapshots["h1_support_orchestrator_828.py"].sha256,
        "--manifest", str(external["support_manifest"].path),
        "--manifest-sha256", external["support_manifest"].sha256,
        "--certificate", str(external["middle_certificate"].path),
        "--certificate-sha256", external["middle_certificate"].sha256,
        "--workspace", str(WS),
        "--timeout", "30.0",
        "--startup-timeout", "120.0",
        "--request-timeout", "5.0",
        "--telemetry-max-age", "1.0",
    ]
    flow = [
        "/usr/bin/python3",
        str(NEW / "h1_candidate3_flow_fast_831_v3.py"),
        "roundtrip",
        "--workspace", str(WS),
        "--right-executor", str(NEW / "h1_right_handle_executor_828.py"),
        "--right-executor-sha256",
        immutable["h1_right_handle_executor_828.py"].sha256,
        "--right-manifest",
        str(NEW / "controller_execution_authorization.json"),
        "--right-manifest-sha256",
        immutable["controller_execution_authorization.json"].sha256,
        "--right-time-scale", "1.0",
        "--discovery-timeout", "10.0",
        "--wall-timeout-factor", "20.0",
        "--support-orchestrator",
        str(NEW / "h1_support_orchestrator_828.py"),
        "--support-orchestrator-sha256",
        snapshots["h1_support_orchestrator_828.py"].sha256,
        "--support-manifest", str(external["support_manifest"].path),
        "--support-manifest-sha256", external["support_manifest"].sha256,
        "--support-certificate", str(external["middle_certificate"].path),
        "--support-certificate-sha256",
        external["middle_certificate"].sha256,
        "--support-timeout", "30.0",
        "--scene", str(external["scene"].path),
        "--scene-sha256", external["scene"].sha256,
        "--plan", str(external["middle_plan"].path),
        "--plan-sha256", external["middle_plan"].sha256,
        "--certificate", str(external["middle_certificate"].path),
        "--certificate-sha256", external["middle_certificate"].sha256,
        "--result", str(external["middle_exact_result"].path),
        "--result-sha256", external["middle_exact_result"].sha256,
        "--middle-executor",
        str(NEW / "h1_candidate3_middle_executor_fast_831_v3.py"),
        "--middle-executor-sha256",
        snapshots["h1_candidate3_middle_executor_fast_831_v3.py"].sha256,
        "--middle-admission", str(external["middle_admission"].path),
        "--middle-admission-sha256", external["middle_admission"].sha256,
        "--middle-contract", str(external["middle_contract"].path),
        "--middle-contract-sha256", external["middle_contract"].sha256,
        "--middle-speed-scale", "1.0",
        "--fast-support-daemon",
        str(NEW / "h1_fast_support_daemon_831.py"),
        "--fast-support-daemon-sha256",
        snapshots["h1_fast_support_daemon_831.py"].sha256,
        "--fast-support-daemon-socket", str(socket_path),
        "--return-plan", str(NEW / "candidate3_return_plan_831.json"),
        "--return-plan-sha256", return_plan.sha256,
        "--return-authorization",
        str(NEW / "return_execution_authorization.json"),
        "--return-authorization-sha256", return_authorization_sha256,
    ]
    processes = [
        {"role": "launch", "argv": launch},
        {"role": "recorder", "argv": recorder},
        {"role": "support_daemon", "argv": support_daemon},
        {"role": "flow", "argv": flow},
    ]
    return {
        "schema": COMMANDS_SCHEMA,
        "run_id": run_id,
        "authority_root": str(NEW),
        "run_root": str(RUN_ROOT),
        "process_order": list(PROCESS_ROLES),
        "processes": processes,
        "roundtrip": True,
        "live_supervisor_required": False,
        "live_certificate_required": False,
        "controller_commands_issued": 0,
        "valid": True,
        "status": "READY",
    }


def prepare_authority() -> PreparedAuthority:
    """Read inputs and prepare every output byte without writing anything."""
    if (
        not WS.is_absolute() or not NEW.is_absolute() or
        not RUN_ROOT.is_absolute() or NEW.parent != RUNTIME_ASSETS or
        RUN_ROOT.parent != RUNTIME_ASSETS or NEW == RUN_ROOT
    ):
        raise RuntimeError("FIXED_PATH_CONTRACT")

    base_blob = read_regular(
        BASE / "manifest.json", BASE_MANIFEST_SHA256,
        allowed_modes=(0o400,))
    base_manifest = parse_json(base_blob.raw, "BASE_MANIFEST")
    if (
        base_manifest.get("schema") !=
            "h1_candidate3_live_fast_authority/831-v14" or
        base_manifest.get("sealed") is not True or
        base_manifest.get("valid") is not True or
        base_manifest.get("status") != "READY"
    ):
        raise RuntimeError("BASE_MANIFEST_CONTRACT")
    base_commands_blob = _validate_base_binding(
        base_manifest, "commands_829.json", 0o400)
    if not hmac.compare_digest(
            base_commands_blob.sha256, BASE_COMMANDS_SHA256):
        raise RuntimeError("BASE_COMMANDS_SHA256")
    base_commands = parse_json(base_commands_blob.raw, "BASE_COMMANDS")
    if (
        base_commands.get("schema") !=
            "h1_candidate3_live_commands/829-v1" or
        base_commands.get("valid") is not True or
        base_commands.get("status") != "READY"
    ):
        raise RuntimeError("BASE_COMMANDS_CONTRACT")
    immutable = {
        name: _validate_base_binding(base_manifest, name, mode)
        for name, mode in IMMUTABLE_MODES.items()
    }
    for name, transform in IMMUTABLE_BYTE_TRANSFORMS.items():
        if name not in immutable or not callable(transform):
            raise RuntimeError(f"IMMUTABLE_TRANSFORM_CONTRACT:{name}")
        original = immutable[name]
        transformed = transform(original.raw)
        if type(transformed) is not bytes or transformed == original.raw:
            raise RuntimeError(f"IMMUTABLE_TRANSFORM_OUTPUT:{name}")
        immutable[name] = InputBlob(
            path=original.path, raw=transformed, mode=original.mode,
            sha256=sha(transformed))

    # A direct absolute launch-file command is used, not a live supervisor.
    # Still require the currently installed world/launch copies to match the
    # frozen v14 inputs because the launch description resolves its world via
    # the installed package share.
    installed_launch = read_regular(
        INSTALLED_FAST_PACKAGE / "launch/gazebo.launch.py",
        immutable["gazebo.launch.py"].sha256, allowed_modes=(0o644, 0o444))
    installed_world = read_regular(
        INSTALLED_FAST_PACKAGE / "worlds/h1_world.sdf",
        immutable["h1_world.sdf"].sha256, allowed_modes=(0o644, 0o444))
    if (
        installed_launch.raw != immutable["gazebo.launch.py"].raw or
        installed_world.raw != immutable["h1_world.sdf"].raw
    ):
        raise RuntimeError("INSTALLED_FAST_PACKAGE_MISMATCH")

    snapshots = {
        name: read_regular(path, allowed_modes=(0o500, 0o700))
        for name, path in SNAPSHOT_SOURCES.items()
    }
    external = {
        name: read_regular(path, expected, allowed_modes=(mode,))
        for name, (path, expected, mode) in EXTERNAL_INPUTS.items()
    }
    expected_external_schemas = {
        "support_manifest": "h1_support_orchestration_828/v1",
        "middle_certificate": "h1_candidate3_middle_certificate/828-e0-v1",
        "middle_plan": "h1_candidate3_middle_plan/828-e0-v1",
        "middle_exact_result": "h1_candidate3_middle_exact_result/828-e0-v1",
    }
    for name, expected_schema in expected_external_schemas.items():
        value = parse_json(external[name].raw, name.upper())
        if value.get("schema") != expected_schema:
            raise RuntimeError(f"EXTERNAL_SCHEMA:{name}")
    base_processes = base_commands.get("processes")
    if type(base_processes) is not list:
        raise RuntimeError("BASE_COMMANDS_PROCESSES")
    base_by_role = {
        row.get("role"): row.get("argv")
        for row in base_processes if type(row) is dict
    }
    middle_argv = base_by_role.get("middle_suffix")
    if type(middle_argv) is not list:
        raise RuntimeError("BASE_MIDDLE_COMMAND")
    base_external_flags = {
        "support_manifest": ("--support-manifest", "--support-manifest-sha256"),
        "middle_certificate": ("--certificate", "--certificate-sha256"),
        "scene": ("--scene", "--scene-sha256"),
        "middle_plan": ("--plan", "--plan-sha256"),
        "middle_exact_result": ("--result", "--result-sha256"),
        "middle_admission": (
            "--middle-admission", "--middle-admission-sha256"),
        "middle_contract": ("--middle-contract", "--middle-contract-sha256"),
    }
    for name, (path_flag, sha_flag) in base_external_flags.items():
        blob = external[name]
        if (
            _flag(middle_argv, path_flag) != str(blob.path) or
            _flag(middle_argv, sha_flag) != blob.sha256
        ):
            raise RuntimeError(f"BASE_EXTERNAL_BINDING:{name}")
    return_plan = read_regular(
        RETURN_SOURCE_ROOT / "candidate3_return_plan_831.json",
        RETURN_PLAN_SHA256, allowed_modes=(0o400, 0o444))
    return_plan_value = parse_json(return_plan.raw, "RETURN_PLAN")
    _validate_return_plan(return_plan_value)

    external_identity = {
        name: {"path": _relative(blob.path), "sha256": blob.sha256}
        for name, blob in sorted(external.items())
    }
    seed_value = {
        "schema": AUTHORITY_SCHEMA,
        "base_manifest_sha256": base_blob.sha256,
        "immutable_sha256": {
            name: blob.sha256 for name, blob in sorted(immutable.items())},
        "snapshot_sha256": {
            name: blob.sha256 for name, blob in sorted(snapshots.items())},
        "return_plan_sha256": return_plan.sha256,
        "external": external_identity,
    }
    if RUN_ID_EXTRA_BINDING:
        seed_value["run_id_extra_binding"] = RUN_ID_EXTRA_BINDING
    seed = canonical(seed_value)
    run_id = sha(seed)
    socket_path = Path(
        f"/run/user/{os.geteuid()}/h1rt831_{run_id[:32]}.sock")
    if len(os.fsencode(socket_path)) > 100:
        raise RuntimeError("SUPPORT_SOCKET_PATH_TOO_LONG")

    return_authorization = _make_return_authorization(
        run_id, return_plan, external)
    return_authorization_raw = pretty(return_authorization)
    commands = _make_commands(
        run_id, socket_path, immutable, snapshots, external,
        return_plan, sha(return_authorization_raw))
    commands_raw = pretty(commands)

    output_files: dict[str, OutputBlob] = {
        name: OutputBlob(
            blob.raw, IMMUTABLE_MODES[name],
            (f"generated:transform:{_relative(BASE / name)}"
             if name in IMMUTABLE_BYTE_TRANSFORMS
             else _relative(BASE / name)))
        for name, blob in immutable.items()
    }
    output_files.update({
        name: OutputBlob(blob.raw, 0o500, _relative(blob.path))
        for name, blob in snapshots.items()
    })
    output_files["candidate3_return_plan_831.json"] = OutputBlob(
        return_plan.raw, 0o400, _relative(return_plan.path))
    output_files["return_execution_authorization.json"] = OutputBlob(
        return_authorization_raw, 0o400, "generated")
    output_files["commands_831.json"] = OutputBlob(
        commands_raw, 0o400, "generated")

    file_bindings = {
        name: _output_binding(name, blob)
        for name, blob in sorted(output_files.items())
    }
    manifest = {
        "schema": AUTHORITY_SCHEMA,
        "run_id": run_id,
        "authority_root": str(NEW),
        "run_root": str(RUN_ROOT),
        "base_authority": {
            "path": _relative(BASE),
            "manifest_sha256": base_blob.sha256,
            "schema": base_manifest["schema"],
        },
        "process_roles": list(PROCESS_ROLES),
        "commands": {
            "path": _relative(NEW / "commands_831.json"),
            "sha256": sha(commands_raw),
            "schema": COMMANDS_SCHEMA,
        },
        "return_execution_authorization": {
            "path": _relative(
                NEW / "return_execution_authorization.json"),
            "sha256": sha(return_authorization_raw),
            "schema": RETURN_AUTH_SCHEMA,
        },
        "external_middle_support_bindings": {
            name: _external_binding(blob)
            for name, blob in sorted(external.items())
        },
        "provenance": {
            name: blob.provenance
            for name, blob in sorted(output_files.items())
        },
        "execution_contract": {
            "operation": "roundtrip",
            "cycles": 2,
            "sequence": [
                "open", "insert_cube", "close_and_lock",
                "reopen", "return_cube_to_origin", "close_and_relock",
            ],
            "exact_inverse_return": True,
            "inverse_transform": "reverse_points_mirror_time",
            "requires_live_final_validation": True,
            "controller_message_count": 60,
            "left_hand_handle_contact_forbidden": True,
            "right_handle_only": True,
            "hard_maximum_abs_torso_deg": 44.5,
            "live_supervisor_required": False,
            "live_certificate_required": False,
        },
        "files": file_bindings,
        "controller_execution_authorized": True,
        "live_execution_passed": False,
        "controller_commands_issued": 0,
        "sealed": True,
        "valid": True,
        "status": "READY",
    }
    output_files["manifest.json"] = OutputBlob(
        pretty(manifest), 0o400, "generated")
    return PreparedAuthority(
        run_id=run_id,
        socket_path=socket_path,
        output_files=output_files,
        commands=commands,
        return_authorization=return_authorization,
        manifest=manifest,
    )


def write_exclusive(root: Path, name: str, raw: bytes, mode: int) -> None:
    if Path(name).name != name or not name:
        raise RuntimeError(f"OUTPUT_NAME:{name}")
    path = root / name
    flags = (
        os.O_WRONLY | os.O_CREAT | os.O_EXCL |
        getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0))
    descriptor = os.open(path, flags, mode)
    try:
        view = memoryview(raw)
        while view:
            count = os.write(descriptor, view)
            if count <= 0:
                raise RuntimeError(f"OUTPUT_SHORT_WRITE:{path}")
            view = view[count:]
        os.fchmod(descriptor, mode)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    written = read_regular(path, allowed_modes=(mode,))
    if written.raw != raw:
        raise RuntimeError(f"OUTPUT_READBACK:{path}")


def _rename_noreplace(source: Path, destination: Path) -> None:
    """Atomically publish a directory with Linux RENAME_NOREPLACE."""
    library = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(library, "renameat2", None)
    if renameat2 is None:
        raise RuntimeError("RENAME_NOREPLACE_UNAVAILABLE")
    renameat2.argtypes = (
        ctypes.c_int, ctypes.c_char_p,
        ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    renameat2.restype = ctypes.c_int
    result = renameat2(
        AT_FDCWD, os.fsencode(source),
        AT_FDCWD, os.fsencode(destination), RENAME_NOREPLACE)
    if result != 0:
        number = ctypes.get_errno()
        if number == errno.EEXIST:
            raise FileExistsError(number, os.strerror(number), destination)
        raise OSError(number, os.strerror(number), destination)


def _fsync_directory(path: Path) -> None:
    flags = (
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
        getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0))
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _safe_cleanup_temporary(
    path: Path, expected_identity: tuple[int, int] | None = None,
) -> None:
    expected_prefix = f".{NEW.name}.building-"
    if (
        path.parent != NEW.parent or
        not path.name.startswith(expected_prefix) or
        path == NEW or path == RUN_ROOT
    ):
        raise RuntimeError(f"TEMP_CLEANUP_SCOPE:{path}")
    try:
        metadata = path.lstat()
    except FileNotFoundError:
        return
    if path.is_symlink() or not stat.S_ISDIR(metadata.st_mode):
        raise RuntimeError(f"TEMP_CLEANUP_IDENTITY:{path}")
    if (
        expected_identity is not None and
        (metadata.st_dev, metadata.st_ino) != expected_identity
    ):
        raise RuntimeError(f"TEMP_CLEANUP_REPLACED:{path}")
    # Publication failures can happen after the directory was sealed 0555.
    # Restore owner-only traversal/write solely on the verified temp inode so
    # its partial files can be unlinked; NEW/RUN/base are never candidates.
    os.chmod(path, 0o700, follow_symlinks=False)
    shutil.rmtree(path)


def build() -> PreparedAuthority:
    if (
        NEW.exists() or NEW.is_symlink() or
        RUN_ROOT.exists() or RUN_ROOT.is_symlink()
    ):
        raise FileExistsError("ROUNDTRIP_AUTHORITY_OR_RUN_ROOT_EXISTS")
    prepared = prepare_authority()
    temporary = NEW.parent / (
        f".{NEW.name}.building-{os.getpid()}-{prepared.run_id[:16]}")
    if temporary.exists() or temporary.is_symlink():
        raise FileExistsError("ROUNDTRIP_AUTHORITY_TEMP_EXISTS")
    os.mkdir(temporary, 0o700)
    temporary_metadata = temporary.lstat()
    temporary_identity = (
        temporary_metadata.st_dev, temporary_metadata.st_ino)
    try:
        for name, blob in sorted(prepared.output_files.items()):
            write_exclusive(temporary, name, blob.raw, blob.mode)
        os.chmod(temporary, 0o555)
        _fsync_directory(temporary)
        if RUN_ROOT.exists() or RUN_ROOT.is_symlink():
            raise FileExistsError("ROUNDTRIP_RUN_ROOT_APPEARED")
        _rename_noreplace(temporary, NEW)
        _fsync_directory(NEW.parent)
    except BaseException:
        _safe_cleanup_temporary(temporary, temporary_identity)
        raise
    if (
        stat.S_IMODE(NEW.stat().st_mode) != 0o555 or
        RUN_ROOT.exists() or RUN_ROOT.is_symlink()
    ):
        raise RuntimeError("PUBLISHED_AUTHORITY_CONTRACT")
    return prepared


def self_test() -> Mapping[str, Any]:
    before_target = NEW.exists() or NEW.is_symlink()
    before_run_root = RUN_ROOT.exists() or RUN_ROOT.is_symlink()
    prepared = prepare_authority()
    after_target = NEW.exists() or NEW.is_symlink()
    after_run_root = RUN_ROOT.exists() or RUN_ROOT.is_symlink()
    if before_target != after_target or before_run_root != after_run_root:
        raise RuntimeError("SELF_TEST_WROTE_FIXED_OUTPUT")

    commands = prepared.commands
    processes = commands.get("processes")
    if (
        commands.get("schema") != COMMANDS_SCHEMA or
        commands.get("process_order") != list(PROCESS_ROLES) or
        type(processes) is not list or
        [row.get("role") for row in processes] != list(PROCESS_ROLES)
    ):
        raise RuntimeError("SELF_TEST_PROCESS_ROLES")
    by_role = {row["role"]: row["argv"] for row in processes}
    if "--roundtrip" not in by_role["support_daemon"]:
        raise RuntimeError("SELF_TEST_DAEMON_ROUNDTRIP")
    flow = by_role["flow"]
    if len(flow) < 3 or flow[2] != "roundtrip":
        raise RuntimeError("SELF_TEST_FLOW_ROUNDTRIP")
    expected_files = prepared.output_files
    for flag, name in (
        ("--middle-executor-sha256",
         "h1_candidate3_middle_executor_fast_831_v3.py"),
        ("--support-orchestrator-sha256", "h1_support_orchestrator_828.py"),
        ("--fast-support-daemon-sha256", "h1_fast_support_daemon_831.py"),
        ("--return-plan-sha256", "candidate3_return_plan_831.json"),
        ("--return-authorization-sha256",
         "return_execution_authorization.json"),
    ):
        if _flag(flow, flag) != sha(expected_files[name].raw):
            raise RuntimeError(f"SELF_TEST_FLOW_HASH:{flag}")
    if (
        _flag(flow, "--return-plan") !=
            str(NEW / "candidate3_return_plan_831.json") or
        _flag(flow, "--return-authorization") !=
            str(NEW / "return_execution_authorization.json")
    ):
        raise RuntimeError("SELF_TEST_RETURN_PATHS")

    authorization = prepared.return_authorization
    contract = authorization.get("execution_contract")
    if (
        authorization.get("schema") != RETURN_AUTH_SCHEMA or
        authorization.get("controller_execution_authorized") is not True or
        authorization.get("live_execution_passed") is not False or
        type(contract) is not dict or
        contract.get("exact_inverse") is not True or
        contract.get("requires_live_final_validation") is not True or
        contract.get("controller_message_count") != 60 or
        contract.get("left_hand_handle_contact_forbidden") is not True or
        contract.get("hard_maximum_abs_torso_deg") != 44.5
    ):
        raise RuntimeError("SELF_TEST_RETURN_AUTHORIZATION")
    for name in SNAPSHOT_SOURCES:
        if expected_files[name].mode != 0o500:
            raise RuntimeError(f"SELF_TEST_SNAPSHOT_MODE:{name}")
    if expected_files["candidate3_return_plan_831.json"].mode != 0o400:
        raise RuntimeError("SELF_TEST_RETURN_PLAN_MODE")
    forbidden = {
        "h1_candidate3_live_supervisor_829.py",
        "h1_candidate3_live_certificate_829.py",
    }
    if forbidden.intersection(expected_files):
        raise RuntimeError("SELF_TEST_LIVE_DEPENDENCY")
    flow_source = expected_files[
        "h1_candidate3_flow_fast_831_v3.py"].raw
    middle_source = expected_files[
        "h1_candidate3_middle_executor_fast_831_v3.py"].raw
    daemon_source = expected_files["h1_fast_support_daemon_831.py"].raw
    for token in (
        b"--return-plan", b"--return-plan-sha256",
        b"--return-authorization", b"--return-authorization-sha256",
    ):
        if token not in flow_source or token not in middle_source:
            raise RuntimeError(f"SELF_TEST_SOURCE_INTERFACE:{token!r}")
    if b"--roundtrip" not in daemon_source:
        raise RuntimeError("SELF_TEST_DAEMON_INTERFACE")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v3",
        "run_id": prepared.run_id,
        "authority": str(NEW),
        "run_root": str(RUN_ROOT),
        "output_file_count": len(prepared.output_files),
        "process_roles": list(PROCESS_ROLES),
        "return_plan_sha256": RETURN_PLAN_SHA256,
        "return_authorization_sha256": sha(
            prepared.output_files[
                "return_execution_authorization.json"].raw),
        "writes_performed": False,
        "valid": True,
        "status": "PASS",
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--self-test", action="store_true",
        help="perform read-only input and in-memory contract checks")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.self_test:
        print(canonical(self_test()).decode("ascii"), end="")
        return 0
    prepared = build()
    print(canonical({
        "schema": "h1_candidate3_roundtrip_builder_result/831-v1",
        "authority": str(NEW),
        "run_root": str(RUN_ROOT),
        "run_id": prepared.run_id,
        "commands_sha256": sha(
            prepared.output_files["commands_831.json"].raw),
        "manifest_sha256": sha(
            prepared.output_files["manifest.json"].raw),
        "controller_commands_issued": 0,
        "valid": True,
        "status": "PASS",
    }).decode("ascii"), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
