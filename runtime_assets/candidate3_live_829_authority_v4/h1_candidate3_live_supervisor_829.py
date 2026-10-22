#!/usr/bin/python3
"""Fail-closed prepare and live-capture supervisor for Candidate 3 (829).

``prepare`` seals the immutable run plan and ordered formal authority
bindings.  ``run`` is read-only unless ``--execute`` is present; execution
then owns exactly six direct children, records every proof topic to a SQLite3
bag, publishes the certificate marker sequence, captures the Gazebo window,
and seals the resulting evidence without replacing any path.

ROS imports are lazy.  The deterministic preparation, process, marker,
SQLite, and audit helpers remain testable on a host without ROS installed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import hashlib
from importlib.machinery import ModuleSpec
import json
import math
import os
from pathlib import Path
import re
import selectors
import signal
import sqlite3
import stat
import subprocess
import sys
import tempfile
import time
from types import ModuleType
from typing import Any, Callable, Iterable, Mapping, Protocol, Sequence, Tuple


COMMAND_SCHEMA = "h1_candidate3_live_commands/829-v1"
MARKER_PAYLOAD_SCHEMA = "h1_candidate3_live_marker_payload/829-v1"
PLAN_SCHEMA = "h1_candidate3_live_run_plan/829-v1"
FORMAL_SCHEMA = "h1_candidate3_formal_authority_bindings/829-v1"
PROC_SCHEMA = "h1_candidate3_live_proc_snapshot/829-v1"
VIDEO_SCHEMA = "h1_candidate3_live_video_audit/829-v1"
VIDEO_WINDOW_SCHEMA = "h1_candidate3_video_window_capture/829-v1"
GUI_CAMERA_SCHEMA = "h1_candidate3_gui_camera_pose/829-v1"
POSTFLIGHT_SCHEMA = "h1_candidate3_live_postflight/829-v1"
EXACT_FCL_REPLAY_SCHEMA = "h1_candidate3_live_exact_fcl_replay/829-v1"

PROCESS_ROLES = (
    "launch", "recorder", "video", "prefix", "middle_suffix",
    "postflight",
)
FORMAL_AUTHORITY_IDS = (
    "right_execution_authorization", "right_manifest", "middle_plan",
    "middle_exact_result", "middle_certificate", "middle_support_manifest",
    "right_executor_source", "middle_executor_source", "flow_source",
    "support_source", "middle_admission", "middle_execution_contract",
    "gazebo_launch_source", "gazebo_world_sdf", "incubator_model_sdf",
)
EXACT_FCL_FILE_IDS = (
    "auditor_source", "auditor_path_base_source",
    "auditor_middle_base_source", "auditor_binary", "contact_path",
    "original_handle_stl", "scene_cdr", "urdf", "srdf",
)
EXACT_FCL_AUTHORITY_RELATIVE = Path(
    "runtime_assets/right_handle_8263_natural/variable_torso_v4_829/"
    "live_fcl_authority_v3_31j")
EXACT_FCL_AUTHORITY_MANIFEST_SHA256 = (
    "f7877f4742ab170b4eadd1aad5bdeeb33f553aca4266ce2804e0e0c9448d1b06")
EXACT_FCL_AUDITOR_BINARY_SHA256 = (
    "12483870b7c6eb3546aaeddecb235dbe374ade78cf14f05f221e5643f197fc83")
EXACT_FCL_MANIFEST_FILE_IDS = {
    "auditor_source": "auditor_source",
    "auditor_path_base_source": "auditor_path_base_source",
    "auditor_middle_base_source": "auditor_middle_base_source",
    "auditor_binary": "auditor_binary",
    "contact_path": "contact_path",
    "original_handle_stl": "original_handle_stl",
    "scene_cdr": "scene_cdr",
    "urdf": "urdf",
    "srdf": "srdf",
}
INPUT_IDS = (
    "rosbag", "markers", "proc_snapshot", "video_audit", "postflight",
    "workcell_pose_audit", "gui_camera_audit", "video_window_audit",
    "formal_authorities", "exact_fcl_replay",
)
EVIDENCE_TOOL_IDS = ("supervisor_source", "certificate_source")
WORKCELL_POSE_SCHEMA = "h1_candidate3_live_workcell_pose_audit/829-v1"
REQUIRED_BAG_TOPICS = (
    "/clock", "/joint_states", "/incubator/joint_states",
    "/torso_controller/joint_trajectory",
    "/right_arm_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
    "/right_gripper_controller/joint_trajectory",
)
CONTROLLER_TOPICS = REQUIRED_BAG_TOPICS[3:]
MARKER_TOPIC = "/h1_candidate3/live_829/markers"
MARKER_TOPIC_TYPE = "std_msgs/msg/String"
RECORDER_INCLUDE_UNPUBLISHED_FLAG = "--include-unpublished-topics"
RECORDER_TOPICS_FLAG = "--topics"
RECORDER_TOPICS = (*REQUIRED_BAG_TOPICS, MARKER_TOPIC)
MAX_JOINT_GAP_NS = 50_000_000
MAX_ABS_TORSO_RAD = math.radians(44.5)
MAX_CAPTURE_BYTES = 128 * 1024 * 1024
SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
RUN_ID_RE = SHA256_RE
EMPTY_SHA256 = hashlib.sha256(b"").hexdigest()

FINAL_ROBOT_POSITIONS = {
    "torso_joint": 0.0,
    "left_shoulder_pitch_joint": -0.000012933885986937658,
    "left_shoulder_roll_joint": 0.24998646984876535,
    "left_shoulder_yaw_joint": 0.000023774721497885907,
    "left_elbow_joint": 0.6000753185207717,
    "left_wrist_roll_joint": 0.000023572560034304773,
    "left_wrist_pitch_joint": -0.00008730492997967213,
    "left_wrist_yaw_joint": -0.00001654021997178485,
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": -0.25,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.60,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
    "left_dex1_finger_1_joint": -0.019,
    "left_dex1_finger_2_joint": -0.019,
    "right_dex1_finger_1_joint": -0.019,
    "right_dex1_finger_2_joint": -0.019,
}


class SupervisorError(RuntimeError):
    """A preparation, identity, ordering, capture, or sealing failure."""


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise SupervisorError(status)


def _canonical_bytes(value: Any) -> bytes:
    try:
        return (json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
            allow_nan=False).encode("utf-8") + b"\n")
    except (TypeError, ValueError) as error:
        raise SupervisorError(f"CANONICAL_JSON:{error}") from error


def _argv_sha256(argv: Sequence[str]) -> str:
    return hashlib.sha256(
        b"".join(item.encode("utf-8") + b"\0" for item in argv)).hexdigest()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _absolute(value: str | Path, status: str) -> Path:
    path = Path(value)
    _require(path.is_absolute() and os.path.normpath(str(path)) == str(path),
             f"{status}_NOT_CANONICAL_ABSOLUTE")
    return path


def _validate_argv(value: Any, status: str) -> Tuple[str, ...]:
    _require(type(value) is list and len(value) > 0 and all(
        type(item) is str and item and "\0" not in item for item in value),
        f"{status}_ARGV")
    argv = tuple(value)
    executable = _absolute(argv[0], f"{status}_EXECUTABLE")
    system_python = (executable == Path("/usr/bin/python3") and
                     executable.resolve(strict=True).parent == Path("/usr/bin"))
    _require(executable.is_file() and
             (not executable.is_symlink() or system_python) and
             os.access(executable, os.X_OK), f"{status}_EXECUTABLE_IDENTITY")
    return argv


def _open_regular(path: Path, mode: int | None, status: str) -> os.stat_result:
    _require(path.is_absolute(), f"{status}_PATH")
    try:
        metadata = path.lstat()
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise SupervisorError(f"{status}_LSTAT:{error}") from error
    _require(resolved == path and not path.is_symlink() and
             stat.S_ISREG(metadata.st_mode) and metadata.st_nlink == 1 and
             metadata.st_uid == os.geteuid(), f"{status}_IDENTITY")
    if mode is not None:
        _require(stat.S_IMODE(metadata.st_mode) == mode, f"{status}_MODE")
    return metadata


def _binding(path: Path, status: str) -> Mapping[str, Any]:
    metadata = _open_regular(path, None, status)
    mode = stat.S_IMODE(metadata.st_mode)
    _require(mode in (0o400, 0o500), f"{status}_MODE")
    return {
        "path": str(path),
        "sha256": _file_sha256(path),
        "size_bytes": metadata.st_size,
        "mode": f"{mode:04o}",
    }


def _atomic_publish_no_replace(path: Path, raw: bytes) -> str:
    """Publish canonical evidence through a same-directory hard-link gate."""
    _require(path.is_absolute(), "PUBLISH_PATH_NOT_ABSOLUTE")
    _require(not path.exists() and not path.is_symlink(), "PUBLISH_EXISTS")
    parent = path.parent
    _require(parent.is_dir() and not parent.is_symlink(), "PUBLISH_PARENT")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".partial", dir=parent)
    temporary = Path(temporary_name)
    published = False
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o400)
        os.link(temporary, path, follow_symlinks=False)
        published = True
        temporary.unlink()
        directory_descriptor = os.open(parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    except BaseException:
        if temporary.exists() or temporary.is_symlink():
            temporary.unlink()
        if published and (path.exists() or path.is_symlink()):
            path.unlink()
        raise
    metadata = _open_regular(path, 0o400, "PUBLISHED")
    _require(metadata.st_size == len(raw), "PUBLISHED_SIZE")
    return hashlib.sha256(raw).hexdigest()


def _seal_existing(path: Path, status: str) -> Mapping[str, Any]:
    metadata = _open_regular(path, None, status)
    _require(stat.S_IMODE(metadata.st_mode) in (0o600, 0o640, 0o644, 0o400),
             f"{status}_INITIAL_MODE")
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    try:
        before = os.fstat(descriptor)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    os.chmod(path, 0o400)
    after = _open_regular(path, 0o400, status)
    _require((before.st_dev, before.st_ino, before.st_size) ==
             (after.st_dev, after.st_ino, after.st_size), f"{status}_RACE")
    return _binding(path, status)


def _read_canonical_sealed(path: Path, status: str) -> Tuple[Mapping[str, Any], str]:
    _open_regular(path, 0o400, status)
    raw = path.read_bytes()
    try:
        def reject_pairs(pairs: Sequence[Tuple[str, Any]]) -> Mapping[str, Any]:
            result = {}
            for key, item in pairs:
                _require(key not in result, f"{status}_DUPLICATE_KEY:{key}")
                result[key] = item
            return result
        value = json.loads(
            raw.decode("utf-8"), object_pairs_hook=reject_pairs,
            parse_constant=lambda token: (
            _ for _ in ()).throw(SupervisorError(
                f"{status}_NONFINITE:{token}")))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SupervisorError(f"{status}_JSON:{error}") from error
    _require(type(value) is dict and raw == _canonical_bytes(value),
             f"{status}_NONCANONICAL")
    return value, hashlib.sha256(raw).hexdigest()


_CERTIFICATE_MODULE: ModuleType | None = None
_BOUND_CERTIFICATE_SOURCE: Path | None = None
CERTIFICATE_MODULE_NAME = "h1_candidate3_live_certificate_829_for_supervisor"


def _certificate_module() -> ModuleType:
    global _CERTIFICATE_MODULE
    if _CERTIFICATE_MODULE is not None:
        return _CERTIFICATE_MODULE
    source = (_BOUND_CERTIFICATE_SOURCE if _BOUND_CERTIFICATE_SOURCE is not None
              else Path(__file__).resolve().with_name(
                  "h1_candidate3_live_certificate_829.py"))
    source = _absolute(source, "CERTIFICATE_SOURCE")
    _open_regular(source, 0o500, "CERTIFICATE_SOURCE")
    try:
        source_bytes = source.read_bytes()
        code = compile(source_bytes, str(source), "exec", dont_inherit=True)
    except (OSError, SyntaxError, ValueError) as error:
        raise SupervisorError(f"CERTIFICATE_SOURCE_COMPILE:{error}") from error
    spec = ModuleSpec(CERTIFICATE_MODULE_NAME, loader=None,
                      origin=str(source))
    module = ModuleType(CERTIFICATE_MODULE_NAME)
    module.__file__ = str(source)
    module.__package__ = ""
    module.__loader__ = None
    module.__spec__ = spec
    module.__cached__ = None
    sys.modules[CERTIFICATE_MODULE_NAME] = module
    try:
        exec(code, module.__dict__)
    except BaseException:
        sys.modules.pop(CERTIFICATE_MODULE_NAME, None)
        raise
    for name, expected in (
        ("PLAN_SCHEMA", PLAN_SCHEMA), ("FORMAL_SCHEMA", FORMAL_SCHEMA),
        ("PROC_SCHEMA", PROC_SCHEMA), ("VIDEO_SCHEMA", VIDEO_SCHEMA),
        ("VIDEO_WINDOW_SCHEMA", VIDEO_WINDOW_SCHEMA),
        ("GUI_CAMERA_SCHEMA", GUI_CAMERA_SCHEMA),
        ("POSTFLIGHT_SCHEMA", POSTFLIGHT_SCHEMA),
        ("WORKCELL_POSE_SCHEMA", WORKCELL_POSE_SCHEMA),
        ("EXACT_FCL_REPLAY_SCHEMA", EXACT_FCL_REPLAY_SCHEMA),
    ):
        _require(getattr(module, name) == expected,
                 f"CERTIFICATE_SCHEMA_MISMATCH:{name}")
    _require(tuple(module.PROCESS_ROLES) == PROCESS_ROLES and
             tuple(module.FORMAL_AUTHORITY_IDS) == FORMAL_AUTHORITY_IDS and
             tuple(module.INPUT_IDS) == INPUT_IDS and
             tuple(module.EVIDENCE_TOOL_IDS) == EVIDENCE_TOOL_IDS and
             tuple(module.FCL_PREBOUND_FILE_IDS) == EXACT_FCL_FILE_IDS,
             "CERTIFICATE_ORDER_MISMATCH")
    _CERTIFICATE_MODULE = module
    return module


@dataclass(frozen=True)
class EvidencePaths:
    root: Path
    plan: Path
    formal: Path
    bag_directory: Path
    bag: Path
    markers: Path
    proc_snapshot: Path
    video: Path
    video_audit: Path
    postflight: Path
    exact_fcl_replay: Path
    exact_fcl_telemetry: Path
    exact_fcl_stdout: Path
    exact_fcl_stderr: Path
    workcell_pose_audit: Path
    video_window_audit: Path
    gui_camera_audit: Path
    logs: Path

    @classmethod
    def from_root(cls, root: Path) -> "EvidencePaths":
        root = _absolute(root, "RUN_ROOT")
        bag_directory = root / "rosbag_829"
        return cls(
            root=root,
            plan=root / "live_plan_829.json",
            formal=root / "formal_authorities_829.json",
            bag_directory=bag_directory,
            bag=bag_directory / "rosbag_829_0.db3",
            markers=root / "markers_829.jsonl",
            proc_snapshot=root / "proc_snapshot_829.json",
            video=root / "candidate3_829.mp4",
            video_audit=root / "video_audit_829.json",
            postflight=root / "postflight_829.json",
            exact_fcl_replay=root / "exact_fcl_replay_829.json",
            exact_fcl_telemetry=root / "exact_fcl_telemetry_829.tsv",
            exact_fcl_stdout=root / "exact_fcl_auditor_829.stdout",
            exact_fcl_stderr=root / "exact_fcl_auditor_829.stderr",
            workcell_pose_audit=root / "workcell_pose_audit_829.json",
            video_window_audit=root / "video_window_audit_829.json",
            gui_camera_audit=root / "gui_camera_pose_829.json",
            logs=root / "logs",
        )

    def input_paths(self) -> Mapping[str, str]:
        return {
            "rosbag": str(self.bag),
            "markers": str(self.markers),
            "proc_snapshot": str(self.proc_snapshot),
            "video_audit": str(self.video_audit),
            "postflight": str(self.postflight),
            "formal_authorities": str(self.formal),
            "exact_fcl_replay": str(self.exact_fcl_replay),
            "workcell_pose_audit": str(self.workcell_pose_audit),
            "gui_camera_audit": str(self.gui_camera_audit),
            "video_window_audit": str(self.video_window_audit),
        }


@dataclass(frozen=True)
class CommandBundle:
    run_id: str
    marker_topic: str
    processes: Mapping[str, Tuple[str, ...]]
    exact_fcl_argv: Tuple[str, ...]
    exact_fcl_files: Mapping[str, Path]
    raw_sha256: str


def _flag_value(argv: Sequence[str], flags: Sequence[str], status: str) -> str:
    indices = [index for index, item in enumerate(argv) if item in flags]
    _require(len(indices) == 1 and indices[0] + 1 < len(argv), status)
    return argv[indices[0] + 1]


def _validate_recorder_argv(argv: Sequence[str], paths: EvidencePaths) -> None:
    _require("bag" in argv and "record" in argv, "RECORDER_OPERATION")
    _require(
        argv.count(RECORDER_INCLUDE_UNPUBLISHED_FLAG) == 1 and
        argv.index(RECORDER_INCLUDE_UNPUBLISHED_FLAG) > argv.index("record"),
        "RECORDER_INCLUDE_UNPUBLISHED_TOPICS")
    storage = _flag_value(argv, ("-s", "--storage"), "RECORDER_STORAGE")
    output = _flag_value(argv, ("-o", "--output"), "RECORDER_OUTPUT")
    _require(storage == "sqlite3", "RECORDER_NOT_SQLITE3")
    _require(output == str(paths.bag_directory), "RECORDER_OUTPUT_PATH")
    _require("-a" not in argv and "--all" not in argv,
             "RECORDER_ALL_TOPICS_FORBIDDEN")
    for topic in RECORDER_TOPICS:
        _require(argv.count(topic) == 1, f"RECORDER_TOPIC:{topic}")
    _require(
        argv.count(RECORDER_TOPICS_FLAG) == 1 and
        argv.index(RECORDER_TOPICS_FLAG) >
        argv.index(RECORDER_INCLUDE_UNPUBLISHED_FLAG),
        "RECORDER_TOPICS_FLAG")
    topics_index = argv.index(RECORDER_TOPICS_FLAG)
    _require(tuple(argv[topics_index + 1:]) == RECORDER_TOPICS,
             "RECORDER_TOPICS_EXACT_ORDER")


def _validate_video_argv(argv: Sequence[str], paths: EvidencePaths) -> None:
    _require(len(argv) >= 4 and argv[0] == "/usr/bin/python3" and
             Path(argv[1]).is_absolute() and
             argv[2] == "_video-child", "VIDEO_HELPER_ENTRY")
    helper_source = _absolute(argv[1], "VIDEO_HELPER_SOURCE")
    helper_binding = _binding(helper_source, "VIDEO_HELPER_SOURCE")
    _require(helper_binding["mode"] == "0500", "VIDEO_HELPER_SOURCE_MODE")
    expected_sha = _flag_value(
        argv, ("--self-sha256",), "VIDEO_HELPER_SELF_SHA")
    _require(SHA256_RE.fullmatch(expected_sha) is not None and
             expected_sha == helper_binding["sha256"],
             "VIDEO_HELPER_SELF_BINDING")
    _require(_flag_value(argv, ("--output",), "VIDEO_HELPER_OUTPUT") ==
             str(paths.video), "VIDEO_OUTPUT_PATH")
    _require(_flag_value(
        argv, ("--window-audit",), "VIDEO_HELPER_WINDOW_AUDIT") ==
        str(paths.video_window_audit), "VIDEO_WINDOW_AUDIT_PATH")
    _require(_flag_value(argv, ("--run-id",), "VIDEO_HELPER_RUN_ID") != "",
             "VIDEO_HELPER_RUN_ID_VALUE")
    _require(Path(_flag_value(
        argv, ("--xwininfo",), "VIDEO_HELPER_XWININFO")).is_absolute() and
        Path(_flag_value(
            argv, ("--ffmpeg",), "VIDEO_HELPER_FFMPEG")).is_absolute(),
        "VIDEO_HELPER_TOOLS")
    _require(_flag_value(argv, ("--display",), "VIDEO_HELPER_DISPLAY") != "",
             "VIDEO_HELPER_DISPLAY_VALUE")
    helper_plan = _absolute(
        _flag_value(argv, ("--plan",), "VIDEO_HELPER_PLAN"),
        "VIDEO_HELPER_PLAN")
    _require(helper_plan == paths.plan, "VIDEO_HELPER_PLAN_BINDING")
    _require(_flag_value(argv, ("--crop",), "VIDEO_HELPER_CROP") ==
             "1160:1530:0:80" and
             _flag_value(argv, ("--framerate",), "VIDEO_HELPER_RATE") ==
             "12" and _flag_value(
                 argv, ("--crf",), "VIDEO_HELPER_CRF") == "20",
             "VIDEO_HELPER_FIXED_ENCODING")


def _float_flag(argv: Sequence[str], flag: str, status: str) -> float:
    try:
        value = float(_flag_value(argv, (flag,), status))
    except ValueError as error:
        raise SupervisorError(f"{status}_PARSE") from error
    _require(math.isfinite(value), f"{status}_FINITE")
    return value


def _validate_flow_argv(processes: Mapping[str, Sequence[str]]) -> None:
    prefix = processes["prefix"]
    middle = processes["middle_suffix"]
    _require(prefix.count("prefix") == 1, "PREFIX_OPERATION")
    _require(middle.count("middle-and-suffix") == 1,
             "MIDDLE_SUFFIX_OPERATION")
    right_prefix = _float_flag(prefix, "--right-time-scale", "PREFIX_SCALE")
    right_middle = _float_flag(
        middle, "--right-time-scale", "MIDDLE_RIGHT_SCALE")
    middle_scale = _float_flag(
        middle, "--middle-speed-scale", "MIDDLE_SPEED_SCALE")
    _require(1.0 <= right_prefix <= 10.0 and right_prefix == right_middle,
             "RIGHT_TIME_SCALE")
    _require(0.05 <= middle_scale <= 1.0, "MIDDLE_SPEED_SCALE_RANGE")
    _require(processes["postflight"].count("finalize") == 1,
             "POSTFLIGHT_FINALIZE_OPERATION")


def _validate_launch_argv(argv: Sequence[str]) -> None:
    _require(len(argv) >= 4 and argv[0] == "/usr/bin/python3" and
             Path(argv[1]).is_absolute() and argv[2] == "_launch-child",
             "LAUNCH_HELPER_ENTRY")
    helper = _binding(
        _absolute(argv[1], "LAUNCH_HELPER_SOURCE"), "LAUNCH_HELPER_SOURCE")
    _require(helper["mode"] == "0500" and _flag_value(
        argv, ("--self-sha256",), "LAUNCH_HELPER_SHA") == helper["sha256"],
        "LAUNCH_HELPER_SOURCE_BINDING")
    _absolute(_flag_value(argv, ("--plan",), "LAUNCH_HELPER_PLAN"),
              "LAUNCH_HELPER_PLAN")
    _absolute(_flag_value(argv, ("--workspace",), "LAUNCH_WORKSPACE"),
              "LAUNCH_WORKSPACE")
    for path_flag, sha_flag in (
        ("--gazebo-launch-source", "--gazebo-launch-source-sha256"),
        ("--gazebo-world-sdf", "--gazebo-world-sdf-sha256"),
        ("--incubator-model-sdf", "--incubator-model-sdf-sha256"),
    ):
        source = _absolute(_flag_value(argv, (path_flag,), path_flag), path_flag)
        _require(_file_sha256(source) == _flag_value(
            argv, (sha_flag,), sha_flag), f"LAUNCH_HELPER_FILE:{path_flag}")
    assignments = {}
    for item in argv:
        if ":=" in item:
            key, value = item.split(":=", 1)
            assignments[key] = value
    _require(assignments == {
        "spawn_incubator": "true", "table_x": "0.600",
        "cube_x": "0.334", "incubator_x": "0.590",
        "incubator_y": "0.00", "incubator_z": "0.815",
        "bridge_support_states": "false",
    }, "LAUNCH_ARGUMENT_CONTRACT")
    try:
        table_x = float(assignments["table_x"])
        incubator_x = float(assignments["incubator_x"])
        incubator_y = float(assignments["incubator_y"])
        incubator_z = float(assignments["incubator_z"])
        cube_x = float(assignments["cube_x"])
    except (KeyError, ValueError) as error:
        raise SupervisorError("LAUNCH_FIXED_GEOMETRY") from error
    _require(table_x == 0.600 and incubator_x == 0.590 and
             incubator_y == 0.0 and incubator_z == 0.815 and
             cube_x == 0.334,
             "LAUNCH_FIXED_GEOMETRY")


def _validate_fcl_argv(
    argv: Sequence[str], files: Mapping[str, Path], paths: EvidencePaths,
) -> None:
    _require(len(argv) == 13 and set(argv[1::2]) == {
        "--scene", "--path", "--telemetry-tsv", "--urdf", "--srdf",
        "--handle-stl",
    }, "FCL_ARGV_SHAPE")
    expected = {
        "--scene": files["scene_cdr"],
        "--path": files["contact_path"],
        "--telemetry-tsv": paths.exact_fcl_telemetry,
        "--urdf": files["urdf"],
        "--srdf": files["srdf"],
        "--handle-stl": files["original_handle_stl"],
    }
    _require(Path(argv[0]) == files["auditor_binary"], "FCL_BINARY_PATH")
    for flag, expected_path in expected.items():
        _require(_flag_value(argv, (flag,), f"FCL_FLAG:{flag}") ==
                 str(expected_path), f"FCL_FLAG_VALUE:{flag}")


def _validate_exact_fcl_authority(files: Mapping[str, Path]) -> None:
    """Bind every exact-FCL input to the immutable 31-joint v2 bundle."""
    roots = {path.parent for path in files.values()}
    _require(len(roots) == 1, "FCL_AUTHORITY_SINGLE_DIRECTORY")
    authority_root = next(iter(roots))
    relative_parts = EXACT_FCL_AUTHORITY_RELATIVE.parts
    _require(authority_root.parts[-len(relative_parts):] == relative_parts,
             "FCL_AUTHORITY_DIRECTORY")
    manifest_path = authority_root / "manifest.json"
    manifest_binding = _binding(manifest_path, "FCL_AUTHORITY_MANIFEST")
    _require(manifest_binding["mode"] == "0400" and
             manifest_binding["sha256"] ==
             EXACT_FCL_AUTHORITY_MANIFEST_SHA256,
             "FCL_AUTHORITY_MANIFEST_BINDING")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SupervisorError(f"FCL_AUTHORITY_MANIFEST_JSON:{error}") from error
    _require(type(manifest) is dict and
             manifest.get("schema") ==
             "h1_moveit/right_handle_live_fcl_authority/829-v3-31j" and
             manifest.get("status") == "PASS" and
             manifest.get("valid") is True and
             manifest.get("sealed") is True and
             manifest.get("controller_commands_issued") == 0,
             "FCL_AUTHORITY_MANIFEST_HEADER")
    manifest_files = manifest.get("files")
    _require(type(manifest_files) is dict and
             set(manifest_files) == set(EXACT_FCL_MANIFEST_FILE_IDS.values()),
             "FCL_AUTHORITY_MANIFEST_FILES")
    for identifier in EXACT_FCL_FILE_IDS:
        manifest_id = EXACT_FCL_MANIFEST_FILE_IDS[identifier]
        entry = manifest_files[manifest_id]
        _require(type(entry) is dict and
                 set(entry) == {"path", "sha256", "mode", "nlink"} and
                 type(entry["path"]) is str and
                 type(entry["sha256"]) is str and
                 SHA256_RE.fullmatch(entry["sha256"]) is not None and
                 entry["mode"] == (
                     "0500" if identifier == "auditor_binary" else "0400") and
                 entry["nlink"] == 1,
                 f"FCL_AUTHORITY_MANIFEST_ENTRY:{identifier}")
        relative_path = Path(entry["path"])
        _require(not relative_path.is_absolute() and
                 relative_path.parent == EXACT_FCL_AUTHORITY_RELATIVE and
                 files[identifier] == authority_root / relative_path.name,
                 f"FCL_AUTHORITY_FILE_PATH:{identifier}")
        actual = _binding(files[identifier],
                          f"FCL_AUTHORITY_FILE:{identifier}")
        _require(actual["sha256"] == entry["sha256"] and
                 actual["mode"] == entry["mode"],
                 f"FCL_AUTHORITY_FILE_BINDING:{identifier}")
    _require(_file_sha256(files["auditor_binary"]) ==
             EXACT_FCL_AUDITOR_BINARY_SHA256,
             "FCL_AUTHORITY_AUDITOR_BINARY")


def load_commands(path: Path, paths: EvidencePaths) -> CommandBundle:
    value, raw_sha = _read_canonical_sealed(path, "COMMANDS")
    _require(set(value) == {
        "schema", "run_id", "marker_topic", "processes",
        "exact_fcl_replay", "valid", "status",
    }, "COMMANDS_KEYS")
    _require(value["schema"] == COMMAND_SCHEMA and
             type(value["run_id"]) is str and
             RUN_ID_RE.fullmatch(value["run_id"]) is not None and
             value["marker_topic"] == MARKER_TOPIC and
             value["valid"] is True and value["status"] == "READY",
             "COMMANDS_HEADER")
    rows = value["processes"]
    _require(type(rows) is list and len(rows) == len(PROCESS_ROLES),
             "COMMANDS_PROCESS_COUNT")
    processes = {}
    for index, row in enumerate(rows):
        _require(type(row) is dict and set(row) == {"role", "argv"} and
                 row["role"] == PROCESS_ROLES[index],
                 f"COMMANDS_PROCESS_{index}")
        processes[row["role"]] = _validate_argv(
            row["argv"], f"COMMANDS_PROCESS_{row['role']}")
        _require(processes[row["role"]][0] == "/usr/bin/python3" and
                 len(processes[row["role"]]) >= 2 and
                 Path(processes[row["role"]][1]).is_absolute(),
                 f"COMMANDS_PROCESS_EXPLICIT_SYSTEM_PYTHON:{row['role']}")
    exact = value["exact_fcl_replay"]
    _require(type(exact) is dict and set(exact) == {"argv", "files"},
             "COMMANDS_FCL_KEYS")
    exact_argv = _validate_argv(exact["argv"], "COMMANDS_FCL")
    _require(type(exact["files"]) is dict and
             set(exact["files"]) == set(EXACT_FCL_FILE_IDS),
             "COMMANDS_FCL_FILE_SET")
    exact_files = {
        identifier: _absolute(exact["files"][identifier],
                              f"COMMANDS_FCL_FILE_{identifier}")
        for identifier in EXACT_FCL_FILE_IDS
    }
    for identifier, file_path in exact_files.items():
        binding = _binding(file_path, f"COMMANDS_FCL_FILE_{identifier}")
        expected_mode = "0500" if identifier == "auditor_binary" else "0400"
        _require(binding["mode"] == expected_mode,
                 f"COMMANDS_FCL_FILE_MODE:{identifier}")
    _validate_exact_fcl_authority(exact_files)
    _validate_launch_argv(processes["launch"])
    _validate_recorder_argv(processes["recorder"], paths)
    _validate_video_argv(processes["video"], paths)
    _require(_flag_value(
        processes["video"], ("--run-id",), "VIDEO_HELPER_RUN_ID") ==
        value["run_id"], "VIDEO_HELPER_RUN_ID_BINDING")
    _validate_flow_argv(processes)
    _validate_fcl_argv(exact_argv, exact_files, paths)
    return CommandBundle(
        run_id=value["run_id"], marker_topic=value["marker_topic"],
        processes=processes, exact_fcl_argv=exact_argv,
        exact_fcl_files=exact_files, raw_sha256=raw_sha)


def _authority_arguments(values: Sequence[str]) -> Mapping[str, Path]:
    _require(len(values) == len(FORMAL_AUTHORITY_IDS), "AUTHORITY_COUNT")
    result = {}
    for index, value in enumerate(values):
        _require(value.count("=") == 1, f"AUTHORITY_{index}_SYNTAX")
        identifier, raw_path = value.split("=", 1)
        _require(identifier == FORMAL_AUTHORITY_IDS[index],
                 f"AUTHORITY_{index}_ORDER")
        result[identifier] = _absolute(raw_path, f"AUTHORITY_{identifier}")
    return result


def _evidence_tool_arguments(values: Sequence[str]) -> Mapping[str, Path]:
    _require(len(values) == len(EVIDENCE_TOOL_IDS), "EVIDENCE_TOOL_COUNT")
    result = {}
    for index, value in enumerate(values):
        _require(value.count("=") == 1, f"EVIDENCE_TOOL_{index}_SYNTAX")
        identifier, raw_path = value.split("=", 1)
        _require(identifier == EVIDENCE_TOOL_IDS[index],
                 f"EVIDENCE_TOOL_{index}_ORDER")
        result[identifier] = _absolute(
            raw_path, f"EVIDENCE_TOOL_{identifier}")
    return result


def prepare(
    *, run_root: Path, commands_path: Path,
    authorities: Mapping[str, Path], evidence_tools: Mapping[str, Path],
    monotonic_ns: int | None = None,
) -> Mapping[str, Any]:
    """Create the immutable formal binding and run plan, without execution."""
    paths = EvidencePaths.from_root(run_root)
    _require(not paths.root.exists() and not paths.root.is_symlink(),
             "RUN_ROOT_EXISTS")
    paths.root.mkdir(mode=0o700, parents=False)
    paths.logs.mkdir(mode=0o700)
    bundle = load_commands(_absolute(commands_path, "COMMANDS_PATH"), paths)
    _require(tuple(authorities) == FORMAL_AUTHORITY_IDS,
             "AUTHORITY_ORDER")
    authority_rows = []
    for identifier in FORMAL_AUTHORITY_IDS:
        item = {"id": identifier}
        item.update(_binding(
            _absolute(authorities[identifier], f"AUTHORITY_{identifier}"),
            f"AUTHORITY_{identifier}"))
        authority_rows.append(item)
    authority_by_id = {item["id"]: item for item in authority_rows}
    for identifier, path_flag, sha_flag in (
        ("gazebo_launch_source", "--gazebo-launch-source",
         "--gazebo-launch-source-sha256"),
        ("gazebo_world_sdf", "--gazebo-world-sdf",
         "--gazebo-world-sdf-sha256"),
        ("incubator_model_sdf", "--incubator-model-sdf",
         "--incubator-model-sdf-sha256"),
    ):
        launch_argv = bundle.processes["launch"]
        _require(_flag_value(launch_argv, (path_flag,), path_flag) ==
                 authority_by_id[identifier]["path"] and
                 _flag_value(launch_argv, (sha_flag,), sha_flag) ==
                 authority_by_id[identifier]["sha256"],
                 f"LAUNCH_FORMAL_CROSS_BIND:{identifier}")
    formal = {
        "schema": FORMAL_SCHEMA,
        "run_id": bundle.run_id,
        "authorities": authority_rows,
        "valid": True,
        "status": "PASS",
    }
    formal_sha = _atomic_publish_no_replace(
        paths.formal, _canonical_bytes(formal))
    _require(tuple(evidence_tools) == EVIDENCE_TOOL_IDS,
             "EVIDENCE_TOOL_ORDER")
    evidence_tool_bindings = {}
    for identifier in EVIDENCE_TOOL_IDS:
        binding = _binding(
            _absolute(evidence_tools[identifier],
                      f"EVIDENCE_TOOL_{identifier}"),
            f"EVIDENCE_TOOL_{identifier}")
        _require(binding["mode"] == "0500",
                 f"EVIDENCE_TOOL_MODE:{identifier}")
        evidence_tool_bindings[identifier] = binding
    _require(Path(bundle.processes["video"][1]) ==
             Path(evidence_tool_bindings["supervisor_source"]["path"]) and
             _flag_value(bundle.processes["video"], ("--self-sha256",),
                         "VIDEO_HELPER_SHA") ==
             evidence_tool_bindings["supervisor_source"]["sha256"],
             "VIDEO_HELPER_EVIDENCE_TOOL_BINDING")
    sealed_monotonic = monotonic_ns if monotonic_ns is not None else time.monotonic_ns()
    _require(type(sealed_monotonic) is int and sealed_monotonic > 0,
             "PLAN_MONOTONIC")
    plan = {
        "schema": PLAN_SCHEMA,
        "run_id": bundle.run_id,
        "sealed_monotonic_ns": sealed_monotonic,
        "input_paths": paths.input_paths(),
        "formal_authorities_sha256": formal_sha,
        "expected_process_argv_sha256": {
            role: _argv_sha256(bundle.processes[role])
            for role in PROCESS_ROLES
        },
        "expected_exact_fcl_replay_argv_sha256":
            _argv_sha256(bundle.exact_fcl_argv),
        "expected_exact_fcl_file_sha256": {
            identifier: _file_sha256(bundle.exact_fcl_files[identifier])
            for identifier in EXACT_FCL_FILE_IDS
        },
        "evidence_tools": evidence_tool_bindings,
        "max_joint_gap_ns": MAX_JOINT_GAP_NS,
        "valid": True,
        "status": "READY",
    }
    plan_sha = _atomic_publish_no_replace(paths.plan, _canonical_bytes(plan))
    return {
        "run_id": bundle.run_id,
        "run_root": str(paths.root),
        "commands_sha256": bundle.raw_sha256,
        "formal_authorities_sha256": formal_sha,
        "plan_sha256": plan_sha,
        "valid": True,
        "status": "READY",
    }


def _proc_stat_fields(path: Path) -> Tuple[int, int]:
    try:
        raw = path.read_text(encoding="ascii")
    except OSError as error:
        raise SupervisorError(f"PROC_STAT_READ:{error}") from error
    closing = raw.rfind(")")
    _require(closing >= 0, "PROC_STAT_FORMAT")
    fields = raw[closing + 2:].split()
    # fields[0] is kernel field 3 (state); ppid is 4 and starttime is 22.
    _require(len(fields) >= 20, "PROC_STAT_FIELDS")
    try:
        return int(fields[1]), int(fields[19])
    except ValueError as error:
        raise SupervisorError("PROC_STAT_INTEGER") from error


def _proc_argv(path: Path) -> Tuple[str, ...]:
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise SupervisorError(f"PROC_CMDLINE_READ:{error}") from error
    _require(raw.endswith(b"\0") and raw != b"\0", "PROC_CMDLINE_FORMAT")
    try:
        argv = tuple(item.decode("utf-8") for item in raw[:-1].split(b"\0"))
    except UnicodeDecodeError as error:
        raise SupervisorError("PROC_CMDLINE_ENCODING") from error
    _require(all(argv), "PROC_CMDLINE_EMPTY_ARG")
    return argv


def _read_proc_identity(
    pid: int, expected_argv: Sequence[str], *, proc_root: Path = Path("/proc"),
    expected_ppid: int | None = None, timeout_s: float = 2.0,
) -> Mapping[str, Any]:
    deadline = time.monotonic() + timeout_s
    last_status = "PROC_IDENTITY_NOT_READY"
    while time.monotonic() < deadline:
        try:
            ppid, start_ticks = _proc_stat_fields(proc_root / str(pid) / "stat")
            actual_argv = _proc_argv(proc_root / str(pid) / "cmdline")
        except SupervisorError as error:
            last_status = str(error)
            time.sleep(0.01)
            continue
        if tuple(expected_argv) != actual_argv:
            last_status = "PROC_ARGV_NOT_EXACT"
            time.sleep(0.01)
            continue
        if expected_ppid is not None and ppid != expected_ppid:
            last_status = "PROC_PPID_NOT_DIRECT"
            time.sleep(0.01)
            continue
        _require(start_ticks > 0, "PROC_START_TICKS")
        return {
            "pid": pid, "ppid": ppid, "start_ticks": start_ticks,
            "argv": list(actual_argv),
            "cmdline_sha256": _argv_sha256(actual_argv),
        }
    raise SupervisorError(last_status)


def _forbidden_process(argv: Sequence[str]) -> bool:
    if not argv:
        return False
    executable = Path(argv[0]).name
    tokens = tuple(Path(item).name if "/" in item else item for item in argv)
    if executable == "gz" and "sim" in argv[1:]:
        return True
    if executable in ("ros2", "python3", "python"):
        if "launch" in argv and "gazebo.launch.py" in tokens:
            return True
        if "bag" in argv and "record" in argv:
            return True
        if "_launch-child" in argv or "_video-child" in argv:
            return True
    if executable == "ffmpeg" and "x11grab" in argv:
        return True
    resident_names = {
        "ruby", "move_group", "parameter_bridge", "ros2_control_node",
        "robot_state_publisher", "controller_manager",
    }
    if executable in resident_names:
        return True
    controlled_scripts = {
        "h1_candidate3_flow_828.py", "h1_right_handle_executor_828.py",
        "h1_candidate3_middle_executor_828.py",
        "h1_support_orchestrator_828.py",
    }
    return any(token in controlled_scripts for token in tokens)


def audit_preexisting_processes(
    *, proc_root: Path = Path("/proc"), excluded_pids: Iterable[int] = (),
) -> list[Mapping[str, Any]]:
    excluded = set(excluded_pids)
    result = []
    try:
        entries = sorted(
            (entry for entry in proc_root.iterdir() if entry.name.isdigit()),
            key=lambda entry: int(entry.name))
    except OSError as error:
        raise SupervisorError(f"PREEXISTING_PROC_LIST:{error}") from error
    for entry in entries:
        pid = int(entry.name)
        if pid in excluded:
            continue
        try:
            argv = _proc_argv(entry / "cmdline")
            ppid, start_ticks = _proc_stat_fields(entry / "stat")
        except SupervisorError:
            continue
        if _forbidden_process(argv):
            result.append({
                "pid": pid, "ppid": ppid, "start_ticks": start_ticks,
                "argv": list(argv), "cmdline_sha256": _argv_sha256(argv),
            })
    return result


def _publish_existing_no_replace(staged: Path, destination: Path) -> None:
    _require(staged.is_absolute() and destination.is_absolute(),
             "STAGED_PATH")
    _require(not destination.exists() and not destination.is_symlink(),
             "STAGED_DESTINATION_EXISTS")
    _open_regular(staged, None, "STAGED_SOURCE")
    os.chmod(staged, 0o400)
    os.link(staged, destination, follow_symlinks=False)
    staged.unlink()
    _open_regular(destination, 0o400, "STAGED_DESTINATION")


def _staging_path(log_directory: Path, role: str, channel: str) -> Path:
    _require(role in PROCESS_ROLES and channel in ("stdout", "stderr"),
             "LOG_NAME")
    return log_directory / f".{role}.{channel}.{os.getpid()}.partial"


@dataclass
class DirectChild:
    role: str
    argv: Tuple[str, ...]
    process: Any
    identity: Mapping[str, Any]
    started_monotonic_ns: int
    stdout_staged: Path
    stderr_staged: Path
    foreground: bool
    ended_monotonic_ns: int | None = None
    exit_code: int | None = None

    def record(self, run_id: str) -> Mapping[str, Any]:
        _require(self.ended_monotonic_ns is not None and
                 self.exit_code is not None, f"CHILD_NOT_ENDED:{self.role}")
        return {
            "role": self.role,
            "pid": self.identity["pid"],
            "ppid": self.identity["ppid"],
            "start_ticks": self.identity["start_ticks"],
            "started_monotonic_ns": self.started_monotonic_ns,
            "ended_monotonic_ns": self.ended_monotonic_ns,
            "exit_code": self.exit_code,
            "run_id": run_id,
            "argv": self.identity["argv"],
            "cmdline_sha256": self.identity["cmdline_sha256"],
        }


def _spawn_child(
    role: str, argv: Sequence[str], paths: EvidencePaths, *, foreground: bool,
    popen_factory: Callable[..., Any] = subprocess.Popen,
    proc_identity_reader: Callable[..., Mapping[str, Any]] = _read_proc_identity,
) -> DirectChild:
    _require(role in PROCESS_ROLES, "SPAWN_ROLE")
    stdout_staged = _staging_path(paths.logs, role, "stdout")
    stderr_staged = _staging_path(paths.logs, role, "stderr")
    for staged in (stdout_staged, stderr_staged):
        _require(not staged.exists() and not staged.is_symlink(),
                 "SPAWN_STAGING_EXISTS")
    stdout_descriptor = None
    if not foreground:
        stdout_descriptor = os.open(
            stdout_staged, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    else:
        descriptor = os.open(
            stdout_staged, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        os.close(descriptor)
    stderr_descriptor = os.open(
        stderr_staged, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    started = time.monotonic_ns()
    stdin_target = subprocess.PIPE if role == "video" else subprocess.DEVNULL
    try:
        process = popen_factory(
            list(argv), stdin=stdin_target,
            stdout=subprocess.PIPE if foreground else stdout_descriptor,
            stderr=stderr_descriptor, text=False, bufsize=0,
            start_new_session=True, close_fds=True)
    except BaseException:
        if stdout_descriptor is not None:
            os.close(stdout_descriptor)
        os.close(stderr_descriptor)
        for staged in (stdout_staged, stderr_staged):
            if staged.exists() or staged.is_symlink():
                staged.unlink()
        raise
    if stdout_descriptor is not None:
        os.close(stdout_descriptor)
    os.close(stderr_descriptor)
    identity = proc_identity_reader(
        process.pid, tuple(argv), expected_ppid=os.getpid())
    _require(identity["cmdline_sha256"] == _argv_sha256(argv),
             f"SPAWN_ARGV:{role}")
    return DirectChild(
        role=role, argv=tuple(argv), process=process, identity=identity,
        started_monotonic_ns=started, stdout_staged=stdout_staged,
        stderr_staged=stderr_staged, foreground=foreground)


def _seal_child_logs(child: DirectChild, paths: EvidencePaths) -> None:
    for channel, staged in (
            ("stdout", child.stdout_staged),
            ("stderr", child.stderr_staged)):
        _require(staged.stat().st_size <= MAX_CAPTURE_BYTES,
                 f"CHILD_{channel.upper()}_BUDGET:{child.role}")
        destination = paths.logs / f"{child.role}.{channel}.log"
        _publish_existing_no_replace(staged, destination)


class MarkerPublisher(Protocol):
    def wait_for_recorder(self, timeout_s: float) -> Mapping[str, Any]:
        """Wait until the rosbag subscription is discovered."""

    def publish(self, payload: str) -> None:
        """Publish one std_msgs/String payload exactly once."""


@dataclass
class MarkerSequencer:
    run_id: str
    specs: Sequence[Any]
    publisher: MarkerPublisher
    payloads: list[Mapping[str, Any]] = field(default_factory=list)

    def _emit(self, role: str, name: str, line: str) -> None:
        index = len(self.payloads)
        _require(index < len(self.specs), "MARKER_EXTRA")
        spec = self.specs[index]
        _require(spec.process_role == role and spec.name == name and
                 spec.pattern.fullmatch(line) is not None,
                 f"MARKER_OUT_OF_ORDER:{role}:{name}")
        now = time.monotonic_ns()
        if self.payloads:
            _require(now > self.payloads[-1]["monotonic_ns"],
                     "MARKER_MONOTONIC")
        payload = {
            "schema": MARKER_PAYLOAD_SCHEMA,
            "run_id": self.run_id,
            "sequence": index,
            "monotonic_ns": now,
            "process_role": role,
            "name": name,
            "line": line,
        }
        self.publisher.publish(
            _canonical_bytes(payload).decode("ascii").rstrip("\n"))
        self.payloads.append(payload)

    def supervisor(self, name: str, line: str) -> None:
        self._emit("supervisor", name, line)

    def child_line(self, role: str, line: str) -> None:
        matches = [spec for spec in self.specs
                   if spec.process_role == role and
                   spec.pattern.fullmatch(line) is not None]
        if not matches:
            return
        _require(len(matches) == 1, f"MARKER_AMBIGUOUS:{role}:{line}")
        self._emit(role, matches[0].name, line)

    def complete(self) -> None:
        _require(len(self.payloads) == len(self.specs), "MARKER_INCOMPLETE")


def _wait_foreground_child(
    child: DirectChild, sequencer: MarkerSequencer, *, timeout_s: float,
) -> None:
    _require(child.foreground and child.process.stdout is not None,
             "FOREGROUND_CHILD")
    deadline = time.monotonic() + timeout_s
    total = 0
    selector = selectors.DefaultSelector()
    selector.register(child.process.stdout, selectors.EVENT_READ)
    pending = b""
    with child.stdout_staged.open("ab", buffering=0) as transcript:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise SupervisorError(f"CHILD_TIMEOUT:{child.role}")
            events = selector.select(timeout=min(0.1, remaining))
            if not events:
                if child.process.poll() is not None:
                    chunk = os.read(child.process.stdout.fileno(), 65536)
                    if not chunk:
                        break
                else:
                    continue
            else:
                chunk = os.read(child.process.stdout.fileno(), 65536)
            if not chunk:
                break
            total += len(chunk)
            _require(total <= MAX_CAPTURE_BYTES,
                     f"CHILD_STDOUT_BUDGET:{child.role}")
            transcript.write(chunk)
            pending += chunk
            _require(b"\0" not in pending, f"CHILD_STDOUT_NUL:{child.role}")
            while b"\n" in pending:
                raw_line, pending = pending.split(b"\n", 1)
                try:
                    line = raw_line.decode("utf-8")
                except UnicodeDecodeError as error:
                    raise SupervisorError(
                        f"CHILD_STDOUT_ENCODING:{child.role}") from error
                if line.endswith("\r"):
                    line = line[:-1]
                sequencer.child_line(child.role, line)
    selector.close()
    _require(pending == b"", f"CHILD_STDOUT_FINAL_NEWLINE:{child.role}")
    child.exit_code = child.process.wait(timeout=1.0)
    child.ended_monotonic_ns = time.monotonic_ns()
    _require(child.exit_code == 0, f"CHILD_EXIT:{child.role}:{child.exit_code}")


def _stop_background_child(
    child: DirectChild, *, method: str, timeout_s: float,
) -> None:
    _require(not child.foreground and method in ("sigint", "video"),
             "STOP_CHILD")
    if child.process.poll() is None:
        if method == "video":
            _require(child.process.stdin is not None, "VIDEO_STDIN")
            child.process.stdin.write(b"q\n")
            child.process.stdin.flush()
            child.process.stdin.close()
        else:
            os.killpg(child.process.pid, signal.SIGINT)
    try:
        child.exit_code = child.process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired as error:
        try:
            os.killpg(child.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        raise SupervisorError(f"CHILD_STOP_TIMEOUT:{child.role}") from error
    child.ended_monotonic_ns = time.monotonic_ns()
    _require(child.exit_code == 0, f"CHILD_EXIT:{child.role}:{child.exit_code}")


def _emergency_stop(children: Sequence[DirectChild]) -> None:
    for child in reversed(children):
        if child.process.poll() is None:
            try:
                os.killpg(child.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    for child in reversed(children):
        if child.process.poll() is None:
            try:
                child.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


class StringDecoder(Protocol):
    def decode(self, payload: bytes) -> str:
        """Decode a serialized std_msgs/msg/String payload."""


def extract_marker_jsonl(
    *, bag_path: Path, marker_topic: str,
    expected_payloads: Sequence[Mapping[str, Any]], decoder: StringDecoder,
    marker_schema: str,
) -> bytes:
    _open_regular(bag_path, 0o400, "MARKER_BAG")
    uri = f"file:{bag_path.as_posix()}?mode=ro&immutable=1"
    try:
        connection = sqlite3.connect(uri, uri=True)
        rows = list(connection.execute(
            "SELECT id,type,serialization_format FROM topics WHERE name=?",
            (marker_topic,)))
        _require(len(rows) == 1 and rows[0][1] == MARKER_TOPIC_TYPE and
                 rows[0][2] == "cdr", "MARKER_BAG_TOPIC")
        message_rows = list(connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? "
            "ORDER BY timestamp,id", (rows[0][0],)))
    except sqlite3.Error as error:
        raise SupervisorError(f"MARKER_BAG_SQL:{error}") from error
    finally:
        if "connection" in locals():
            connection.close()
    _require(len(message_rows) == len(expected_payloads),
             "MARKER_BAG_MESSAGE_COUNT")
    result = []
    previous_timestamp = -1
    for index, ((timestamp, raw), expected) in enumerate(zip(
            message_rows, expected_payloads)):
        _require(type(timestamp) is int and timestamp > previous_timestamp and
                 type(raw) is bytes, f"MARKER_BAG_ROW:{index}")
        previous_timestamp = timestamp
        decoded = decoder.decode(raw)
        _require(decoded.encode("ascii") == _canonical_bytes(expected)[:-1],
                 f"MARKER_BAG_PAYLOAD:{index}")
        result.append({
            "schema": marker_schema,
            "run_id": expected["run_id"],
            "sequence": expected["sequence"],
            "monotonic_ns": expected["monotonic_ns"],
            "bag_timestamp_ns": timestamp,
            "process_role": expected["process_role"],
            "name": expected["name"],
            "line": expected["line"],
        })
    return b"".join(_canonical_bytes(item) for item in result)


class RclpyLiveAdapter:
    """The single supervisor ROS node: marker publisher and live probes."""

    def __init__(self, run_id: str, marker_topic: str) -> None:
        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from rclpy.serialization import deserialize_message
            from controller_manager_msgs.srv import ListControllers
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
        except ImportError as error:
            raise SupervisorError(f"RCLPY_IMPORT:{error}") from error
        self._rclpy = rclpy
        self._String = String
        self._deserialize = deserialize_message
        self._ListControllers = ListControllers
        rclpy.init(args=None)
        self.node = rclpy.create_node(
            f"h1_candidate3_live_supervisor_829_{run_id[:12]}")
        qos = QoSProfile(depth=100)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._marker_publisher = self.node.create_publisher(
            String, marker_topic, qos)
        self._robot: Tuple[int, Mapping[str, float]] | None = None
        self._incubator: Tuple[int, Mapping[str, float]] | None = None
        self._latch: Tuple[int, str] | None = None
        self._clocks: list[Tuple[int, int]] = []

        def joint_mapping(message: Any) -> Mapping[str, float]:
            if len(message.name) != len(message.position):
                return {}
            return {name: float(position)
                    for name, position in zip(message.name, message.position)}

        def robot_callback(message: Any) -> None:
            self._robot = (time.monotonic_ns(), joint_mapping(message))

        def incubator_callback(message: Any) -> None:
            self._incubator = (time.monotonic_ns(), joint_mapping(message))

        def latch_callback(message: Any) -> None:
            self._latch = (time.monotonic_ns(), str(message.data))

        def clock_callback(message: Any) -> None:
            value = int(message.clock.sec) * 1_000_000_000 + int(
                message.clock.nanosec)
            self._clocks.append((time.monotonic_ns(), value))
            if len(self._clocks) > 100:
                del self._clocks[:-100]

        self._subscriptions = (
            self.node.create_subscription(
                JointState, "/joint_states", robot_callback, 100),
            self.node.create_subscription(
                JointState, "/incubator/joint_states", incubator_callback, 100),
            self.node.create_subscription(
                String, "/incubator/latch/state", latch_callback, 10),
            self.node.create_subscription(Clock, "/clock", clock_callback, 100),
        )
        self._controllers = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers")

    def _spin_until(self, predicate: Callable[[], bool], timeout_s: float,
                    status: str) -> None:
        deadline = time.monotonic() + timeout_s
        while True:
            if predicate():
                return
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                break
            self._rclpy.spin_once(
                self.node, timeout_sec=min(0.05, remaining_s))
        raise SupervisorError(status)

    def _controller_states(self, timeout_s: float) -> list[str]:
        deadline = time.monotonic() + timeout_s
        _require(self._controllers.wait_for_service(timeout_sec=max(
            0.0, deadline - time.monotonic())),
                 "CONTROLLER_SERVICE")
        future = self._controllers.call_async(self._ListControllers.Request())
        self._spin_until(lambda: future.done(), max(
            0.0, deadline - time.monotonic()),
                         "CONTROLLER_SERVICE_TIMEOUT")
        try:
            response = future.result()
        except Exception as error:
            raise SupervisorError(f"CONTROLLER_SERVICE_RESULT:{error}") from error
        _require(response is not None, "CONTROLLER_SERVICE_NONE")
        return [str(controller.state) for controller in response.controller]

    def wait_launch_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s

        def telemetry_ready() -> bool:
            return (self._robot is not None and self._incubator is not None and
                    self._latch is not None and len(self._clocks) >= 2 and
                    self._clocks[-1][1] > self._clocks[-2][1])
        self._spin_until(
            telemetry_ready, max(0.0, deadline - time.monotonic()),
            "LAUNCH_TELEMETRY_TIMEOUT")
        while True:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                raise SupervisorError("LAUNCH_CONTROLLERS")
            states = self._controller_states(remaining_s)
            if time.monotonic() <= deadline and len(states) == 6 and all(
                    state == "active" for state in states):
                return
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                raise SupervisorError("LAUNCH_CONTROLLERS")
            time.sleep(min(0.05, remaining_s))

    def wait_for_recorder(self, timeout_s: float) -> Mapping[str, Any]:
        topics = tuple(sorted((*REQUIRED_BAG_TOPICS, MARKER_TOPIC)))

        def recorder_identity_set(topic: str) -> set[Tuple[str, str]]:
            return {(item.node_namespace, item.node_name)
                    for item in self.node.get_subscriptions_info_by_topic(topic)}

        common: set[Tuple[str, str]] = set()

        def ready() -> bool:
            nonlocal common
            sets = [recorder_identity_set(topic) for topic in topics]
            if any(not identities for identities in sets):
                return False
            common = set.intersection(*sets)
            return (len(common) == 1 and
                    self._marker_publisher.get_subscription_count() == 1)

        self._spin_until(ready, timeout_s, "RECORDER_ALL_SUBSCRIPTIONS")
        return {
            "captured_monotonic_ns": time.monotonic_ns(),
            "subscription_topics": list(topics),
        }

    def publish(self, payload: str) -> None:
        message = self._String()
        message.data = payload
        self._marker_publisher.publish(message)
        self._rclpy.spin_once(self.node, timeout_sec=0.02)

    def decode(self, payload: bytes) -> str:
        try:
            message = self._deserialize(payload, self._String)
        except Exception as error:
            raise SupervisorError(f"MARKER_CDR:{error}") from error
        return str(message.data)

    def capture_postflight(
        self, *, after_monotonic_ns: int, timeout_s: float,
    ) -> Mapping[str, Any]:
        def fresh() -> bool:
            samples = (self._robot, self._incubator, self._latch)
            fresh_samples = all(sample is not None and sample[0] >= after_monotonic_ns
                                for sample in samples)
            fresh_clocks = [item for item in self._clocks
                            if item[0] >= after_monotonic_ns]
            return (fresh_samples and len(fresh_clocks) >= 2 and
                    fresh_clocks[-1][1] > fresh_clocks[-2][1])

        self._spin_until(fresh, timeout_s, "POSTFLIGHT_TELEMETRY_TIMEOUT")
        _require(self._robot is not None and self._incubator is not None and
                 self._latch is not None, "POSTFLIGHT_INTERNAL")
        robot = self._robot[1]
        incubator = self._incubator[1]
        _require(all(name in robot for name in FINAL_ROBOT_POSITIONS),
                 "POSTFLIGHT_ROBOT_JOINTS")
        robot_at_launch = all(abs(robot[name] - expected) <= (
            0.0015 if name.startswith("right_dex1_finger_") else 0.003)
            for name, expected in FINAL_ROBOT_POSITIONS.items())
        _require("door_hinge_joint" in incubator and
                 "handle_latch_joint" in incubator,
                 "POSTFLIGHT_INCUBATOR_JOINTS")
        states = self._controller_states(timeout_s)
        publisher_count = sum(len(self.node.get_publishers_info_by_topic(topic))
                              for topic in CONTROLLER_TOPICS)
        return {
            "captured_monotonic_ns": time.monotonic_ns(),
            "door_angle_rad": incubator["door_hinge_joint"],
            "handle_angle_rad": incubator["handle_latch_joint"],
            "torso_angle_rad": robot["torso_joint"],
            "latch_locked": self._latch[1] == "locked",
            "robot_at_launch": robot_at_launch,
            "controller_count": len(states),
            "trajectory_publisher_count": publisher_count,
            "clock_advancing": True,
        }

    def close(self) -> None:
        self.node.destroy_node()
        self._rclpy.shutdown()


GUI_CAMERA_REQUEST = (
    "pose { position { x: 0.06292473524808884 "
    "y: -0.46163156628608704 z: 1.8398312330245972 } "
    "orientation { x: -0.15106521546840668 y: 0.24716269969940186 "
    "z: 0.49844232201576233 w: 0.8170956969261169 } }"
)
GUI_CAMERA_ARGV = (
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz", "service", "-s",
    "/gui/move_to/pose",
    "--reqtype", "gz.msgs.GUICamera", "--reptype", "gz.msgs.Boolean",
    "--timeout", "5000", "--req", GUI_CAMERA_REQUEST,
)


def set_gui_camera_pose(
    *, run_id: str, plan_sha256: str, output_path: Path,
    run_factory: Callable[..., Any] = subprocess.run,
) -> Mapping[str, Any]:
    completed = run_factory(
        list(GUI_CAMERA_ARGV), stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    _require(completed.returncode == 0 and
             type(completed.stdout) is bytes and
             type(completed.stderr) is bytes and
             len(completed.stdout) <= MAX_CAPTURE_BYTES and
             len(completed.stderr) <= MAX_CAPTURE_BYTES,
             "GUI_CAMERA_COMMAND")
    try:
        response = completed.stdout.decode("utf-8")
        diagnostic = completed.stderr.decode("utf-8")
    except UnicodeDecodeError as error:
        raise SupervisorError("GUI_CAMERA_ENCODING") from error
    _require(re.search(r"(?:^|\n)\s*data:\s*true\s*(?:\n|$)", response)
             is not None and diagnostic == "", "GUI_CAMERA_REJECTED")
    value = {
        "schema": GUI_CAMERA_SCHEMA,
        "run_id": run_id,
        "sealed_plan_sha256": plan_sha256,
        "captured_monotonic_ns": time.monotonic_ns(),
        "source_command_argv": list(GUI_CAMERA_ARGV),
        "source_command_argv_sha256": _argv_sha256(GUI_CAMERA_ARGV),
        "request": GUI_CAMERA_REQUEST,
        "response_stdout": response,
        "response_stderr": diagnostic,
        "exit_code": completed.returncode,
        "valid": True,
        "status": "PASS",
    }
    _atomic_publish_no_replace(output_path, _canonical_bytes(value))
    return value


def _launch_child(args: argparse.Namespace) -> int:
    """Verify the sealed launch/physics sources, then proxy ros2 launch."""
    try:
        source = Path(__file__).resolve()
        _require(args.self_sha256 == _file_sha256(source),
                 "LAUNCH_CHILD_SELF_SHA256")
        plan_path = _absolute(args.plan, "LAUNCH_CHILD_PLAN")
        plan, plan_sha256 = _read_canonical_sealed(
            plan_path, "LAUNCH_CHILD_PLAN")
        tool_binding = plan.get("evidence_tools", {}).get("supervisor_source")
        _require(type(tool_binding) is dict and
                 tool_binding.get("path") == str(source) and
                 tool_binding.get("sha256") == args.self_sha256 and
                 tool_binding.get("mode") == "0500",
                 "LAUNCH_CHILD_PLAN_SOURCE_BINDING")
        helper_argv = [sys.executable, *sys.argv]
        _require(_argv_sha256(helper_argv) ==
                 plan.get("expected_process_argv_sha256", {}).get("launch"),
                 "LAUNCH_CHILD_PLAN_ARGV")
        formal_path = Path(plan["input_paths"]["formal_authorities"])
        formal, formal_sha = _read_canonical_sealed(
            formal_path, "LAUNCH_CHILD_FORMAL")
        _require(formal_sha == plan.get("formal_authorities_sha256"),
                 "LAUNCH_CHILD_FORMAL_SHA")
        authorities = {item["id"]: item for item in formal["authorities"]}
        supplied = {
            "gazebo_launch_source": (
                args.gazebo_launch_source,
                args.gazebo_launch_source_sha256),
            "gazebo_world_sdf": (
                args.gazebo_world_sdf, args.gazebo_world_sdf_sha256),
            "incubator_model_sdf": (
                args.incubator_model_sdf,
                args.incubator_model_sdf_sha256),
        }
        for identifier, (path, digest) in supplied.items():
            binding = authorities.get(identifier)
            actual = _binding(
                _absolute(path, f"LAUNCH_CHILD_FORMAL_PATH:{identifier}"),
                f"LAUNCH_CHILD_FORMAL_FILE:{identifier}")
            _require(type(binding) is dict and binding.get("path") == str(path) and
                     binding.get("sha256") == digest and
                     binding.get("size_bytes") == actual["size_bytes"] and
                     binding.get("mode") == actual["mode"] == "0400" and
                     actual["path"] == str(path) and
                     actual["sha256"] == digest,
                     f"LAUNCH_CHILD_FORMAL_BINDING:{identifier}")
        workspace = _absolute(args.workspace, "LAUNCH_CHILD_WORKSPACE")
        installed = {
            "gazebo_launch_source": workspace / "install/h1_2_moveit_config/"
                "share/h1_2_moveit_config/launch/gazebo.launch.py",
            "gazebo_world_sdf": workspace / "install/h1_2_description/"
                "share/h1_2_description/worlds/h1_world.sdf",
            "incubator_model_sdf": workspace / "install/h1_2_description/"
                "share/h1_2_description/models/incubator/model.sdf",
        }
        for identifier, installed_path in installed.items():
            _require(installed_path.is_file() and _file_sha256(installed_path) ==
                     supplied[identifier][1],
                     f"LAUNCH_CHILD_INSTALLED_BINDING:{identifier}")
        assignments = {}
        for item in args.launch_arguments:
            _require(item.count(":=") == 1, "LAUNCH_CHILD_ARGUMENT")
            key, value = item.split(":=", 1)
            _require(key not in assignments, "LAUNCH_CHILD_ARGUMENT_DUPLICATE")
            assignments[key] = value
        _require(assignments == {
            "spawn_incubator": "true", "table_x": "0.600",
            "cube_x": "0.334", "incubator_x": "0.590",
            "incubator_y": "0.00", "incubator_z": "0.815",
            "bridge_support_states": "false",
        }, "LAUNCH_CHILD_FIXED_ARGUMENTS")
        ros2 = _absolute(args.ros2, "LAUNCH_CHILD_ROS2")
        command = [
            "/usr/bin/python3", str(ros2), "launch", "h1_2_moveit_config",
            "gazebo.launch.py", *args.launch_arguments,
        ]
        interrupted = False

        def handle_sigint(signum: int, frame: Any) -> None:
            del signum, frame
            nonlocal interrupted
            interrupted = True

        previous = signal.signal(signal.SIGINT, handle_sigint)
        try:
            process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=None, stderr=None,
                start_new_session=False, close_fds=True)
            while process.poll() is None:
                time.sleep(0.05)
            returncode = process.wait()
        finally:
            signal.signal(signal.SIGINT, previous)
        _require(returncode == 0,
                 f"LAUNCH_CHILD_ROS2_EXIT:{returncode}:interrupted={interrupted}")
        return 0
    except (KeyError, OSError, SupervisorError) as error:
        print(f"H1_CANDIDATE3_LAUNCH_HELPER_829=FAIL:{error}", file=sys.stderr)
        return 1


def _video_child(args: argparse.Namespace) -> int:
    """Resolve exactly one Gazebo GUI X11 client, then proxy ffmpeg."""
    try:
        source = Path(__file__).resolve()
        _require(args.self_sha256 == _file_sha256(source),
                 "VIDEO_CHILD_SELF_SHA256")
        plan_path = _absolute(args.plan, "VIDEO_CHILD_PLAN")
        plan, plan_sha256 = _read_canonical_sealed(
            plan_path, "VIDEO_CHILD_PLAN")
        _require(plan.get("run_id") == args.run_id,
                 "VIDEO_CHILD_PLAN_RUN_ID")
        tool_binding = plan.get("evidence_tools", {}).get("supervisor_source")
        _require(type(tool_binding) is dict and
                 tool_binding.get("path") == str(source) and
                 tool_binding.get("sha256") == args.self_sha256 and
                 tool_binding.get("mode") == "0500",
                 "VIDEO_CHILD_PLAN_SOURCE_BINDING")
        helper_argv = [sys.executable, *sys.argv]
        _require(_argv_sha256(helper_argv) ==
                 plan.get("expected_process_argv_sha256", {}).get("video"),
                 "VIDEO_CHILD_PLAN_ARGV")
        output = _absolute(args.output, "VIDEO_CHILD_OUTPUT")
        window_audit = _absolute(
            args.window_audit, "VIDEO_CHILD_WINDOW_AUDIT")
        _require(not output.exists() and not output.is_symlink() and
                 not window_audit.exists() and not window_audit.is_symlink(),
                 "VIDEO_CHILD_OUTPUT_EXISTS")
        xwininfo = _absolute(args.xwininfo, "VIDEO_CHILD_XWININFO")
        ffmpeg = _absolute(args.ffmpeg, "VIDEO_CHILD_FFMPEG")
        environment = dict(os.environ)
        environment["DISPLAY"] = args.display
        tree_argv = [str(xwininfo), "-root", "-tree"]
        tree = subprocess.run(
            tree_argv, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
            env=environment)
        _require(tree.returncode == 0 and len(tree.stdout) <= MAX_CAPTURE_BYTES,
                 "VIDEO_CHILD_WINDOW_TREE")
        try:
            lines = tree.stdout.decode("utf-8").splitlines()
        except UnicodeDecodeError as error:
            raise SupervisorError("VIDEO_CHILD_WINDOW_TREE_ENCODING") from error
        matches = []
        for line in lines:
            if '("gz-sim-gui" "Gazebo GUI")' not in line:
                continue
            tokens = line.split()
            _require(tokens and re.fullmatch(r"0x[0-9a-fA-F]+", tokens[0]),
                     "VIDEO_CHILD_WINDOW_TREE_ROW")
            matches.append(tokens[0].lower())
        _require(len(matches) == 1, "VIDEO_CHILD_WINDOW_UNIQUE")
        window_hex = matches[0]
        window_decimal = int(window_hex, 16)
        details_argv = [str(xwininfo), "-id", window_hex]
        details = subprocess.run(
            details_argv, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
            env=environment)
        _require(details.returncode == 0 and
                 len(details.stdout) <= MAX_CAPTURE_BYTES,
                 "VIDEO_CHILD_WINDOW_DETAILS")
        try:
            detail_text = details.stdout.decode("utf-8")
        except UnicodeDecodeError as error:
            raise SupervisorError("VIDEO_CHILD_WINDOW_DETAILS_ENCODING") from error
        width_match = re.search(r"^\s*Width:\s*([1-9][0-9]*)\s*$",
                                detail_text, re.M)
        height_match = re.search(r"^\s*Height:\s*([1-9][0-9]*)\s*$",
                                 detail_text, re.M)
        _require(width_match is not None and height_match is not None,
                 "VIDEO_CHILD_WINDOW_DIMENSIONS")
        width, height = int(width_match.group(1)), int(height_match.group(1))
        _require((width, height) == (2000, 1690),
                 "VIDEO_CHILD_WINDOW_FIXED_DIMENSIONS")
        _require(width % 2 == 0 and height % 2 == 0,
                 "VIDEO_CHILD_WINDOW_DIMENSIONS_EVEN")
        _require(args.framerate == 12 and args.crf == 20 and
                 args.crop == "1160:1530:0:80",
                 "VIDEO_CHILD_ENCODING_OPTIONS")
        crop_width, crop_height, crop_x, crop_y = (
            int(item) for item in args.crop.split(":"))
        _require(crop_width % 2 == 0 and crop_height % 2 == 0 and
                 crop_x >= 0 and crop_y >= 0 and
                 crop_x + crop_width <= width and
                 crop_y + crop_height <= height,
                 "VIDEO_CHILD_CROP_BOUNDS")
        ffmpeg_argv = [
            str(ffmpeg), "-n", "-thread_queue_size", "512", "-f",
            "x11grab", "-window_id", str(window_decimal), "-draw_mouse", "0",
            "-framerate", str(args.framerate), "-video_size",
            f"{width}x{height}", "-i", args.display, "-an", "-vf",
            f"crop={args.crop}", "-c:v", "libx264", "-preset",
            "ultrafast", "-crf", str(args.crf),
            "-pix_fmt", "yuv420p", str(output),
        ]
        capture_started = time.monotonic_ns()
        process = subprocess.Popen(
            ffmpeg_argv, stdin=subprocess.PIPE, stdout=None, stderr=None,
            start_new_session=False, close_fds=True)
        audit = {
            "schema": VIDEO_WINDOW_SCHEMA,
            "run_id": args.run_id,
            "sealed_plan_sha256": plan_sha256,
            "helper_source_sha256": args.self_sha256,
            "helper_process_argv": helper_argv,
            "helper_process_argv_sha256": _argv_sha256(helper_argv),
            "capture_started_monotonic_ns": capture_started,
            "xwindow_id_hex": window_hex,
            "xwindow_id": window_decimal,
            "wm_class": ["gz-sim-gui", "Gazebo GUI"],
            "display": args.display,
            "window_width": width,
            "window_height": height,
            "width": crop_width,
            "height": crop_height,
            "crop": args.crop,
            "window_tree_argv": tree_argv,
            "window_tree_argv_sha256": _argv_sha256(tree_argv),
            "window_details_argv": details_argv,
            "window_details_argv_sha256": _argv_sha256(details_argv),
            "ffmpeg_argv": ffmpeg_argv,
            "ffmpeg_argv_sha256": _argv_sha256(ffmpeg_argv),
            "valid": True,
            "status": "CAPTURE_STARTED",
        }
        _atomic_publish_no_replace(window_audit, _canonical_bytes(audit))
        selector = selectors.DefaultSelector()
        selector.register(sys.stdin.buffer, selectors.EVENT_READ)
        while True:
            returncode = process.poll()
            _require(returncode is None, f"VIDEO_CHILD_FFMPEG_EARLY:{returncode}")
            events = selector.select(timeout=0.1)
            if not events:
                continue
            command = sys.stdin.buffer.readline()
            _require(command == b"q\n", "VIDEO_CHILD_CONTROL")
            _require(process.stdin is not None, "VIDEO_CHILD_FFMPEG_STDIN")
            process.stdin.write(command)
            process.stdin.flush()
            process.stdin.close()
            returncode = process.wait(timeout=60.0)
            _require(returncode == 0, f"VIDEO_CHILD_FFMPEG_EXIT:{returncode}")
            selector.close()
            return 0
    except (OSError, subprocess.TimeoutExpired, SupervisorError) as error:
        print(f"H1_CANDIDATE3_VIDEO_HELPER_829=FAIL:{error}", file=sys.stderr)
        return 1


def _parse_fraction(value: Any, status: str) -> Tuple[int, int]:
    _require(type(value) is str and value.count("/") == 1, status)
    numerator_raw, denominator_raw = value.split("/", 1)
    try:
        numerator, denominator = int(numerator_raw), int(denominator_raw)
    except ValueError as error:
        raise SupervisorError(status) from error
    _require(numerator > 0 and denominator > 0, status)
    return numerator, denominator


def audit_video(
    *, video_path: Path, run_id: str, started_monotonic_ns: int,
    ended_monotonic_ns: int, plan_sha256: str,
    window_audit_path: Path, gui_camera_audit_path: Path,
    ffprobe: Path, ffmpeg: Path,
    run_factory: Callable[..., Any] = subprocess.run,
) -> Mapping[str, Any]:
    video_binding = _binding(video_path, "VIDEO_FILE")
    window, _ = _read_canonical_sealed(
        window_audit_path, "VIDEO_WINDOW_AUDIT")
    camera, _ = _read_canonical_sealed(
        gui_camera_audit_path, "GUI_CAMERA_AUDIT")
    _require(window.get("schema") == VIDEO_WINDOW_SCHEMA and
             camera.get("schema") == GUI_CAMERA_SCHEMA and
             window.get("run_id") == run_id and camera.get("run_id") == run_id and
             window.get("sealed_plan_sha256") == plan_sha256 and
             camera.get("sealed_plan_sha256") == plan_sha256 and
             window.get("valid") is True and camera.get("valid") is True and
             camera.get("status") == "PASS" and
             window.get("status") == "CAPTURE_STARTED",
             "VIDEO_AUXILIARY_EVIDENCE")
    window_started = window.get("capture_started_monotonic_ns")
    _require(type(window_started) is int and
             started_monotonic_ns <= window_started < ended_monotonic_ns and
             camera.get("captured_monotonic_ns", ended_monotonic_ns) <
             window_started, "VIDEO_AUXILIARY_TIMING")
    probe_argv = [
        str(_absolute(ffprobe, "FFPROBE")), "-v", "error",
        "-select_streams", "v:0", "-count_frames", "-show_entries",
        "stream=codec_name,width,height,pix_fmt,avg_frame_rate,"
        "duration,nb_read_frames:format=duration", "-of", "json",
        str(video_path),
    ]
    probe = run_factory(probe_argv, stdin=subprocess.DEVNULL,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                        check=False)
    _require(probe.returncode == 0 and probe.stderr == b"",
             "VIDEO_FFPROBE_EXIT")
    try:
        value = json.loads(probe.stdout.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SupervisorError(f"VIDEO_FFPROBE_JSON:{error}") from error
    streams = value.get("streams") if type(value) is dict else None
    _require(type(streams) is list and len(streams) == 1 and
             type(streams[0]) is dict, "VIDEO_FFPROBE_STREAMS")
    stream = streams[0]
    numerator, denominator = _parse_fraction(
        stream.get("avg_frame_rate"), "VIDEO_FFPROBE_RATE")
    try:
        width = int(stream["width"])
        height = int(stream["height"])
        frame_count = int(stream["nb_read_frames"])
        duration = float(stream.get("duration") or value["format"]["duration"])
    except (KeyError, TypeError, ValueError) as error:
        raise SupervisorError("VIDEO_FFPROBE_FIELDS") from error
    _require(stream.get("codec_name") == "h264" and
             stream.get("pix_fmt") == "yuv420p" and width > 0 and
             height > 0 and numerator == 12 and denominator == 1 and
             frame_count > 0 and math.isfinite(duration) and
             duration > 0.0 and
             abs(frame_count - duration * numerator / denominator) <= 1.0,
             "VIDEO_FFPROBE_CONTRACT")
    _require(window.get("window_width") == 2000 and
             window.get("window_height") == 1690 and
             window.get("crop") == "1160:1530:0:80" and
             window.get("width") == width == 1160 and
             window.get("height") == height == 1530,
             "VIDEO_WINDOW_CROP_BINDING")
    decode_argv = [
        str(_absolute(ffmpeg, "FFMPEG_DECODE")), "-v", "error", "-i",
        str(video_path), "-map", "0:v:0", "-f", "null", "-",
    ]
    decode = run_factory(decode_argv, stdin=subprocess.DEVNULL,
                         stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                         check=False)
    _require(decode.returncode == 0 and decode.stderr == b"",
             "VIDEO_FULL_DECODE")
    return {
        "schema": VIDEO_SCHEMA,
        "run_id": run_id,
        "video": video_binding,
        "capture_started_monotonic_ns": window_started,
        "capture_ended_monotonic_ns": ended_monotonic_ns,
        "ffprobe": {
            "exit_code": probe.returncode,
            "stream_count": 1,
            "codec_name": stream["codec_name"],
            "width": width,
            "height": height,
            "pix_fmt": stream["pix_fmt"],
            "avg_frame_rate_num": numerator,
            "avg_frame_rate_den": denominator,
            "duration_s": duration,
            "frame_count": frame_count,
        },
        "decode": {
            "exit_code": decode.returncode,
            "stderr_size_bytes": len(decode.stderr),
            "stderr_sha256": hashlib.sha256(decode.stderr).hexdigest(),
        },
        "valid": True,
        "status": "PASS",
    }


def build_postflight(
    *, run_id: str, live: Mapping[str, Any], supports_detached: bool,
    unexpected_executor_process_count: int,
) -> Mapping[str, Any]:
    required = {
        "captured_monotonic_ns", "door_angle_rad", "handle_angle_rad",
        "torso_angle_rad", "latch_locked", "robot_at_launch",
        "controller_count", "trajectory_publisher_count", "clock_advancing",
    }
    _require(set(live) == required, "POSTFLIGHT_LIVE_KEYS")
    finite = all(type(live[key]) in (int, float) and
                 math.isfinite(float(live[key])) for key in (
                     "door_angle_rad", "handle_angle_rad", "torso_angle_rad"))
    valid = (
        finite and abs(float(live["door_angle_rad"])) <= 0.004 and
        abs(float(live["handle_angle_rad"])) <= 0.004 and
        abs(float(live["torso_angle_rad"])) <= MAX_ABS_TORSO_RAD and
        live["latch_locked"] is True and supports_detached is True and
        live["robot_at_launch"] is True and live["controller_count"] == 6 and
        live["trajectory_publisher_count"] == 0 and
        unexpected_executor_process_count == 0 and
        live["clock_advancing"] is True)
    _require(valid, "POSTFLIGHT_NOT_PASS")
    return {
        "schema": POSTFLIGHT_SCHEMA,
        "run_id": run_id,
        "captured_monotonic_ns": live["captured_monotonic_ns"],
        "door_angle_rad": float(live["door_angle_rad"]),
        "handle_angle_rad": float(live["handle_angle_rad"]),
        "torso_angle_rad": float(live["torso_angle_rad"]),
        "latch_locked": True,
        "supports_detached": True,
        "robot_at_launch": True,
        "controller_count": 6,
        "trajectory_publisher_count": 0,
        "unexpected_executor_process_count": 0,
        "clock_advancing": True,
        "valid": True,
        "status": "PASS",
    }


def unexpected_executor_process_count(
    *, proc_root: Path = Path("/proc"), excluded_pids: Iterable[int] = (),
) -> int:
    excluded = set(excluded_pids)
    controlled = {
        "h1_candidate3_flow_828.py", "h1_right_handle_executor_828.py",
        "h1_candidate3_middle_executor_828.py",
        "h1_support_orchestrator_828.py",
    }
    count = 0
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) in excluded:
            continue
        try:
            argv = _proc_argv(entry / "cmdline")
        except SupervisorError:
            continue
        tokens = {Path(item).name for item in argv}
        if tokens.intersection(controlled):
            count += 1
    return count


POSE_COMMAND_ARGV = (
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz", "topic", "-e",
    "--json-output", "-t",
    "/world/h1_world/pose/info", "-n", "1",
)
WORKCELL_POSITION_TOLERANCE_M = 0.003
WORKCELL_STATIC_ORIENTATION_TOLERANCE_RAD = 1e-6
WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD = math.radians(2.0)
TERMINAL_CUBE_TARGET = {
    "position_m": [0.611, -0.030, 0.995],
    "orientation_xyzw": [
        0.0, 0.0, 0.7071067811865475, 0.7071067811865476],
    "position_tolerance_m": WORKCELL_POSITION_TOLERANCE_M,
    "orientation_tolerance_rad": WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD,
}


def _json_frames(raw: bytes, status: str) -> list[Mapping[str, Any]]:
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as error:
        raise SupervisorError(f"{status}_ENCODING") from error
    decoder = json.JSONDecoder(parse_constant=lambda token: (
        _ for _ in ()).throw(SupervisorError(
            f"{status}_NONFINITE:{token}")))
    frames = []
    cursor = 0
    while cursor < len(text):
        while cursor < len(text) and text[cursor].isspace():
            cursor += 1
        if cursor == len(text):
            break
        try:
            frame, cursor = decoder.raw_decode(text, cursor)
        except json.JSONDecodeError as error:
            raise SupervisorError(f"{status}_JSON:{error}") from error
        _require(type(frame) is dict, f"{status}_FRAME")
        frames.append(frame)
    _require(len(frames) == 1, f"{status}_FRAME_COUNT")
    return frames


def _pose_vector(value: Any, keys: Sequence[str], status: str) -> list[float]:
    _require(type(value) is dict, f"{status}_TYPE")
    result = []
    for key in keys:
        item = value.get(key, 0.0)
        _require(type(item) in (int, float) and type(item) is not bool and
                 math.isfinite(float(item)), f"{status}_{key}")
        result.append(float(item))
    return result


def _pose_error(
    actual: Mapping[str, Sequence[float]],
    expected: Mapping[str, Sequence[float]],
) -> Tuple[float, float]:
    position_error = math.sqrt(sum(
        (actual["position_m"][index] - expected["position_m"][index]) ** 2
        for index in range(3)))
    left = actual["orientation_xyzw"]
    right = expected["orientation_xyzw"]
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    _require(abs(left_norm - 1.0) <= 1e-6 and
             abs(right_norm - 1.0) <= 1e-12, "POSE_QUATERNION_NORM")
    dot = abs(sum(left[index] * right[index]
                  for index in range(4)) / (left_norm * right_norm))
    dot = min(1.0, max(0.0, dot))
    orientation_error = 2.0 * math.atan2(
        math.sqrt(max(0.0, 1.0 - dot * dot)), dot)
    return position_error, orientation_error


def _extract_entities(
    frame: Mapping[str, Any], names: Sequence[str], status: str,
) -> Mapping[str, Any]:
    poses = frame.get("pose")
    _require(type(poses) is list and all(type(item) is dict for item in poses),
             f"{status}_POSES")
    result = {}
    for name in names:
        matches = [item for item in poses if item.get("name") == name]
        _require(len(matches) == 1, f"{status}_ENTITY:{name}")
        entity = matches[0]
        raw_id = entity.get("id")
        try:
            entity_id = int(raw_id)
        except (TypeError, ValueError) as error:
            raise SupervisorError(f"{status}_ENTITY_ID:{name}") from error
        _require(entity_id > 0 and str(entity_id) == str(raw_id),
                 f"{status}_ENTITY_ID:{name}")
        result[name] = {
            "entity_id": entity_id,
            "position_m": _pose_vector(
                entity.get("position", {}), ("x", "y", "z"),
                f"{status}_POSITION:{name}"),
            "orientation_xyzw": _pose_vector(
                entity.get("orientation", {}), ("x", "y", "z", "w"),
                f"{status}_ORIENTATION:{name}"),
        }
    return result


def sample_workcell_poses(
    *, phase: str, run_factory: Callable[..., Any] = subprocess.run,
) -> list[Mapping[str, Any]]:
    _require(phase in ("preflight", "postflight"), "POSE_PHASE")
    names = ("h1_2", "table", "incubator", "green_cube")
    samples = []
    for sequence in range(3):
        completed = run_factory(
            list(POSE_COMMAND_ARGV), stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        _require(completed.returncode == 0 and
                 type(completed.stdout) is bytes and
                 len(completed.stdout) <= MAX_CAPTURE_BYTES and
                 type(completed.stderr) is bytes and
                 len(completed.stderr) <= MAX_CAPTURE_BYTES,
                 f"POSE_COMMAND:{phase}:{sequence}")
        frame = _json_frames(
            completed.stdout, f"POSE_{phase.upper()}_{sequence}")[0]
        now = time.monotonic_ns()
        if samples:
            _require(now > samples[-1]["monotonic_ns"],
                     f"POSE_MONOTONIC:{phase}")
        samples.append({
            "sequence": sequence,
            "monotonic_ns": now,
            "entities": _extract_entities(
                frame, names, f"POSE_{phase.upper()}_{sequence}"),
        })
        if sequence != 2:
            time.sleep(0.02)
    return samples


def validate_workcell_pose_samples(
    preflight: Sequence[Mapping[str, Any]],
    postflight: Sequence[Mapping[str, Any]],
) -> list[Mapping[str, Any]]:
    _require(len(preflight) == 3 and len(postflight) == 3,
             "POSE_SAMPLE_COUNT")
    root_half = math.sqrt(0.5)
    expected = {
        "h1_2": {
            "position_m": [0.0, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "table": {
            "position_m": [0.600, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "incubator": {
            "position_m": [0.590, 0.0, 0.815],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "green_cube": {
            "position_m": [0.334, 0.377, 0.830],
            "orientation_xyzw": [0.0, 0.0, root_half, root_half]},
    }
    for sequence, sample in enumerate(preflight):
        _require(sample.get("sequence") == sequence,
                 "POSE_PREFLIGHT_SEQUENCE")
        _require(tuple(sample["entities"]) == tuple(expected),
                 "POSE_PREFLIGHT_ENTITY_ORDER")
        for name, target in expected.items():
            position, orientation = _pose_error(sample["entities"][name], target)
            orientation_limit = (WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD
                                 if name == "green_cube" else 1e-6)
            _require(position <= WORKCELL_POSITION_TOLERANCE_M and
                     orientation <= orientation_limit,
                     f"POSE_PREFLIGHT_GATE:{name}")
    first = preflight[0]["entities"]
    names = tuple(expected)
    for sample in preflight[1:]:
        _require(tuple(sample["entities"]) == names,
                 "POSE_PREFLIGHT_ENTITY_ORDER")
        for name in names:
            _require(sample["entities"][name]["entity_id"] ==
                     first[name]["entity_id"], f"POSE_ENTITY_ID:{name}")
            position, orientation = _pose_error(
                sample["entities"][name], first[name])
            orientation_limit = (WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD
                                 if name == "green_cube" else
                                 WORKCELL_STATIC_ORIENTATION_TOLERANCE_RAD)
            _require(position <= WORKCELL_POSITION_TOLERANCE_M and
                     orientation <= orientation_limit,
                     f"POSE_STABILITY:{name}")

    terminal_target = {
        "position_m": TERMINAL_CUBE_TARGET["position_m"],
        "orientation_xyzw": TERMINAL_CUBE_TARGET["orientation_xyzw"],
    }
    terminal_gates = []
    for sequence, sample in enumerate(postflight):
        _require(sample.get("sequence") == sequence,
                 "POSE_POSTFLIGHT_SEQUENCE")
        _require(tuple(sample["entities"]) == names,
                 "POSE_POSTFLIGHT_ENTITY_ORDER")
        for name in names:
            _require(sample["entities"][name]["entity_id"] ==
                     first[name]["entity_id"], f"POSE_ENTITY_ID:{name}")
        for name in ("h1_2", "table", "incubator"):
            position, orientation = _pose_error(
                sample["entities"][name], first[name])
            _require(position <= WORKCELL_POSITION_TOLERANCE_M and
                     orientation <=
                     WORKCELL_STATIC_ORIENTATION_TOLERANCE_RAD,
                     f"POSE_STABILITY:{name}")
        cube_position_error, cube_orientation_error = _pose_error(
            sample["entities"]["green_cube"], terminal_target)
        position_passed = (
            cube_position_error <= WORKCELL_POSITION_TOLERANCE_M)
        orientation_passed = (
            cube_orientation_error <=
            WORKCELL_CUBE_ORIENTATION_TOLERANCE_RAD)
        _require(position_passed and orientation_passed,
                 "POSE_POSTFLIGHT_GATE:green_cube")
        terminal_gates.append({
            "sequence": sequence,
            "position_error_m": cube_position_error,
            "orientation_error_rad": cube_orientation_error,
            "position_gate_passed": position_passed,
            "orientation_gate_passed": orientation_passed,
            "passed": position_passed and orientation_passed,
        })
    return terminal_gates


def build_workcell_pose_audit(
    *, run_id: str, plan_sha256: str,
    preflight: Sequence[Mapping[str, Any]],
    postflight: Sequence[Mapping[str, Any]],
) -> Mapping[str, Any]:
    terminal_gates = validate_workcell_pose_samples(preflight, postflight)
    return {
        "schema": WORKCELL_POSE_SCHEMA,
        "run_id": run_id,
        "sealed_plan_sha256": plan_sha256,
        "world_name": "h1_world",
        "source_command_argv": list(POSE_COMMAND_ARGV),
        "source_command_argv_sha256": _argv_sha256(POSE_COMMAND_ARGV),
        "preflight_samples": list(preflight),
        "postflight_samples": list(postflight),
        "terminal_cube_target": {
            "position_m": list(TERMINAL_CUBE_TARGET["position_m"]),
            "orientation_xyzw": list(
                TERMINAL_CUBE_TARGET["orientation_xyzw"]),
            "position_tolerance_m":
                TERMINAL_CUBE_TARGET["position_tolerance_m"],
            "orientation_tolerance_rad":
                TERMINAL_CUBE_TARGET["orientation_tolerance_rad"],
        },
        "terminal_cube_sample_gates": terminal_gates,
        "terminal_cube_gate_passed": True,
        "valid": True,
        "status": "PASS",
    }


def _fcl_machine_rows(
    stdout: bytes, telemetry: Any, certificate: ModuleType,
) -> Tuple[list[Mapping[str, str]], Mapping[str, str]]:
    try:
        text = stdout.decode("ascii")
    except UnicodeDecodeError as error:
        raise SupervisorError("FCL_STDOUT_ENCODING") from error
    _require(text.endswith("\n") and "\r" not in text and "\0" not in text,
             "FCL_STDOUT_FRAMING")
    lines = text.splitlines()
    _require(len(lines) == len(telemetry.rows) + 1,
             "FCL_STDOUT_LINE_COUNT")
    try:
        rows = [certificate._parse_machine_line(
            line, "RIGHT_HANDLE_LIVE_FCL_ROW_829", certificate.FCL_ROW_KEYS,
            f"SUPERVISOR_FCL_ROW_{index}")
            for index, line in enumerate(lines[:-1])]
        summary = certificate._parse_machine_line(
            lines[-1], "RIGHT_HANDLE_LIVE_FCL_SUMMARY_829",
            certificate.FCL_SUMMARY_KEYS, "SUPERVISOR_FCL_SUMMARY")
    except Exception as error:
        raise SupervisorError(f"FCL_MACHINE_PARSE:{error}") from error
    return rows, summary


def execute_exact_fcl_replay(
    *, paths: EvidencePaths, bundle: CommandBundle,
    plan: Mapping[str, Any], run_factory: Callable[..., Any] = subprocess.run,
) -> Mapping[str, Any]:
    """Derive the exact TSV, execute the planned C++ auditor, seal wrapper."""
    certificate = _certificate_module()
    try:
        bag_evidence = certificate._read_sealed_file(
            paths.bag, maximum_bytes=certificate.MAX_DB_BYTES,
            status="SUPERVISOR_FCL_BAG")
        _, markers = certificate._load_markers(paths.markers, bundle.run_id)
        _, telemetry = certificate._analyze_rosbag(
            bag_evidence, markers, certificate.RclpyCdrDeserializer())
    except Exception as error:
        raise SupervisorError(f"FCL_TELEMETRY_DERIVATION:{error}") from error
    _atomic_publish_no_replace(paths.exact_fcl_telemetry, telemetry.raw)
    completed = run_factory(
        list(bundle.exact_fcl_argv), stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    _require(type(completed.stdout) is bytes and
             type(completed.stderr) is bytes and
             len(completed.stdout) <= MAX_CAPTURE_BYTES and
             len(completed.stderr) <= MAX_CAPTURE_BYTES,
             "FCL_OUTPUT_BUDGET")
    stdout_sha = _atomic_publish_no_replace(
        paths.exact_fcl_stdout, completed.stdout)
    _atomic_publish_no_replace(paths.exact_fcl_stderr, completed.stderr)
    _require(completed.returncode == 0 and completed.stderr == b"",
             f"FCL_EXIT:{completed.returncode}")
    rows, _ = _fcl_machine_rows(completed.stdout, telemetry, certificate)
    try:
        pad1_valid = [certificate._machine_bool(
            row["pad1_patch_valid"], f"SUPERVISOR_FCL_PAD1_{index}")
            for index, row in enumerate(rows)]
        pad2_valid = [certificate._machine_bool(
            row["pad2_patch_valid"], f"SUPERVISOR_FCL_PAD2_{index}")
            for index, row in enumerate(rows)]
        nonpad = [certificate._machine_float(
            row["nonpad_handle_clearance_m"],
            f"SUPERVISOR_FCL_NONPAD_{index}")
            for index, row in enumerate(rows)]
        left = [certificate._machine_float(
            row["left_handle_clearance_m"], f"SUPERVISOR_FCL_LEFT_{index}")
            for index, row in enumerate(rows)]
    except Exception as error:
        raise SupervisorError(f"FCL_METRIC_PARSE:{error}") from error
    required_count = sum(
        bool(row["contact_required"]) for row in telemetry.rows)
    pad1_count = sum(pad1_valid)
    pad2_count = sum(pad2_valid)
    observed_pairs = []
    if pad1_count:
        observed_pairs.append(list(certificate.ALLOWED_HANDLE_CONTACT_PAIRS[0]))
    if pad2_count:
        observed_pairs.append(list(certificate.ALLOWED_HANDLE_CONTACT_PAIRS[1]))
    geometry = {
        "contact_path": _binding(
            bundle.exact_fcl_files["contact_path"], "FCL_CONTACT_PATH"),
        "original_handle_stl": _binding(
            bundle.exact_fcl_files["original_handle_stl"], "FCL_HANDLE_STL"),
        "scene_cdr": _binding(
            bundle.exact_fcl_files["scene_cdr"], "FCL_SCENE"),
        "urdf": _binding(bundle.exact_fcl_files["urdf"], "FCL_URDF"),
        "srdf": _binding(bundle.exact_fcl_files["srdf"], "FCL_SRDF"),
    }
    replay = {
        "schema": EXACT_FCL_REPLAY_SCHEMA,
        "run_id": bundle.run_id,
        "scope": certificate._exact_fcl_scope(),
        "rosbag": _binding(paths.bag, "FCL_BAG_BINDING"),
        "telemetry_tsv": _binding(
            paths.exact_fcl_telemetry, "FCL_TELEMETRY_BINDING"),
        "geometry": geometry,
        "auditor": {
            "source": _binding(
                bundle.exact_fcl_files["auditor_source"],
                "FCL_AUDITOR_SOURCE"),
            "path_base_source": _binding(
                bundle.exact_fcl_files["auditor_path_base_source"],
                "FCL_AUDITOR_PATH_BASE_SOURCE"),
            "middle_base_source": _binding(
                bundle.exact_fcl_files["auditor_middle_base_source"],
                "FCL_AUDITOR_MIDDLE_BASE_SOURCE"),
            "binary": _binding(
                bundle.exact_fcl_files["auditor_binary"],
                "FCL_AUDITOR_BINARY"),
            "argv": list(bundle.exact_fcl_argv),
            "argv_sha256": _argv_sha256(bundle.exact_fcl_argv),
            "exit_code": completed.returncode,
            "stdout": _binding(paths.exact_fcl_stdout, "FCL_STDOUT"),
            "stderr": _binding(paths.exact_fcl_stderr, "FCL_STDERR"),
        },
        "replay": {
            "source_joint_state_samples": telemetry.source_sample_count,
            "densified_sample_count": len(telemetry.rows),
            "maximum_source_gap_ns": telemetry.maximum_source_gap_ns,
            "maximum_revolute_step_rad": telemetry.maximum_revolute_step_rad,
            "maximum_linear_step_m": telemetry.maximum_linear_step_m,
            "close_command_timestamp_ns":
                telemetry.close_command_timestamp_ns,
            "close_pass_timestamp_ns": telemetry.close_pass_timestamp_ns,
            "release_command_timestamp_ns":
                telemetry.release_command_timestamp_ns,
            "release_pass_timestamp_ns": telemetry.release_pass_timestamp_ns,
            "excluded_close_transition_rows":
                telemetry.excluded_close_transition_rows,
            "excluded_release_transition_rows":
                telemetry.excluded_release_transition_rows,
            "collision_free": True,
            "left_handle_checks": len(telemetry.rows),
            "left_handle_contact_count": 0,
            "minimum_left_handle_clearance_m": min(left),
            "right_contact_required_samples": required_count,
            "right_patch_valid_samples": required_count,
            "right_pad_1_patch_valid_samples": pad1_count,
            "right_pad_2_patch_valid_samples": pad2_count,
            "right_nonpad_handle_contact_count": 0,
            "minimum_right_nonpad_handle_clearance_m": min(nonpad),
            "allowed_handle_contact_pairs": [
                list(pair) for pair in certificate.ALLOWED_HANDLE_CONTACT_PAIRS],
            "observed_handle_contact_pairs": observed_pairs,
            "unauthorized_handle_contact_pairs": [],
            "replay_transcript_sha256": stdout_sha,
        },
        "native_physics_contacts": {
            "backend": "DART",
            "available": False,
            "reason": certificate.NATIVE_CONTACT_UNAVAILABLE_REASON,
            "force_or_wrench_claimed": False,
        },
        "offline_replay": True,
        "live_telemetry_bound": True,
        "valid": True,
        "status": "PASS",
    }
    _atomic_publish_no_replace(paths.exact_fcl_replay,
                               _canonical_bytes(replay))
    try:
        evidence, loaded = certificate._load_canonical_json(
            paths.exact_fcl_replay, status="SUPERVISOR_FCL_REPLAY")
        certificate._validate_exact_fcl_replay(
            loaded, evidence, plan, bag_evidence, telemetry)
    except Exception as error:
        raise SupervisorError(f"FCL_REPLAY_VALIDATION:{error}") from error
    return replay


def _boot_id_sha256() -> str:
    try:
        raw = Path("/proc/sys/kernel/random/boot_id").read_bytes()
    except OSError as error:
        raise SupervisorError(f"BOOT_ID:{error}") from error
    _require(raw.endswith(b"\n") and len(raw) == 37, "BOOT_ID_FORMAT")
    return hashlib.sha256(raw).hexdigest()


def build_proc_snapshot(
    *, run_id: str, plan_sha256: str, preexisting: Sequence[Any],
    children: Sequence[DirectChild], supervisor_start_ticks: int,
    recorder_ready: Mapping[str, Any],
) -> Mapping[str, Any]:
    _require(preexisting == [] and
             tuple(child.role for child in children) == PROCESS_ROLES,
             "PROC_SNAPSHOT_ORDER")
    rows = [child.record(run_id) for child in children]
    _require(all(row["exit_code"] == 0 for row in rows),
             "PROC_SNAPSHOT_EXIT")
    return {
        "schema": PROC_SCHEMA,
        "run_id": run_id,
        "boot_id_sha256": _boot_id_sha256(),
        "supervisor_pid": os.getpid(),
        "supervisor_start_ticks": supervisor_start_ticks,
        "sealed_plan_sha256": plan_sha256,
        "preexisting_processes": [],
        "recorder_ready": recorder_ready,
        "captured_monotonic_ns": time.monotonic_ns(),
        "processes": rows,
        "valid": True,
        "status": "PASS",
    }


def _load_prepared(
    *, run_root: Path, plan_sha256: str, commands_path: Path,
    commands_sha256: str,
) -> Tuple[EvidencePaths, Mapping[str, Any], CommandBundle]:
    global _BOUND_CERTIFICATE_SOURCE, _CERTIFICATE_MODULE
    _require(SHA256_RE.fullmatch(plan_sha256) is not None and
             SHA256_RE.fullmatch(commands_sha256) is not None,
             "PREPARED_SHA256")
    paths = EvidencePaths.from_root(run_root)
    plan, actual_plan_sha = _read_canonical_sealed(paths.plan, "PLAN")
    _require(actual_plan_sha == plan_sha256, "PLAN_SHA256")
    bundle = load_commands(_absolute(commands_path, "COMMANDS_PATH"), paths)
    _require(bundle.raw_sha256 == commands_sha256, "COMMANDS_SHA256")
    _require(plan.get("run_id") == bundle.run_id and
             plan.get("input_paths") == paths.input_paths(),
             "PREPARED_IDENTITY")
    _require(plan.get("expected_process_argv_sha256") == {
        role: _argv_sha256(bundle.processes[role]) for role in PROCESS_ROLES
    }, "PREPARED_PROCESS_ARGV")
    _require(plan.get("expected_exact_fcl_replay_argv_sha256") ==
             _argv_sha256(bundle.exact_fcl_argv), "PREPARED_FCL_ARGV")
    _require(plan.get("expected_exact_fcl_file_sha256") == {
        identifier: _file_sha256(bundle.exact_fcl_files[identifier])
        for identifier in EXACT_FCL_FILE_IDS
    }, "PREPARED_FCL_FILES")
    tool_values = plan.get("evidence_tools")
    _require(type(tool_values) is dict and set(tool_values) ==
             set(EVIDENCE_TOOL_IDS), "PREPARED_EVIDENCE_TOOLS")
    for identifier in EVIDENCE_TOOL_IDS:
        expected = tool_values[identifier]
        _require(type(expected) is dict and set(expected) == {
            "path", "sha256", "size_bytes", "mode"},
            f"PREPARED_EVIDENCE_TOOL_SCHEMA:{identifier}")
        actual = _binding(
            _absolute(expected["path"], f"PREPARED_TOOL_{identifier}"),
            f"PREPARED_TOOL_{identifier}")
        _require(actual == expected and actual["mode"] == "0500",
                 f"PREPARED_EVIDENCE_TOOL_BINDING:{identifier}")
    _require(Path(__file__).resolve() == Path(
        tool_values["supervisor_source"]["path"]),
        "PREPARED_SUPERVISOR_NOT_BOUND_COPY")
    certificate_path = Path(tool_values["certificate_source"]["path"])
    if _BOUND_CERTIFICATE_SOURCE != certificate_path:
        _BOUND_CERTIFICATE_SOURCE = certificate_path
        _CERTIFICATE_MODULE = None
    formal, formal_sha = _read_canonical_sealed(paths.formal, "FORMAL")
    _require(formal_sha == plan.get("formal_authorities_sha256") and
             formal.get("run_id") == bundle.run_id,
             "PREPARED_FORMAL")
    certificate = _certificate_module()
    try:
        certificate._validate_plan(plan)
        certificate._validate_formal_authorities(formal, bundle.run_id)
    except Exception as error:
        raise SupervisorError(f"PREPARED_CERTIFICATE_SCHEMA:{error}") from error
    return paths, plan, bundle


def run_capture(
    *, run_root: Path, plan_sha256: str, commands_path: Path,
    commands_sha256: str, execute: bool,
    startup_timeout_s: float = 180.0,
    prefix_timeout_s: float = 7200.0,
    middle_timeout_s: float = 7200.0,
    postflight_timeout_s: float = 600.0,
    shutdown_timeout_s: float = 60.0,
    ffprobe: Path = Path("/usr/bin/ffprobe"),
    ffmpeg: Path = Path("/usr/bin/ffmpeg"),
    adapter_factory: Callable[[str, str], Any] = RclpyLiveAdapter,
    popen_factory: Callable[..., Any] = subprocess.Popen,
    proc_identity_reader: Callable[..., Mapping[str, Any]] = _read_proc_identity,
    preexisting_auditor: Callable[..., list[Mapping[str, Any]]] =
        audit_preexisting_processes,
    run_factory: Callable[..., Any] = subprocess.run,
    exact_fcl_runner: Callable[..., Mapping[str, Any]] =
        execute_exact_fcl_replay,
) -> Mapping[str, Any]:
    """Validate a prepared run and, only when authorized, execute it."""
    for value, name in (
        (startup_timeout_s, "STARTUP_TIMEOUT"),
        (prefix_timeout_s, "PREFIX_TIMEOUT"),
        (middle_timeout_s, "MIDDLE_TIMEOUT"),
        (postflight_timeout_s, "POSTFLIGHT_TIMEOUT"),
        (shutdown_timeout_s, "SHUTDOWN_TIMEOUT"),
    ):
        _require(math.isfinite(value) and value > 0.0, name)
    paths, plan, bundle = _load_prepared(
        run_root=run_root, plan_sha256=plan_sha256,
        commands_path=commands_path, commands_sha256=commands_sha256)
    if not execute:
        return {
            "run_id": bundle.run_id,
            "plan_sha256": plan_sha256,
            "commands_sha256": commands_sha256,
            "external_execution": False,
            "valid": True,
            "status": "READY_NO_EXECUTION",
        }

    _require(paths.logs.is_dir() and stat.S_IMODE(
        paths.logs.stat().st_mode) == 0o700, "LOG_DIRECTORY")
    forbidden_outputs = (
        paths.bag_directory, paths.markers, paths.proc_snapshot, paths.video,
        paths.video_audit, paths.postflight, paths.exact_fcl_replay,
        paths.exact_fcl_telemetry, paths.exact_fcl_stdout,
        paths.exact_fcl_stderr, paths.workcell_pose_audit,
        paths.video_window_audit, paths.gui_camera_audit,
    )
    _require(all(not path.exists() and not path.is_symlink()
                 for path in forbidden_outputs), "RUN_OUTPUT_EXISTS")
    _require(not any(paths.logs.iterdir()), "RUN_LOGS_NOT_EMPTY")
    supervisor_ppid, supervisor_start_ticks = _proc_stat_fields(
        Path("/proc/self/stat"))
    del supervisor_ppid
    preexisting = preexisting_auditor(excluded_pids=(os.getpid(),))
    _require(preexisting == [], "PREEXISTING_PROCESS_AUDIT")

    children: list[DirectChild] = []
    adapter = None
    postflight_value: Mapping[str, Any] | None = None
    recorder_ready: Mapping[str, Any] | None = None
    preflight_pose_samples: Sequence[Mapping[str, Any]] | None = None
    postflight_pose_samples: Sequence[Mapping[str, Any]] | None = None
    try:
        launch = _spawn_child(
            "launch", bundle.processes["launch"], paths, foreground=False,
            popen_factory=popen_factory,
            proc_identity_reader=proc_identity_reader)
        children.append(launch)
        adapter = adapter_factory(bundle.run_id, bundle.marker_topic)
        adapter.wait_launch_ready(startup_timeout_s)
        set_gui_camera_pose(
            run_id=bundle.run_id, plan_sha256=plan_sha256,
            output_path=paths.gui_camera_audit,
            run_factory=run_factory)

        recorder = _spawn_child(
            "recorder", bundle.processes["recorder"], paths,
            foreground=False, popen_factory=popen_factory,
            proc_identity_reader=proc_identity_reader)
        children.append(recorder)
        recorder_ready = adapter.wait_for_recorder(startup_timeout_s)
        _require(set(recorder_ready) == {
            "captured_monotonic_ns", "subscription_topics"} and
            recorder_ready["subscription_topics"] == sorted((
                *REQUIRED_BAG_TOPICS, MARKER_TOPIC)),
            "RECORDER_READY_SCHEMA")
        _require(recorder.process.poll() is None, "RECORDER_EARLY_EXIT")

        video = _spawn_child(
            "video", bundle.processes["video"], paths, foreground=False,
            popen_factory=popen_factory,
            proc_identity_reader=proc_identity_reader)
        children.append(video)
        video_deadline = time.monotonic() + startup_timeout_s
        while not paths.video_window_audit.exists():
            _require(video.process.poll() is None, "VIDEO_EARLY_EXIT")
            _require(time.monotonic() < video_deadline,
                     "VIDEO_WINDOW_AUDIT_TIMEOUT")
            time.sleep(0.02)
        _open_regular(paths.video_window_audit, 0o400,
                      "VIDEO_WINDOW_AUDIT_READY")
        _require(video.process.poll() is None, "VIDEO_EARLY_EXIT")

        certificate = _certificate_module()
        sequencer = MarkerSequencer(
            bundle.run_id, certificate.MARKER_SPECS, adapter)
        sequencer.supervisor(
            "CAPTURE_START", "H1_CANDIDATE3_LIVE_829=START")
        preflight_pose_samples = sample_workcell_poses(
            phase="preflight", run_factory=run_factory)

        prefix = _spawn_child(
            "prefix", bundle.processes["prefix"], paths, foreground=True,
            popen_factory=popen_factory,
            proc_identity_reader=proc_identity_reader)
        children.append(prefix)
        _wait_foreground_child(
            prefix, sequencer, timeout_s=prefix_timeout_s)

        middle = _spawn_child(
            "middle_suffix", bundle.processes["middle_suffix"], paths,
            foreground=True, popen_factory=popen_factory,
            proc_identity_reader=proc_identity_reader)
        children.append(middle)
        _wait_foreground_child(
            middle, sequencer, timeout_s=middle_timeout_s)

        postflight = _spawn_child(
            "postflight", bundle.processes["postflight"], paths,
            foreground=True, popen_factory=popen_factory,
            proc_identity_reader=proc_identity_reader)
        children.append(postflight)
        _wait_foreground_child(
            postflight, sequencer, timeout_s=postflight_timeout_s)
        _require(sequencer.payloads[-1]["name"] == "POSTFLIGHT_FINALIZE",
                 "POSTFLIGHT_MARKER_MISSING")
        postflight_pose_samples = sample_workcell_poses(
            phase="postflight", run_factory=run_factory)
        live_postflight = adapter.capture_postflight(
            after_monotonic_ns=sequencer.payloads[-1]["monotonic_ns"],
            timeout_s=postflight_timeout_s)
        unexpected = unexpected_executor_process_count(
            excluded_pids=(os.getpid(), *(child.process.pid
                                          for child in children)))
        postflight_value = build_postflight(
            run_id=bundle.run_id, live=live_postflight,
            supports_detached=True,
            unexpected_executor_process_count=unexpected)
        sequencer.supervisor(
            "CAPTURE_END", "H1_CANDIDATE3_LIVE_829=END")
        sequencer.complete()
        time.sleep(0.25)

        _stop_background_child(
            recorder, method="sigint", timeout_s=shutdown_timeout_s)
        _stop_background_child(
            video, method="video", timeout_s=shutdown_timeout_s)
        _stop_background_child(
            launch, method="sigint", timeout_s=shutdown_timeout_s)

        _require(tuple(child.role for child in children) == PROCESS_ROLES,
                 "CHILD_ROLE_ORDER")
        for child in children:
            _seal_child_logs(child, paths)
        _require(paths.bag_directory.is_dir() and
                 len(list(paths.bag_directory.glob("*.db3"))) == 1 and
                 paths.bag.exists(), "ROSBAG_DB3_PATH")
        os.chmod(paths.bag_directory, 0o700)
        _seal_existing(paths.bag, "ROSBAG")
        for suffix in ("-wal", "-shm", "-journal"):
            sidecar = Path(str(paths.bag) + suffix)
            _require(not sidecar.exists() and not sidecar.is_symlink(),
                     f"ROSBAG_SIDECAR:{suffix}")
        metadata_path = paths.bag_directory / "metadata.yaml"
        if metadata_path.exists():
            _seal_existing(metadata_path, "ROSBAG_METADATA")
        _seal_existing(paths.video, "VIDEO")

        marker_raw = extract_marker_jsonl(
            bag_path=paths.bag, marker_topic=bundle.marker_topic,
            expected_payloads=sequencer.payloads, decoder=adapter,
            marker_schema=certificate.MARKER_SCHEMA)
        _atomic_publish_no_replace(paths.markers, marker_raw)
        _require(postflight_value is not None, "POSTFLIGHT_VALUE")
        _atomic_publish_no_replace(
            paths.postflight, _canonical_bytes(postflight_value))
        _require(preflight_pose_samples is not None and
                 postflight_pose_samples is not None,
                 "WORKCELL_POSE_SAMPLES")
        pose_audit = build_workcell_pose_audit(
            run_id=bundle.run_id, plan_sha256=plan_sha256,
            preflight=preflight_pose_samples,
            postflight=postflight_pose_samples)
        _atomic_publish_no_replace(
            paths.workcell_pose_audit, _canonical_bytes(pose_audit))
        _require(recorder_ready is not None, "RECORDER_READY_VALUE")
        proc_value = build_proc_snapshot(
            run_id=bundle.run_id, plan_sha256=plan_sha256,
            preexisting=preexisting, children=children,
            supervisor_start_ticks=supervisor_start_ticks,
            recorder_ready=recorder_ready)
        _atomic_publish_no_replace(
            paths.proc_snapshot, _canonical_bytes(proc_value))
        video_value = audit_video(
            video_path=paths.video, run_id=bundle.run_id,
            started_monotonic_ns=video.started_monotonic_ns,
            ended_monotonic_ns=video.ended_monotonic_ns,
            plan_sha256=plan_sha256,
            window_audit_path=paths.video_window_audit,
            gui_camera_audit_path=paths.gui_camera_audit,
            ffprobe=ffprobe, ffmpeg=ffmpeg, run_factory=run_factory)
        _atomic_publish_no_replace(
            paths.video_audit, _canonical_bytes(video_value))
        exact_fcl_runner(
            paths=paths, bundle=bundle, plan=plan,
            run_factory=run_factory)
        return {
            "run_id": bundle.run_id,
            "plan_sha256": plan_sha256,
            "commands_sha256": commands_sha256,
            "rosbag_sha256": _file_sha256(paths.bag),
            "markers_sha256": _file_sha256(paths.markers),
            "proc_snapshot_sha256": _file_sha256(paths.proc_snapshot),
            "video_sha256": _file_sha256(paths.video),
            "video_audit_sha256": _file_sha256(paths.video_audit),
            "gui_camera_audit_sha256":
                _file_sha256(paths.gui_camera_audit),
            "video_window_audit_sha256":
                _file_sha256(paths.video_window_audit),
            "postflight_sha256": _file_sha256(paths.postflight),
            "workcell_pose_audit_sha256":
                _file_sha256(paths.workcell_pose_audit),
            "exact_fcl_replay_sha256": _file_sha256(paths.exact_fcl_replay),
            "external_execution": True,
            "valid": True,
            "status": "CAPTURE_PASS",
        }
    except BaseException:
        _emergency_stop(children)
        for child in children:
            for staged in (child.stdout_staged, child.stderr_staged):
                if staged.exists() and not staged.is_symlink():
                    try:
                        os.chmod(staged, 0o400)
                    except OSError:
                        pass
        raise
    finally:
        if adapter is not None:
            adapter.close()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)
    prepare_parser = commands.add_parser("prepare")
    prepare_parser.add_argument("--run-root", type=Path, required=True)
    prepare_parser.add_argument("--commands", type=Path, required=True)
    prepare_parser.add_argument(
        "--authority", action="append", default=[],
        metavar="ID=/ABSOLUTE/PATH")
    prepare_parser.add_argument(
        "--evidence-tool", action="append", default=[],
        metavar="ID=/ABSOLUTE/SEALED_SOURCE")
    run_parser = commands.add_parser("run")
    run_parser.add_argument("--run-root", type=Path, required=True)
    run_parser.add_argument("--plan-sha256", required=True)
    run_parser.add_argument("--commands", type=Path, required=True)
    run_parser.add_argument("--commands-sha256", required=True)
    run_parser.add_argument("--startup-timeout", type=float, default=180.0)
    run_parser.add_argument("--prefix-timeout", type=float, default=7200.0)
    run_parser.add_argument("--middle-timeout", type=float, default=7200.0)
    run_parser.add_argument("--postflight-timeout", type=float, default=600.0)
    run_parser.add_argument("--shutdown-timeout", type=float, default=60.0)
    run_parser.add_argument(
        "--ffprobe", type=Path, default=Path("/usr/bin/ffprobe"))
    run_parser.add_argument(
        "--ffmpeg", type=Path, default=Path("/usr/bin/ffmpeg"))
    run_parser.add_argument("--execute", action="store_true")
    video_child = commands.add_parser("_video-child", help=argparse.SUPPRESS)
    video_child.add_argument("--self-sha256", required=True)
    video_child.add_argument("--run-id", required=True)
    video_child.add_argument("--plan", type=Path, required=True)
    video_child.add_argument("--output", type=Path, required=True)
    video_child.add_argument("--window-audit", type=Path, required=True)
    video_child.add_argument("--xwininfo", type=Path, required=True)
    video_child.add_argument("--ffmpeg", type=Path, required=True)
    video_child.add_argument("--display", required=True)
    video_child.add_argument("--crop", default="1160:1530:0:80")
    video_child.add_argument("--framerate", type=int, default=12)
    video_child.add_argument("--crf", type=int, default=20)
    launch_child = commands.add_parser("_launch-child", help=argparse.SUPPRESS)
    launch_child.add_argument("--self-sha256", required=True)
    launch_child.add_argument("--plan", type=Path, required=True)
    launch_child.add_argument("--workspace", type=Path, required=True)
    launch_child.add_argument("--ros2", type=Path, required=True)
    launch_child.add_argument("--gazebo-launch-source", type=Path, required=True)
    launch_child.add_argument("--gazebo-launch-source-sha256", required=True)
    launch_child.add_argument("--gazebo-world-sdf", type=Path, required=True)
    launch_child.add_argument("--gazebo-world-sdf-sha256", required=True)
    launch_child.add_argument("--incubator-model-sdf", type=Path, required=True)
    launch_child.add_argument("--incubator-model-sdf-sha256", required=True)
    launch_child.add_argument("launch_arguments", nargs="+")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.operation == "_video-child":
        return _video_child(args)
    if args.operation == "_launch-child":
        return _launch_child(args)
    try:
        if args.operation == "prepare":
            result = prepare(
                run_root=args.run_root, commands_path=args.commands,
                authorities=_authority_arguments(args.authority),
                evidence_tools=_evidence_tool_arguments(args.evidence_tool))
        else:
            result = run_capture(
                run_root=args.run_root, plan_sha256=args.plan_sha256,
                commands_path=args.commands,
                commands_sha256=args.commands_sha256,
                execute=args.execute,
                startup_timeout_s=args.startup_timeout,
                prefix_timeout_s=args.prefix_timeout,
                middle_timeout_s=args.middle_timeout,
                postflight_timeout_s=args.postflight_timeout,
                shutdown_timeout_s=args.shutdown_timeout,
                ffprobe=args.ffprobe, ffmpeg=args.ffmpeg)
        print("H1_CANDIDATE3_LIVE_SUPERVISOR_829=PASS " +
              _canonical_bytes(result).decode("ascii").strip())
        return 0
    except SupervisorError as error:
        print(f"H1_CANDIDATE3_LIVE_SUPERVISOR_829=FAIL:{error}",
              file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
