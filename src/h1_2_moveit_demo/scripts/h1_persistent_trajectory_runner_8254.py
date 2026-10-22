#!/usr/bin/python3
"""Persistent, fail-closed replay worker for the 8254 closed-loop demo.

The worker keeps one ROS node and one publisher per robot controller alive
during the authorized action window.  Requests arrive as sealed, owner-only
files in a private directory.  It never controls Gazebo, a support plugin, or
an incubator joint.  A malformed request or a replay failure writes one error
response, destroys all publishers, and exits.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import math
import os
from pathlib import Path
import signal
import stat
import sys
import threading
import time
from types import MappingProxyType, SimpleNamespace
from typing import Any, Dict, Mapping, NamedTuple, Sequence, Tuple


CONTROLLER_TOPICS: Tuple[str, ...] = (
    "/right_arm_controller/joint_trajectory",
    "/left_arm_controller/joint_trajectory",
    "/right_gripper_controller/joint_trajectory",
    "/left_gripper_controller/joint_trajectory",
)
VALID_STAGES = frozenset(("handle", "left_clear", "door", "cube"))
REQUEST_FIELDS = 10

# Only this already sealed reverse-handle asset may be split, and only at the
# physical q90->q0 / release boundary.  This is deliberately a payload registry,
# not a general authorization for partial handle replay.
SEALED_HANDLE_SEGMENT_RANGES = {
    "b6b15a9ec270cb3158e0c815997cfa85ebefb04ba3cd3d878c5ed1ff8b9b5ce3": {
        (0, 1),
        (1, 5),
    }
}


class RegisteredAsset(NamedTuple):
    """Compile-time binding for one trajectory accepted by the 8254 worker."""

    relative_path: str
    stage: str
    message_count: int
    payload_sha256: str
    file_sha256: str


class CachedAsset(NamedTuple):
    """Startup-validated immutable replay data and its on-disk identity."""

    path: Path
    registration: RegisteredAsset
    identity: Tuple[int, int, int, int, int]
    canonical_payload: bytes
    durations: Tuple[int, ...]
    payload: Mapping[str, Any]
    messages: Tuple[Mapping[str, Any], ...]
    topology: Tuple[Tuple[int, str, Tuple[str, ...], int, int], ...]


REGISTERED_ASSETS: Tuple[RegisteredAsset, ...] = (
    RegisteredAsset(
        "runtime_assets/closed_loop_90_8254/handle_position_only.trajectory.json",
        "handle", 5,
        "c03df7c51daac6d2059ab5316d17dd9d486191d1bc2aadd0395cfe73044adc6b",
        "89e35a80b3d012871935dff280a93e878d3f9e7d0eb8f51e23bcfc1f0a310f0b",
    ),
    RegisteredAsset(
        "runtime_assets/full_robot_flow_8251/left_clear.trajectory.json",
        "left_clear", 2,
        "066b2437fa320d2db895a927e24f0d5044ff2a9786638e335f8117695e105b88",
        "9e3daae8d7fdc20702a31ec60c475d78da08f05b91550279779845ab22dcc0b3",
    ),
    RegisteredAsset(
        "runtime_assets/full_robot_flow_8251/door.trajectory.json",
        "door", 13,
        "867548548197ae577580f394051e8118cc0fccefdea1494c1bbb41ccaa4a4a19",
        "c481657ac205017a5be16646532236c97dedf909eb0571297e70f9d8690131c9",
    ),
    RegisteredAsset(
        "runtime_assets/parallel_cube_8262/cube.parallel90.neutral.trajectory.json",
        "cube", 6,
        "acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705",
        "09919a92a65b5ca90b3a11420bc9209ceead1e90112f94da2c4c18711af8e134",
    ),
    RegisteredAsset(
        "runtime_assets/close_and_relock_8253/door_close.trajectory.json",
        "door", 13,
        "5b212d78bb204fdde047c875d1e2e9585efb18d7d053c847be0a0ef59a4a9038",
        "3cbb2decfe3128e6a7ca95238085f4285b06a79486c40733db7d426d0a3e5afb",
    ),
    RegisteredAsset(
        "runtime_assets/close_and_relock_8253/left_regrasp.trajectory.json",
        "left_clear", 2,
        "5acd13b1a497b2496ca50d06982316d18b4d482a3288126e1a66fe36f8fd6f2d",
        "8d4443c50e9d5d18bc4b1df0cef6a5af132fd4b577c4dada964f3350ec6ef443",
    ),
    RegisteredAsset(
        "runtime_assets/close_and_relock_8253/handle_relock.trajectory.json",
        "handle", 5,
        "b6b15a9ec270cb3158e0c815997cfa85ebefb04ba3cd3d878c5ed1ff8b9b5ce3",
        "3d88b71fc9c748b6669831467b7cbd6b6f21f67a5026ea88da99e4bc93fa07e9",
    ),
)


class RunnerError(RuntimeError):
    """A fail-closed worker or request error."""


def _require_authorized_range(
    stage: str, start: int, end: int, message_count: int, digest: str
) -> None:
    if not (0 <= start < end <= message_count):
        raise RunnerError("requested range is outside the sealed asset")
    if (start, end) == (0, message_count) or stage in ("door", "cube"):
        return
    if (
        stage == "handle"
        and message_count == 5
        and (start, end) in SEALED_HANDLE_SEGMENT_RANGES.get(digest, set())
    ):
        return
    raise RunnerError(
        "segmented handle replay is restricted to the sealed q90-to-q0 / "
        "release-retreat ranges; other partial assets are forbidden"
    )


def _regular_owned_file(path: Path, mode: int) -> os.stat_result:
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise RunnerError(f"cannot inspect {path}: {exc}") from exc
    if not stat.S_ISREG(metadata.st_mode) or path.is_symlink():
        raise RunnerError(f"not a regular non-symlink file: {path}")
    if metadata.st_uid != os.geteuid() or stat.S_IMODE(metadata.st_mode) != mode:
        raise RunnerError(f"wrong owner or mode for {path}")
    if metadata.st_nlink != 1:
        raise RunnerError(f"file must have exactly one link: {path}")
    return metadata


def _file_identity(metadata: os.stat_result) -> Tuple[int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _secure_file_sha256(path: Path) -> Tuple[str, os.stat_result]:
    """Hash one protected file descriptor and reject changes during the read."""
    flags = os.O_RDONLY
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise RunnerError(f"cannot securely open registered asset {path}: {exc}") from exc
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise RunnerError(f"registered asset is not a regular file: {path}")
        if before.st_uid != os.geteuid() or stat.S_IMODE(before.st_mode) != 0o400:
            raise RunnerError(f"wrong owner or mode for registered asset {path}")
        if before.st_nlink != 1 or before.st_size <= 0:
            raise RunnerError(f"wrong link count or empty registered asset: {path}")
        digest = hashlib.sha256()
        remaining = before.st_size
        while remaining:
            chunk = os.read(descriptor, min(1024 * 1024, remaining))
            if not chunk:
                raise RunnerError(f"registered asset shrank while hashing: {path}")
            digest.update(chunk)
            remaining -= len(chunk)
        if os.read(descriptor, 1):
            raise RunnerError(f"registered asset grew while hashing: {path}")
        after = os.fstat(descriptor)
        if _file_identity(before) != _file_identity(after):
            raise RunnerError(f"registered asset changed while hashing: {path}")
        return digest.hexdigest(), after
    finally:
        os.close(descriptor)


def _deep_freeze(value: Any) -> Any:
    """Convert strict JSON data into recursively immutable Python values."""
    if type(value) is dict:
        return MappingProxyType({key: _deep_freeze(item) for key, item in value.items()})
    if type(value) is list:
        return tuple(_deep_freeze(item) for item in value)
    return value


def _deep_thaw(value: Any) -> Any:
    """Build request-local JSON containers without mutating the sealed cache."""
    if isinstance(value, Mapping):
        return {key: _deep_thaw(item) for key, item in value.items()}
    if type(value) is tuple:
        return [_deep_thaw(item) for item in value]
    return value


def _message_topology(
    messages: Sequence[Mapping[str, Any]], durations: Sequence[int]
) -> Tuple[Tuple[int, str, Tuple[str, ...], int, int], ...]:
    if len(messages) != len(durations):
        raise RunnerError("asset duration topology does not match its messages")
    return tuple(
        (
            int(message["sequence"]),
            str(message["topic"]),
            tuple(message["joint_names"]),
            len(message["points"]),
            int(durations[index]),
        )
        for index, message in enumerate(messages)
    )


def _preload_registered_assets(
    asset_module: Any,
    workspace: Path,
    registrations: Sequence[RegisteredAsset] = REGISTERED_ASSETS,
) -> Mapping[Path, CachedAsset]:
    """Parse and validate every registered JSON before ROS publishers exist."""
    if not workspace.is_absolute():
        raise RunnerError("H1_FULL_WORKSPACE must be absolute")
    try:
        resolved_workspace = workspace.resolve(strict=True)
    except OSError as exc:
        raise RunnerError(f"cannot resolve H1_FULL_WORKSPACE: {exc}") from exc
    cache: Dict[Path, CachedAsset] = {}
    for registration in registrations:
        path = resolved_workspace / registration.relative_path
        try:
            resolved_path = path.resolve(strict=True)
        except OSError as exc:
            raise RunnerError(f"cannot resolve registered asset {path}: {exc}") from exc
        if resolved_path != path or resolved_workspace not in resolved_path.parents:
            raise RunnerError(f"registered asset escapes or aliases its bound path: {path}")
        path_metadata = _regular_owned_file(path, 0o400)
        file_digest, hashed_metadata = _secure_file_sha256(path)
        if _file_identity(path_metadata) != _file_identity(hashed_metadata):
            raise RunnerError(f"registered asset changed during startup validation: {path}")
        if file_digest != registration.file_sha256:
            raise RunnerError(f"registered asset file SHA-256 mismatch: {path}")
        payload, durations, payload_digest = asset_module.load_asset(path)
        if payload_digest != registration.payload_sha256:
            raise RunnerError(f"registered asset payload SHA-256 mismatch: {path}")
        if payload["stage"] != registration.stage:
            raise RunnerError(f"registered asset stage mismatch: {path}")
        if len(payload["messages"]) != registration.message_count:
            raise RunnerError(f"registered asset message count mismatch: {path}")
        canonical_payload = asset_module._canonical_bytes(payload)
        if hashlib.sha256(canonical_payload).hexdigest() != registration.payload_sha256:
            raise RunnerError(f"registered asset canonical payload binding mismatch: {path}")
        frozen_payload = _deep_freeze(payload)
        frozen_messages = tuple(frozen_payload["messages"])
        frozen_durations = tuple(int(value) for value in durations)
        topology = _message_topology(frozen_messages, frozen_durations)
        cached = CachedAsset(
            path=path,
            registration=registration,
            identity=_file_identity(hashed_metadata),
            canonical_payload=canonical_payload,
            durations=frozen_durations,
            payload=frozen_payload,
            messages=frozen_messages,
            topology=topology,
        )
        if path in cache:
            raise RunnerError(f"duplicate registered asset path: {path}")
        cache[path] = cached
    return MappingProxyType(cache)


def _cached_asset_for_request(
    request: Mapping[str, Any], cache: Mapping[Path, CachedAsset]
) -> CachedAsset:
    """Rebind a request to unchanged bytes and an immutable startup contract."""
    requested_path = request["asset"]
    cached = cache.get(requested_path)
    if cached is None:
        raise RunnerError("request asset path is not in the sealed startup registry")
    metadata = _regular_owned_file(requested_path, 0o400)
    if _file_identity(metadata) != cached.identity:
        raise RunnerError("registered asset metadata or file identity changed after startup")
    file_digest, hashed_metadata = _secure_file_sha256(requested_path)
    if _file_identity(hashed_metadata) != cached.identity:
        raise RunnerError("registered asset changed while revalidating the request")
    if file_digest != cached.registration.file_sha256:
        raise RunnerError("registered asset file SHA-256 changed after startup")
    registration = cached.registration
    if request["expected_digest"] != registration.payload_sha256:
        raise RunnerError("asset payload SHA-256 differs from the request binding")
    if hashlib.sha256(cached.canonical_payload).hexdigest() != registration.payload_sha256:
        raise RunnerError("cached canonical payload SHA-256 changed after startup")
    if request["stage"] != registration.stage or cached.payload["stage"] != registration.stage:
        raise RunnerError("request, registration, and cached payload stage contract differ")
    if len(cached.messages) != registration.message_count:
        raise RunnerError("cached message count differs from the registered topology")
    if _message_topology(cached.messages, cached.durations) != cached.topology:
        raise RunnerError("cached payload topology contract changed after startup")
    for expected_sequence, item in enumerate(cached.topology):
        sequence, topic, joint_names, point_count, duration_ns = item
        if sequence != expected_sequence or topic not in CONTROLLER_TOPICS:
            raise RunnerError("cached payload has an invalid sequence or controller topic")
        if not joint_names or point_count < 1 or duration_ns < 0:
            raise RunnerError("cached payload topology has an empty trajectory component")
    return cached


def _private_directory(path: Path) -> None:
    if not path.is_absolute():
        raise RunnerError("command directory must be absolute")
    metadata = path.lstat()
    if not stat.S_ISDIR(metadata.st_mode) or path.is_symlink():
        raise RunnerError("command directory must be a real directory")
    if metadata.st_uid != os.geteuid() or stat.S_IMODE(metadata.st_mode) != 0o700:
        raise RunnerError("command directory must be owned by this user with mode 0700")


def _parse_request_line(line: str) -> Mapping[str, Any]:
    if "\x00" in line or "\r" in line or "\n" in line:
        raise RunnerError("request must be exactly one newline-free text row")
    fields = line.split("\t")
    if len(fields) != REQUEST_FIELDS:
        raise RunnerError(f"request must contain exactly {REQUEST_FIELDS} tab-separated fields")
    (
        operation,
        request_id,
        asset_text,
        stage,
        start_text,
        end_text,
        scales,
        expected_digest,
        start_tolerance_text,
        endpoint_tolerance_text,
    ) = fields
    if operation not in ("run", "stop"):
        raise RunnerError("request operation must be run or stop")
    if not request_id.isdigit() or int(request_id) <= 0:
        raise RunnerError("request id must be a positive integer")
    if operation == "stop":
        if any(
            (
                asset_text,
                stage,
                start_text,
                end_text,
                scales,
                expected_digest,
                start_tolerance_text,
                endpoint_tolerance_text,
            )
        ):
            raise RunnerError("stop request has unexpected arguments")
        return {"operation": operation, "request_id": int(request_id)}
    asset = Path(asset_text)
    if not asset.is_absolute() or not asset_text or "\t" in asset_text:
        raise RunnerError("asset path must be absolute")
    if stage not in VALID_STAGES:
        raise RunnerError("stage is not recognized")
    if not start_text.isdigit() or not end_text.isdigit():
        raise RunnerError("range bounds must be unsigned integers")
    start = int(start_text)
    end = int(end_text)
    if start >= end:
        raise RunnerError("range must satisfy start < end")
    if len(expected_digest) != 64 or any(
        character not in "0123456789abcdef" for character in expected_digest
    ):
        raise RunnerError("expected payload SHA-256 must be 64 lowercase hex characters")
    try:
        start_tolerance = float(start_tolerance_text)
        endpoint_tolerance = float(endpoint_tolerance_text)
    except ValueError as exc:
        raise RunnerError("request tolerances must be decimal numbers") from exc
    if any(
        not math.isfinite(value) or not (0.0 < value <= 0.012)
        for value in (start_tolerance, endpoint_tolerance)
    ):
        raise RunnerError("request tolerances must be in (0, 0.012]")
    if (
        start_tolerance > 0.003 or endpoint_tolerance > 0.003
    ) and not (stage == "cube" and (start, end) == (2, 4)):
        raise RunnerError(
            "relaxed simulation tolerances are scoped only to supported cube carry"
        )
    return {
        "operation": operation,
        "request_id": int(request_id),
        "asset": asset,
        "stage": stage,
        "start": start,
        "end": end,
        "scales": scales,
        "expected_digest": expected_digest,
        "start_tolerance": start_tolerance,
        "endpoint_tolerance": endpoint_tolerance,
    }


def _exclusive_text(path: Path, text: str, mode: int = 0o600) -> None:
    temporary = path.with_name(
        f".{path.name}.tmp.{os.getpid()}.{time.monotonic_ns()}"
    )
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        payload = text.encode("utf-8")
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise RunnerError(f"short write for {path}")
            written += count
        os.fsync(descriptor)
        os.fchmod(descriptor, mode)
    finally:
        os.close(descriptor)
    try:
        os.link(temporary, path, follow_symlinks=False)
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _require_advancing_clock(node: Any, stop_event: threading.Event, timeout: float) -> None:
    import rclpy

    start_sim_ns = node.get_clock().now().nanoseconds
    deadline = time.monotonic() + timeout
    while node.get_clock().now().nanoseconds <= start_sim_ns:
        if stop_event.is_set() or not rclpy.ok():
            raise RunnerError("interrupted while proving an advancing simulation clock")
        if time.monotonic() >= deadline:
            raise RunnerError("simulation clock is not advancing")
        rclpy.spin_once(node, timeout_sec=0.01)


def _load_asset_module(path: Path) -> Any:
    if not path.is_absolute():
        raise RunnerError("trajectory tool path must be absolute")
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise RunnerError(f"cannot resolve trajectory tool: {exc}") from exc
    _regular_owned_file(resolved, 0o755)
    spec = importlib.util.spec_from_file_location(
        "h1_joint_trajectory_asset_8254", resolved
    )
    if spec is None or spec.loader is None:
        raise RunnerError("could not load the trajectory asset module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _prepare_replay(
    asset_module: Any,
    request: Mapping[str, Any],
    publishers: Mapping[str, Any],
    cache: Mapping[Path, CachedAsset],
) -> Tuple[Sequence[Mapping[str, Any]], Mapping[int, Any], Sequence[Sequence[Mapping[str, Any]]], str]:
    cached = _cached_asset_for_request(request, cache)
    payload = cached.payload
    durations = cached.durations
    digest = cached.registration.payload_sha256
    stage = request["stage"]
    if payload["stage"] != stage:
        raise RunnerError(f"stage mismatch: asset={payload['stage']!r}, request={stage!r}")
    # The asset helper deliberately requires exact JSON dict/list types.  Give
    # it request-local containers; the recursively immutable startup cache is
    # never exposed to ROS message construction or replay code.
    messages = tuple(_deep_thaw(message) for message in cached.messages)
    start = request["start"]
    end = request["end"]
    _require_authorized_range(stage, start, end, len(messages), digest)
    scales_text = request["scales"]
    if not scales_text:
        raise RunnerError("every selected message requires an explicit retime scale")
    parsed_scales = asset_module._parse_message_time_scales(scales_text)
    selected_scales = asset_module._selected_message_time_scales(
        parsed_scales, start, end, len(messages)
    )
    if set(selected_scales) != set(range(start, end)):
        raise RunnerError("retime schedule must explicitly bind every selected message")
    for index in range(start, end):
        scale = float(selected_scales[index])
        if not math.isfinite(scale) or not (0.0 < scale <= 1.0):
            raise RunnerError(f"invalid scale for message {index}")
        asset_module._retimed_message_points(
            messages[index], scale, True, f"trajectory[{index}]"
        )
        if messages[index]["topic"] not in publishers:
            raise RunnerError(f"unsupported controller topic in message {index}")
    groups = asset_module._replay_overlap_groups(
        messages, durations, start, end, 1.0, 0.02, selected_scales
    )
    outgoing = {
        index: asset_module._trajectory_message(
            messages[index], selected_scales[index], True, f"trajectory[{index}]"
        )
        for index in range(start, end)
    }
    return messages, outgoing, groups, digest


def _response_path(command_dir: Path, request_id: int) -> Path:
    return command_dir / f"response_{request_id:06d}.txt"


def _request_path(command_dir: Path, request_id: int) -> Path:
    return command_dir / f"request_{request_id:06d}.txt"


def run_worker(args: argparse.Namespace) -> int:
    command_dir = Path(args.command_dir)
    _private_directory(command_dir)
    asset_module = _load_asset_module(Path(args.trajectory_tool))
    workspace_text = os.environ.get("H1_FULL_WORKSPACE", "")
    if not workspace_text:
        raise RunnerError("H1_FULL_WORKSPACE is required for the sealed asset registry")
    asset_cache = _preload_registered_assets(asset_module, Path(workspace_text))

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(
        f"h1_persistent_trajectory_runner_8254_{os.getpid()}",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        automatically_declare_parameters_from_overrides=True,
    )
    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_event.set()

    previous_handlers = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    }
    feedback = asset_module.JointFeedback()
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(
        JointState, "/joint_states", feedback.update, qos_profile_sensor_data
    )
    publishers = {
        topic: node.create_publisher(JointTrajectory, topic, qos)
        for topic in CONTROLLER_TOPICS
    }
    run_args = SimpleNamespace(
        state_timeout=args.state_timeout,
        timeout_factor=args.timeout_factor,
        timeout_padding=args.timeout_padding,
    )
    ready_path = command_dir / "ready"
    stopped_path = command_dir / "stopped"
    next_request_id = 1
    graceful_stop = False
    publication_started = False

    def mark_publication(_index: int, _topic: str) -> None:
        nonlocal publication_started
        publication_started = True

    def publish_failure_hold() -> None:
        """Preempt an accepted trajectory with fresh measured joint holds."""
        if not publication_started:
            return
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        names_by_topic = {
            "/right_arm_controller/joint_trajectory": (
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint", "right_elbow_joint",
                "right_wrist_roll_joint", "right_wrist_pitch_joint",
                "right_wrist_yaw_joint",
            ),
            "/left_arm_controller/joint_trajectory": (
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint", "left_elbow_joint",
                "left_wrist_roll_joint", "left_wrist_pitch_joint",
                "left_wrist_yaw_joint",
            ),
            "/right_gripper_controller/joint_trajectory": (
                "right_dex1_finger_1_joint", "right_dex1_finger_2_joint",
            ),
            "/left_gripper_controller/joint_trajectory": (
                "left_dex1_finger_1_joint", "left_dex1_finger_2_joint",
            ),
        }
        all_names = sorted({name for names in names_by_topic.values() for name in names})
        hold_wait_event = threading.Event()
        fault_ns = time.monotonic_ns()
        fresh = asset_module._fresh_positions(
            node,
            feedback,
            all_names,
            hold_wait_event,
            min(args.state_timeout, 1.0),
            newer_than_ns=fault_ns,
        )
        hold_targets: Dict[str, Tuple[Tuple[str, ...], Sequence[float]]] = {}
        hold_publish_ns = 0
        for topic, names in names_by_topic.items():
            if publishers[topic].get_subscription_count() < 1:
                raise RunnerError(f"failure hold has no controller subscriber on {topic}")
            message = JointTrajectory()
            message.joint_names = list(names)
            point = JointTrajectoryPoint()
            point.positions = [fresh[name] for name in names]
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 100_000_000
            message.points.append(point)
            publishers[topic].publish(message)
            hold_targets[topic] = (names, tuple(point.positions))
            hold_publish_ns = time.monotonic_ns()
        hold_until = time.monotonic() + 0.25
        while time.monotonic() < hold_until and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
        measured = asset_module._fresh_positions(
            node,
            feedback,
            all_names,
            hold_wait_event,
            min(args.state_timeout, 1.0),
            newer_than_ns=hold_publish_ns,
        )
        for topic, (names, positions) in hold_targets.items():
            asset_module._require_close_to_positions(
                measured,
                names,
                positions,
                asset_module.ENDPOINT_TOLERANCE_RAD,
                f"failure hold {topic}",
            )
        print("PERSISTENT_RUNNER_8254_FAILURE_HOLD=PASS", flush=True)
    try:
        deadline = time.monotonic() + args.discovery_timeout
        while time.monotonic() < deadline and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
            if all(
                publisher.get_subscription_count()
                >= args.initial_minimum_subscriptions
                for publisher in publishers.values()
            ):
                break
        else:
            raise RunnerError("controller subscribers were not discovered")
        clock_deadline = time.monotonic() + args.discovery_timeout
        while time.monotonic() < clock_deadline and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
            if bool(getattr(node.get_clock(), "ros_time_is_active", False)) and node.get_clock().now().nanoseconds > 0:
                break
        else:
            raise RunnerError("active nonzero simulation clock was not discovered")
        _require_advancing_clock(node, stop_event, 1.0)
        _exclusive_text(ready_path, f"PERSISTENT_RUNNER_8254=READY pid={os.getpid()}\n", 0o400)

        while not stop_event.is_set():
            request_path = _request_path(command_dir, next_request_id)
            if not request_path.exists():
                rclpy.spin_once(node, timeout_sec=0.02)
                continue
            _regular_owned_file(request_path, 0o600)
            raw = request_path.read_text(encoding="utf-8")
            if not raw.endswith("\n") or raw.count("\n") != 1:
                raise RunnerError("request file must contain exactly one terminated row")
            request = _parse_request_line(raw[:-1])
            if request["request_id"] != next_request_id:
                raise RunnerError("request id is not the expected monotonic sequence")
            response_path = _response_path(command_dir, next_request_id)
            if request["operation"] == "stop":
                _exclusive_text(
                    response_path,
                    f"PERSISTENT_RUNNER_8254=STOPPING request={next_request_id}\n",
                    0o400,
                )
                graceful_stop = True
                break
            request_received_ns = time.monotonic_ns()
            messages, outgoing, groups, digest = _prepare_replay(
                asset_module, request, publishers, asset_cache
            )
            request_run_args = SimpleNamespace(
                state_timeout=run_args.state_timeout,
                timeout_factor=run_args.timeout_factor,
                timeout_padding=run_args.timeout_padding,
                start_tolerance=request["start_tolerance"],
                endpoint_tolerance=request["endpoint_tolerance"],
            )
            selected_topics = {
                messages[int(item["index"])]["topic"]
                for group in groups
                for item in group
            }
            required_subscriptions = (
                args.initial_minimum_subscriptions if next_request_id == 1 else 1
            )
            subscription_deadline = time.monotonic() + args.discovery_timeout
            while time.monotonic() < subscription_deadline:
                if all(
                    publishers[topic].get_subscription_count()
                    >= required_subscriptions
                    for topic in selected_topics
                ):
                    break
                rclpy.spin_once(node, timeout_sec=0.02)
            else:
                raise RunnerError(
                    "required controller-topic subscriptions disappeared before replay"
                )
            selected_joints = sorted(
                {
                    joint
                    for group in groups
                    for item in group
                    for joint in messages[int(item["index"])]["joint_names"]
                }
            )
            asset_module._fresh_positions(
                node,
                feedback,
                selected_joints,
                stop_event,
                args.state_timeout,
                newer_than_ns=request_received_ns,
            )
            for group in groups:
                _require_advancing_clock(node, stop_event, 1.0)
                asset_module._run_replay_group(
                    node,
                    stop_event,
                    feedback,
                    publishers,
                    messages,
                    outgoing,
                    group,
                    request_run_args,
                    mark_publication,
                )
                publication_started = False
            _exclusive_text(
                response_path,
                "PERSISTENT_REPLAY_8254=PASS "
                f"request={next_request_id} stage={request['stage']} "
                f"range=[{request['start']},{request['end']}) "
                f"messages={request['end'] - request['start']} sha256={digest}\n",
                0o400,
            )
            next_request_id += 1
    except Exception as exc:  # fail-closed boundary; publisher destruction is below
        try:
            publish_failure_hold()
        except Exception as hold_exc:
            print(
                f"h1_persistent_trajectory_runner_8254: ERROR: failure hold also failed: {hold_exc}",
                file=sys.stderr,
                flush=True,
            )
        error_path = command_dir / f"error_{next_request_id:06d}.txt"
        try:
            _exclusive_text(error_path, f"PERSISTENT_RUNNER_8254=ERROR {exc}\n", 0o400)
        except Exception:
            pass
        print(f"h1_persistent_trajectory_runner_8254: ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        node.destroy_subscription(subscription)
        for publisher in publishers.values():
            node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.try_shutdown()
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
    if not graceful_stop:
        return 130
    _exclusive_text(stopped_path, "PERSISTENT_RUNNER_8254=STOPPED publishers=0\n", 0o400)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--command-dir", required=True)
    parser.add_argument("--trajectory-tool", required=True)
    parser.add_argument("--discovery-timeout", type=float, default=8.0)
    parser.add_argument(
        "--initial-minimum-subscriptions", type=int, choices=range(1, 9), default=1
    )
    parser.add_argument("--state-timeout", type=float, default=3.0)
    parser.add_argument("--timeout-factor", type=float, default=3.0)
    parser.add_argument("--timeout-padding", type=float, default=4.0)
    return parser


def main() -> int:
    try:
        return run_worker(build_parser().parse_args())
    except (OSError, ValueError, RunnerError) as exc:
        print(f"h1_persistent_trajectory_runner_8254: ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
