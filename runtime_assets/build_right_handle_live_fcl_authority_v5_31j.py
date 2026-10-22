#!/usr/bin/python3
"""Build the sealed 829 v5 31-joint live-FCL authority exactly once.

This builder is intentionally file-only: it neither imports ROS nor issues any
controller command.  The v4 manifest is the immutable semantic baseline.  Of
its nine payloads, only the live auditor source and binary are replaced; the
other seven payloads are copied byte-for-byte from v4.

The destination is written directly with exclusive creates.  Any failed build
therefore leaves a conspicuous, non-overwritable partial destination instead of
silently replacing or repairing evidence on a later invocation.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import stat
from typing import Any


WS = Path(
    "/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
)
BASE = (
    WS
    / "runtime_assets/right_handle_8263_natural/variable_torso_v4_829"
    / "live_fcl_authority_v4_31j"
)
NEW = (
    WS
    / "runtime_assets/right_handle_8263_natural/variable_torso_v4_829"
    / "live_fcl_authority_v5_31j"
)
LIVE_SOURCE = (
    WS / "src/h1_2_moveit_demo/src/right_handle_live_fcl_auditor_829.cpp"
)
LIVE_BINARY = WS / "build/h1_2_moveit_demo/right_handle_live_fcl_auditor_829"

BASE_MANIFEST_SHA256 = (
    "f44efe1dcb9147c49cafa74d387ccbe0096c5e21cfe3c69be259bd0dd1d90c40"
)
BASE_SCHEMA = "h1_moveit/right_handle_live_fcl_authority/829-v4-31j"
NEW_SCHEMA = "h1_moveit/right_handle_live_fcl_authority/829-v5-31j"

# The tuple order is deliberate and is also checked against the v4 manifest.
# There are exactly nine authority payloads: two live replacements and seven
# byte-identical geometry/source-closure payloads from v4.
FILE_LAYOUT: tuple[tuple[str, str, int, str], ...] = (
    (
        "auditor_source",
        "right_handle_live_fcl_auditor_829.cpp",
        0o400,
        "live_source",
    ),
    (
        "auditor_path_base_source",
        "right_handle_path_exact_auditor.cpp",
        0o400,
        "baseline_copy",
    ),
    (
        "auditor_middle_base_source",
        "candidate3_middle_exact_auditor.cpp",
        0o400,
        "baseline_copy",
    ),
    (
        "auditor_binary",
        "right_handle_live_fcl_auditor_829.bin",
        0o500,
        "live_binary",
    ),
    (
        "contact_path",
        "contact_path_v4_44p25_candidate.json",
        0o400,
        "baseline_copy",
    ),
    (
        "original_handle_stl",
        "incubator_handle.STL",
        0o400,
        "baseline_copy",
    ),
    ("scene_cdr", "scene.cdr", 0o400, "baseline_copy"),
    ("urdf", "clearance_audit_robot.urdf", 0o400, "baseline_copy"),
    ("srdf", "clearance_audit_robot.srdf", 0o400, "baseline_copy"),
)

EXPECTED_BASE_FILE_SHA256: dict[str, str] = {
    "auditor_source":
        "368dba55c778f0cc0ba5888171fc911b3e22f3b1ee6b8a03d2e06b7fb2861068",
    "auditor_path_base_source":
        "476b3714da8ecd459aed4892e5efa997b20ca28bcabd311375566736536dc309",
    "auditor_middle_base_source":
        "f3589a08b1f2ab59ee10bcd8830c39ba7a2779ff58bc4c2183e4118262eb7203",
    "auditor_binary":
        "e374fc6d5804d1f4b23c80d9eff4f4e14f31e5c122c78f9b8c208aaec456761b",
    "contact_path":
        "92d0f131324db1755ecdf2a1e03b102b450fbd783a647625e9baf9dfb0a0fb98",
    "original_handle_stl":
        "ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e",
    "scene_cdr":
        "dacf67c055f5ba1b8d46c30b6be4aafdd80a6d453faee069b2d22e5a0e791568",
    "urdf":
        "3aad4bdc038ff6cde3bef221bb15b8134bb45f7a87ef5dfa0fadc2126bd44929",
    "srdf":
        "2f268a57406fcea301cd1a58c3912b91aa9a3a676e2ae42b9f9c741fac30d79d",
}

EXPECTED_DELEGATED_SHA256: dict[str, str] = {
    "middle_exact_result":
        "3e49dd9a34b206da4699545cff5656fb0db3fb38796ffd9a9435ebf55048e1fb",
    "middle_certificate":
        "58a683347c95be35c7c9c39e814c7be23dfab7311a037a0a0e6d7bb5f62aaade",
}


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
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_mode,
        metadata.st_nlink,
        metadata.st_uid,
        metadata.st_gid,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def require_absolute_identity(path: Path) -> None:
    if not path.is_absolute() or path.resolve(strict=True) != path:
        raise RuntimeError(f"ABSOLUTE_IDENTITY:{path}")


def require_directory(path: Path, expected_mode: int) -> None:
    require_absolute_identity(path)
    metadata = path.lstat()
    if (
        path.is_symlink()
        or not stat.S_ISDIR(metadata.st_mode)
        or stat.S_IMODE(metadata.st_mode) != expected_mode
    ):
        raise RuntimeError(f"DIRECTORY_IDENTITY:{path}")


def snapshot_regular(path: Path, expected_mode: int | None = None) -> Snapshot:
    """Read through a no-follow fd and bind both path and fd identities."""

    require_absolute_identity(path)
    before = path.lstat()
    mode = stat.S_IMODE(before.st_mode)
    if (
        path.is_symlink()
        or not stat.S_ISREG(before.st_mode)
        or before.st_nlink != 1
        or (expected_mode is not None and mode != expected_mode)
    ):
        raise RuntimeError(f"INPUT_IDENTITY:{path}")

    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        opened = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened.st_mode)
            or opened.st_nlink != 1
            or (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino)
            or (expected_mode is not None
                and stat.S_IMODE(opened.st_mode) != expected_mode)
        ):
            raise RuntimeError(f"INPUT_OPEN_IDENTITY:{path}")
        chunks: list[bytes] = []
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            chunks.append(block)
        raw = b"".join(chunks)
        after_fd = os.fstat(descriptor)
    finally:
        os.close(descriptor)

    after_path = path.lstat()
    if (
        signature(before) != signature(opened)
        or signature(opened) != signature(after_fd)
        or signature(after_fd) != signature(after_path)
        or len(raw) != after_fd.st_size
    ):
        raise RuntimeError(f"INPUT_TOCTOU:{path}")
    return Snapshot(path, raw, sha_bytes(raw), mode, signature(after_path))


def revalidate(snapshot: Snapshot, expected_mode: int | None = None) -> None:
    current = snapshot_regular(snapshot.path, expected_mode)
    if (
        current.signature != snapshot.signature
        or current.sha256 != snapshot.sha256
        or current.raw != snapshot.raw
    ):
        raise RuntimeError(f"INPUT_CHANGED:{snapshot.path}")


def reject_json_constant(value: str) -> None:
    raise ValueError(f"NONFINITE_JSON:{value}")


def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for key, value in pairs:
        if key in output:
            raise ValueError(f"DUPLICATE_JSON_KEY:{key}")
        output[key] = value
    return output


def parse_json(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=unique_object,
            parse_constant=reject_json_constant,
        )
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as error:
        raise RuntimeError(f"JSON:{label}:{error}") from error
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON_ROOT:{label}")
    return value


def pretty(value: Any) -> bytes:
    return (
        json.dumps(
            value,
            sort_keys=True,
            indent=2,
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
        + b"\n"
    )


def exclusive_write(path: Path, raw: bytes, mode: int) -> Snapshot:
    if path.parent != NEW or not path.is_absolute() or lexists(path):
        raise RuntimeError(f"OUTPUT_PRECONDITION:{path}")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
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
    output = snapshot_regular(path, mode)
    if output.raw != raw or output.sha256 != sha_bytes(raw):
        raise RuntimeError(f"OUTPUT_READBACK:{path}")
    return output


def workspace_relative(path: Path) -> str:
    require_absolute_identity(path)
    try:
        relative = path.relative_to(WS)
    except ValueError as error:
        raise RuntimeError(f"OUTSIDE_WORKSPACE:{path}") from error
    if relative.is_absolute() or ".." in relative.parts:
        raise RuntimeError(f"UNSAFE_RELATIVE:{path}")
    return relative.as_posix()


def validate_live_contract(
    source: Snapshot,
    path_base: Snapshot,
    middle_base: Snapshot,
    binary: Snapshot,
) -> None:
    source_tokens = (
        b'#include "right_handle_path_exact_auditor.cpp"',
        b"constexpr double kLiveTorsoLimit = 44.5 * kPi / 180.0;",
        b"constexpr double kLiveJointBoundsTolerance = 1e-6;",
        b"constexpr double kLiveLeftHandleClearance = 0.050;",
        b"constexpr double kLiveRightPadGripProximity = 0.0003;",
        b'"right_dex1_finger_1_link", "right_dex1_finger_2_link"',
        b"std::min(metric.pad1_distance, metric.pad2_distance)",
        b"metric.right_pad_grip_engaged = minimum_right_pad_distance <=",
        b"if (!metric.right_pad_grip_engaged)",
        b"RIGHT_PAD_GRIP_PROXIMITY_FORBIDDEN",
        b"metric.adjacent_finger_clearance < -kLiveNumericTolerance",
        b"RIGHT_FINGER_BODY_HANDLE_PENETRATION",
        b"metric.nonpad_clearance + kLiveNumericTolerance < "
        b"kNonpadHandleClearance",
        b"LEFT_HANDLE_50MM_CLEARANCE",
        b"state.satisfiesBounds(kLiveJointBoundsTolerance)",
        b"std::abs(row.torso) > kLiveTorsoLimit",
        b"collision_acm.setEntry(kHandleId, pad, true);",
        b"geometric_only=true force_or_wrench_claimed=false",
    )
    missing = [token.decode("ascii") for token in source_tokens
               if token not in source.raw]
    if missing:
        raise RuntimeError("LIVE_SOURCE_CONTRACT_TOKENS:" + repr(missing))
    if source.raw.count(
        b"collision_acm.setEntry(kHandleId, pad, true);") != 1:
        raise RuntimeError("LIVE_SOURCE_COLLISION_EXCEPTION_COUNT")
    forbidden_source_tokens = (
        b"collision_acm.setEntry(kHandleId, adjacent",
        b"collision_acm.setEntry(kHandleId, finger",
        b"force_or_wrench_claimed=true",
    )
    present = [token.decode("ascii") for token in forbidden_source_tokens
               if token in source.raw]
    if present:
        raise RuntimeError("LIVE_SOURCE_FORBIDDEN_TOKENS:" + repr(present))

    path_tokens = (
        b'#include "candidate3_middle_exact_auditor.cpp"',
        b"constexpr double kNonpadHandleClearance = 0.0006;",
    )
    if any(token not in path_base.raw for token in path_tokens):
        raise RuntimeError("PATH_BASE_CONTRACT_TOKENS")
    middle_tokens = (
        b'"right_dex1_finger_1_pad_link", '
        b'"right_dex1_finger_2_pad_link"',
        b"const std::array<std::string, 2> kRightHandlePads",
    )
    if any(token not in middle_base.raw for token in middle_tokens):
        raise RuntimeError("MIDDLE_BASE_CONTRACT_TOKENS")
    if not binary.raw.startswith(b"\x7fELF"):
        raise RuntimeError("LIVE_BINARY_NOT_ELF")


def validate_base_manifest(
    manifest: dict[str, Any], snapshots: dict[str, Snapshot]
) -> None:
    identifiers = tuple(identifier for identifier, _, _, _ in FILE_LAYOUT)
    if len(identifiers) != 9 or len(set(identifiers)) != 9:
        raise RuntimeError("FILE_LAYOUT_CARDINALITY")
    if sum(kind == "baseline_copy" for _, _, _, kind in FILE_LAYOUT) != 7:
        raise RuntimeError("BASELINE_COPY_CARDINALITY")
    if set(EXPECTED_BASE_FILE_SHA256) != set(identifiers):
        raise RuntimeError("BASELINE_SHA_CARDINALITY")
    if (
        manifest.get("schema") != BASE_SCHEMA
        or manifest.get("status") != "PASS"
        or manifest.get("valid") is not True
        or manifest.get("sealed") is not True
        or manifest.get("controller_commands_issued") != 0
    ):
        raise RuntimeError("BASE_MANIFEST_STATUS")
    files = manifest.get("files")
    if not isinstance(files, dict) or set(files) != set(identifiers):
        raise RuntimeError("BASE_MANIFEST_FILE_SET")
    for identifier, name, mode, _ in FILE_LAYOUT:
        entry = files.get(identifier)
        expected_path = (BASE / name).relative_to(WS).as_posix()
        if entry != {
            "path": expected_path,
            "sha256": EXPECTED_BASE_FILE_SHA256[identifier],
            "mode": f"{mode:04o}",
            "nlink": 1,
        }:
            raise RuntimeError(f"BASE_MANIFEST_BINDING:{identifier}")
        payload = snapshots[identifier]
        if (
            payload.sha256 != EXPECTED_BASE_FILE_SHA256[identifier]
            or payload.mode != mode
        ):
            raise RuntimeError(f"BASE_PAYLOAD_BINDING:{identifier}")

    graph = manifest.get("build_source_dependency_graph")
    if graph != {
        "right_handle_live_fcl_auditor_829.cpp":
            "right_handle_path_exact_auditor.cpp",
        "right_handle_path_exact_auditor.cpp":
            "candidate3_middle_exact_auditor.cpp",
    }:
        raise RuntimeError("BASE_SOURCE_CLOSURE")
    scope = manifest.get("scope")
    if (
        not isinstance(scope, dict)
        or scope.get("all_31_robot_joints_replayed") is not True
        or scope.get("recursive_build_source_closure_sealed") is not True
        or scope.get("dynamic_cube_objects_included") is not False
        or scope.get("dynamic_cube_authority") !=
            "candidate3_middle_exact_result_and_certificate"
        or len(scope.get("leg_joint_order", [])) != 12
    ):
        raise RuntimeError("BASE_SCOPE")
    delegated = manifest.get("delegated_dynamic_cube_bindings")
    if not isinstance(delegated, dict) or set(delegated) != set(
        EXPECTED_DELEGATED_SHA256
    ):
        raise RuntimeError("BASE_DELEGATION")
    for identifier, expected_sha in EXPECTED_DELEGATED_SHA256.items():
        entry = delegated[identifier]
        if not isinstance(entry, dict) or entry.get("sha256") != expected_sha:
            raise RuntimeError(f"BASE_DELEGATION_BINDING:{identifier}")


def binding(snapshot: Snapshot) -> dict[str, Any]:
    return {
        "path": workspace_relative(snapshot.path),
        "sha256": snapshot.sha256,
        "mode": f"{snapshot.mode:04o}",
        "nlink": 1,
    }


def build_manifest(
    base_manifest: dict[str, Any],
    outputs: dict[str, Snapshot],
    live_source: Snapshot,
    live_binary: Snapshot,
) -> dict[str, Any]:
    # JSON round-trip gives a detached tree without importing mutable helpers.
    manifest = parse_json(pretty(base_manifest), "BASE_MANIFEST_COPY")
    manifest["schema"] = NEW_SCHEMA
    manifest["status"] = "PASS"
    manifest["valid"] = True
    manifest["sealed"] = True
    manifest["controller_commands_issued"] = 0
    manifest["scope"] = {
        **base_manifest["scope"],
        "right_handle_geometric_replay": True,
        "all_31_robot_joints_replayed": True,
        "left_hand_handle_clearance_checked": True,
        "non_cube_world_collision_checked": True,
        "dynamic_cube_objects_included": False,
        "dynamic_cube_authority":
            "candidate3_middle_exact_result_and_certificate",
        "gripper_transition_windows_included": False,
        "geometric_contact_only": True,
        "force_or_wrench_claimed": False,
        "authorized_right_handle_pad_links": [
            "right_dex1_finger_1_pad_link",
            "right_dex1_finger_2_pad_link",
        ],
        "contact_required_grip_rule": (
            "minimum signed distance from the exact handle STL to at least "
            "one authorized right pad is <= 0.0003 m"
        ),
        "contact_forbidden_rule": (
            "both authorized right pads have signed distance > 0.0003 m"
        ),
        "adjacent_right_finger_body_links": [
            "right_dex1_finger_1_link",
            "right_dex1_finger_2_link",
        ],
        "adjacent_right_finger_body_rule":
            "signed distance to exact handle STL is >= 0 m",
        "buffered_nonpad_rule": (
            "all remaining non-left, non-authorized-pad, "
            "non-adjacent-finger collision links keep >= 0.0006 m from "
            "the exact handle STL"
        ),
        "handle_collision_exceptions_during_contact_required": [
            ["incubator_handle_exact", "right_dex1_finger_1_pad_link"],
            ["incubator_handle_exact", "right_dex1_finger_2_pad_link"],
        ],
        "adjacent_right_finger_body_collision_allowed": False,
        "recursive_build_source_closure_sealed": True,
    }
    manifest["numeric_contract"] = {
        **base_manifest["numeric_contract"],
        "right_pad_grip_proximity_max_m": 0.0003,
        "right_pad_grip_required_during_contact": True,
        "right_pad_grip_forbidden_outside_contact": True,
        "adjacent_right_finger_body_signed_distance_min_m": 0.0,
        "buffered_nonpad_handle_clearance_min_m": 0.0006,
        "left_hand_handle_clearance_min_m": 0.05,
        "torso_absolute_limit_deg": 44.5,
        "joint_bounds_tolerance_rad": 1e-6,
        "force_or_wrench_claimed": False,
    }
    manifest["build_source_dependency_graph"] = {
        "right_handle_live_fcl_auditor_829.cpp":
            "right_handle_path_exact_auditor.cpp",
        "right_handle_path_exact_auditor.cpp":
            "candidate3_middle_exact_auditor.cpp",
    }
    manifest["files"] = {
        identifier: binding(outputs[identifier])
        for identifier, _, _, _ in FILE_LAYOUT
    }
    manifest["authority_provenance"] = {
        "immutable_geometry_baseline": {
            "path": workspace_relative(BASE),
            "manifest_sha256": BASE_MANIFEST_SHA256,
            "schema": BASE_SCHEMA,
        },
        "baseline_manifest_is_eighth_sealed_baseline_input": True,
        "baseline_payloads_copied_byte_for_byte": [
            identifier
            for identifier, _, _, kind in FILE_LAYOUT
            if kind == "baseline_copy"
        ],
        "live_replacements": {
            "auditor_source": {
                "path": workspace_relative(live_source.path),
                "sha256": live_source.sha256,
            },
            "auditor_binary": {
                "path": workspace_relative(live_binary.path),
                "sha256": live_binary.sha256,
            },
        },
    }
    return manifest


def validate_delegated_bindings(
    base_manifest: dict[str, Any]
) -> list[Snapshot]:
    snapshots: list[Snapshot] = []
    delegated = base_manifest["delegated_dynamic_cube_bindings"]
    for identifier, expected_sha in EXPECTED_DELEGATED_SHA256.items():
        raw_path = delegated[identifier]["path"]
        if not isinstance(raw_path, str):
            raise RuntimeError(f"DELEGATED_PATH:{identifier}")
        relative = Path(raw_path)
        if relative.is_absolute() or ".." in relative.parts:
            raise RuntimeError(f"DELEGATED_RELATIVE:{identifier}")
        snapshot = snapshot_regular(WS / relative, 0o400)
        if snapshot.sha256 != expected_sha:
            raise RuntimeError(f"DELEGATED_SHA256:{identifier}")
        snapshots.append(snapshot)
    return snapshots


def fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def main() -> None:
    require_directory(WS, stat.S_IMODE(WS.lstat().st_mode))
    require_directory(BASE, 0o500)
    if lexists(NEW):
        raise FileExistsError(f"NO_OVERWRITE:{NEW}")

    base_manifest_snapshot = snapshot_regular(BASE / "manifest.json", 0o400)
    if base_manifest_snapshot.sha256 != BASE_MANIFEST_SHA256:
        raise RuntimeError("V4_MANIFEST_SHA256")
    base_manifest = parse_json(base_manifest_snapshot.raw, "V4_MANIFEST")

    base_snapshots: dict[str, Snapshot] = {}
    for identifier, name, mode, _ in FILE_LAYOUT:
        payload = snapshot_regular(BASE / name, mode)
        base_snapshots[identifier] = payload
    validate_base_manifest(base_manifest, base_snapshots)

    live_source = snapshot_regular(LIVE_SOURCE)
    live_binary = snapshot_regular(LIVE_BINARY)
    if live_source.mode & 0o002 or live_binary.mode & 0o002:
        raise RuntimeError("LIVE_INPUT_WORLD_WRITABLE")
    delegated_snapshots = validate_delegated_bindings(base_manifest)
    validate_live_contract(
        live_source,
        base_snapshots["auditor_path_base_source"],
        base_snapshots["auditor_middle_base_source"],
        live_binary,
    )

    # Recheck every input immediately before the first irreversible output.
    revalidate(base_manifest_snapshot, 0o400)
    for identifier, _, mode, _ in FILE_LAYOUT:
        revalidate(base_snapshots[identifier], mode)
    revalidate(live_source)
    revalidate(live_binary)
    for delegated in delegated_snapshots:
        revalidate(delegated, 0o400)

    previous_umask = os.umask(0o077)
    try:
        # mkdir is the atomic no-overwrite claim on the final destination.
        os.mkdir(NEW, 0o700)
        fsync_directory(NEW.parent)
        require_directory(NEW, 0o700)

        outputs: dict[str, Snapshot] = {}
        for identifier, name, mode, kind in FILE_LAYOUT:
            if kind == "live_source":
                raw = live_source.raw
            elif kind == "live_binary":
                raw = live_binary.raw
            elif kind == "baseline_copy":
                raw = base_snapshots[identifier].raw
            else:
                raise RuntimeError(f"UNKNOWN_SOURCE_KIND:{identifier}:{kind}")
            outputs[identifier] = exclusive_write(NEW / name, raw, mode)

        manifest = build_manifest(
            base_manifest, outputs, live_source, live_binary
        )
        manifest_raw = pretty(manifest)
        manifest_output = exclusive_write(
            NEW / "manifest.json", manifest_raw, 0o400
        )

        # Seal only after every payload and the manifest have been read back.
        fsync_directory(NEW)
        os.chmod(NEW, 0o555)
        require_directory(NEW, 0o555)
        fsync_directory(NEW)
        fsync_directory(NEW.parent)

        expected_names = {name for _, name, _, _ in FILE_LAYOUT}
        expected_names.add("manifest.json")
        actual_names = {entry.name for entry in os.scandir(NEW)}
        if actual_names != expected_names:
            raise RuntimeError("FINAL_FILE_SET")

        loaded_manifest = parse_json(
            snapshot_regular(NEW / "manifest.json", 0o400).raw,
            "V5_MANIFEST_READBACK",
        )
        if loaded_manifest != manifest:
            raise RuntimeError("FINAL_MANIFEST_SEMANTIC_READBACK")
        if manifest_output.sha256 != sha_bytes(manifest_raw):
            raise RuntimeError("FINAL_MANIFEST_SHA256")

        for identifier, name, mode, kind in FILE_LAYOUT:
            output = snapshot_regular(NEW / name, mode)
            if binding(output) != manifest["files"][identifier]:
                raise RuntimeError(f"FINAL_BINDING:{identifier}")
            if kind == "baseline_copy" and output.raw != base_snapshots[identifier].raw:
                raise RuntimeError(f"FINAL_BASELINE_BYTE_COPY:{identifier}")
            if kind == "live_source" and output.raw != live_source.raw:
                raise RuntimeError("FINAL_LIVE_SOURCE_COPY")
            if kind == "live_binary" and output.raw != live_binary.raw:
                raise RuntimeError("FINAL_LIVE_BINARY_COPY")

        # A final input revalidation proves that the manifest describes the
        # same snapshots observed before and throughout construction.
        revalidate(base_manifest_snapshot, 0o400)
        for identifier, _, mode, _ in FILE_LAYOUT:
            revalidate(base_snapshots[identifier], mode)
        revalidate(live_source)
        revalidate(live_binary)
        for delegated in delegated_snapshots:
            revalidate(delegated, 0o400)

        print(json.dumps({
            "status": "PASS",
            "valid": True,
            "sealed": True,
            "controller_commands_issued": 0,
            "authority": str(NEW),
            "schema": NEW_SCHEMA,
            "manifest_sha256": manifest_output.sha256,
            "payload_count": len(FILE_LAYOUT),
            "baseline_payload_byte_copies": 7,
            "live_replacements": 2,
            "source_contract_tokens": "PASS",
            "input_toctou_revalidation": "PASS",
            "directory_mode": "0555",
        }, sort_keys=True, indent=2))
    finally:
        os.umask(previous_umask)


if __name__ == "__main__":
    main()
