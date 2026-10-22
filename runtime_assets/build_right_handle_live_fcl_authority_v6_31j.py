#!/usr/bin/python3
"""Instantiate the reviewed v5 FCL builder for fast-831 authority v8 once."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import stat
import sys
from types import ModuleType


WS = Path("/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
UPSTREAM = WS / "runtime_assets/build_right_handle_live_fcl_authority_v5_31j.py"
UPSTREAM_SHA256 = (
    "4289b5d1eb28c0006f483ed5ca42bbe4911aa90779f072472c3449f86eee2e56")
LIVE_SOURCE = WS / "src/h1_2_moveit_demo/src/right_handle_live_fcl_auditor_fast_831.cpp"
LIVE_BINARY = WS / "build/h1_2_moveit_demo/right_handle_live_fcl_auditor_fast_831"
LIVE_SOURCE_SHA256 = (
    "45f2fc8bbe73639774443a0ce8af4ed110cb89af7e2b6b275682daf7539bd06a")
LIVE_BINARY_SHA256 = (
    "fa5c926222b2cb87c5a20ff92b85ba951d974c3a7c7d7eb2e72fd0dcaba3698d")


def read_single_link(path: Path, expected_sha256: str) -> bytes:
    metadata = path.lstat()
    if (path.is_symlink() or not stat.S_ISREG(metadata.st_mode) or
            metadata.st_nlink != 1 or path.resolve(strict=True) != path):
        raise RuntimeError(f"INPUT_IDENTITY:{path}")
    before = (
        metadata.st_dev, metadata.st_ino, metadata.st_mode,
        metadata.st_nlink, metadata.st_size, metadata.st_mtime_ns,
        metadata.st_ctime_ns)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        blocks = []
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            blocks.append(block)
        raw = b"".join(blocks)
        opened = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    after = path.lstat()
    opened_signature = (
        opened.st_dev, opened.st_ino, opened.st_mode, opened.st_nlink,
        opened.st_size, opened.st_mtime_ns, opened.st_ctime_ns)
    after_signature = (
        after.st_dev, after.st_ino, after.st_mode, after.st_nlink,
        after.st_size, after.st_mtime_ns, after.st_ctime_ns)
    if before != opened_signature or opened_signature != after_signature:
        raise RuntimeError(f"INPUT_TOCTOU:{path}")
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise RuntimeError(f"INPUT_SHA256:{path}")
    return raw


def replace_once(raw: bytes, old: bytes, new: bytes, label: str) -> bytes:
    count = raw.count(old)
    if count != 1:
        raise RuntimeError(f"TRANSFORM_CARDINALITY:{label}:{count}")
    return raw.replace(old, new, 1)


def main() -> None:
    upstream = read_single_link(UPSTREAM, UPSTREAM_SHA256)
    source = read_single_link(LIVE_SOURCE, LIVE_SOURCE_SHA256)
    binary = read_single_link(LIVE_BINARY, LIVE_BINARY_SHA256)
    if not binary.startswith(b"\x7fELF"):
        raise RuntimeError("LIVE_BINARY_NOT_ELF")
    for token in (
            b"constexpr double kLiveJointBoundsTolerance = 2e-6;",
            b"constexpr double kLiveMechanismPhysicsTolerance = 2.5e-4;",
            b"void silence_native_library_stderr()",
            b'std::freopen("/dev/null", "w", stderr)',
            b"silence_native_library_stderr();",
            b"print_live_summary(",
            b"return valid ? 0 : 3;"):
        if token not in source:
            raise RuntimeError(f"LIVE_SOURCE_CONTRACT:{token!r}")

    transformed = replace_once(
        upstream, b"live_fcl_authority_v5_31j",
        b"live_fcl_authority_v8_31j", "DESTINATION")
    transformed = replace_once(
        transformed, b"829-v5-31j", b"831-v8-31j", "SCHEMA")
    transformed = replace_once(
        transformed,
        b'WS / "src/h1_2_moveit_demo/src/right_handle_live_fcl_auditor_829.cpp"',
        b'WS / "src/h1_2_moveit_demo/src/right_handle_live_fcl_auditor_fast_831.cpp"',
        "LIVE_SOURCE_PATH")
    transformed = replace_once(
        transformed,
        b'WS / "build/h1_2_moveit_demo/right_handle_live_fcl_auditor_829"',
        b'WS / "build/h1_2_moveit_demo/right_handle_live_fcl_auditor_fast_831"',
        "LIVE_BINARY_PATH")
    transformed = replace_once(
        transformed,
        b"kLiveJointBoundsTolerance = 1e-6",
        b"kLiveJointBoundsTolerance = 2e-6",
        "SOURCE_TOKEN_TOLERANCE")
    transformed = replace_once(
        transformed,
        b'joint_bounds_tolerance_rad": 1e-6',
        b'joint_bounds_tolerance_rad": 2e-6',
        "MANIFEST_TOLERANCE")
    transformed = replace_once(
        transformed,
        b'        "joint_bounds_tolerance_rad": 2e-6,\n',
        b'        "joint_bounds_tolerance_rad": 2e-6,\n'
        b'        "door_physics_range_tolerance_rad": 2.5e-4,\n'
        b'        "handle_range_tolerance_rad": 2.5e-4,\n',
        "MECHANISM_NUMERIC_CONTRACT")
    transformed = replace_once(
        transformed,
        b'    manifest["build_source_dependency_graph"] = {\n',
        b'    manifest.pop("door_physics_tolerance_basis", None)\n'
        b'    manifest["mechanism_physics_tolerance_basis"] = {\n'
        b'        "run": "candidate3_live_831_fast_r2",\n'
        b'        "physics_step_ns": 5000000,\n'
        b'        "observed_fcl_door_min_rad": -4.898644035105899e-05,\n'
        b'        "observed_fcl_door_max_rad": 1.0474034762809068,\n'
        b'        "observed_fcl_handle_min_rad": -1.358372165135551e-10,\n'
        b'        "observed_fcl_handle_max_rad": 1.5703507263580976,\n'
        b'        "selected_common_tolerance_rad": 2.5e-4,\n'
        b'        "angles_are_not_clamped": True,\n'
        b'    }\n'
        b'    manifest["build_source_dependency_graph"] = {\n',
        "MECHANISM_TOLERANCE_BASIS")
    if (b"live_fcl_authority_v5_31j" in transformed or
            b"829-v5-31j" in transformed or
            b"kLiveJointBoundsTolerance = 1e-6" in transformed or
            b'joint_bounds_tolerance_rad": 1e-6' in transformed):
        raise RuntimeError("STALE_V5_TARGET")

    module_name = "right_handle_live_fcl_authority_v8_31j_builder"
    module = ModuleType(module_name)
    module.__file__ = str(UPSTREAM) + "<v8-in-memory>"
    sys.modules[module_name] = module
    exec(compile(transformed, module.__file__, "exec"), module.__dict__)
    module.main()


if __name__ == "__main__":
    main()
