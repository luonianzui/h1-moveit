#!/usr/bin/python3
"""Instantiate the reviewed v14 builder for fresh authority v15/r14.

The transformation is fail-closed and performed only in memory.  It binds the
exact reviewed v14 builder bytes, changes only the new authority/run targets,
schema, diagnostic labels, and supersession explanation, then invokes the
same no-overwrite construction logic.  The failed r13 startup is preserved.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import stat
import sys
from types import ModuleType


WS = Path("/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
UPSTREAM = WS / "runtime_assets/build_candidate3_live_829_authority_v14.py"
UPSTREAM_SHA256 = (
    "79ccedf2404e1ffa4213c02f6199dcf03bbc73493bf91b0f1b4cdca429abd596")


def replace_once(raw: bytes, old: bytes, new: bytes, label: str) -> bytes:
    count = raw.count(old)
    if count != 1:
        raise RuntimeError(f"TRANSFORM_CARDINALITY:{label}:{count}")
    return raw.replace(old, new, 1)


def main() -> None:
    metadata = UPSTREAM.lstat()
    if (not UPSTREAM.is_absolute() or UPSTREAM.is_symlink() or
            not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or
            UPSTREAM.resolve(strict=True) != UPSTREAM):
        raise RuntimeError("UPSTREAM_IDENTITY")
    before = (
        metadata.st_dev, metadata.st_ino, metadata.st_mode,
        metadata.st_nlink, metadata.st_size, metadata.st_mtime_ns,
        metadata.st_ctime_ns)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(UPSTREAM, flags)
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
    after = UPSTREAM.lstat()
    after_signature = (
        after.st_dev, after.st_ino, after.st_mode, after.st_nlink,
        after.st_size, after.st_mtime_ns, after.st_ctime_ns)
    opened_signature = (
        opened.st_dev, opened.st_ino, opened.st_mode, opened.st_nlink,
        opened.st_size, opened.st_mtime_ns, opened.st_ctime_ns)
    if before != opened_signature or opened_signature != after_signature:
        raise RuntimeError("UPSTREAM_TOCTOU")
    if hashlib.sha256(raw).hexdigest() != UPSTREAM_SHA256:
        raise RuntimeError("UPSTREAM_SHA256")

    transformed = replace_once(
        raw,
        b'runtime_assets/candidate3_live_829_authority_v14"',
        b'runtime_assets/candidate3_live_829_authority_v15"',
        "NEW_AUTHORITY")
    transformed = replace_once(
        transformed,
        b'runtime_assets/candidate3_live_829_final_r13"',
        b'runtime_assets/candidate3_live_829_final_r14"',
        "NEW_RUN_ROOT")
    transformed = replace_once(
        transformed,
        b'"candidate3_live_829_authority_v14_supervisor"',
        b'"candidate3_live_829_authority_v15_supervisor"',
        "IMPORT_NAME")
    transformed = replace_once(
        transformed,
        b'"h1_candidate3_live_production_authority/829-v14"',
        b'"h1_candidate3_live_production_authority/829-v15"',
        "SCHEMA")
    transformed = replace_once(
        transformed,
        b'"path": "runtime_assets/candidate3_live_829_authority_v13",',
        b'"path": "runtime_assets/candidate3_live_829_authority_v14",',
        "SUPERSEDES_PATH")
    transformed = replace_once(
        transformed,
        b'"v14 preserves the successful v13/r12 controller trajectory "',
        b'"v15 preserves the tested v14 controller trajectory "',
        "SUPERSEDES_REASON_PREFIX")
    transformed = replace_once(
        transformed,
        b'"and 826.3 posture while binding the corrected scheduled-time "',
        b'"and 826.3 posture after r13 startup failed before actions; "',
        "SUPERSEDES_REASON_MIDDLE")
    transformed = replace_once(
        transformed,
        b'"31-joint FCL authority v5, and a fresh absent r13 root"',
        b'"31-joint FCL authority v5, and a fresh absent r14 root"',
        "SUPERSEDES_REASON_SUFFIX")
    transformed = replace_once(
        transformed, b'"V14_OR_R13_EXISTS"', b'"V15_OR_R14_EXISTS"',
        "EXISTS_LABEL")
    transformed = replace_once(
        transformed,
        b'"R13_CREATED_DURING_AUTHORITY_BUILD"',
        b'"R14_CREATED_DURING_AUTHORITY_BUILD"',
        "CREATED_LABEL")
    transformed = replace_once(
        transformed, b'"R13_FINAL_LEXISTS"', b'"R14_FINAL_LEXISTS"',
        "FINAL_LABEL")
    transformed = replace_once(
        transformed, b'"r13_lexists": False', b'"r14_lexists": False',
        "OUTPUT_LABEL")

    if transformed.count(
            b'"path": "runtime_assets/candidate3_live_829_authority_v14",') != 1:
        raise RuntimeError("SUPERSEDES_V14_BINDING")
    for stale in (
            b'runtime_assets/candidate3_live_829_final_r13"',
            b'"h1_candidate3_live_production_authority/829-v14"'):
        if stale in transformed:
            raise RuntimeError(f"STALE_TARGET:{stale!r}")
    module_name = "candidate3_live_829_authority_v15_builder"
    module = ModuleType(module_name)
    module.__file__ = str(UPSTREAM) + "<v15-in-memory>"
    sys.modules[module_name] = module
    exec(compile(transformed, module.__file__, "exec"), module.__dict__)
    module.main()


if __name__ == "__main__":
    main()
