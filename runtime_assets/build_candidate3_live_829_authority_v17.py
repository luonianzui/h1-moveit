#!/usr/bin/python3
"""Instantiate the reviewed v16 wrapper for fresh authority v17/r16.

The transformation is fail-closed and performed only in memory.  It preserves
the tested trajectory, certificate, and immutable FCL v6 inputs while binding
a fresh run after r15 recording was invalidated by an online SQLite diagnostic.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import stat
import sys
from types import ModuleType


WS = Path("/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
UPSTREAM = WS / "runtime_assets/build_candidate3_live_829_authority_v16.py"
UPSTREAM_SHA256 = (
    "a20af2936793204a34a24896a3cbc45af4145f54417f9c0365b2cc2d76ad4e73")


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
    opened_signature = (
        opened.st_dev, opened.st_ino, opened.st_mode, opened.st_nlink,
        opened.st_size, opened.st_mtime_ns, opened.st_ctime_ns)
    after_signature = (
        after.st_dev, after.st_ino, after.st_mode, after.st_nlink,
        after.st_size, after.st_mtime_ns, after.st_ctime_ns)
    if before != opened_signature or opened_signature != after_signature:
        raise RuntimeError("UPSTREAM_TOCTOU")
    if hashlib.sha256(raw).hexdigest() != UPSTREAM_SHA256:
        raise RuntimeError("UPSTREAM_SHA256")

    transformed = replace_once(
        raw,
        b'"""Instantiate the reviewed v14 builder for fresh authority v16/r15.',
        b'"""Instantiate the reviewed v14 builder for fresh authority v17/r16.',
        "WRAPPER_DOCSTRING")
    transformed = replace_once(
        transformed,
        b'tested evidence tools, and a fresh absent r15 root.  Earlier authorities and',
        b'tested evidence tools, and a fresh absent r16 root.  Earlier authorities and',
        "WRAPPER_DESCRIPTION")
    transformed = replace_once(
        transformed,
        b'Create sealed Candidate 3 authority v16 for a fresh absent r15 root.',
        b'Create sealed Candidate 3 authority v17 for a fresh absent r16 root.',
        "TARGET_DOCSTRING")
    transformed = replace_once(
        transformed,
        b'runtime_assets/candidate3_live_829_authority_v16"',
        b'runtime_assets/candidate3_live_829_authority_v17"',
        "NEW_AUTHORITY")
    transformed = replace_once(
        transformed,
        b'runtime_assets/candidate3_live_829_final_r15"',
        b'runtime_assets/candidate3_live_829_final_r16"',
        "NEW_RUN_ROOT")
    transformed = replace_once(
        transformed,
        b'"candidate3_live_829_authority_v16_supervisor"',
        b'"candidate3_live_829_authority_v17_supervisor"',
        "IMPORT_NAME")
    transformed = replace_once(
        transformed,
        b'"h1_candidate3_live_production_authority/829-v16"',
        b'"h1_candidate3_live_production_authority/829-v17"',
        "SCHEMA")
    old_supersedes = (
        b'"path": "runtime_assets/candidate3_live_829_authority_v15",')
    new_supersedes = (
        b'"path": "runtime_assets/candidate3_live_829_authority_v16",')
    if transformed.count(old_supersedes) != 2:
        raise RuntimeError(
            f"TRANSFORM_CARDINALITY:SUPERSEDES_PATH:"
            f"{transformed.count(old_supersedes)}")
    transformed = transformed.replace(old_supersedes, new_supersedes)
    transformed = replace_once(
        transformed,
        b'"v16 preserves the action-complete r14 controller trajectory "',
        b'"v17 preserves the action-complete r14 controller trajectory "',
        "SUPERSEDES_REASON_PREFIX")
    transformed = replace_once(
        transformed,
        b'fresh absent r15 root after r14 had five ',
        b'fresh absent r16 root after r15 recording ',
        "SUPERSEDES_REASON_RUN")
    transformed = replace_once(
        transformed, b'solver-boundary rows',
        b'was invalidated by an online SQLite diagnostic lock',
        "SUPERSEDES_REASON_FAILURE")
    transformed = replace_once(
        transformed, b'"V16_OR_R15_EXISTS"', b'"V17_OR_R16_EXISTS"',
        "EXISTS_LABEL")
    transformed = replace_once(
        transformed,
        b'"R15_CREATED_DURING_AUTHORITY_BUILD"',
        b'"R16_CREATED_DURING_AUTHORITY_BUILD"',
        "CREATED_LABEL")
    transformed = replace_once(
        transformed, b'"V16_MANIFEST_READBACK"',
        b'"V17_MANIFEST_READBACK"', "READBACK_LABEL")
    transformed = replace_once(
        transformed, b'"R15_FINAL_LEXISTS"', b'"R16_FINAL_LEXISTS"',
        "FINAL_LABEL")
    transformed = replace_once(
        transformed, b'"r15_lexists": False', b'"r16_lexists": False',
        "OUTPUT_LABEL")
    transformed = replace_once(
        transformed,
        b'module_name = "candidate3_live_829_authority_v16_builder"',
        b'module_name = "candidate3_live_829_authority_v17_builder"',
        "MODULE_NAME")

    for stale in (
            b'runtime_assets/candidate3_live_829_final_r15"',
            b'"h1_candidate3_live_production_authority/829-v16"',
            b'"V16_OR_R15_EXISTS"', b'"r15_lexists": False'):
        if stale in transformed:
            raise RuntimeError(f"STALE_TARGET:{stale!r}")
    if transformed.count(
            b'"path": "runtime_assets/candidate3_live_829_authority_v16",') != 2:
        raise RuntimeError("SUPERSEDES_V16_BINDING")

    module_name = "candidate3_live_829_authority_v17_wrapper"
    module = ModuleType(module_name)
    module.__file__ = str(UPSTREAM) + "<v17-in-memory>"
    sys.modules[module_name] = module
    exec(compile(transformed, module.__file__, "exec"), module.__dict__)
    module.main()


if __name__ == "__main__":
    main()
