#!/usr/bin/python3
"""Create corrected Candidate 3 authority v18 for a fresh r17 run.

The reviewed v16 wrapper is transformed only in memory and still pins the
fixed v14 builder bytes.  This revision additionally binds the candidate
manifest's exact-FCL metadata to the immutable v6 bundle.  The sealed v17
authority and its prepared-but-never-executed r16 root are verified and kept.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import stat
import sys
from types import ModuleType
from typing import Any


WS = Path("/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
UPSTREAM = WS / "runtime_assets/build_candidate3_live_829_authority_v16.py"
UPSTREAM_SHA256 = (
    "a20af2936793204a34a24896a3cbc45af4145f54417f9c0365b2cc2d76ad4e73")
V17 = WS / "runtime_assets/candidate3_live_829_authority_v17"
V17_MANIFEST_SHA256 = (
    "4786e8fe5003d3e9181998fcaca8f2fb91b2d8d9b03a46ff67b076221a1713af")
V17_COMMANDS_SHA256 = (
    "606604a604228feacfc2cd5db444e01fafe6a758d6c758362938cc284d78fad6")
R16 = WS / "runtime_assets/candidate3_live_829_final_r16"
R16_PLAN_SHA256 = (
    "55a4ec0def42f48f692f4ccf298a265fa4d4bf2c37c986f29596fc1086b15edc")
R16_FORMAL_SHA256 = (
    "8114955bc028865a114d2979e59e38b8fd779f35ed06507b31e80fae35136021")
STALE_FCL_V4_MANIFEST_SHA256 = (
    "f44efe1dcb9147c49cafa74d387ccbe0096c5e21cfe3c69be259bd0dd1d90c40")


def read_bound(path: Path, expected_sha256: str, expected_mode: int) -> bytes:
    metadata = path.lstat()
    if (not path.is_absolute() or path.is_symlink() or
            not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or
            stat.S_IMODE(metadata.st_mode) != expected_mode or
            path.resolve(strict=True) != path):
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


def parse_object(raw: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"INPUT_JSON:{label}:{error}") from error
    if type(value) is not dict:
        raise RuntimeError(f"INPUT_JSON_ROOT:{label}")
    return value


def replace_once(raw: bytes, old: bytes, new: bytes, label: str) -> bytes:
    count = raw.count(old)
    if count != 1:
        raise RuntimeError(f"TRANSFORM_CARDINALITY:{label}:{count}")
    return raw.replace(old, new, 1)


def require_directory(path: Path, mode: int) -> None:
    metadata = path.lstat()
    if (path.is_symlink() or not stat.S_ISDIR(metadata.st_mode) or
            stat.S_IMODE(metadata.st_mode) != mode or
            path.resolve(strict=True) != path):
        raise RuntimeError(f"DIRECTORY_IDENTITY:{path}")


def main() -> None:
    upstream = read_bound(UPSTREAM, UPSTREAM_SHA256, 0o664)
    require_directory(V17, 0o555)
    manifest_raw = read_bound(
        V17 / "manifest.json", V17_MANIFEST_SHA256, 0o400)
    commands_raw = read_bound(
        V17 / "commands_829.json", V17_COMMANDS_SHA256, 0o400)
    manifest = parse_object(manifest_raw, "V17_MANIFEST")
    commands = parse_object(commands_raw, "V17_COMMANDS")
    stale = manifest.get("exact_fcl_authority")
    if (manifest.get("schema") !=
            "h1_candidate3_live_production_authority/829-v17" or
            manifest.get("run_root") != str(R16) or
            commands.get("run_id") != manifest.get("run_id") or
            stale != {
                "file_count": 9,
                "path": (
                    "runtime_assets/right_handle_8263_natural/"
                    "variable_torso_v4_829/live_fcl_authority_v4_31j/"
                    "manifest.json"),
                "sha256": STALE_FCL_V4_MANIFEST_SHA256,
            }):
        raise RuntimeError("V17_SEMANTICS")

    require_directory(R16, 0o700)
    entries = {entry.name: entry for entry in os.scandir(R16)}
    if set(entries) != {"formal_authorities_829.json", "live_plan_829.json",
                        "logs"}:
        raise RuntimeError("R16_PREPARED_FILE_SET")
    require_directory(R16 / "logs", 0o700)
    if any(os.scandir(R16 / "logs")):
        raise RuntimeError("R16_LOGS_NOT_EMPTY")
    plan_raw = read_bound(
        R16 / "live_plan_829.json", R16_PLAN_SHA256, 0o400)
    read_bound(
        R16 / "formal_authorities_829.json", R16_FORMAL_SHA256, 0o400)
    plan = parse_object(plan_raw, "R16_PLAN")
    input_paths = plan.get("input_paths")
    if (plan.get("schema") != "h1_candidate3_live_run_plan/829-v1" or
            plan.get("status") != "READY" or plan.get("valid") is not True or
            plan.get("run_id") != manifest.get("run_id") or
            plan.get("formal_authorities_sha256") != R16_FORMAL_SHA256 or
            type(input_paths) is not dict or not input_paths or
            any(type(value) is not str or
                not Path(value).is_relative_to(R16)
                for value in input_paths.values())):
        raise RuntimeError("R16_PREPARED_SEMANTICS")

    transformed = replace_once(
        upstream,
        b'"""Instantiate the reviewed v14 builder for fresh authority v16/r15.',
        b'"""Instantiate the reviewed v14 builder for fresh authority v18/r17.',
        "WRAPPER_DOCSTRING")
    transformed = replace_once(
        transformed,
        b'tested evidence tools, and a fresh absent r15 root.  Earlier authorities and',
        b'tested evidence tools, and a fresh absent r17 root.  Earlier authorities and',
        "WRAPPER_DESCRIPTION")
    transformed = replace_once(
        transformed,
        b'Create sealed Candidate 3 authority v16 for a fresh absent r15 root.',
        b'Create sealed Candidate 3 authority v18 for a fresh absent r17 root.',
        "TARGET_DOCSTRING")
    transformed = replace_once(
        transformed,
        b'runtime_assets/candidate3_live_829_authority_v16"',
        b'runtime_assets/candidate3_live_829_authority_v18"',
        "NEW_AUTHORITY")
    transformed = replace_once(
        transformed,
        b'runtime_assets/candidate3_live_829_final_r15"',
        b'runtime_assets/candidate3_live_829_final_r17"',
        "NEW_RUN_ROOT")
    transformed = replace_once(
        transformed,
        b'"candidate3_live_829_authority_v16_supervisor"',
        b'"candidate3_live_829_authority_v18_supervisor"',
        "IMPORT_NAME")
    transformed = replace_once(
        transformed,
        b'"h1_candidate3_live_production_authority/829-v16"',
        b'"h1_candidate3_live_production_authority/829-v18"',
        "SCHEMA")
    old_supersedes = (
        b'"path": "runtime_assets/candidate3_live_829_authority_v15",')
    new_supersedes = (
        b'"path": "runtime_assets/candidate3_live_829_authority_v17",')
    if transformed.count(old_supersedes) != 2:
        raise RuntimeError(
            f"TRANSFORM_CARDINALITY:SUPERSEDES_PATH:"
            f"{transformed.count(old_supersedes)}")
    transformed = transformed.replace(old_supersedes, new_supersedes)
    transformed = replace_once(
        transformed,
        b'"v16 preserves the action-complete r14 controller trajectory "',
        b'"v18 preserves the action-complete r14 controller trajectory "',
        "SUPERSEDES_REASON_PREFIX")
    transformed = replace_once(
        transformed, b'fresh absent r15 root after r14 had five ',
        b'fresh absent r17 root after r16 was prepared but never ',
        "SUPERSEDES_REASON_RUN")
    transformed = replace_once(
        transformed, b'solver-boundary rows',
        b'executed because v17 retained a stale FCL v4 metadata label',
        "SUPERSEDES_REASON_METADATA")
    transformed = replace_once(
        transformed, b'"V16_OR_R15_EXISTS"', b'"V18_OR_R17_EXISTS"',
        "EXISTS_LABEL")
    transformed = replace_once(
        transformed,
        b'"R15_CREATED_DURING_AUTHORITY_BUILD"',
        b'"R17_CREATED_DURING_AUTHORITY_BUILD"',
        "CREATED_LABEL")
    transformed = replace_once(
        transformed, b'"V16_MANIFEST_READBACK"',
        b'"V18_MANIFEST_READBACK"', "READBACK_LABEL")
    transformed = replace_once(
        transformed, b'"R15_FINAL_LEXISTS"', b'"R17_FINAL_LEXISTS"',
        "FINAL_LABEL")
    transformed = replace_once(
        transformed, b'"r15_lexists": False', b'"r17_lexists": False',
        "OUTPUT_LABEL")
    transformed = replace_once(
        transformed,
        b'module_name = "candidate3_live_829_authority_v16_builder"',
        b'module_name = "candidate3_live_829_authority_v18_builder"',
        "MODULE_NAME")
    transformed = replace_once(
        transformed, b'<v16-in-memory>', b'<v18-in-memory>',
        "MODULE_FILE_LABEL")

    for stale_target in (
            b'runtime_assets/candidate3_live_829_final_r15"',
            b'runtime_assets/candidate3_live_829_authority_v16"',
            b'"h1_candidate3_live_production_authority/829-v16"',
            b'"V16_OR_R15_EXISTS"', b'"r15_lexists": False',
            b'<v16-in-memory>'):
        if stale_target in transformed:
            raise RuntimeError(f"STALE_TARGET:{stale_target!r}")
    if transformed.count(new_supersedes) != 2:
        raise RuntimeError("SUPERSEDES_V17_BINDING")

    module_name = "candidate3_live_829_authority_v18_wrapper"
    module = ModuleType(module_name)
    module.__file__ = str(UPSTREAM) + "<v18-wrapper>"
    sys.modules[module_name] = module
    exec(compile(transformed, module.__file__, "exec"), module.__dict__)

    original_replace = module.replace_once
    metadata_injected = False

    def replace_with_metadata(
        raw: bytes, old: bytes, new: bytes, label: str,
    ) -> bytes:
        nonlocal metadata_injected
        result = original_replace(raw, old, new, label)
        if label != "FCL_MANIFEST_SHA256":
            return result
        if metadata_injected:
            raise RuntimeError("FCL_METADATA_DUPLICATE")
        manifest_anchor = (
            b"        manifest = replace_tree(base_manifest, replacements)\n")
        manifest_bound = manifest_anchor + (
            b'        exact_fcl_manifest = parse_json(\n'
            b'            fcl_manifest.raw, "NEW_FCL_MANIFEST")\n'
            b'        exact_fcl_files = exact_fcl_manifest.get("files")\n'
            b'        if (exact_fcl_manifest.get("schema") !=\n'
            b'                "h1_moveit/right_handle_live_fcl_authority/829-v6-31j" or\n'
            b'                exact_fcl_manifest.get("status") != "PASS" or\n'
            b'                exact_fcl_manifest.get("valid") is not True or\n'
            b'                exact_fcl_manifest.get("sealed") is not True or\n'
            b'                type(exact_fcl_files) is not dict or\n'
            b'                len(exact_fcl_files) != 9):\n'
            b'            raise RuntimeError("NEW_FCL_MANIFEST_SEMANTICS")\n'
            b'        manifest["exact_fcl_authority"] = {\n'
            b'            "file_count": len(exact_fcl_files),\n'
            b'            "path": str((NEW_FCL / "manifest.json").relative_to(WS)),\n'
            b'            "sha256": fcl_manifest.sha256,\n'
            b'        }\n')
        result = original_replace(
            result, manifest_anchor, manifest_bound,
            "FCL_METADATA_BINDING")
        readback_anchor = (
            b'        if loaded_manifest != manifest:\n'
            b'            raise RuntimeError("MANIFEST_READBACK")\n')
        readback_bound = readback_anchor + (
            b'        if loaded_manifest.get("exact_fcl_authority") != {\n'
            b'                "file_count": len(exact_fcl_files),\n'
            b'                "path": str(\n'
            b'                    (NEW_FCL / "manifest.json").relative_to(WS)),\n'
            b'                "sha256": fcl_manifest.sha256,\n'
            b'        }:\n'
            b'            raise RuntimeError("EXACT_FCL_METADATA_READBACK")\n')
        result = original_replace(
            result, readback_anchor, readback_bound,
            "FCL_METADATA_READBACK")
        metadata_injected = True
        return result

    module.replace_once = replace_with_metadata
    module.main()
    if not metadata_injected:
        raise RuntimeError("FCL_METADATA_NOT_INJECTED")


if __name__ == "__main__":
    main()
