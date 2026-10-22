#!/usr/bin/python3
"""Build the non-overwriting Candidate-3 831 round-trip authority v5."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
BASE_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v3.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v5_base", BASE_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V5_BASE_BUILDER_IMPORT")
BUILDER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BUILDER
SPEC.loader.exec_module(BUILDER)


def _extend_right_endpoint_observation(raw: bytes) -> bytes:
    old = b"HARD_ENDPOINT_SETTLE_SIM_S = 0.5\n"
    new = b"HARD_ENDPOINT_SETTLE_SIM_S = 2.0\n"
    if raw.count(old) != 1:
        raise RuntimeError("V5_RIGHT_SETTLE_SOURCE")
    return raw.replace(old, new)


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v5")
BUILDER.RUN_ROOT = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r13")
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v5"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v5"
BUILDER.IMMUTABLE_BYTE_TRANSFORMS = {
    "h1_right_handle_executor_828.py": _extend_right_endpoint_observation,
}


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
