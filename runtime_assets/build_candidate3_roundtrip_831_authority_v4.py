#!/usr/bin/python3
"""Build the non-overwriting Candidate-3 831 round-trip authority v4."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


HERE = Path(__file__).resolve().parent
BASE_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v3.py"
SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v4_base", BASE_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V4_BASE_BUILDER_IMPORT")
BUILDER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BUILDER
SPEC.loader.exec_module(BUILDER)

# Reuse the audited atomic builder implementation while publishing to new,
# non-overwriting paths and schemas.  Mutable source inputs are re-read and
# re-hashed by the base implementation during preparation.
BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v4")
BUILDER.RUN_ROOT = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r12")
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v4"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v4"


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
