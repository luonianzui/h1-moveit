"""Test migration safety gates using tiny files in isolated temporary roots.

No test invokes migrate() or opens an existing project trace. Entrypoint tests
execute a temporary copy and must stop before acquiring its migration lock.
"""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zlib

import pytest


SOURCE = Path(__file__).with_name("migrate.py")
CONTENT = (b'{"step":1,"payload":"small fixture"}\n' * 19
           + '{"step":2,"payload":"中文"}\n'.encode("utf-8"))


@pytest.fixture
def isolated(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("trace_migrate_test_subject", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    workspace = tmp_path / "workspace"
    audit = workspace / "runtime_evidence" / "test_archive"
    audit.mkdir(parents=True)
    monkeypatch.setattr(module, "ROOT", workspace)
    monkeypatch.setattr(module, "AUDIT", audit)
    # Several independently compressed members, including splits within UTF-8.
    monkeypatch.setattr(module, "CHUNK", 37)
    disk_usage = shutil.disk_usage(tmp_path)
    monkeypatch.setattr(module.shutil, "disk_usage", lambda _: disk_usage._replace(free=10**9))
    raw = workspace / "fixture.jsonl"
    raw.write_bytes(CONTENT)
    row = {
        "id": 0,
        "size": len(CONTENT),
        "sha256": hashlib.sha256(CONTENT).hexdigest(),
        "stat": module.metadata(raw.stat()),
    }
    return module, raw, audit / "fixture.jsonl.gz.partial", row


def member_count(encoded):
    count = 0
    while encoded:
        decoder = zlib.decompressobj(16 + zlib.MAX_WBITS)
        decoder.decompress(encoded)
        assert decoder.eof, "Incomplete gzip member"
        encoded = decoder.unused_data
        count += 1
    return count


def test_multimember_roundtrip_checks_sha_bytes_and_lines(isolated):
    module, raw, archive, row = isolated
    lines = module.compress_one(raw, archive, row)
    assert member_count(archive.read_bytes()) > 1
    assert lines == CONTENT.count(b"\n")
    assert module.decode_verify(archive, row, lines) == {
        "sha256": row["sha256"], "size": len(CONTENT), "lines": lines,
    }
    assert raw.read_bytes() == CONTENT


def test_original_sha_mismatch_refuses_compression_result(isolated):
    module, raw, archive, row = isolated
    row["sha256"] = "0" * 64
    with pytest.raises(AssertionError, match="Source differs from historical SHA"):
        module.compress_one(raw, archive, row)
    assert raw.read_bytes() == CONTENT


@pytest.mark.parametrize("field,value", [("sha256", "0" * 64), ("size", len(CONTENT) + 1)])
def test_decoded_identity_mismatch_is_rejected(isolated, field, value):
    module, raw, archive, row = isolated
    lines = module.compress_one(raw, archive, row)
    changed = dict(row, **{field: value})
    with pytest.raises(AssertionError, match="Decoded SHA/size mismatch"):
        module.decode_verify(archive, changed, lines)
    assert raw.read_bytes() == CONTENT


def test_decoded_line_count_mismatch_is_rejected(isolated):
    module, raw, archive, row = isolated
    lines = module.compress_one(raw, archive, row)
    with pytest.raises(AssertionError, match="Line count mismatch"):
        module.decode_verify(archive, row, lines + 1)
    assert raw.read_bytes() == CONTENT


def test_truncated_final_gzip_trailer_is_rejected(isolated):
    module, raw, archive, row = isolated
    lines = module.compress_one(raw, archive, row)
    archive.write_bytes(archive.read_bytes()[:-4])
    with pytest.raises(AssertionError, match="gzip_decode_failed"):
        module.decode_verify(archive, row, lines)
    assert raw.read_bytes() == CONTENT


def test_changed_source_metadata_is_rejected(isolated):
    module, raw, archive, row = isolated
    changed = CONTENT + b'{"unexpected":true}\n'
    raw.write_bytes(changed)
    with pytest.raises(AssertionError):
        module.compress_one(raw, archive, row)
    assert raw.read_bytes() == changed


def test_existing_temporary_output_is_never_overwritten(isolated):
    module, raw, archive, row = isolated
    archive.write_bytes(b"existing temporary output")
    with pytest.raises(FileExistsError):
        module.compress_one(raw, archive, row)
    assert archive.read_bytes() == b"existing temporary output"
    assert raw.read_bytes() == CONTENT


def test_disk_reserve_stops_before_writing_member(isolated, monkeypatch):
    module, raw, archive, row = isolated
    disk_usage = shutil.disk_usage(raw.parent)
    monkeypatch.setattr(module.shutil, "disk_usage", lambda _: disk_usage._replace(free=module.RESERVE))
    with pytest.raises(AssertionError, match="Disk reserve reached"):
        module.compress_one(raw, archive, row)
    assert archive.read_bytes() == b""
    assert raw.read_bytes() == CONTENT


def test_atomic_json_survives_old_interrupted_new_file(isolated):
    module, raw, archive, row = isolated
    destination = module.AUDIT / "registry.json"
    destination.write_text('{"previous":true}\n')
    abandoned = destination.with_name(destination.name + ".new")
    abandoned.write_bytes(b'{"interrupted":')
    module.atomic_json(destination, {"updated": 1})
    module.atomic_json(destination, {"updated": 2})
    assert json.loads(destination.read_text()) == {"updated": 2}
    assert abandoned.read_bytes() == b'{"interrupted":'
    assert list(module.AUDIT.glob("registry.json.new.*")) == []
    assert raw.read_bytes() == CONTENT


@pytest.mark.parametrize(
    "environment,arguments,message",
    [
        ({"PYTHONOPTIMIZE": "1"}, [], "Optimized Python disables safety assertions"),
        ({}, ["-O"], "Optimized Python disables safety assertions"),
        ({"H1_TRACE_ARCHIVE_REGISTRY": "unavailable-test-registry.json"}, [],
         "Run migration without compatibility mapping"),
    ],
)
def test_entrypoint_rejects_unsafe_environment_before_any_io(
    tmp_path, environment, arguments, message
):
    # A temporary copy guarantees a regression cannot access the real audit root.
    workspace = tmp_path / "isolated-entrypoint"
    script = workspace / "tools" / "trace_archive" / "migrate.py"
    script.parent.mkdir(parents=True)
    script.write_bytes(SOURCE.read_bytes())
    child_environment = os.environ.copy()
    for name in ("PYTHONPATH", "PYTHONOPTIMIZE", "H1_TRACE_ARCHIVE_REGISTRY"):
        child_environment.pop(name, None)
    child_environment.update(environment)
    child_environment["PYTHONDONTWRITEBYTECODE"] = "1"
    result = subprocess.run(
        [sys.executable, "-S", *arguments, str(script), "migrate"],
        cwd=tmp_path, env=child_environment, capture_output=True, text=True, timeout=10,
    )
    assert result.returncode != 0
    assert message in result.stderr, result.stdout + result.stderr
    assert not (workspace / "runtime_evidence").exists()
