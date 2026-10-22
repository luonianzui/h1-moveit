"""Exercise the opt-in archive view using tiny, isolated temporary evidence."""

import gzip
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys

import pytest


TOOLS = Path(__file__).resolve().parent
RUN = TOOLS / "run"
CLI = TOOLS / "archive_cli.py"
RELATIVE = (
    "runtime_evidence/centrifuge_48box_full_cycle_20261009_r161/"
    "run/loading_trace.jsonl"
)
ARCHIVE_RELATIVE = (
    "runtime_evidence/trace_archive_20261014_r1/data/"
    "centrifuge_48box_full_cycle_20261009_r161.loading_trace.jsonl.gz"
)
CONTENT = b'{"step":1,"value":"first"}\n{"step":2,"value":"second"}\n'


def sha256(content):
    return hashlib.sha256(content).hexdigest()


@pytest.fixture
def evidence(tmp_path):
    workspace = tmp_path / "workspace"
    raw = workspace / RELATIVE
    archive = workspace / ARCHIVE_RELATIVE
    raw.parent.mkdir(parents=True)
    archive.parent.mkdir(parents=True)
    raw.write_bytes(CONTENT)
    raw.chmod(0o444)
    original = raw.stat()
    archive.write_bytes(gzip.compress(CONTENT, compresslevel=6, mtime=0))
    archive.chmod(0o444)
    metadata = {
        name: getattr(original, "st_" + name)
        for name in (
            "mode", "ino", "dev", "nlink", "uid", "gid", "size",
            "atime", "mtime", "ctime", "atime_ns", "mtime_ns", "ctime_ns",
        )
    }
    entry = {
        "archive": ARCHIVE_RELATIVE,
        "sha256": sha256(CONTENT),
        "size": len(CONTENT),
        "lines": CONTENT.count(b"\n"),
        "archive_sha256": sha256(archive.read_bytes()),
        "stat": metadata,
    }
    document = {"version": 1, "workspace": str(workspace),
                "entries": {RELATIVE: entry}}
    registry = tmp_path / "registry.json"
    registry.write_text(json.dumps(document))
    manifest = raw.parent.parent / "SHA256SUMS"
    manifest.write_text(sha256(CONTENT) + "  run/loading_trace.jsonl\n")
    raw.unlink()
    return {"workspace": workspace, "raw": raw, "archive": archive,
            "registry": registry, "document": document, "entry": entry,
            "manifest": manifest}


def save_registry(evidence):
    evidence["registry"].write_text(json.dumps(evidence["document"]))


def invoke(evidence, code, *, check=True):
    environment = os.environ.copy()
    environment.pop("H1_TRACE_ARCHIVE_REGISTRY", None)
    result = subprocess.run(
        [str(RUN), "--registry", str(evidence["registry"]), "--",
         sys.executable, "-c", code],
        env=environment, text=True, capture_output=True, timeout=30,
    )
    if check:
        assert result.returncode == 0, result.stdout + result.stderr
    return result


def test_path_builtin_and_io_reads_seek_and_original_metadata(evidence):
    raw = str(evidence["raw"])
    metadata = evidence["entry"]["stat"]
    invoke(evidence, f"""
import io, os, stat
from pathlib import Path
path = Path({raw!r})
expected = {CONTENT!r}
assert path.exists() and path.is_file() and not path.is_dir()
assert path.read_bytes() == expected
assert path.read_text(encoding='utf-8') == expected.decode()
for opener in (open, io.open):
    with opener(path, 'rb') as stream:
        assert stream.read(9) == expected[:9]
        assert stream.tell() == 9
        assert stream.seek(0) == 0
        assert stream.readline() == expected.splitlines(keepends=True)[0]
        assert stream.seek(-4, 2) == len(expected) - 4
        assert stream.read() == expected[-4:]
with path.open('r', encoding='utf-8', newline='') as stream:
    assert list(stream) == expected.decode().splitlines(keepends=True)
current = path.stat()
assert stat.S_ISREG(current.st_mode)
assert current.st_mode & 0o777 == 0o444
assert current.st_size == len(expected)
assert current.st_mtime_ns == {metadata['mtime_ns']!r}
""")


def test_original_raw_takes_precedence(evidence):
    original = b"original raw is still present\n"
    evidence["raw"].write_bytes(original)
    invoke(evidence, f"""
from pathlib import Path
path = Path({str(evidence['raw'])!r})
assert path.read_bytes() == {original!r}
assert path.stat().st_size == {len(original)}
""")


def test_unlisted_paths_keep_normal_filesystem_behavior(evidence):
    normal = evidence["workspace"] / "ordinary.txt"
    missing = evidence["raw"].with_name("not_registered.jsonl")
    missing.with_name(missing.name + ".gz").write_bytes(gzip.compress(b"not a mapping"))
    invoke(evidence, f"""
from pathlib import Path
normal = Path({str(normal)!r})
normal.write_text('unchanged normal writes')
assert normal.read_text() == 'unchanged normal writes'
missing = Path({str(missing)!r})
assert not missing.exists() and not missing.is_file()
try:
    missing.read_bytes()
except FileNotFoundError:
    pass
else:
    raise AssertionError('Unlisted gzip was incorrectly auto-discovered')
""")
    assert normal.read_text() == "unchanged normal writes"


@pytest.mark.parametrize("mode", ["w", "wb", "a", "x", "r+", "w+b"])
@pytest.mark.parametrize("raw_present", [False, True])
def test_registered_writes_are_rejected(evidence, mode, raw_present):
    if raw_present:
        evidence["raw"].write_bytes(CONTENT)
    before = evidence["archive"].read_bytes()
    result = invoke(evidence, f"""
from pathlib import Path
with Path({str(evidence['raw'])!r}).open({mode!r}) as stream:
    raise AssertionError('Mapped writes must be rejected before returning a stream')
""", check=False)
    assert result.returncode != 0
    assert "Mapped writes must be rejected" not in result.stderr
    assert evidence["archive"].read_bytes() == before
    assert evidence["raw"].exists() == raw_present
    if raw_present:
        assert evidence["raw"].read_bytes() == CONTENT


def test_child_python_inherits_only_explicit_archive_context(evidence):
    child = (
        "from pathlib import Path; "
        f"assert Path({str(evidence['raw'])!r}).read_bytes() == {CONTENT!r}"
    )
    disabled = (
        "from pathlib import Path; "
        f"assert not Path({str(evidence['raw'])!r}).exists()"
    )
    invoke(evidence, f"""
import os, subprocess, sys
subprocess.run([sys.executable, '-c', {child!r}], check=True)
environment = os.environ.copy()
environment.pop('H1_TRACE_ARCHIVE_REGISTRY', None)
subprocess.run([sys.executable, '-c', {disabled!r}], env=environment, check=True)
""")
    assert not evidence["raw"].exists()


@pytest.mark.parametrize("damage", ["archive_sha", "gzip_crc", "raw_sha"])
def test_corrupt_or_mismatched_archives_fail(evidence, damage):
    if damage == "archive_sha":
        evidence["entry"]["archive_sha256"] = "0" * 64
    elif damage == "gzip_crc":
        encoded = bytearray(evidence["archive"].read_bytes())
        encoded[-8] ^= 1
        evidence["archive"].chmod(0o644)
        evidence["archive"].write_bytes(encoded)
        evidence["archive"].chmod(0o444)
        evidence["entry"]["archive_sha256"] = sha256(encoded)
    else:
        evidence["entry"]["sha256"] = "0" * 64
    save_registry(evidence)
    result = invoke(evidence, f"""
from pathlib import Path
Path({str(evidence['raw'])!r}).read_bytes()
print('UNEXPECTED_SUCCESS')
""", check=False)
    assert result.returncode != 0, result.stdout + result.stderr
    assert "UNEXPECTED_SUCCESS" not in result.stdout


def test_missing_archive_is_not_treated_as_empty(evidence):
    evidence["archive"].unlink()
    result = invoke(evidence, f"""
from pathlib import Path
Path({str(evidence['raw'])!r}).read_bytes()
""", check=False)
    assert result.returncode != 0


def test_manifest_verifies_original_hash_and_never_skips_missing(evidence):
    manifest = evidence["manifest"]
    command = [str(CLI), "--registry", str(evidence["registry"]),
               "verify-manifest", str(manifest)]
    invoke(evidence, f"""
import subprocess, sys
subprocess.run([sys.executable] + {command!r}, check=True)
""")
    manifest.write_text(manifest.read_text() + "0" * 64 + "  missing-file.txt\n")
    failed = invoke(evidence, f"""
import subprocess, sys
subprocess.run([sys.executable] + {command!r}, check=True)
""", check=False)
    assert failed.returncode != 0


def test_no_adjacent_gzip_or_ambiguous_legacy_trace_paths(evidence):
    adjacent = evidence["raw"].with_name("loading_trace.jsonl.gz")
    invoke(evidence, f"""
from pathlib import Path
original = Path({str(evidence['raw'])!r})
compressed = Path({str(adjacent)!r})
assert original.exists() and not compressed.exists()
assert original.read_bytes() == {CONTENT!r}
assert not (original.exists() and compressed.exists())
""")
    assert not adjacent.exists()
    assert not evidence["raw"].exists()


def test_registry_cannot_expand_the_frozen_path_allowlist(evidence):
    unknown = "runtime_evidence/unapproved/run/loading_trace.jsonl"
    evidence["document"]["entries"] = {unknown: evidence["entry"]}
    save_registry(evidence)
    result = invoke(evidence, "print('UNEXPECTED_SUCCESS')", check=False)
    assert result.returncode != 0
    assert "UNEXPECTED_SUCCESS" not in result.stdout
