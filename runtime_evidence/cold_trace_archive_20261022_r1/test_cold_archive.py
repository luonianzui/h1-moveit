import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

import archive_cold
import cold_access


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
CONTENT = b'{"timestamp":1,"value":"preserved"}\n' * 128


@pytest.fixture
def fixture_registry(tmp_path):
    workspace = tmp_path / 'workspace'
    relative = 'runtime_evidence/' + archive_cold.TARGETS[0] + '/run/loading_trace.jsonl'
    raw = workspace / relative
    raw.parent.mkdir(parents=True)
    raw.write_bytes(CONTENT)
    raw.chmod(0o444)
    metadata = archive_cold.helpers.metadata(raw.stat())
    archive_relative = cold_access.ARCHIVE_ROOT + '/data/' + archive_cold.TARGETS[0] + '.loading_trace.jsonl.gz'
    compressed = workspace / archive_relative
    compressed.parent.mkdir(parents=True)
    compressed.write_bytes(gzip.compress(CONTENT, mtime=0))
    entry = dict(sha256=hashlib.sha256(CONTENT).hexdigest(), size=len(CONTENT),
        lines=CONTENT.count(b'\n'), stat=metadata, archive=archive_relative,
        archive_sha256=hashlib.sha256(compressed.read_bytes()).hexdigest())
    document = dict(version=1, workspace=str(workspace), entries={relative: entry})
    registry = tmp_path / 'registry.json'
    registry.write_text(json.dumps(document))
    raw.unlink()
    return registry, raw, compressed, document


def invoke(registry, code, success=True):
    process = subprocess.run([sys.executable, '-c',
        'import cold_access; cold_access.install(' + repr(str(registry)) + ');\n' + code],
        env=dict(os.environ, PYTHONPATH=str(ROOT) + ':' + os.environ.get('PYTHONPATH', '')),
        text=True, capture_output=True)
    assert (process.returncode == 0) == success, process.stderr
    return process


def test_transparent_reads_seek_and_original_metadata(fixture_registry):
    registry, raw, compressed, document = fixture_registry
    invoke(registry, f"""
from pathlib import Path
import io
path = Path({str(raw)!r})
assert path.read_bytes() == {CONTENT!r}
assert path.exists() and path.is_file() and path.stat().st_size == {len(CONTENT)}
assert path.stat().st_mode & 0o777 == 0o444
with io.open(path, 'rb') as stream:
    assert stream.read(10) == {CONTENT[:10]!r}
    stream.seek(0)
    assert stream.read() == {CONTENT!r}
""")
    assert not raw.exists() and compressed.is_file()


@pytest.mark.parametrize('mode', ['w', 'a', 'x', 'r+', 'wb'])
def test_mapped_writes_refused(fixture_registry, mode):
    registry, raw, compressed, document = fixture_registry
    invoke(registry, f"open({str(raw)!r}, {mode!r})", success=False)
    assert not raw.exists()


@pytest.mark.parametrize('damage', ['raw_sha', 'archive_sha', 'crc', 'unknown_path', 'archive_location'])
def test_integrity_and_exact_scope(fixture_registry, damage):
    registry, raw, compressed, document = fixture_registry
    relative, entry = next(iter(document['entries'].items()))
    if damage == 'raw_sha':
        entry['sha256'] = '0' * 64
    elif damage == 'archive_sha':
        entry['archive_sha256'] = '0' * 64
    elif damage == 'crc':
        data = bytearray(compressed.read_bytes())
        data[-8] ^= 1
        compressed.write_bytes(data)
        entry['archive_sha256'] = hashlib.sha256(data).hexdigest()
    elif damage == 'unknown_path':
        document['entries'] = {'runtime_evidence/unapproved/run/loading_trace.jsonl': entry}
    else:
        entry['archive'] = '../unapproved.gz'
    registry.write_text(json.dumps(document))
    invoke(registry, f"from pathlib import Path; Path({str(raw)!r}).read_bytes()", success=False)


def test_original_31_registry_not_extended(fixture_registry):
    registry, raw, compressed, document = fixture_registry
    invoke(registry, """
import compat
from pathlib import Path
import os
active = compat.install(os.environ['H1_TRACE_ARCHIVE_REGISTRY'])
assert len(active.relative_entries) == len(compat.ALLOWED_PATHS) == 31
assert Path(__import__('sitecustomize').__file__).parent.name == 'trace_archive'
""")


def test_manifest_preserves_original_sha_and_missing_members_fail(fixture_registry, tmp_path):
    registry, raw, compressed, document = fixture_registry
    manifest = raw.parent.parent / 'SHA256SUMS'
    manifest.write_text(hashlib.sha256(CONTENT).hexdigest() + '  ./run/loading_trace.jsonl\n')
    invoke(registry, f"""
from archive_cli import verify_manifest
assert verify_manifest(None, {str(manifest)!r})['status'] == 'PASS'
""")
    manifest.write_text(manifest.read_text() + '0' * 64 + '  missing\n')
    invoke(registry, f"""
from archive_cli import verify_manifest
assert verify_manifest(None, {str(manifest)!r})['status'] == 'FAIL'
""")


@pytest.fixture
def migration(tmp_path, monkeypatch):
    workspace = tmp_path / 'workspace'
    audit = workspace / 'runtime_evidence/archive'
    audit.mkdir(parents=True)
    (audit / 'data').mkdir()
    monkeypatch.setattr(archive_cold, 'ROOT', audit)
    monkeypatch.setattr(archive_cold, 'WORKSPACE', workspace)
    monkeypatch.setattr(archive_cold.helpers, 'AUDIT', audit)
    monkeypatch.setattr(archive_cold.helpers, 'ROOT', workspace)
    rows = []
    for name in archive_cold.TARGETS:
        source = workspace / 'runtime_evidence' / name / 'run/loading_trace.jsonl'
        source.parent.mkdir(parents=True)
        source.write_bytes(CONTENT)
        manifest = source.parent.parent / 'SHA256SUMS'
        manifest.write_text(hashlib.sha256(CONTENT).hexdigest() + '  ./run/loading_trace.jsonl\n')
        rows.append(dict(path=str(source.relative_to(workspace)), sha256=hashlib.sha256(CONTENT).hexdigest(),
            size=len(CONTENT), allocated_bytes=source.stat().st_blocks * 512,
            stat=archive_cold.helpers.metadata(source.stat()), manifest=str(manifest.relative_to(workspace)),
            manifest_sha256=archive_cold.helpers.digest(manifest)))
    (audit / 'inventory.json').write_text(json.dumps(dict(entries=rows, original_tools={},
        original_bytes=sum(row['size'] for row in rows),
        original_allocated_bytes=sum(row['allocated_bytes'] for row in rows))))
    (audit / 'runtime_protection_before.json').write_text('{"status":"PASS"}')
    return workspace, audit, rows


def test_commit_decode_and_then_remove_raw(migration):
    workspace, audit, rows = migration
    archive_cold.migrate()
    result = json.loads((audit / 'migration_result.json').read_text())
    assert result['status'] == 'PASS' and result['raw_files_remaining'] == 0
    for row in rows:
        archive = audit / 'data' / (Path(row['path']).parent.parent.name + '.loading_trace.jsonl.gz')
        assert gzip.decompress(archive.read_bytes()) == CONTENT
        assert archive_cold.helpers.digest(workspace / row['manifest']) == row['manifest_sha256']


@pytest.mark.parametrize('stage', ['compress', 'verify', 'source_change'])
def test_failure_keeps_original(migration, monkeypatch, stage):
    workspace, audit, rows = migration
    if stage == 'source_change':
        (workspace / rows[0]['path']).write_bytes(b'changed')
    else:
        def refuse(*args, **kwargs):
            raise RuntimeError('injected verification failure')
        monkeypatch.setattr(archive_cold.helpers,
            'compress_one' if stage == 'compress' else 'decode_verify', refuse)
    with pytest.raises((RuntimeError, AssertionError)):
        archive_cold.migrate()
    assert all((workspace / row['path']).is_file() for row in rows)
    assert not (audit / 'migration_result.json').exists()


def test_manifest_dot_prefix_normalization():
    assert str(Path('./run/loading_trace.jsonl')) == 'run/loading_trace.jsonl'


def test_export_is_exact_and_does_not_overwrite(fixture_registry, tmp_path):
    registry, raw, compressed, document = fixture_registry
    relative = next(iter(document['entries']))
    destination = tmp_path / 'exported.jsonl'
    invoke(registry, f"""
from pathlib import Path
cold_access.ROOT = Path({str(registry.parent)!r})
result = cold_access.export({relative!r}, {str(destination)!r})
assert result['status'] == 'PASS'
assert Path({str(destination)!r}).read_bytes() == {CONTENT!r}
try:
    cold_access.export({relative!r}, {str(destination)!r})
except AssertionError:
    pass
else:
    raise AssertionError('Export overwrote existing output')
""")
    assert compressed.is_file() and not raw.exists()


def test_inventory_cannot_expand_delete_scope(migration):
    workspace, audit, rows = migration
    inventory = json.loads((audit / 'inventory.json').read_text())
    inventory['entries'][0]['path'] = '../unapproved.jsonl'
    (audit / 'inventory.json').write_text(json.dumps(inventory))
    with pytest.raises(AssertionError):
        archive_cold.migrate()
    assert all((workspace / row['path']).exists() for row in rows)
