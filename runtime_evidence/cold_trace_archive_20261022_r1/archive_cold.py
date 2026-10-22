import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
TARGETS = (
    'centrifuge_48box_servo_gain02_20261007_r144',
    'centrifuge_48box_velocity_epsilon_20261007_r147',
    'centrifuge_48box_passive_rotor_settle_20261007_r148',
    'centrifuge_48box_drop_contact_settle_20261007_r149',
    'centrifuge_48box_left_reorient_20261008_r150',
)
LEGACY = WORKSPACE / 'tools/trace_archive/migrate.py'
specification = importlib.util.spec_from_file_location('unchanged_gzip_migration_helpers', LEGACY)
helpers = importlib.util.module_from_spec(specification)
specification.loader.exec_module(helpers)
helpers.AUDIT = ROOT
helpers.RESERVE = 512 * 1024 * 1024


def save(name, value):
    with (ROOT / name).open('x') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.flush()
        os.fsync(stream.fileno())


def prepare():
    rows = []
    for name in TARGETS:
        source = WORKSPACE / 'runtime_evidence' / name / 'run/loading_trace.jsonl'
        metadata = source.lstat()
        assert source.is_file() and not source.is_symlink() and metadata.st_nlink == 1
        assert source.resolve() == source.absolute()
        assert not source.with_name('loading_trace.jsonl.gz').exists()
        manifest = source.parent.parent / 'SHA256SUMS'
        members = dict((str(Path(relative.lstrip('*'))), expected)
            for expected, relative in (line.split(maxsplit=1) for line in manifest.read_text().splitlines()))
        assert helpers.digest(source) == members['run/loading_trace.jsonl']
        rows.append(dict(path=str(source.relative_to(WORKSPACE)),
            sha256=members['run/loading_trace.jsonl'], size=metadata.st_size,
            allocated_bytes=metadata.st_blocks * 512, stat=helpers.metadata(metadata),
            manifest=str(manifest.relative_to(WORKSPACE)), manifest_sha256=helpers.digest(manifest)))
    rows.sort(key=lambda row: row['size'])
    existing = WORKSPACE / 'runtime_evidence/trace_archive_20261014_r1/registry.json'
    old_document = json.loads(existing.read_text())
    assert not set(old_document['entries']).intersection(row['path'] for row in rows)
    originals = {}
    for path in [LEGACY, existing, *(WORKSPACE / 'tools/trace_archive').glob('*.py'),
                 WORKSPACE / 'tools/trace_archive/run']:
        originals[str(path.relative_to(WORKSPACE))] = helpers.digest(path)
    save('inventory.json', dict(status='PASS_INVENTORY_NOT_MIGRATED', version=1,
        workspace=str(WORKSPACE), entries=rows, original_tools=originals,
        original_bytes=sum(row['size'] for row in rows),
        original_allocated_bytes=sum(row['allocated_bytes'] for row in rows),
        original_archive_registry_entries=31,
        free_bytes=shutil.disk_usage(WORKSPACE).free))
    documents = {}
    for name in ('AGENTS.md', 'HANDOFF_CURRENT.md', 'QUICKSTART.md', 'CENTRIFUGE_FULL_CYCLE.md',
                 'HANDOFF_PROMPT.md', 'NEXT_CHAT_HANDOFF.md', 'README.md'):
        data = (WORKSPACE / name).read_bytes()
        documents[name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
        if name != 'AGENTS.md':
            documents[name]['text'] = data.decode()
    save('documentation_before.json', documents)
    print('INVENTORY_PASS', len(rows), sum(row['size'] for row in rows), flush=True)


def migrate():
    inventory = json.loads((ROOT / 'inventory.json').read_text())
    expected_paths = {'runtime_evidence/' + name + '/run/loading_trace.jsonl' for name in TARGETS}
    assert len(inventory['entries']) == len(TARGETS)
    assert {row['path'] for row in inventory['entries']} == expected_paths
    assert (ROOT / 'data').resolve() == (ROOT / 'data').absolute()
    assert json.loads((ROOT / 'runtime_protection_before.json').read_text())['status'] == 'PASS'
    for name, expected in inventory['original_tools'].items():
        assert helpers.digest(WORKSPACE / name) == expected, name
    registry_path = ROOT / 'registry.json'
    registry = json.loads(registry_path.read_text()) if registry_path.exists() else dict(
        version=1, workspace=str(WORKSPACE), entries={})
    assert registry['version'] == 1 and registry['workspace'] == str(WORKSPACE)
    assert set(registry['entries']).issubset(expected_paths)
    for row in inventory['entries']:
        source = WORKSPACE / row['path']
        assert row['manifest'] == str(Path(row['path']).parents[1] / 'SHA256SUMS')
        assert row['size'] == row['stat']['size']
        archive = ROOT / 'data' / (source.parent.parent.name + '.loading_trace.jsonl.gz')
        partial = archive.with_name(archive.name + '.partial')
        assert not archive.is_symlink() and not source.is_symlink()
        assert helpers.digest(WORKSPACE / row['manifest']) == row['manifest_sha256']
        if not source.exists():
            entry = registry['entries'][row['path']]
            assert helpers.digest(archive) == entry['archive_sha256']
            helpers.decode_verify(archive, row, entry['lines'])
            continue
        assert helpers.stable(source.lstat(), row['stat'])
        references = helpers.fd_references(source, (row['stat']['dev'], row['stat']['ino']))
        assert not references['found'], references
        helpers.journal('START', path=row['path'], size=row['size'], references=references)
        if archive.exists():
            decoded = helpers.decode_verify(archive, row)
        else:
            assert not partial.exists(), 'Keep partial and raw on interruption; investigate before any retry'
            line_count = helpers.compress_one(source, partial, row)
            decoded = helpers.decode_verify(partial, row, line_count)
            assert partial.stat().st_size < row['allocated_bytes']
            partial.chmod(0o444)
            with partial.open('rb') as stream:
                os.fsync(stream.fileno())
            os.link(partial, archive)
            partial.unlink()
            helpers.sync_dir(archive.parent)
        entry = {key: row[key] for key in ('sha256', 'size', 'stat')}
        entry.update(archive=str(archive.relative_to(WORKSPACE)),
            archive_sha256=helpers.digest(archive), lines=decoded['lines'],
            compressed_bytes=archive.stat().st_size,
            compressed_allocated_bytes=archive.stat().st_blocks * 512)
        registry['entries'][row['path']] = entry
        helpers.atomic_json(registry_path, registry)
        helpers.journal('VERIFIED_COMMITTED', path=row['path'], **entry)
        references = helpers.fd_references(source, (row['stat']['dev'], row['stat']['ino']))
        assert not references['found'] and helpers.stable(source.lstat(), row['stat'])
        assert helpers.digest(WORKSPACE / row['manifest']) == row['manifest_sha256']
        source.unlink()
        helpers.sync_dir(source.parent)
        helpers.journal('RAW_REMOVED_AFTER_FULL_DECODE_VERIFICATION', path=row['path'],
            references=references, free_bytes=shutil.disk_usage(WORKSPACE).free)
    assert len(registry['entries']) == len(TARGETS)
    allocated = sum(entry['compressed_allocated_bytes'] for entry in registry['entries'].values())
    save('migration_result.json', dict(status='PASS', files=len(TARGETS),
        original_bytes=inventory['original_bytes'],
        original_allocated_bytes=inventory['original_allocated_bytes'],
        compressed_bytes=sum(entry['compressed_bytes'] for entry in registry['entries'].values()),
        compressed_allocated_bytes=allocated,
        saved_allocated_bytes=inventory['original_allocated_bytes'] - allocated,
        raw_files_remaining=sum((WORKSPACE / name).exists() for name in registry['entries']),
        free_bytes=shutil.disk_usage(WORKSPACE).free))
    print('COLD_TRACE_MIGRATION_PASS', len(TARGETS), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('prepare', 'migrate'))
    arguments = parser.parse_args()
    assert not sys.flags.optimize
    assert os.environ.get('H1_TRACE_ARCHIVE_REGISTRY') == str(
        WORKSPACE / 'runtime_evidence/trace_archive_20261014_r1/registry.json')
    with (ROOT / 'migration.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        (prepare if arguments.mode == 'prepare' else migrate)()


if __name__ == '__main__':
    main()
