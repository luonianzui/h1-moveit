import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys

from archive_cold import TARGETS


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
COMPAT = WORKSPACE / 'tools/trace_archive/compat.py'
COMPAT_SHA256 = '31eb7267245f211e5ab3b685d5f0f8473cde998ef7d8adb073f5e3216206b374'
ARCHIVE_ROOT = 'runtime_evidence/cold_trace_archive_20261022_r1'
_ADAPTER = None


def install(registry_path=None):
    global _ADAPTER
    registry_path = Path(registry_path or ROOT / 'registry.json').absolute()
    if _ADAPTER is None:
        assert hashlib.sha256(COMPAT.read_bytes()).hexdigest() == COMPAT_SHA256
        specification = importlib.util.spec_from_file_location('explicit_cold_trace_view', COMPAT)
        _ADAPTER = importlib.util.module_from_spec(specification)
        specification.loader.exec_module(_ADAPTER)
        _ADAPTER.ARCHIVE_ROOT = ARCHIVE_ROOT
        _ADAPTER.ALLOWED_PATHS = frozenset(
            'runtime_evidence/' + name + '/run/loading_trace.jsonl' for name in TARGETS)
    return _ADAPTER.install(str(registry_path))


def verify(manifest):
    from archive_cli import verify_manifest
    registry = install()
    return verify_manifest(registry, manifest)


def export(relative, destination):
    registry = install()
    assert relative in registry.relative_entries
    entry = registry.relative_entries[relative]
    destination = Path(destination).absolute()
    assert not destination.exists() and not destination.is_symlink()
    assert destination != Path(registry.workspace) / relative
    assert not any((parent / 'SHA256SUMS').is_file() for parent in destination.parents)
    assert shutil.disk_usage(destination.parent).free >= entry['size'] + 512 * 1024 * 1024
    registry.verify_logical(entry)
    hasher, size, lines = hashlib.sha256(), 0, 0
    with (Path(registry.workspace) / relative).open('rb') as source, destination.open('xb') as output:
        for data in iter(lambda: source.read(8 * 1024 * 1024), b''):
            assert shutil.disk_usage(destination.parent).free >= len(data) + 512 * 1024 * 1024
            hasher.update(data)
            size += len(data)
            lines += data.count(b'\n')
            output.write(data)
        output.flush()
        os.fsync(output.fileno())
    assert (hasher.hexdigest(), size, lines) == (entry['sha256'], entry['size'], entry['lines'])
    destination.chmod(0o444)
    return dict(status='PASS', path=str(destination), sha256=hasher.hexdigest(), bytes=size, lines=lines)


def main():
    parser = argparse.ArgumentParser(description='Explicit read-only access to five cold traces; runtime launcher unchanged')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('doctor')
    check = commands.add_parser('verify-manifest')
    check.add_argument('manifest')
    restore = commands.add_parser('export')
    restore.add_argument('relative')
    restore.add_argument('destination')
    arguments = parser.parse_args()
    if arguments.command == 'doctor':
        from archive_cli import doctor
        result = doctor(install())
    elif arguments.command == 'verify-manifest':
        result = verify(arguments.manifest)
    else:
        result = export(arguments.relative, arguments.destination)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
