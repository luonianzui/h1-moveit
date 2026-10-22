import hashlib
import json
import os
from pathlib import Path

import compat


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
REGISTRY = WORKSPACE / 'runtime_evidence/trace_archive_20261014_r1/registry.json'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def require_active_registry():
    if os.environ.get(compat.ENVIRONMENT) != str(REGISTRY):
        raise ValueError('Exact authorized trace archive registry environment required')
    active = compat.install(str(REGISTRY))
    if len(active.relative_entries) != 31 or set(active.relative_entries) != set(compat.ALLOWED_PATHS):
        raise ValueError('Only the exact 31 authorized historical traces may be virtualized')
    return active


def main():
    registry = require_active_registry()
    files = [REGISTRY, WORKSPACE / 'tools/trace_archive/run', WORKSPACE / 'tools/trace_archive/compat.py',
             WORKSPACE / 'tools/trace_archive/sitecustomize.py', WORKSPACE / 'tools/trace_archive/archive_cli.py']
    logical = []
    for relative, entry in registry.relative_entries.items():
        manifest = WORKSPACE / Path(relative).parents[1] / 'SHA256SUMS'
        entries = {name.lstrip('*'): checksum for checksum, name in
                   (line.split(maxsplit=1) for line in manifest.read_text().splitlines())}
        if entries['run/loading_trace.jsonl'] != entry['sha256']:
            raise ValueError('Archived logical identity disagrees with original sealed manifest')
        registry.verify_logical(entry)
        archive = WORKSPACE / entry['archive']
        files.append(archive)
        logical.append(dict(original_path=relative, original_sha256=entry['sha256'],
                            original_bytes=entry['size'], original_lines=entry['lines'],
                            archive_path=str(archive), compressed_sha256=entry['archive_sha256']))
    records = []
    for path in files:
        if path.is_symlink():
            raise ValueError('Archive runtime dependencies must be regular files')
        metadata = path.stat()
        records.append(dict(path=str(path), sha256=digest(path), bytes=metadata.st_size,
                            device=metadata.st_dev, inode=metadata.st_ino,
                            mtime_ns=metadata.st_mtime_ns, ctime_ns=metadata.st_ctime_ns))
    with (ROOT / 'archive_runtime_dependencies.json').open('x') as stream:
        json.dump(dict(status='PASS', external_runtime_dependencies=records,
                       logical_archives_crc_sha_bytes_lines_verified=logical,
                       original_manifests_unchanged=True, exact_readonly_registry_only=True,
                       no_historical_raw_restoration_or_deletion=True), stream, indent=2)
    print('ARCHIVE_RUNTIME_BINDING_PASS', len(logical), len(records), flush=True)


def verify_bound_dependencies():
    require_active_registry()
    binding = json.loads((ROOT / 'archive_runtime_dependencies.json').read_text())
    for record in binding['external_runtime_dependencies']:
        path = Path(record['path'])
        if path.is_symlink() or digest(path) != record['sha256']:
            raise ValueError('Archive runtime dependency changed: ' + str(path))
    return len(binding['external_runtime_dependencies'])


if __name__ == '__main__':
    main()
