#!/usr/bin/env python3
"""Explicit integrity checks for the opt-in historical trace archive view."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

from compat import ENVIRONMENT, Registry, default_registry, install


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def doctor(registry):
    results = []
    for relative, entry in registry.relative_entries.items():
        result = {'path': relative}
        try:
            registry.verify_logical(entry)
            result.update(status='PASS', bytes=entry['size'], lines=entry['lines'])
        except Exception as error:
            result.update(status='FAIL', error=str(error))
        results.append(result)
    return {'status': 'PASS' if all(row['status'] == 'PASS' for row in results) else 'FAIL',
            'registry': registry.filename, 'registered_files': len(results), 'results': results,
            'view': 'read-only Python streams and logical stat for exact registered paths',
            'limits': 'Native descriptors, mmap, directory listings and external cat/sha256sum are not virtualized'}


def verify_manifest(registry, filename):
    manifest = Path(filename).absolute()
    rows = []
    for number, line in enumerate(manifest.read_text(encoding='utf-8').splitlines(), 1):
        if not line.strip():
            continue
        result = {'line': number}
        try:
            expected, relative = line.split(maxsplit=1)
            if len(expected) != 64 or any(char not in '0123456789abcdefABCDEF' for char in expected):
                raise ValueError('Malformed SHA-256 manifest line')
            if relative.startswith('*'):
                relative = relative[1:]
            path = manifest.parent / relative
            actual = digest(path)
            result.update(path=relative, expected=expected.lower(), actual=actual,
                          status='PASS' if actual == expected.lower() else 'FAIL')
        except Exception as error:
            result.update(status='FAIL', error=str(error))
        rows.append(result)
    if not rows:
        raise ValueError('Empty checksum manifest')
    return {'status': 'PASS' if all(row['status'] == 'PASS' for row in rows) else 'FAIL',
            'manifest': str(manifest), 'checked_files': len(rows), 'results': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', default=os.environ.get(ENVIRONMENT, default_registry()))
    commands = parser.add_subparsers(dest='command', required=True)
    health = commands.add_parser('doctor', help='Verify every registered gzip and its decompressed original identity')
    health.add_argument('--output')
    verify = commands.add_parser('verify-manifest', help='Check every SHA256SUMS member, resolving archived raw traces')
    verify.add_argument('--output')
    verify.add_argument('manifest')
    arguments = parser.parse_args()
    try:
        registry = install(arguments.registry)
        result = doctor(registry) if arguments.command == 'doctor' else verify_manifest(registry, arguments.manifest)
        rendered = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
        if arguments.output:
            with Path(arguments.output).open('x', encoding='utf-8') as stream:
                stream.write(rendered)
        sys.stdout.write(rendered)
        return 0 if result['status'] == 'PASS' else 1
    except Exception as error:
        sys.stderr.write(f'TRACE_ARCHIVE_CHECK_FAILED: {error}\n')
        return 1


if __name__ == '__main__':
    sys.exit(main())
