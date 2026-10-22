#!/usr/bin/env python3
"""Explicit, resumable migration of the 31 user-authorized historical traces.

Run without the compatibility wrapper: this program checks physical files.
No historical manifest or control source is rewritten.
"""
import collections
import concurrent.futures
import datetime
import fcntl
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
AUDIT = ROOT / 'runtime_evidence/trace_archive_20261014_r1'
IDS = {161,162,163,164,165,166,167,171,172,175,176,177,178,181,183,185,
       188,190,193,194,195,196,198,199,201,205,206,208,209,210,211}
CHUNK = 8 * 1024 * 1024
RESERVE = 128 * 1024 * 1024

def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(CHUNK), b''):
            h.update(data)
    return h.hexdigest()

def sync_dir(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)

def atomic_json(path, value):
    temp = path.with_name(path.name + f'.new.{os.getpid()}.{time.time_ns()}')
    with temp.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    os.replace(temp, path)
    sync_dir(path.parent)

def journal(event, **kwargs):
    row = dict(time=now(), event=event, **kwargs)
    with (AUDIT / 'journal.jsonl').open('a') as stream:
        stream.write(json.dumps(row) + '\n'); stream.flush(); os.fsync(stream.fileno())
    print(json.dumps(row), flush=True)

def metadata(st):
    return {key: getattr(st, 'st_' + key) for key in
            ('mode','ino','dev','nlink','uid','gid','size','atime','mtime','ctime',
             'atime_ns','mtime_ns','ctime_ns')}

def stable(st, expected):
    return all(getattr(st, 'st_' + key) == expected[key]
               for key in ('mode','ino','dev','nlink','size','mtime_ns','ctime_ns'))

def fd_references(path, inode):
    """Readable /proc only; permission blind spots are recorded, not hidden."""
    found, blind = [], []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid(): continue
        try:
            if proc.stat().st_uid != os.getuid(): continue
            fds = list((proc / 'fd').iterdir())
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            blind.append(proc.name); continue
        for fd in fds:
            try:
                s = fd.stat()
                if (s.st_dev, s.st_ino) == inode: found.append(str(fd))
            except (PermissionError, FileNotFoundError, ProcessLookupError): pass
    return dict(found=found, unreadable_same_uid_pids=blind)

def prepare():
    assert not (AUDIT / 'inventory.json').exists(), 'Existing inventory: do not overwrite'
    rows = []
    for p in (ROOT / 'runtime_evidence').glob('centrifuge_48box_*/run/loading_trace.jsonl'):
        match = re.search(r'_r(\d+)$', p.parent.parent.name)
        if not match or int(match[1]) not in IDS: continue
        s = p.lstat()
        assert stat.S_ISREG(s.st_mode) and s.st_nlink == 1 and not p.is_symlink()
        assert not p.with_suffix('.jsonl.gz').exists()
        manifest = p.parent.parent / 'SHA256SUMS'
        hashes = [line.split()[0] for line in manifest.read_text().splitlines()
                  if len(line.split(maxsplit=1)) == 2 and
                  line.split(maxsplit=1)[1].lstrip('*') == 'run/loading_trace.jsonl']
        assert len(hashes) == 1 and re.fullmatch('[a-f0-9]{64}', hashes[0])
        rows.append(dict(path=str(p.relative_to(ROOT)), id=int(match[1]), sha256=hashes[0],
                         size=s.st_size, allocated_bytes=s.st_blocks*512, stat=metadata(s),
                         manifest=str(manifest.relative_to(ROOT)), manifest_sha256=digest(manifest)))
    assert len(rows) == 31 and {r['id'] for r in rows} == IDS
    rows.sort(key=lambda r:r['size'])
    inventory = dict(version=1, workspace=str(ROOT), created_at=now(), entries=rows,
                     original_bytes=sum(r['size'] for r in rows),
                     original_allocated_bytes=sum(r['allocated_bytes'] for r in rows))
    atomic_json(AUDIT / 'inventory.json', inventory)
    atomic_json(AUDIT / 'registry.json', dict(version=1, workspace=str(ROOT), entries={}))
    journal('PREPARED', files=31, original_bytes=inventory['original_bytes'],
            free_bytes=shutil.disk_usage(ROOT).free)

def decode_verify(archive, row, expected_lines=None):
    # Independent system gzip validates every concatenated member's CRC and trailer.
    h = hashlib.sha256(); size = lines = 0
    with subprocess.Popen(['/usr/bin/gzip', '-dc', '--', str(archive)], stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE) as proc:
        for data in iter(lambda: proc.stdout.read(CHUNK), b''):
            h.update(data); size += len(data); lines += data.count(b'\n')
        error = proc.stderr.read().decode(errors='replace')
        code = proc.wait()
    assert code == 0, ('gzip_decode_failed', code, error)
    assert h.hexdigest() == row['sha256'] and size == row['size'], 'Decoded SHA/size mismatch'
    if expected_lines is not None: assert lines == expected_lines, 'Line count mismatch'
    return dict(sha256=h.hexdigest(), size=size, lines=lines)

def compressed_member(data):
    return gzip.compress(data, compresslevel=6, mtime=0)

def compress_one(source, temp, row):
    h = hashlib.sha256(); size = lines = 0; pending = collections.deque()
    last_notice = time.monotonic()
    with source.open('rb') as inp, temp.open('xb') as out, \
         concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        assert stable(os.fstat(inp.fileno()), row['stat'])
        eof = False
        while pending or not eof:
            while not eof and len(pending) < 12:
                data = inp.read(CHUNK)
                if not data: eof = True; break
                h.update(data); size += len(data); lines += data.count(b'\n')
                pending.append(pool.submit(compressed_member, data))
            if pending:
                member = pending.popleft().result()
                assert shutil.disk_usage(ROOT).free >= RESERVE + len(member), 'Disk reserve reached'
                out.write(member)
            if time.monotonic() - last_notice > 30:
                print(json.dumps(dict(event='COMPRESS_PROGRESS', id=row['id'],
                                      read_bytes=size, total_bytes=row['size'])), flush=True)
                last_notice = time.monotonic()
        out.flush(); os.fsync(out.fileno())
        assert stable(os.fstat(inp.fileno()), row['stat'])
    assert h.hexdigest() == row['sha256'] and size == row['size'], 'Source differs from historical SHA'
    return lines

def migrate():
    for filename in ('protected_before.json', 'dual_protected_before.json'):
        if json.loads((AUDIT / filename).read_text()).get('status') != 'PASS':
            raise RuntimeError('Baseline protection must pass before migration: ' + filename)
    inventory = json.loads((AUDIT / 'inventory.json').read_text())
    registry = json.loads((AUDIT / 'registry.json').read_text())
    assert len(inventory['entries']) == 31
    for row in inventory['entries']:
        source = ROOT / row['path']
        dest = AUDIT / 'data' / (source.parent.parent.name + '.loading_trace.jsonl.gz')
        temp = dest.with_name(dest.name + '.partial')
        assert digest(ROOT / row['manifest']) == row['manifest_sha256'], 'Original manifest changed'
        if not source.exists():
            entry = registry['entries'][row['path']]
            assert dest.is_file() and digest(dest) == entry['archive_sha256']
            decode_verify(dest, row, entry['lines'])
            journal('RESUME_REVERIFIED', path=row['path']); continue
        assert stable(source.lstat(), row['stat']), 'Source metadata changed'
        refs = fd_references(source, (row['stat']['dev'], row['stat']['ino']))
        assert not refs['found'], refs
        journal('START', path=row['path'], size=row['size'], references=refs)
        if not dest.exists():
            # Only this migration's unfinished output can be discarded, and only
            # after re-hashing the still-present original against its historical SHA.
            if temp.exists():
                assert stat.S_ISREG(temp.lstat().st_mode) and not temp.is_symlink()
                assert temp.stat().st_nlink == 1 and digest(source) == row['sha256']
                assert stable(source.lstat(), row['stat'])
                refs = fd_references(temp, (temp.stat().st_dev, temp.stat().st_ino))
                assert not refs['found'], refs
                journal('INTERRUPTED_TEMP_REMOVAL_VERIFIED',
                        path=str(temp.relative_to(ROOT)), size=temp.stat().st_size)
                temp.unlink(); sync_dir(temp.parent)
            try:
                line_count = compress_one(source, temp, row)
                decoded = decode_verify(temp, row, line_count)
                archive_sha = digest(temp)
                os.chmod(temp, 0o444)
                with temp.open('rb') as stream: os.fsync(stream.fileno())
                os.link(temp, dest)  # exclusive final commit: never overwrite another archive
                temp.unlink(); sync_dir(dest.parent)
            except BaseException:
                journal('FAILED_OR_INTERRUPTED', path=row['path'], raw_preserved=source.exists(),
                        partial_exists=temp.exists())
                raise
        else:
            decoded = decode_verify(dest, row)
            archive_sha = digest(dest)
        entry = {key:row[key] for key in ('sha256','size','stat')}
        entry.update(archive=str(dest.relative_to(ROOT)), archive_sha256=archive_sha,
                     lines=decoded['lines'], compressed_bytes=dest.stat().st_size,
                     compressed_allocated_bytes=dest.stat().st_blocks*512)
        registry['entries'][row['path']] = entry
        atomic_json(AUDIT / 'registry.json', registry)
        journal('VERIFIED_COMMITTED', path=row['path'], archive=entry['archive'],
                sha256=row['sha256'], size=row['size'], lines=entry['lines'],
                compressed_bytes=entry['compressed_bytes'], archive_sha256=archive_sha)
        refs = fd_references(source, (row['stat']['dev'], row['stat']['ino']))
        assert not refs['found'] and stable(source.lstat(), row['stat']), 'Delete gate failed'
        assert digest(ROOT / row['manifest']) == row['manifest_sha256']
        source.unlink(); sync_dir(source.parent)
        journal('RAW_REMOVED', path=row['path'], free_bytes=shutil.disk_usage(ROOT).free,
                references=refs)
    assert len(registry['entries']) == 31
    report = dict(status='PASS', files=31, original_bytes=inventory['original_bytes'],
                  original_allocated_bytes=inventory['original_allocated_bytes'],
                  compressed_bytes=sum(r['compressed_bytes'] for r in registry['entries'].values()),
                  compressed_allocated_bytes=sum(r['compressed_allocated_bytes'] for r in registry['entries'].values()),
                  raw_files_remaining=sum((ROOT / p).exists() for p in registry['entries']),
                  free_bytes=shutil.disk_usage(ROOT).free, finished_at=now())
    report['saved_allocated_bytes'] = report['original_allocated_bytes'] - report['compressed_allocated_bytes']
    atomic_json(AUDIT / 'migration_result.json', report)
    journal('MIGRATION_COMPLETE', **report)

if __name__ == '__main__':
    if sys.flags.optimize:
        raise RuntimeError('Optimized Python disables safety assertions; migration is refused')
    if os.environ.get('H1_TRACE_ARCHIVE_REGISTRY'):
        raise RuntimeError('Run migration without compatibility mapping')
    with (AUDIT / 'migration.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        {'prepare': prepare, 'migrate': migrate}[sys.argv[1]]()
