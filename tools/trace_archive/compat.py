"""Opt-in, read-only Python view of the 31 archived historical trace files.

This is deliberately not a filesystem mount. Native file descriptors, directory
enumeration and external programs continue to see the physical filesystem.
"""

import builtins
import errno
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import re
import stat
import threading


ENVIRONMENT = 'H1_TRACE_ARCHIVE_REGISTRY'
ARCHIVE_ROOT = 'runtime_evidence/trace_archive_20261014_r1'
_NAMES = (
    *(f'centrifuge_48box_full_cycle_20261009_r{n}' for n in
      (161, 162, 163, 164, 165, 166, 167, 171, 172, 175, 176, 177, 178, 181, 183, 185)),
    'centrifuge_48box_full_cycle_20261010_r188',
    'centrifuge_48box_single_extraction_20261010_r190',
    'centrifuge_48box_full_cycle_20261010_r193',
    'centrifuge_48box_soft_grasp_20261010_r194',
    'centrifuge_48box_full_cycle_20261010_r195',
    'centrifuge_48box_clearance_unloading_20261010_r196',
    'centrifuge_48box_terminal_unloading_20261010_r198',
    'centrifuge_48box_leg_posture_unloading_20261010_r199',
    'centrifuge_48box_setdown_braking_20261010_r201',
    'centrifuge_48box_setdown_braking_20261010_r205',
    'centrifuge_48box_full_cycle_20261010_r206',
    'centrifuge_48box_pilot_braking_20261010_r208',
    'centrifuge_48box_fast_complete_20261010_r209',
    'centrifuge_48box_bounded_speed_20261010_r210',
    'centrifuge_48box_button_speed_20261010_r211',
)
ALLOWED_PATHS = frozenset(f'runtime_evidence/{name}/run/loading_trace.jsonl' for name in _NAMES)
_OPEN, _IO_OPEN, _STAT, _LSTAT = builtins.open, io.open, os.stat, os.lstat
_ACTIVE = None


class ArchiveIntegrityError(OSError):
    """An archived representation does not satisfy its registered identity."""


def _absolute(path):
    return os.path.abspath(os.fsdecode(os.fspath(path)))


def _regular_stat(path):
    value = _LSTAT(path)
    if not stat.S_ISREG(value.st_mode):
        raise ArchiveIntegrityError(f'Archive must be a regular file, not a symlink: {path}')
    return value


def _signature(value):
    return (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns)


class Registry:
    def __init__(self, filename):
        self.filename = _absolute(filename)
        with _OPEN(self.filename, encoding='utf-8') as stream:
            document = json.load(stream)
        if document.get('version') != 1 or not isinstance(document.get('entries'), dict):
            raise ArchiveIntegrityError('Invalid trace archive registry schema')
        workspace = document.get('workspace')
        if not isinstance(workspace, str) or not os.path.isabs(workspace):
            raise ArchiveIntegrityError('Registry workspace must be an absolute path')
        self.workspace = _absolute(workspace)
        self.entries = {}
        self.relative_entries = {}
        self._archive_verified = {}
        self._logical_verified = {}
        self._lock = threading.RLock()
        for relative, entry in document['entries'].items():
            if relative not in ALLOWED_PATHS or not isinstance(entry, dict):
                raise ArchiveIntegrityError(f'Unapproved historical trace: {relative}')
            expected_archive = f'{ARCHIVE_ROOT}/data/{relative.split("/")[1]}.loading_trace.jsonl.gz'
            if entry.get('archive') != expected_archive:
                raise ArchiveIntegrityError(f'Unapproved archive location: {relative}')
            for key in ('sha256', 'archive_sha256'):
                if not isinstance(entry.get(key), str) or not re.fullmatch('[0-9a-f]{64}', entry[key]):
                    raise ArchiveIntegrityError(f'Invalid {key}: {relative}')
            for key in ('size', 'lines'):
                if type(entry.get(key)) is not int or entry[key] < 0:
                    raise ArchiveIntegrityError(f'Invalid {key}: {relative}')
            metadata = entry.get('stat', {})
            integer_fields = ('mode', 'ino', 'dev', 'nlink', 'uid', 'gid', 'size',
                              'atime_ns', 'mtime_ns', 'ctime_ns')
            if (not isinstance(metadata, dict) or
                    any(type(metadata.get(key)) is not int for key in integer_fields) or
                    any(type(metadata.get(key)) not in (int, float) for key in ('atime', 'mtime', 'ctime')) or
                    metadata['size'] != entry['size'] or not stat.S_ISREG(metadata['mode'])):
                raise ArchiveIntegrityError(f'Invalid original stat metadata: {relative}')
            self.relative_entries[relative] = entry
            self.entries[os.path.join(self.workspace, relative)] = entry

    def lookup(self, path):
        if isinstance(path, int):
            return None, None
        try:
            absolute = _absolute(path)
        except TypeError:
            return None, None
        return absolute, self.entries.get(absolute)

    def archive_path(self, entry):
        return os.path.join(self.workspace, entry['archive'])

    def validate_archive(self, entry):
        """Check the physical compressed bytes once per unchanged file identity."""
        path = self.archive_path(entry)
        with self._lock:
            before = _regular_stat(path)
            signature = _signature(before)
            if self._archive_verified.get(path) != signature:
                digest = hashlib.sha256()
                with _OPEN(path, 'rb') as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                        digest.update(chunk)
                if digest.hexdigest() != entry['archive_sha256']:
                    raise ArchiveIntegrityError(f'Compressed SHA-256 mismatch: {path}')
                if _signature(_regular_stat(path)) != signature:
                    raise ArchiveIntegrityError(f'Archive changed while validating: {path}')
                self._archive_verified[path] = signature
                self._logical_verified.pop(path, None)
            return signature

    def verify_logical(self, entry):
        signature = self.validate_archive(entry)
        path = self.archive_path(entry)
        if self._logical_verified.get(path) != signature:
            digest, size, lines = hashlib.sha256(), 0, 0
            with gzip.open(path, 'rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
                    size += len(chunk)
                    lines += chunk.count(b'\n')
            self.check_logical(entry, digest.hexdigest(), size, lines, signature)
        return signature

    def check_logical(self, entry, digest, size, lines, signature):
        path = self.archive_path(entry)
        if digest != entry['sha256'] or size != entry['size'] or lines != entry['lines']:
            raise ArchiveIntegrityError(f'Decompressed SHA-256, size or line count mismatch: {path}')
        if _signature(_regular_stat(path)) != signature:
            raise ArchiveIntegrityError(f'Archive changed while reading: {path}')
        self._logical_verified[path] = signature

    def virtual_stat(self, entry):
        self.validate_archive(entry)
        value = entry['stat']
        # This is explicitly the logical original-file view, not disk allocation.
        return os.stat_result(
            (value['mode'], value['ino'], value['dev'], value['nlink'], value['uid'],
             value['gid'], value['size'], value['atime'], value['mtime'], value['ctime']),
            {f'st_{key}': value[key] for key in ('atime_ns', 'mtime_ns', 'ctime_ns')})


class _TraceReader(io.BufferedIOBase):
    """A gzip stream that checks logical identity when consumed to EOF."""

    def __init__(self, registry, entry, original):
        super().__init__()
        self.registry, self.entry, self.name = registry, entry, original
        self.mode = 'rb'
        self.signature = registry.validate_archive(entry)
        self._stream = gzip.open(registry.archive_path(entry), 'rb')
        self._digest, self._size, self._lines = hashlib.sha256(), 0, 0
        self._tracking = True
        self._complete = False

    def readable(self):
        return True

    def seekable(self):
        return True

    def _record(self, data, eof=False):
        if self._tracking and not self._complete:
            self._digest.update(data)
            self._size += len(data)
            self._lines += data.count(b'\n')
            if self._size > self.entry['size']:
                raise ArchiveIntegrityError(f'Decompressed trace exceeds registered size: {self.name}')
            if eof:
                self.registry.check_logical(self.entry, self._digest.hexdigest(),
                                            self._size, self._lines, self.signature)
                self._complete = True
        return data

    def read(self, size=-1):
        data = self._stream.read(size)
        return self._record(data, size is None or size < 0 or (size != 0 and not data))

    def read1(self, size=-1):
        data = self._stream.read1(size)
        return self._record(data, size != 0 and not data)

    def readline(self, size=-1):
        data = self._stream.readline(size)
        return self._record(data, size != 0 and not data)

    def readinto(self, buffer):
        data = self.read(len(buffer))
        buffer[:len(data)] = data
        return len(data)

    def tell(self):
        return self._stream.tell()

    def seek(self, offset, whence=os.SEEK_SET):
        self.registry.verify_logical(self.entry)
        result = self._stream.seek(offset, whence)
        self._tracking = False
        return result

    def fileno(self):
        raise io.UnsupportedOperation('Archived traces have no raw file descriptor; use the Python stream')

    def close(self):
        try:
            self._stream.close()
        finally:
            super().close()


def _readonly_open(file, mode='r', buffering=-1, encoding=None, errors=None,
                   newline=None, closefd=True, opener=None):
    absolute, entry = _ACTIVE.lookup(file)
    if entry is None:
        return _OPEN(file, mode, buffering, encoding, errors, newline, closefd, opener)
    if mode not in ('r', 'rt', 'rb'):
        raise PermissionError(errno.EROFS, 'Archived historical traces are read-only', absolute)
    try:
        _STAT(absolute)
    except FileNotFoundError:
        pass
    else:
        return _OPEN(file, mode, buffering, encoding, errors, newline, closefd, opener)
    if not closefd:
        raise ValueError('Cannot use closefd=False with a file name')
    if opener is not None or buffering == 0:
        raise io.UnsupportedOperation('Custom opener and unbuffered access are unavailable for archived traces')
    if mode == 'rb' and any(value is not None for value in (encoding, errors, newline)):
        raise ValueError('Binary mode does not accept encoding, errors or newline')
    stream = _TraceReader(_ACTIVE, entry, absolute)
    if mode == 'rb':
        return stream
    try:
        return io.TextIOWrapper(stream, encoding=encoding, errors=errors, newline=newline)
    except BaseException:
        stream.close()
        raise


def _logical_stat(path, *, dir_fd=None, follow_symlinks=True):
    try:
        return _STAT(path, dir_fd=dir_fd, follow_symlinks=follow_symlinks)
    except FileNotFoundError:
        if dir_fd is not None:
            raise
        _, entry = _ACTIVE.lookup(path)
        if entry is None:
            raise
        return _ACTIVE.virtual_stat(entry)


def install(registry_path):
    """Install only in this interpreter; descendants opt in via their environment."""
    global _ACTIVE
    if _ACTIVE is not None:
        if _ACTIVE.filename != _absolute(registry_path):
            raise ArchiveIntegrityError('A different trace registry is already installed')
        return _ACTIVE
    registry = Registry(registry_path)
    _ACTIVE = registry
    builtins.open = _readonly_open
    io.open = _readonly_open
    os.stat = _logical_stat
    return registry


def default_registry():
    return str(Path(__file__).resolve().parents[2] / ARCHIVE_ROOT / 'registry.json')
