"""Metadata-only selection for the source snapshot; never modify source files.

collect_candidates(source, config) returns ({relative_path: reason}, exclusion
counts). Counts include rejected files and pruned directories, not the unknown
number of files below a pruned directory. allowed_file returns (bool, reason).
No payloads are read, hashes computed, commands executed, or timestamps used to
choose what is new. Copying callers must separately check content and races.
"""
from __future__ import annotations

from collections import Counter
import os
from pathlib import Path, PurePosixPath
import re
import stat
from typing import Any

_GENERATED = frozenset({
    '.git', '.github_sync', '__pycache__', '.pytest_cache', '.mypy_cache',
    '.ruff_cache', '.cache', '.tox', '.venv', 'venv', 'node_modules',
    'build', 'install', 'log', 'logs', 'cache', 'colcon_log', 'cmakefiles',
    'dist', 'coverage', 'htmlcov',
})
_PRIVATE_DIRS = frozenset({'.ssh', '.aws', '.gnupg', '.secrets'})
_PRIVATE_NAMES = frozenset({
    '.netrc', '.npmrc', '.pypirc', '.git-credentials', 'credentials',
    'credentials.json', 'credentials.yml', 'credentials.yaml', 'hosts.yml',
    'id_rsa', 'id_dsa', 'id_ecdsa', 'id_ed25519',
})
_BLOCKED_SUFFIXES = frozenset({
    '.pyc', '.pyo', '.so', '.o', '.a', '.dll', '.dylib', '.exe', '.bin',
    '.mp4', '.mkv', '.avi', '.mov', '.webm', '.m4v', '.wmv', '.mpeg', '.mpg',
    '.mp3', '.wav', '.flac', '.ogg', '.aac', '.db3', '.mcap', '.cdr', '.bag',
    '.sqlite', '.sqlite3', '.jsonl', '.ndjson', '.log', '.stdout', '.stderr', '.prof', '.profile',
    '.perf', '.tar', '.gz', '.gzip', '.bz2', '.xz', '.zst', '.zip', '.7z',
    '.rar', '.tgz', '.tbz2', '.txz', '.deb', '.rpm', '.pem', '.p12', '.pfx', '.key',
})
_EVIDENCE_SUFFIXES = frozenset({
    '.py', '.sh', '.bash', '.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp',
    '.hxx', '.cmake', '.json', '.yaml', '.yml', '.toml', '.ini', '.cfg',
    '.config', '.xml', '.xacro', '.urdf', '.sdf', '.rviz', '.qml', '.qrc',
    '.msg', '.srv', '.action', '.idl', '.md', '.rst', '.txt', '.csv', '.tsv', '.svg',
})
_EVIDENCE_NAMES = frozenset({
    'CMakeLists.txt', 'SHA256SUMS', 'SHA256SUMS.sha256', 'Makefile', 'Dockerfile',
    'LICENSE', 'COPYING', 'NOTICE', 'COLCON_IGNORE', 'requirements.txt', 'run',
})
_RUNTIME_ASSET_SUFFIXES = frozenset({
    '.py', '.sh', '.cpp', '.c', '.cc', '.cxx', '.h', '.hh', '.hpp', '.hxx',
    '.json', '.yaml', '.yml', '.xml', '.xacro', '.urdf', '.sdf', '.rviz',
    '.md', '.rst', '.stl', '.dae', '.obj', '.mtl', '.png', '.jpg', '.jpeg',
    '.csv', '.svg', '.cmake', '.qml', '.qrc', '.config', '.toml', '.cfg',
})


def blocked_path(rel: str) -> str | None:
    """Return a path-only exclusion category, without filesystem access."""
    if not isinstance(rel, str) or not rel or '\\' in rel or '\x00' in rel:
        return 'invalid_relative_path'
    parts = rel.split('/')
    if rel.startswith('/') or any(p in ('', '.', '..') for p in parts):
        return 'invalid_relative_path'
    for part in (p.lower() for p in parts):
        if part in _GENERATED:
            return 'generated_directory'
        if not PurePosixPath(part).suffix and part.startswith(('build_', 'install_', 'log_', 'cache_', 'colcon_log_')):
            # Authored build_cache_cleanup_* evidence is not a build cache.
            if 'cleanup' not in part:
                return 'generated_directory'
        if part in _PRIVATE_DIRS:
            return 'credential_directory'
    name = parts[-1].lower()
    if name in _PRIVATE_NAMES or name == '.env' or name.startswith('.env.'):
        return 'credential_filename'
    if name.endswith('.pub') and name.startswith('id_'):
        return 'credential_filename'
    if '.so.' in name or name.endswith('.so'):
        return 'shared_library'
    if re.search(r'\.(?:prof|profile|perf)(?:\.|$)', name):
        return 'profiling_data'
    if PurePosixPath(name).suffix in _BLOCKED_SUFFIXES:
        return 'excluded_file_type'
    return None


def allowed_file(path: Path, rel: str, config: dict[str, Any],
                 evidence: bool = False, *, legacy_selected: bool = False
                 ) -> tuple[bool, str | None]:
    """Check metadata only. Exact historical compact .log files are an exception.

    legacy_selected never exempts paths, symlinks, binaries or size limits; it
    only allows a .log explicitly listed in historical_evidence_files or
    legacy_source_files from the original published snapshot.
    """
    path = Path(path)
    legacy_log = False
    if legacy_selected and path.suffix.lower() == '.log':
        legacy_names = {v['path'] if isinstance(v, dict) else v for v in
                        config.get('historical_evidence_files', []) + config.get('legacy_source_files', [])}
        legacy_log = rel in legacy_names
    failure = blocked_path(rel)
    if failure and not (legacy_log and failure == 'excluded_file_type'):
        return False, failure
    try:
        metadata = path.lstat()
        if stat.S_ISLNK(metadata.st_mode):
            return False, 'symlink'
        if not stat.S_ISREG(metadata.st_mode):
            return False, 'not_regular_file'
        for ancestor in list(path.parents)[:len(PurePosixPath(rel).parts)]:
            if ancestor.is_symlink():
                return False, 'symlink_ancestor'
    except FileNotFoundError:
        return False, 'missing'
    except OSError:
        return False, 'unreadable_metadata'
    limit = int(config.get('max_file_bytes', 50 * 1024 * 1024))
    if evidence:
        limit = min(limit, int(config.get('evidence_max_file_bytes', 5 * 1024 * 1024)))
        if not legacy_log and path.suffix.lower() not in _EVIDENCE_SUFFIXES and path.name not in _EVIDENCE_NAMES:
            return False, 'evidence_type_not_allowed'
    elif rel.startswith('runtime_assets/'):
        # Match the original snapshot's sources-only policy here. These old
        # execution bundles also contain extensionless ELF and raw telemetry.
        if path.suffix.lower() not in _RUNTIME_ASSET_SUFFIXES and path.name not in {'CMakeLists.txt', 'SHA256SUMS'}:
            return False, 'runtime_asset_type_not_allowed'
    if limit <= 0:
        raise ValueError('File size limits must be positive')
    if metadata.st_size > limit:
        return False, 'evidence_size_limit' if evidence else 'file_size_limit'
    return True, None


def _relative(value: str) -> str:
    if (not isinstance(value, str) or not value or value.startswith('/') or
            '\\' in value or any(p in ('', '.', '..') for p in value.split('/'))):
        raise ValueError('Configured selections must be normalized relative paths')
    return value


def _evidence_dir(value: str) -> str:
    value = _relative(value)
    if '/' not in value:
        value = 'runtime_evidence/' + value
    parts = value.split('/')
    if len(parts) != 2 or parts[0] != 'runtime_evidence':
        raise ValueError('Evidence directory entries must name a direct runtime_evidence child')
    return value


def collect_candidates(source: Path, config: dict[str, Any]
                       ) -> tuple[dict[str, str], dict[str, int]]:
    """Return deterministic candidates and exclusion counts, using stat only."""
    source = Path(source)
    if not source.is_absolute() or source.is_symlink() or not source.is_dir():
        raise ValueError('source must be an existing absolute non-symlink directory')
    selected: dict[str, str] = {}
    excluded: Counter[str] = Counter()

    def offer(path: Path, reason: str, *, evidence=False, legacy=False):
        rel = path.relative_to(source).as_posix()
        ok, failure = allowed_file(path, rel, config, evidence=evidence, legacy_selected=legacy)
        if ok:
            selected.setdefault(rel, reason)
        else:
            excluded[failure or 'unknown'] += 1

    def walk(relative: str, reason: str, *, evidence=False):
        base = source / _relative(relative)
        failure = blocked_path(relative)
        if failure:
            excluded[failure] += 1
            return
        if base.is_symlink():
            excluded['symlink'] += 1
            return
        if not base.is_dir():
            excluded['missing_tree'] += 1
            return
        for ancestor in list(base.parents)[:len(PurePosixPath(relative).parts)]:
            if ancestor.is_symlink():
                excluded['symlink_ancestor'] += 1
                return
        def onerror(_error):
            excluded['unreadable_directory'] += 1
        for current, dirs, files in os.walk(base, followlinks=False, onerror=onerror):
            current = Path(current)
            retained = []
            for name in sorted(dirs):
                child = current / name
                if child.is_symlink():
                    excluded['symlink'] += 1
                elif (failure := blocked_path(child.relative_to(source).as_posix())):
                    excluded[failure] += 1
                else:
                    retained.append(name)
            dirs[:] = retained
            for name in sorted(files):
                offer(current / name, reason, evidence=evidence)

    for tree in config.get('source_trees', ['src', 'tools', '文档', 'runtime_assets']):
        walk(tree, f'Source tree: {tree}')
    root_names = set(config.get('root_files', ['.gitignore']))
    for path in sorted(source.iterdir()):
        if path.suffix.lower() in ('.md', '.sh') or path.name in root_names:
            offer(path, 'Workspace root documentation or entry point')
    for item in config.get('historical_evidence_files', []):
        rel = _relative(item['path'] if isinstance(item, dict) else item)
        if not rel.startswith('runtime_evidence/'):
            raise ValueError('Historical evidence files must be within runtime_evidence')
        reason = 'Legacy selected compact evidence (exact historical path)'
        offer(source / rel, reason, evidence=True, legacy=True)
    for item in config.get('legacy_source_files', []):
        rel = _relative(item['path'] if isinstance(item, dict) else item)
        offer(source / rel, 'Legacy selected compact evidence (exact published source attachment)', legacy=True)
    historical = {_evidence_dir(v) for v in config.get('historical_evidence_dirs', [])}
    extra = {_evidence_dir(v) for v in config.get('extra_evidence_dirs', [])}
    base = source / 'runtime_evidence'
    if base.is_symlink():
        excluded['symlink'] += 1
    elif base.is_dir():
        seen_extra = set()
        for path in sorted(base.iterdir()):
            rel = path.relative_to(source).as_posix()
            if rel in extra:
                seen_extra.add(rel)
            if path.is_symlink():
                excluded['symlink'] += 1
                continue
            if not path.is_dir():
                continue
            if rel in historical and rel not in extra:
                excluded['historical_directory_not_recursed'] += 1
                continue
            reason = ('Explicitly selected historical evidence directory' if rel in extra
                      else 'New evidence directory outside the fixed historical boundary')
            walk(rel, reason, evidence=True)
        excluded['missing_extra_evidence_directory'] += len(extra - seen_extra)
    else:
        excluded['missing_evidence_root'] += 1
    return dict(sorted(selected.items())), dict(sorted((k, v) for k, v in excluded.items() if v))
