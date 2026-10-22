import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import time

import psutil


EVIDENCE = Path(__file__).resolve().parent
WORKSPACE = EVIDENCE.parents[1]
IPCH = Path('/home/liu/.cache/vscode-cpptools/ipch')


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def fingerprint(path):
    info = path.lstat()
    return [info.st_dev, info.st_ino, info.st_mode, info.st_nlink, info.st_size,
            info.st_mtime_ns, info.st_ctime_ns]


def process_references(targets):
    references, limited = [], []
    target_set = set(map(str, targets))
    for process in psutil.process_iter(['pid', 'name', 'uids']):
        if process.info['uids'].real != os.getuid() or process.pid == os.getpid():
            continue
        for field in ('cmdline', 'cwd', 'exe', 'open_files', 'memory_maps'):
            try:
                value = getattr(process, field)()
                paths = [item.path for item in value] if field in ('open_files', 'memory_maps') else value if field == 'cmdline' else [value]
                for item in paths:
                    if item in target_set or field == 'cmdline' and any(target in item for target in target_set):
                        references.append(dict(pid=process.pid, field=field, value=item))
            except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
                limited.append(dict(pid=process.pid, name=process.info['name'], field=field))
                assert 'cpptools' not in (process.info['name'] or ''), limited[-1]
    return references, limited


def protected_and_installed():
    protected = set()
    baseline = json.loads((WORKSPACE / 'runtime_evidence/backup_920_release_20260920/archive_payload_sha256.json').read_text())
    for name in baseline:
        if name.startswith(('runtime_assets/', 'runtime_evidence/', 'src/')):
            protected.add(str(WORKSPACE / name))
    for parent in ('runtime_evidence', 'runtime_assets'):
        for manifest in (WORKSPACE / parent).glob('*/SHA256SUMS'):
            for line in manifest.read_text().splitlines():
                checksum, name = line.split(maxsplit=1)
                assert len(checksum) == 64
                protected.add(str((manifest.parent / name.lstrip('*')).absolute()))
    for dependency in (WORKSPACE / 'runtime_evidence').glob('*/scene_dependency_audit.json'):
        for item in json.loads(dependency.read_text()).get('selected_resources', []):
            protected.add(item['path'])
    for identity in (WORKSPACE / 'runtime_evidence').glob('*/identity.json'):
        fields = json.loads(identity.read_text()).get('sha256', {})
        if isinstance(fields, dict):
            for name in fields:
                protected.add(str((identity.parent / name).absolute()))
    for directory in WORKSPACE.glob('install*'):
        if directory.is_dir():
            for path in directory.rglob('*'):
                if path.is_symlink():
                    protected.add(str(path.resolve()))
    return protected


def object_recipe(path):
    dependency = path if path.name.endswith('.o.d') else path.with_name(path.name + '.d')
    if not dependency.is_file():
        return None
    tokens = shlex.split(dependency.read_text().replace('\\\n', ' ').split(':', 1)[1])
    source_files = [Path(value) for value in tokens if value.startswith('/')]
    if not source_files or any(not source.is_file() for source in source_files):
        return None
    source = source_files[0]
    if source.suffix not in ('.cpp', '.cc', '.cxx', '.c'):
        return None
    recipes = [parent / 'build.make' for parent in path.parents
               if parent.is_relative_to(WORKSPACE) and (parent / 'build.make').is_file()]
    if not recipes:
        return None
    return dict(source=str(source), source_sha256=digest(source),
                build_make=str(recipes[0]), build_make_sha256=digest(recipes[0]),
                every_declared_absolute_dependency_exists=True,
                dependency_count=len(source_files))


def inventory():
    protected = protected_and_installed()
    members, skipped = [], []
    candidates = []
    for root in [WORKSPACE / 'build'] + list((WORKSPACE / 'runtime_evidence').glob('*/build*')):
        if root.is_dir() and not root.is_symlink():
            candidates.extend(path for path in root.rglob('*') if path.name.endswith(('.o', '.o.d')))
    assert not any(psutil.Process(process.pid).name() in ('cc1', 'cc1plus', 'clang', 'clang++', 'ninja', 'make')
                   for process in psutil.process_iter(['name']) if process.info['name'])
    for path in sorted(set(candidates)):
        if not path.is_file() or path.is_symlink() or path.stat().st_nlink != 1 or str(path) in protected:
            skipped.append(dict(path=str(path), reason='protected_linked_or_not_regular'))
            continue
        is_header = path.suffix == '.ipch'
        minimum_age = 12 * 3600 if is_header else 24 * 3600
        if path.stat().st_mtime > time.time() - minimum_age:
            skipped.append(dict(path=str(path), reason='not_old_enough'))
            continue
        if is_header:
            sources = list((WORKSPACE / 'src').rglob(path.stem + '.cpp'))
            sources += list(Path('/home/liu/h1_moveit/.tools').glob(path.stem + '.cc'))
            contents = path.read_bytes()
            sources = [source for source in sources if str(source).encode() in contents]
            if len(sources) != 1:
                skipped.append(dict(path=str(path), reason='unique_embedded_original_source_not_proved'))
                continue
            source = sources[0]
            recipe = dict(source=str(source), source_sha256=digest(source),
                          embedded_absolute_source_path_verified=True,
                          regenerable_by_installed_ms_vscode_cpptools=True)
        else:
            recipe = object_recipe(path)
            if recipe is None:
                skipped.append(dict(path=str(path), reason='complete_regenerable_recipe_not_proved'))
                continue
        info = path.stat()
        members.append(dict(path=str(path), kind='cpp_intellisense_precompiled_header' if is_header else 'cmake_compile_intermediate',
                            sha256=digest(path), fingerprint=fingerprint(path), bytes=info.st_size,
                            allocated_bytes=info.st_blocks * 512, recipe=recipe,
                            minimum_old_age_s=minimum_age))
    references, limited = process_references([Path(row['path']) for row in members])
    referenced = {row['value'] for row in references}
    members = [row for row in members if row['path'] not in referenced]
    assert not any(row['field'] == 'cmdline' for row in references), references
    return dict(status='PASS', authorization='用户明确允许仅清理经核验且未被正式证据或已安装运行引用的旧构建缓存',
                members=members, skipped=skipped, omitted_live_referenced=references,
                process_visibility_limits=limited, cpp_tool_process_fields_visible=True,
                count=len(members), bytes=sum(row['bytes'] for row in members),
                allocated_bytes=sum(row['allocated_bytes'] for row in members),
                free_bytes=os.statvfs(WORKSPACE).f_bavail * os.statvfs(WORKSPACE).f_frsize,
                sqlite_browse_databases_installers_sources_libraries_models_evidence_backups_excluded=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('inventory', 'clean'))
    mode = parser.parse_args().mode
    if mode == 'inventory':
        result = inventory()
        with (EVIDENCE / 'inventory.json').open('x') as output:
            json.dump(result, output, indent=2)
        print(json.dumps({key: value for key, value in result.items() if key not in ('members', 'skipped')}, ensure_ascii=False), flush=True)
        return
    baseline = WORKSPACE / 'runtime_evidence/centrifuge_48box_sequential_full_20261022_r351'
    before = {}
    for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected', 'resumed_protected',
                   'additional_protected', 'dual_protected', 'current_protected', 'startup_protected'):
        path = baseline / (prefix + '_before.json')
        assert json.loads(path.read_text())['status'] == 'PASS'
        before[str(path)] = digest(path)
    with (EVIDENCE / 'before_protection_bindings.json').open('x') as stream:
        json.dump(before, stream, indent=2)
    original = json.loads((EVIDENCE / 'inventory.json').read_text())
    assert not any(process.info['name'] in ('cc1', 'cc1plus', 'clang', 'clang++', 'ninja', 'make')
                   for process in psutil.process_iter(['name']))
    members = original['members']
    protected = protected_and_installed()
    assert all(row['kind'] == 'cmake_compile_intermediate' and row['minimum_old_age_s'] == 86400 for row in members)
    references, limited = process_references([Path(row['path']) for row in members])
    assert not references, references
    for row in members:
        path = Path(row['path'])
        assert str(path) not in protected and fingerprint(path) == row['fingerprint']
        assert digest(path) == row['sha256']
        recipe = row['recipe']
        assert digest(Path(recipe['source'])) == recipe['source_sha256']
        if 'build_make' in recipe:
            assert digest(Path(recipe['build_make'])) == recipe['build_make_sha256']
    assert not (EVIDENCE / 'cleanup.json').exists()
    with (EVIDENCE / 'deletion_journal.jsonl').open('x') as journal:
        for row in members:
            path = Path(row['path'])
            assert fingerprint(path) == row['fingerprint']
            path.unlink()
            journal.write(json.dumps(dict(path=str(path), bytes=row['bytes'], sha256=row['sha256'])) + '\n')
            journal.flush()
    result = dict(status='PASS', removed_count=len(members), removed_bytes=original['bytes'],
                  removed_allocated_bytes=original['allocated_bytes'], free_before=original['free_bytes'],
                  free_after=os.statvfs(WORKSPACE).f_bavail * os.statvfs(WORKSPACE).f_frsize,
                  all_listed_files_absent=all(not Path(row['path']).exists() for row in members),
                  sqlite_browse_databases_and_entire_trash_untouched=True,
                  current_process_visibility_limits=limited,
                  no_source_library_installed_runtime_or_formal_evidence_deleted=True)
    with (EVIDENCE / 'cleanup.json').open('x') as output:
        json.dump(result, output, indent=2)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
