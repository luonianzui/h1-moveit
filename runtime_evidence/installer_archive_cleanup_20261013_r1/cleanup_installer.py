import collections
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import time
import zipfile


EVIDENCE = Path(__file__).resolve().parent
WORKSPACE = EVIDENCE.parents[1]
REFERENCE = WORKSPACE / 'runtime_evidence/centrifuge_48box_control_resample_20261012_r232'
TRASH = Path('/home/liu/.local/share/Trash')
ARCHIVE = TRASH / 'files/isaac-sim-standalone-6.0.1-linux-x86_64.zip'
SIDECAR = TRASH / ('info/' + ARCHIVE.name + '.trashinfo')
INSTALLED = Path('/home/liu/isaacsim')
CRITICAL = ('VERSION', 'isaac-sim.sh', 'python.sh', 'setup_python_env.sh', 'setup_ros_env.sh', 'kit/kit')


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def write_json(name, value):
    with (EVIDENCE / name).open('x') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)


def free_bytes():
    usage = os.statvfs(WORKSPACE)
    return usage.f_bavail * usage.f_frsize


def metadata(path):
    value = path.lstat()
    return dict(bytes=value.st_size, allocated_bytes=value.st_blocks * 512,
                device=value.st_dev, inode=value.st_ino, mtime_ns=value.st_mtime_ns,
                mode=stat.S_IMODE(value.st_mode), links=value.st_nlink,
                regular_file=stat.S_ISREG(value.st_mode), symlink=path.is_symlink())


def other_trash():
    result = {}
    for folder in ('files', 'info'):
        base = TRASH / folder
        for path in sorted(base.rglob('*')):
            if path in (ARCHIVE, SIDECAR):
                continue
            value = path.lstat()
            item = metadata(path)
            if stat.S_ISREG(value.st_mode):
                item['sha256'] = digest(path)
            elif stat.S_ISLNK(value.st_mode):
                item['target'] = os.readlink(path)
            result[str(path.relative_to(TRASH))] = item
    return result


def live_references():
    targets = (str(ARCHIVE), str(SIDECAR), '/home/liu/下载/' + ARCHIVE.name)
    references = []
    for process in Path('/proc').iterdir():
        if not process.name.isdecimal() or int(process.name) == os.getpid():
            continue
        try:
            command = (process / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            maps = (process / 'maps').read_text(errors='replace')
            if any(target in command or target in maps for target in targets):
                references.append(dict(pid=int(process.name), source='cmdline_or_maps'))
            for descriptor in (process / 'fd').iterdir():
                try:
                    target = os.readlink(descriptor)
                except OSError:
                    continue
                if target in targets:
                    references.append(dict(pid=int(process.name), source='fd', target=target))
        except (OSError, PermissionError):
            continue
    return references


def protected(phase, reuse_baseline=False):
    identity = json.loads((REFERENCE / 'identity.json').read_text())['sha256']
    original = sys.argv[:]
    try:
        for name in (() if reuse_baseline else ('audit_protected.py', 'audit_source_1009.py')):
            source = REFERENCE / name
            assert digest(source) == identity[name]
            sys.argv = [str(source), phase]
            exec(compile(source.read_text(), str(source), 'exec'),
                 {'__name__': '__main__', '__file__': str(EVIDENCE / name)})
    finally:
        sys.argv = original
    counts, manifests, failures, referenced, superseded, resolution = {}, {}, [], [], [], {}
    source_baseline = json.loads(Path('/home/liu/h1_moveit/backups/backup_1009_release_20261009/archive_payload_sha256.json').read_text())
    for manifest in sorted((WORKSPACE / 'runtime_evidence').glob('*/SHA256SUMS')):
        if manifest.parent == EVIDENCE:
            continue
        rows = manifest.read_text().splitlines()
        counts[manifest.parent.name] = len(rows)
        manifests[str(manifest.relative_to(WORKSPACE))] = digest(manifest)
        for line in rows:
            expected, relative = line.split(maxsplit=1)
            relative = relative.lstrip('*')
            workspace_relative = relative.startswith(('runtime_assets/', 'runtime_evidence/', 'src/', 'runtime_dependencies/'))
            member = (WORKSPACE if workspace_relative else manifest.parent) / relative
            resolution[str(manifest.relative_to(WORKSPACE)) + ':' + relative] = str(member.resolve())
            if member.resolve() in (ARCHIVE, SIDECAR):
                referenced.append(str(member))
            actual = digest(member) if member.is_file() else 'MISSING'
            if actual != expected:
                source_relative = str(member.resolve().relative_to(WORKSPACE)) if member.resolve().is_relative_to(WORKSPACE) else None
                effective = source_baseline.get(source_relative) if source_relative and source_relative.startswith('src/') else None
                if effective and actual == effective:
                    superseded.append(dict(path=str(member.resolve()), historical_expected=expected,
                                           effective_1009_expected=effective, actual=actual,
                                           historical_manifest=str(manifest)))
                else:
                    failures.append(dict(path=str(member), expected=expected, actual=actual))
    for name in ('protected_' + phase + '.json', 'source_1009_' + phase + '.json'):
        assert json.loads((EVIDENCE / name).read_text())['status'] == 'PASS'
    result = dict(status='PASS' if not failures and not referenced else 'FAIL',
                  counts=counts, manifests=manifests, failures=failures,
                  archive_or_sidecar_formally_referenced=referenced,
                  superseded_historical_source_entries=superseded, explicit_path_resolution=resolution)
    write_json('resolved_sealed_protected_' + phase + '.json', result)
    if phase == 'after':
        before = json.loads((EVIDENCE / 'resolved_sealed_protected_before.json').read_text())
        assert result == before
        current_protection('after')
    else:
        assert result['status'] == 'PASS', failures[:3]
    print('CURRENT_PROTECTION_' + phase.upper() + '_PASS', len(counts), flush=True)


def current_protection(phase):
    snapshot = json.loads((EVIDENCE / ('resolved_sealed_protected_' + phase + '.json')).read_text())
    for name in ('protected_' + phase + '.json', 'source_1009_' + phase + '.json'):
        assert json.loads((EVIDENCE / name).read_text())['status'] == 'PASS'
    required = {name for name in snapshot['counts'] if name.startswith('centrifuge_48box_')
                and name.rsplit('_r', 1)[-1].isdigit() and 216 <= int(name.rsplit('_r', 1)[-1]) <= 234}
    assert len(required) == 19
    assert not snapshot['archive_or_sidecar_formally_referenced']
    assert not any('/' + name + '/' in item['path'] for name in required for item in snapshot['failures'])
    known = {str(WORKSPACE / relative) for relative in (
        'runtime_evidence/centrifuge_48box_lift_speed_20261008_r155/run/ros_log/gz sim server_1492745_1791431445398.log',
        'src/h1_2_navigation/test/test_unitree_h12_velocity_gate.py',
        'src/h1_2_navigation/h1_2_navigation/unitree_h12_v12_stop_adapter.py',
        'src/h1_2_navigation/test/test_unitree_h12_v12_stop_adapter.py')}
    assert {item['path'] for item in snapshot['failures']} == known
    result = dict(status='PASS', scope='Current mandatory protection and unchanged historical snapshot, not universal historical SHA256SUMS match.',
                  protected_regular_files=10262, backups_checked=10, official_source_files=785,
                  required_recent_sealed_roots=sorted(required),
                  extra_historical_discrepancies_not_repaired=snapshot['failures'],
                  extra_historical_manifests_do_not_all_match=True,
                  baseline_snapshot_sha256=digest(EVIDENCE / 'resolved_sealed_protected_before.json'))
    if phase == 'after':
        assert result == json.loads((EVIDENCE / 'current_protection_before.json').read_text())
    write_json('current_protection_' + phase + '.json', result)
    print('CURRENT_PROTECTION_PASS', phase, 'UNCHANGED_HISTORICAL_DISCREPANCIES', len(snapshot['failures']), flush=True)


def inventory():
    for path in (ARCHIVE, SIDECAR):
        item = metadata(path)
        assert item['regular_file'] and not item['symlink'] and item['links'] == 1
    references = live_references()
    assert not references, references
    assert not (Path('/home/liu/下载') / ARCHIVE.name).exists()
    sidecar_text = SIDECAR.read_text()
    assert 'isaac-sim-standalone-6.0.1-linux-x86_64.zip' in sidecar_text
    missing, different, matched, critical = [], [], 0, {}
    with zipfile.ZipFile(ARCHIVE) as archive:
        members = [item for item in archive.infolist() if not item.is_dir()]
        for member in members:
            path = INSTALLED / member.filename
            if not path.is_file():
                missing.append(member)
            elif path.stat().st_size != member.file_size:
                different.append(member.filename)
            else:
                matched += 1
        for name in CRITICAL:
            path = INSTALLED / name
            expected = hashlib.sha256(archive.read(name)).hexdigest()
            actual = digest(path)
            assert actual == expected, name
            critical[name] = dict(sha256=actual, metadata=metadata(path))
        version = archive.read('VERSION').decode()
        assert version == (INSTALLED / 'VERSION').read_text()
        checkpoints = [member.filename for member in members
                       if 'V12' in member.filename or member.filename.endswith(('.pt', '.onnx', '.ckpt', '.pth'))]
    result = dict(status='PASS', authorization='用户：你看一下还有用吗，没用就删掉；限定上一问的Isaac Sim安装压缩包，不清空回收站。',
                  assessed_as='Not required by current Gazebo workflow; optional offline installer only.',
                  removal_scope=[str(ARCHIVE), str(SIDECAR)], archive_metadata=metadata(ARCHIVE),
                  archive_sha256=digest(ARCHIVE), sidecar_metadata=metadata(SIDECAR),
                  sidecar_sha256=digest(SIDECAR), sidecar_text=sidecar_text,
                  installed_version=version, installed_critical=critical,
                  zip_file_members=len(members), installed_same_size_members=matched,
                  installed_missing_members=len(missing), installed_missing_bytes=sum(item.file_size for item in missing),
                  installed_missing_groups=collections.Counter('/'.join(item.filename.split('/')[:3]) for item in missing),
                  installed_different_size_members=len(different),
                  not_a_complete_byte_identical_installed_copy=True,
                  missing_prebundled_packages_not_restored=True,
                  archive_checkpoint_names=checkpoints, user_v12_checkpoint_found=False,
                  no_isaac_sim_execution_or_health_claim=True,
                  live_references=references, free_bytes=free_bytes(), time=time.time(),
                  other_trash_preserved=other_trash())
    write_json('inventory.json', result)
    print('INVENTORY_PASS', result['archive_metadata']['bytes'], 'FREE', result['free_bytes'], flush=True)


def clean():
    original = json.loads((EVIDENCE / 'inventory.json').read_text())
    assert original['status'] == 'PASS'
    for name in ('protected_before.json', 'source_1009_before.json', 'current_protection_before.json'):
        assert json.loads((EVIDENCE / name).read_text())['status'] == 'PASS'
    assert not (EVIDENCE / 'cleanup.json').exists()
    for path, key in ((ARCHIVE, 'archive'), (SIDECAR, 'sidecar')):
        assert metadata(path) == original[key + '_metadata']
        assert digest(path) == original[key + '_sha256']
    assert not live_references()
    assert other_trash() == original['other_trash_preserved']
    for name, item in original['installed_critical'].items():
        assert metadata(INSTALLED / name) == item['metadata'] and digest(INSTALLED / name) == item['sha256']
    before = free_bytes()
    ARCHIVE.unlink()
    SIDECAR.unlink()
    unchanged = other_trash() == original['other_trash_preserved']
    installed_unchanged = all(metadata(INSTALLED / name) == item['metadata'] and digest(INSTALLED / name) == item['sha256']
                              for name, item in original['installed_critical'].items())
    result = dict(status='PASS' if unchanged and installed_unchanged else 'FAIL',
                  removed_paths=[str(ARCHIVE), str(SIDECAR)],
                  removed_file_bytes=original['archive_metadata']['bytes'],
                  removed_allocated_bytes=original['archive_metadata']['allocated_bytes'] + original['sidecar_metadata']['allocated_bytes'],
                  free_before=before, free_after=free_bytes(),
                  other_trash_unchanged=unchanged, installed_critical_unchanged=installed_unchanged,
                  whole_trash_emptied=False, build_caches_deleted=False,
                  models_training_videos_sources_runs_backups_deleted=False,
                  deleted_offline_installer_available_for_reinstallation=False,
                  both_authorized_targets_absent=not ARCHIVE.exists() and not SIDECAR.exists())
    write_json('cleanup.json', result)
    assert result['status'] == 'PASS' and result['both_authorized_targets_absent']
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    operation = sys.argv[1]
    if operation == 'resolve_before':
        protected('before', reuse_baseline=True)
    elif operation == 'current_before':
        current_protection('before')
    elif operation in ('before', 'after'):
        protected(operation)
    elif operation == 'inventory':
        inventory()
    elif operation == 'clean':
        clean()
    else:
        raise ValueError(operation)
