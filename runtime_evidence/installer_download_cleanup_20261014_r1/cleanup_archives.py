import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import psutil


EVIDENCE = Path(__file__).resolve().parent
WORKSPACE = EVIDENCE.parent.parent
CACHE = Path('/home/liu/.cache/codex-update-manager')
TARGETS = {
    CACHE / 'downloads/Codex-2ddfc5863814a6850b1fce952dcc92d922ce53a54a9a445d9fdc755e49b82aa0.dmg':
        (623791660, '2ddfc5863814a6850b1fce952dcc92d922ce53a54a9a445d9fdc755e49b82aa0'),
    CACHE / 'packages/chatgpt-26.1007.21434-amd64-276b5341c3d74b261d3ad52106f7fa7edde06841e4767fe2a40cb54099b1f3ed.deb':
        (497075610, '276b5341c3d74b261d3ad52106f7fa7edde06841e4767fe2a40cb54099b1f3ed'),
}
CURRENT = CACHE / 'packages/chatgpt-26.1007.71326-amd64-0e006b4931712f5a90d4cd04972c6e0b533cedcdfdb2b51946d5b9367471f556.deb'


def save(name, value):
    with (EVIDENCE / name).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)


def helpers():
    root = EVIDENCE.parent / 'video_cleanup_20261014_r1'
    source = root / 'audit_videos.py'
    manifest = dict((relative.lstrip('*'), checksum) for checksum, relative in
                    (line.split(maxsplit=1) for line in (root / 'SHA256SUMS').read_text().splitlines()))
    assert hashlib.sha256((root / 'SHA256SUMS').read_bytes()).hexdigest() == 'cdfb30fbdaf7edeb0452bfcb838f0af02a14d63a4e3557670daf917ccccf416c'
    assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[source.name]
    spec = importlib.util.spec_from_file_location('verified_video_cleanup_helpers', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for relative, expected in manifest.items():
        assert module.digest(root / relative) == expected and (root / relative).stat().st_mode & 0o777 == 0o444
    return module


def runtime_snapshot(module):
    paths = {CURRENT, Path('/opt/codex-desktop/resources/codex'), Path('/usr/bin/codex-update-manager')}
    processes = []
    for process in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            command = process.info['cmdline'] or []
            if process.info['name'] == 'unattended-upgr' and command == [
                    '/usr/bin/python3', '/usr/share/unattended-upgrades/unattended-upgrade-shutdown', '--wait-for-signal']:
                continue
            if process.info['name'] in ('apt', 'apt-get', 'dpkg', 'gdebi', 'unattended-upgr'):
                raise RuntimeError('Package installation active: ' + str(process.info))
            if process.info['name'] in ('codex', 'codex-update-ma', 'codex-update-manager', 'codex-code-mode'):
                executable = Path(process.exe())
                assert executable not in TARGETS
                paths.add(executable)
                processes.append(dict(pid=process.pid, ctime=process.create_time(), executable=str(executable), argv=command))
        except psutil.NoSuchProcess:
            continue
    rows = {str(path): module.metadata(path) for path in paths if path.is_file() and not path.is_symlink()}
    assert str(CURRENT) in rows and rows[str(CURRENT)]['sha256'] == '0e006b4931712f5a90d4cd04972c6e0b533cedcdfdb2b51946d5b9367471f556'
    return dict(files=rows, processes=processes)


def inventory():
    module = helpers()
    selected = []
    protected = module.protected_paths()
    for path, (size, checksum) in TARGETS.items():
        row = module.metadata(path)
        assert row['bytes'] == size and row['sha256'] == checksum and row['links'] == 1
        assert str(path) not in protected
        selected.append(row)
    visibility = module.process_scan(selected)
    updater_pids = [process.pid for process in psutil.process_iter(['name'])
                   if process.info['name'] in ('codex-update-ma', 'codex-update-manager')]
    assert updater_pids and not any(row['pid'] in updater_pids for row in visibility['visibility_limitations'])
    suffix = '_retry2' if (EVIDENCE / 'process_scan_before.json').exists() else ''
    save('process_scan_before' + suffix + '.json', visibility)
    for index, path in enumerate(TARGETS):
        command = ['rg', '-l', '-F', path.name, str(WORKSPACE), '--glob', '*.json', '--glob', '*.md',
                   '--glob', '*.py', '--glob', '*.sh', '--glob', '*.yaml', '--glob', '*.txt',
                   '--glob', '*SHA256*', '--glob', '!**/build*/**', '--glob', '!**/colcon*/**',
                   '--glob', '!**/ros_log/**', '--glob', '!**/installer_download_cleanup_20261014_r1/**']
        result = subprocess.run(command, capture_output=True, text=True)
        save('formal_reference_search_' + str(index) + suffix + '.json', dict(argv=command, exit_code=result.returncode,
                                                                 stdout=result.stdout, stderr=result.stderr))
        assert result.returncode == 1 and not result.stdout and not result.stderr
    save('runtime_before.json', runtime_snapshot(module))
    save('supplementary_before.json', module.supplementary())
    save('inventory.json', dict(status='PASS', selected=selected, authorized_exact_files=2,
                                no_program_or_current_package_or_project_artifact_selected=True))
    print('EXACT_TWO_ARCHIVES_VERIFIED', sum(row['bytes'] for row in selected))


def remove():
    module = helpers()
    for name in ('protected_before.json', 'source_1009_before.json', 'additional_protected_before.json', 'dual_protected_before.json'):
        assert json.loads((EVIDENCE / name).read_text())['status'] == 'PASS'
    selected = json.loads((EVIDENCE / 'inventory.json').read_text())['selected']
    assert {Path(row['path']) for row in selected} == set(TARGETS)
    assert module.supplementary() == json.loads((EVIDENCE / 'supplementary_before.json').read_text())
    save('process_scan_pre_delete.json', module.process_scan(selected))
    before = json.loads((EVIDENCE / 'runtime_before.json').read_text())
    for path, row in before['files'].items():
        assert module.metadata(Path(path)) == row
    for row in selected:
        assert module.metadata(Path(row['path'])) == row
    free_before = psutil.disk_usage(str(WORKSPACE)).free
    with (EVIDENCE / 'deletion_journal.jsonl').open('x') as stream:
        for row in selected:
            path = Path(row['path'])
            assert path in TARGETS and module.metadata(path) == row
            path.unlink()
            stream.write(json.dumps(dict(path=str(path), bytes=row['bytes'], allocated_bytes=row['allocated_bytes'], sha256=row['sha256'])) + '\n')
            stream.flush()
    for path, row in before['files'].items():
        assert module.metadata(Path(path)) == row
    save('runtime_after.json', runtime_snapshot(module))
    save('supplementary_after.json', module.supplementary())
    save('deletion_result.json', dict(status='PASS', deleted_files=2, logical_bytes=sum(row['bytes'] for row in selected),
                                     allocated_bytes=sum(row['allocated_bytes'] for row in selected), free_before=free_before,
                                     free_after=psutil.disk_usage(str(WORKSPACE)).free,
                                     installed_runtime_and_current_package_unchanged=True))
    print('EXACT_TWO_AUTHORIZED_ARCHIVES_DELETED')


if __name__ == '__main__':
    {'inventory': inventory, 'remove': remove}[sys.argv[1]]()
