import hashlib
import json
import os
from pathlib import Path
import stat
import zipfile

import psutil


ROOT = Path(__file__).resolve().parent
BODY = Path('/home/liu/.cache/pip/http-v2/5/f/0/c/f/5f0cfe9031b2dd26c1748b016f63f2829715613528a11611040321e0.body')
INDEX = BODY.with_suffix('')
assert (ROOT / 'reference_scan_exit.txt').read_text().strip() == '1'
assert (ROOT / 'reference_scan.txt').read_text() == ''
assert BODY.stat().st_size == 3021340845
paths = (BODY, INDEX)
records = []
for path in paths:
    info = path.lstat()
    assert stat.S_ISREG(info.st_mode) and info.st_uid == os.getuid() and info.st_nlink == 1
    with path.open('rb') as stream:
        checksum = hashlib.file_digest(stream, 'sha256').hexdigest()
    records.append(dict(path=str(path), bytes=info.st_size, sha256=checksum, inode=info.st_ino,
                        device=info.st_dev, mtime_ns=info.st_mtime_ns, uid=info.st_uid))
with zipfile.ZipFile(BODY) as archive:
    metadata_name = 'isaacsim_extscache_kit-5.1.0.0.dist-info/METADATA'
    metadata = archive.read(metadata_name).decode()
    assert '\nName: isaacsim-extscache-kit\n' in metadata and '\nVersion: 5.1.0.0\n' in metadata
    members = len(archive.infolist())
processes = []
unavailable_desktop_processes = []
for process in psutil.process_iter(['pid', 'uids', 'name', 'cmdline', 'status']):
    try:
        if (process.info['uids'].real != os.getuid() or process.pid == os.getpid()
                or process.info['status'] == psutil.STATUS_ZOMBIE):
            continue
        command = process.info['cmdline'] or []
        assert not any('pip' in argument and ('install' in command or 'download' in command) for argument in command), command
        opened = {item.path for item in process.open_files()}
        assert not opened.intersection(str(path) for path in paths), process.pid
        maps = Path(f'/proc/{process.pid}/maps').read_text()
        assert not any(str(path) in maps for path in paths), process.pid
        processes.append(process.pid)
    except (psutil.NoSuchProcess, psutil.ZombieProcess, FileNotFoundError, ProcessLookupError):
        continue
    except (psutil.AccessDenied, PermissionError):
        assert process.info['name'] in {'systemd', '(sd-pam)', 'fusermount3', 'fcclientCore',
                                        'ssh-agent', 'chrome-sandbox', 'qq'}
        unavailable_desktop_processes.append(dict(pid=process.pid, name=process.info['name']))
before = dict(status='PASS_EXACT_AUTHORIZED_DOWNLOAD_CACHE_NO_LIVE_REFERENCE', files=records,
              package='isaacsim-extscache-kit', version='5.1.0.0', archive_members=members,
              same_user_processes_checked=processes, project_reference_scan_exit=1,
              inaccessible_nonproject_desktop_processes=unavailable_desktop_processes,
              all_system_process_reference_absence_not_claimed=True,
              installed_software_project_outputs_and_backups_not_targets=True,
              free_bytes=psutil.disk_usage(str(ROOT)).free)
with (ROOT / 'before.json').open('x') as stream:
    json.dump(before, stream, indent=2)
for record in records:
    path = Path(record['path'])
    info = path.lstat()
    assert (info.st_ino, info.st_dev, info.st_size, info.st_mtime_ns) == (
        record['inode'], record['device'], record['bytes'], record['mtime_ns'])
    path.unlink()
assert not any(path.exists() or path.is_symlink() for path in paths)
result = dict(status='PASS_EXACT_AUTHORIZED_TWO_CACHE_FILES_REMOVED',
              removed_bytes=sum(record['bytes'] for record in records), files=records,
              free_bytes_after=psutil.disk_usage(str(ROOT)).free,
              no_installed_software_project_evidence_videos_or_backups_deleted=True)
with (ROOT / 'result.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
