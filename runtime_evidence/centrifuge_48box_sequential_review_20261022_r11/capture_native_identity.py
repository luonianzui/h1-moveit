import json
from pathlib import Path

import psutil


REVIEW = Path(__file__).resolve().parent
RUN = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r353/run'
assert (RUN / 'process_identities.json').is_file()
owner = json.loads((RUN / 'startup_review_freeze.json').read_text())['server']
process = psutil.Process(owner['pid'])
assert abs(process.create_time() - owner['created']) < .01
maps = Path(f'/proc/{process.pid}/maps').read_text()
assert 'libTaskCpuProfile.so' in maps and 'libprofiler.so' in maps
with (REVIEW / 'native_process_maps.txt').open('x') as stream:
    stream.write(maps)
with (REVIEW / 'native_process_identity.json').open('x') as stream:
    json.dump(dict(pid=process.pid, created=process.create_time(), executable=process.exe(),
        argv=process.cmdline(), tids=[thread.id for thread in process.threads()],
        exact_owned_process_identity_checked=True, read_only=True), stream, indent=2)
print('OWNED_NATIVE_PROFILE_PROCESS_CAPTURED', process.pid, flush=True)
