import hashlib
import json
from pathlib import Path
import time

import psutil
from startup_review_hold import require_scene

ROOT = Path(__file__).resolve().parent
require_scene(ROOT)
for name, expected in json.loads((ROOT / 'dart_cache_dependencies.json').read_text())['sha256'].items():
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == expected
path = ROOT / 'run/dart_constraint_samples.jsonl'
deadline = time.monotonic() + 5
while not path.exists() and time.monotonic() < deadline:
    time.sleep(.05)
assert path.exists()
rows = [json.loads(row) for row in path.read_text().splitlines()]
assert len(rows) > 3
identities = {(row['pid'], row['tid']) for row in rows}
assert len(identities) == 1
pid, tid = next(iter(identities))
process = psutil.Process(pid)
environment = process.environ()
assert environment['GZ_PARTITION'] == 'h1_seq_full_r351'
assert environment['ROS_DOMAIN_ID'] == '159'
assert environment['LD_PRELOAD'] == str(ROOT / 'inputs/libDartConstraintCache.so')
assert environment['H1_DART_CACHE_MODE'] == 'cached'
assert environment['GZ_TRANSPORT_RCVHWM'] == environment['GZ_TRANSPORT_SNDHWM'] == '20000'
maps = Path(f'/proc/{pid}/maps').read_text()
assert str(ROOT / 'inputs/libDartConstraintCache.so') in maps
assert '/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2' in maps
assert all(row['dimension'] > 0 and row['assembly_s'] > 0 for row in rows)
with (ROOT / 'run/dart_cache_runtime_identity.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_NEW_GAZEBO_CACHED_SOLVER', pid=pid, tid=tid,
                   created=process.create_time(), command=process.cmdline(),
                   sample_count=len(rows), mapped_candidate=True,
                   environment={key: environment[key] for key in
                       ('GZ_PARTITION', 'ROS_DOMAIN_ID', 'LD_PRELOAD', 'H1_DART_CACHE_MODE', 'H1_DART_CACHE_LOG')}),
              stream, indent=2)
with (ROOT / 'run/dart_cache_process_maps.txt').open('x') as stream:
    stream.write(maps)
print('ACTUAL_DART_CACHE_RUNTIME_BOUND', pid, tid, len(rows), flush=True)
