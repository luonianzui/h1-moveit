import gzip
import hashlib
import json
from pathlib import Path
import time

from batched_trace import BatchedLosslessTrace
from trace_io import LosslessTrace, verify_trace


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r328'
source = PARENT / 'run/loading_trace.jsonl.gz'
assert hashlib.sha256((PARENT / 'SHA256SUMS').read_bytes()).hexdigest() == '19f0dadae90f05d5d29c46000d42cb9ade52b16b27927417286820c9df18c7ef'
rows = []
size = 0
with gzip.open(source, 'rt') as stream:
    for line in stream:
        rows.append(line)
        size += len(line.encode())
        if size >= 16 * 1024 * 1024:
            break
results = []
for name, constructor in [('original', LosslessTrace), ('batched', BatchedLosslessTrace)]:
    folder = ROOT / ('trace_component_' + name)
    folder.mkdir()
    (folder / 'run').mkdir()
    writer = constructor(folder / 'run')
    began = time.perf_counter()
    for line in rows:
        writer.write(line)
        writer.flush()
    writer.close()
    elapsed = time.perf_counter() - began
    audit = verify_trace(folder)
    assert audit['status'] == 'PASS'
    results.append(dict(implementation=name, wall_s=elapsed, audit=audit))
assert results[0]['audit']['decoded_sha256'] == results[1]['audit']['decoded_sha256']
with (ROOT / 'batched_trace_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_IDENTICAL_ACTUAL_TRACE_BYTES_NOT_TASK_SPEED',
                   source=str(source), real_prefix_bytes=size, rows=len(rows),
                   unchanged_every_row_sha256_crc_and_closed_audit=True, results=results,
                   component_speedup=results[0]['wall_s'] / results[1]['wall_s']), stream, indent=2)
print('BATCH_TRACE_EQUIVALENT', size, len(rows), [row['wall_s'] for row in results], flush=True)
