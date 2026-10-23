import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r356'
REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261023_r3'

def read(path):
    return json.loads(path.read_text())

assert read(RUN / 'run/result.json')['status'] == 'PASS'
assert read(RUN / 'run/completion_finalization.json')['status'] == 'PASS'
capture = read(RUN / 'run/full_cycle.capture.json')
assert capture['status'] == 'PASS' and capture['normal_sigint']
executor = read(RUN / 'run/process_identities.json')['executor']
recorder = read(RUN / 'run/video_pid.json')
wall = read(RUN / 'run/executor_wall_clock.json')
offset = executor['created'] - recorder['created']
start = wall['started_ns'] / 1e9
phases = []
for line in (REVIEW / 'server_thread_samples.jsonl').open():
    row = json.loads(line)
    relative = row['monotonic'] - start + offset
    if not phases or phases[-1]['phase'] != row['phase']:
        if phases:
            phases[-1]['end_video_s'] = relative
        phases.append(dict(phase=row['phase'], begin_video_s=relative, begin_sim_s=row['sim']))
phases[-1]['end_video_s'] = capture['frames'] / 12
source = RUN / 'run/full_cycle.mp4'
with source.open('rb') as stream:
    sha = hashlib.file_digest(stream, 'sha256').hexdigest()
result = dict(source=str(source), source_sha256=sha, original_duration_s=capture['frames']/12,
    source_task_wall_s=wall['elapsed_wall_s'], source_task_pass=True,
    alignment='approximate video time from process creation plus monotonic samples, reviewed visually',
    timeline=phases, original_preserved=True, purpose='edited briefing, not a task duration certificate')
with (ROOT / 'source_timeline.json').open('x') as stream:
    json.dump(result, stream, indent=2, ensure_ascii=False)
for phase in phases:
    print(f"{phase['begin_video_s']:7.2f} {phase['end_video_s']:7.2f} {phase['phase']}")
