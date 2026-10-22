import json
from pathlib import Path
import time

from startup_review_hold import world_stats


ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r350/run'
deadline = time.monotonic() + 1800
with (ROOT / 'world_stats.jsonl').open('x') as stream:
    while not (RUN / 'process_exit_status.json').exists() and time.monotonic() < deadline:
        began = time.monotonic()
        try:
            stats = world_stats()
            phase = json.loads((RUN / 'trace_progress.json').read_text())
            row = dict(status='READ_ONLY_SAMPLE', monotonic=time.monotonic(), request_began=began,
                       stats=stats, phase=phase['phase'], observer_started_after_executor=True)
        except Exception as error:
            row = dict(status='SAMPLE_ERROR', monotonic=time.monotonic(), error=str(error))
        stream.write(json.dumps(row) + '\n')
        stream.flush()
        for attempt in range(15):
            if (RUN / 'process_exit_status.json').exists():
                break
            time.sleep(1)
print('READ_ONLY_WORLD_STATS_FINISHED', flush=True)
