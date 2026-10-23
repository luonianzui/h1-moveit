import json
from pathlib import Path
import statistics


ROOT = Path(__file__).resolve().parent


def seconds(stamp):
    return int(stamp.get('sec', 0)) + int(stamp.get('nsec', 0)) * 1e-9


rows = [json.loads(line) for line in (ROOT / 'world_stats.jsonl').open()]
valid = [row for row in rows if row['status'] == 'READ_ONLY_SAMPLE']
elapsed_sim = seconds(valid[-1]['stats']['simTime']) - seconds(valid[0]['stats']['simTime'])
elapsed_real = seconds(valid[-1]['stats']['realTime']) - seconds(valid[0]['stats']['realTime'])
factors = [row['stats']['realTimeFactor'] for row in valid if 'realTimeFactor' in row['stats']]
result = dict(status='PARTIAL_WORLD_STATS_NOT_WHOLE_TASK_WALL_CLOCK', samples=len(valid),
              errors=[row for row in rows if row['status'] != 'READ_ONLY_SAMPLE'],
              first=valid[0], last=valid[-1], elapsed_sim_s=elapsed_sim, elapsed_real_s=elapsed_real,
              interval_ratio=elapsed_sim/elapsed_real, minimum_reported_rtf=min(factors),
              median_reported_rtf=statistics.median(factors), maximum_reported_rtf=max(factors),
              missing_reported_rtf_samples=sum('realTimeFactor' not in row['stats'] for row in valid),
              stable_one_times_realtime_not_achieved=elapsed_sim/elapsed_real < .99)
with (ROOT / 'world_stats_summary.json').open('x') as stream:
    json.dump(result, stream, indent=2)
