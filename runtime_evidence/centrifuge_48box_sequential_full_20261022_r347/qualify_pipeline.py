import hashlib
import json
from pathlib import Path
import time

from joined_stages import joined_samples, joined_timing, stage_groups
from speed_planning import plan
from speed_policy import configure_simulation_profile


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r333'
configure_simulation_profile()
assert '15 passed' in (ROOT / 'pipeline_test_revision2.log').read_text()
records = []
first_result = None
for name in ('1_park_left_request.json', '3_load_pilot_left_request.json', '1_park_left_request.json'):
    path = PARENT / 'run' / name
    content = path.read_bytes()
    began = time.monotonic()
    result = plan(json.loads(content))
    records.append(dict(source=str(path), source_sha256=hashlib.sha256(content).hexdigest(),
        mode=result['mode'], checked_samples=result['checked_samples'], wall_s=time.monotonic()-began))
    if first_result is None:
        first_result = result
    elif name.startswith('1_'):
        assert first_result == result, 'Prior loaded request must not contaminate fresh empty scene'
joined = []
for path in sorted((PARENT / 'run').glob('*transport*_result.json')):
    original = json.loads(path.read_text())
    for group in stage_groups(original['stages']):
        if len(group) != 2:
            continue
        timing = joined_timing(group[0]['samples'][0], group, .8)
        assert timing['points'][1:] == joined_samples(group)
        joined.append(dict(source=str(path), original_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            phases=[stage['phase'] for stage in group], duration_sim_s=timing['duration_sim_s']))
assert len(joined) == 4
with (ROOT / 'pipeline_qualification_revision2.json').open('x') as stream:
    json.dump(dict(status='PASS_PERSISTENT_PROTOCOL_AND_JOINED_TIMING_NOT_PHYSICAL',
        original_dense_paths_and_per_segment_ramps_preserved=True,
        fresh_scenes_with_empty_loaded_empty_state_isolation=True, records=records, joined=joined,
        no_guard_or_30_second_spin_reduction=True), stream, indent=2)
print('PIPELINE_QUALIFICATION_PASS_NOT_PHYSICAL', flush=True)
