import hashlib
import json
from pathlib import Path
import time

from alignment_scene import make_alignment_scene
import sequential_geometry
from planner_resources import CACHE, make_resource_scene
from speed_planning import plan
from speed_policy import configure_simulation_profile

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r347'
configure_simulation_profile()
records = []
for source in sorted((PARENT / 'run').glob('[0-9]*_request.json'), key=lambda path: int(path.name.split('_')[0])):
    request = json.loads(source.read_text())
    if request.get('sequential_contract') != sequential_geometry.CONTRACT:
        continue
    results, times = {}, {}
    for name, factory in (('original', make_alignment_scene), ('candidate', make_resource_scene)):
        sequential_geometry.make_alignment_scene = factory
        began = time.monotonic()
        results[name] = plan(request)
        times[name] = time.monotonic() - began
    assert results['original'] == results['candidate'], source
    assert json.loads(source.read_text()) == request
    result_hash = hashlib.sha256(json.dumps(results['candidate'], sort_keys=True).encode()).hexdigest()
    records.append(dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        mode=request['mode'], checked_samples=results['candidate']['checked_samples'],
        exact_same_result_sha256=result_hash, wall_s=times))
    print('EXACT_SAME_ACTUAL_SNAPSHOT', source.name, times, flush=True)
sequential_geometry.make_alignment_scene = make_alignment_scene
assert len(records) == 22 and CACHE.builds == 1 and CACHE.hits > 22
report = dict(status='PASS_SAME_ACTUAL_SNAPSHOT_GEOMETRY_AND_FRESH_SCENES_NOT_PHYSICAL',
    records=records, resources=CACHE.snapshot(),
    total_original_wall_s=sum(row['wall_s']['original'] for row in records),
    total_candidate_wall_s=sum(row['wall_s']['candidate'] for row in records),
    prefetch_paths_still_guarded_against_live_snapshot=True,
    no_guard_or_dense_check_or_observation_or_physics_change=True)
with (ROOT / 'planner_resources_qualification.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print('PLANNER_RESOURCES_QUALIFICATION_PASS', flush=True)
