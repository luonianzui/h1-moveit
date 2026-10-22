import hashlib
import json
from pathlib import Path
import time

from coherent_time import JointTimeline
from indexed_timeline import IndexedJointTimeline
from speed_coherent import select_coherent_pair
from speed_policy import PROFILE


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r327'
sample_path = ROOT.parent / 'centrifuge_48box_sequential_runtime_perf_20261020_r1/actual_samples.json'
sample_bytes = sample_path.read_bytes()
positions = json.loads(sample_bytes)['actual_full_joint_frame']['payload']['positions']
unchanged = {}
for name in ('speed_feedback_cycle.py', 'coherent_time.py',
             'sequential_cycle.py', 'execute_coherent.py', 'execute_dual.py', 'speed_retrieval_geometry.py'):
    previous = (PARENT / name).read_bytes()
    from pipeline_source import restore_pipeline_source
    current = restore_pipeline_source(name, (ROOT / name).read_text()).encode()
    expected = previous.decode().replace(str(PARENT), str(ROOT)).replace(PARENT.name, ROOT.name).encode()
    assert current == expected, name
    unchanged[name] = hashlib.sha256(current).hexdigest()
parent_profile = json.loads((PARENT / 'identity.json').read_text())['speed_profile']
assert dict(parent_profile, retrieval_pilot_rate_rad_s=.1, empty_button_rate_cap_rad_s=.8) == PROFILE
original = JointTimeline(positions)
candidate = IndexedJointTimeline(positions)
for number in range(300):
    for timeline in (original, candidate):
        timeline.add(number * .002, positions)
frames = [(number * .0005, 1., {}) for number in range(1000, 1256)]
latest = frames[-1][0]
assert select_coherent_pair(frames, original, latest) == select_coherent_pair(frames, candidate, latest)
measurements = {}
for name, timeline in [('original', original), ('candidate', candidate), ('candidate_repeat', candidate), ('original_repeat', original)]:
    began = time.perf_counter()
    for number in range(20000):
        pair = select_coherent_pair(frames, timeline, latest)
        assert pair is not None
    measurements[name] = time.perf_counter() - began
result = dict(status='PASS_EXACT_TIMELINE_AND_UNCHANGED_CONTRACT_NOT_PHYSICAL',
              real_joint_value_source=str(sample_path), real_sample_sha256=hashlib.sha256(sample_bytes).hexdigest(),
              synthetic_monotonic_probe_times=True, iterations_each=20000,
              unchanged_add_validation_and_bounded_real_interpolation=True,
              unchanged_contract_sources=unchanged, wall_seconds=measurements,
              component_speedup=min(measurements['original'], measurements['original_repeat']) /
                                max(measurements['candidate'], measurements['candidate_repeat']))
with (ROOT / 'runtime_performance_qualification.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result, indent=2), flush=True)
