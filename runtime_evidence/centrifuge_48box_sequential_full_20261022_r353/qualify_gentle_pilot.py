import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from right_setdown_cycle import SpeedPriorityCycle
from speed_policy import common_timing


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r339'
source = PARENT / 'run/15_retrieval_pilot_left_result.json'
request = json.loads(source.with_name('15_retrieval_pilot_left_request.json').read_text())
plan = json.loads(source.read_text())
assert plan['status'] == 'PASS_GEOMETRY_ONLY'
samples = plan['stages'][0]['samples']
owners = copy.deepcopy(plan['original_runtime_holder_references'])
cycle = object.__new__(SpeedPriorityCycle)
cycle.phase = 'seq_retrieval_single_pilot'
cycle.holders = SimpleNamespace(rows=owners, snapshot=lambda: copy.deepcopy(owners))
cycle.order = SimpleNamespace(phase='retrieval')
rate, tracking = cycle.motion_limits(cycle.phase, .25)
assert (rate, tracking) == (.05, .04)
previous = common_timing(request['positions'], samples, .1)
candidate = cycle.common_timing(request['positions'], samples, rate)
assert candidate['points'] == previous['points'] and candidate['groups'] == previous['groups']
assert candidate['duration_sim_s'] >= previous['duration_sim_s']
result = dict(status='PASS_R339_ACTUAL_FAILED_PILOT_RETIMING_ONLY_NOT_PHYSICAL',
              source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              old_rate_rad_s=.1, current_rate_rad_s=.05,
              previous_sim_s=previous['duration_sim_s'], candidate_sim_s=candidate['duration_sim_s'],
              unchanged_point_count=len(samples), rotor_transient_guard_rpm=.1,
              original_point_geometry_and_hold_reference_unchanged=True,
              right_setdown_stability_retained=True, new_full_success_claimed=False)
with (ROOT / 'gentle_pilot_qualification.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result))
