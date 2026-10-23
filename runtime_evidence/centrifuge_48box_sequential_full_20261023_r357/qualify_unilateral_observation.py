import hashlib
import json
from pathlib import Path

from preload_floor_frame import single_missing_floor_frame
from preload_observation import pending_unilateral_frame
from sequential_support import actual_preload_support
from test_unilateral_observation import actual_fixture

ROOT = Path(__file__).resolve().parent
cycle, metrics, _ = actual_fixture()
assert not single_missing_floor_frame(cycle, 'left', metrics)['valid']
assert not actual_preload_support(cycle, 'left', retrieval=True)['valid']
pending = pending_unilateral_frame(cycle)
assert pending is not None and pending['not_a_support_or_grip_certificate']
result = dict(status='PASS_ACTUAL_R355_PENDING_ELIGIBILITY_NOT_SUPPORT_OR_RECOVERY',
    fixture_sha256=hashlib.sha256((ROOT / 'r355_preload_fixture.json').read_bytes()).hexdigest(),
    original_support_still_rejected=True, holder_created=False, actual_recovery_not_claimed=True,
    maximum_observation_wall_s=.020, maximum_observation_sim_s=.001, maximum_callbacks=64,
    current_single_finger_force_limit_N=1., positive_history_maximum_age_ns=1000000,
    original_support_limits_not_changed=True, pending=pending)
with (ROOT / 'unilateral_observation_qualification.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(result['status'])
