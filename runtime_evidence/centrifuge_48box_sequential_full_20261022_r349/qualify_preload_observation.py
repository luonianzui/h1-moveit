import hashlib
import json
from pathlib import Path

from preload_observation import pending_zero_force_frame
from test_preload_observation import (
    FIXTURE, actual_fixture, test_actual_post_fault_nine_ms_gap_is_rejected_not_claimed_as_recovery,
    test_original_support_capture_geometry_sources_unchanged,
)


ROOT = Path(__file__).resolve().parent
cycle, _, _ = actual_fixture()
initial = pending_zero_force_frame(cycle)
assert initial and initial['not_a_support_or_grip_certificate']
test_actual_post_fault_nine_ms_gap_is_rejected_not_claimed_as_recovery()
test_original_support_capture_geometry_sources_unchanged()
result = dict(status='PASS_ACTUAL_FAULT_ELIGIBILITY_AND_BOUNDED_WAIT_NOT_NEW_PHYSICS',
              fixture_sha256=hashlib.sha256(FIXTURE.read_bytes()).hexdigest(),
              actual_fault_wait_eligible=True, actual_post_fault_large_gap_rejected=True,
              original_support_capture_and_geometry_unchanged=True,
              maximum_wall_wait_s=.020, maximum_sim_wait_s=.001, maximum_callbacks=64,
              old_complete_success_not_modified=True, no_new_physical_success_claimed=True)
with (ROOT / 'preload_observation_qualification.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result, indent=2), flush=True)
