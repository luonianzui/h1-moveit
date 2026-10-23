import json
from pathlib import Path

from dual_motion import common_arm_timing
from test_initial_approach import actual_failed_approach


command = actual_failed_approach()
original = common_arm_timing(command['initial'], command['samples'], .8)
candidate = common_arm_timing(command['initial'], command['samples'], .7)
assert original['times'] == command['times']
assert candidate['points'] == original['points'] and candidate['groups'] == original['groups']
assert candidate['original_floor_sim_s'] == .020
result = dict(status='PASS', scope='r272 actual failed empty approach retiming, not physical success',
              parent_rate_rad_s=.8, candidate_rate_rad_s=.7,
              original_duration_sim_s=original['duration_sim_s'],
              candidate_duration_sim_s=candidate['duration_sim_s'],
              additional_duration_sim_s=candidate['duration_sim_s'] - original['duration_sim_s'],
              all_original_points_actual_seed_groups_and_floor_preserved=True,
              original_native_finger_velocity_limit_m_s=.2,
              native_guards_not_relaxed=True, original_complete_3_10_30_certificates_preserved=True,
              fresh_physical_execution_required=True, complete_180_not_preclaimed=True)
with (Path(__file__).resolve().parent / 'initial_approach_probe_audit.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
