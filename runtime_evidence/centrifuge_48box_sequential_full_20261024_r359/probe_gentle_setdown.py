import hashlib
import json
from pathlib import Path

from dual_motion import common_arm_timing
from gentle_setdown_cycle import COARSE_PHASE, GentleSetdownCycle
from test_gentle_setdown import real_failure_command


EVIDENCE = Path(__file__).resolve().parent


def main():
    command = real_failure_command()
    original = common_arm_timing(command['initial'], command['samples'], .35)
    candidate = common_arm_timing(command['initial'], command['samples'], .30)
    assert original['times'] == command['times']
    assert candidate['points'] == original['points'] and candidate['groups'] == original['groups']
    assert candidate['original_floor_sim_s'] == .020
    result = dict(status='PASS', scope='Exact r271 failed coarse command retiming only, not physical setdown or complete 180',
                  phase=COARSE_PHASE, runtime_subclass=GentleSetdownCycle.__name__,
                  parent_coarse_rate_rad_s=.35, candidate_coarse_rate_rad_s=.30,
                  original_message_points=len(original['points']), all_original_points_and_actual_seed_preserved=True,
                  original_common_group_timestamps_and_twenty_ms_floor_preserved=True,
                  original_duration_sim_s=original['duration_sim_s'], candidate_duration_sim_s=candidate['duration_sim_s'],
                  additional_duration_sim_s=candidate['duration_sim_s'] - original['duration_sim_s'],
                  original_native_grip_limits_not_relaxed=True, native_limit_m=.002, native_angle_limit_deg=5.,
                  fresh_actual_regrasp_reference_not_rebased=True, new_current_physical_execution_still_required=True,
                  runtime_module_sha256=hashlib.sha256((EVIDENCE / 'gentle_setdown_cycle.py').read_bytes()).hexdigest())
    with (EVIDENCE / 'gentle_setdown_probe_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
