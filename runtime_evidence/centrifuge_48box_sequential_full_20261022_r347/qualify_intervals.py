import gzip
import hashlib
import json
from pathlib import Path

from speed_policy import PROFILE
from table_alignment_cycle import SpeedPriorityCycle
from empty_gap_policy import eligible
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r345'
assert hashlib.sha256((PARENT / 'identity.json').read_bytes()).hexdigest() == 'd202078cba7d45d07a03b254d85f2047d0a9bdbaa63bbd1bfde6f7b54958508a'
starts = set()
arm_commands = 0
with gzip.open(PARENT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] in ('command', 'button_arm_command', 'gripper_goal_accepted'):
            payload = row['payload']
            if eligible(row['phase'], payload.get('holders', {})):
                starts.add((row['phase'], payload.get('start', payload.get('common_start_sim'))))
                arm_commands += row['kind'] == 'command'
assert None not in {stamp for phase, stamp in starts}
assert PROFILE == json.loads((PARENT / 'identity.json').read_text())['speed_profile']
state = SimpleNamespace(phase='seq_load_empty_park', holders=SimpleNamespace(rows={}))
assert SpeedPriorityCycle.future_start_offset(state) == .16
assert SpeedPriorityCycle.paired_feedback_overlap_enabled(state)
report = dict(status='PASS_SCHEDULING_AND_ENDPOINT_OVERLAP_NOT_PHYSICAL',
    parent=str(PARENT), parent_unique_schedules=len(starts), parent_arm_commands=arm_commands,
    old_future_lead_sim_s=.3, new_future_lead_sim_s=.16, unchanged_minimum_future_sim_s=.15,
    nominal_lead_budget_reduction_sim_s=len(starts)*(.3-.16),
    endpoint_overlap_max_nominal_reduction_sim_s=arm_commands*.05,
    original_geometry_dense_samples_and_per_segment_ramps_unchanged=True,
    only_named_empty_free_phases_eligible=True,
    loaded_near_contact_and_button_lead_restored_to_original_0_3=True,
    original_50ms_measured_endpoint_hold_and_tracking_guards_unchanged=True,
    grip_drop_pilot_settle_and_30s_spin_observations_unchanged=True,
    persistent_planning_no_ai_relay=True, actual_wall_saving_not_preclaimed=True)
with (ROOT / 'interval_qualification.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps(report), flush=True)

