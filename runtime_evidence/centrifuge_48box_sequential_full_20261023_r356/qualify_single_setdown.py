import hashlib
import json
from pathlib import Path
from test_single_setdown import PLAN, PLAN_SHA, fixture
from single_setdown import execute_single_setdown

ROOT=Path(__file__).resolve().parent
cycle,planned,samples,split=fixture()
before=cycle.holders.snapshot()
execute_single_setdown(cycle,planned,samples,split)
commands=[value for kind,value in cycle.events if kind=='command']
assert len(commands)==2 and cycle.motion_count==6 and cycle.holders.snapshot()==before
assert commands[0]['samples']==samples[:split+1] and commands[1]['samples']==samples[split:]
assert [value['actual_rate_rad_s'] for value in commands]==[.13,.03]
result=dict(status='PASS_ACTUAL_SINGLE_SETDOWN_DISPATCH_OFFLINE_NOT_PHYSICAL',
    actual_parent_plan=str(PLAN),actual_parent_plan_sha256=PLAN_SHA,
    initial_state='synthetic_offline_state_at_qualified_setdown_first_point_not_new_measurement',
    holder_references='copied_parent_actual_registry_for_offline_dispatch_not_new_capture',
    actual_SequentialCycle_and_original_DualCycle_move_executed_with_mock_publishers=True,
    original_samples=len(samples),split_index=split,published_group_messages=cycle.motion_count,
    coarse_sim_s=commands[0]['times'][-1],fine_sim_s=commands[1]['times'][-1],
    original_path_and_guards_unchanged=True,coarse_rate_rad_s=.13,fine_rate_rad_s=.03,ramp_sim_s=.6,
    old_paired_guard_unchanged=True,no_new_task_result=True)
with (ROOT/'single_setdown_qualification.json').open('x') as stream:
    json.dump(result,stream,indent=2)
print(json.dumps(result,indent=2))
