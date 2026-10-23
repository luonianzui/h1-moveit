import json
from pathlib import Path

from test_whole_single_transport import PLAN, PLAN_SHA, execute_transport

ROOT=Path(__file__).resolve().parent
cycle,plan=execute_transport()
commands=[value for kind,value in cycle.events if kind=='command']
assert len(commands)==6 and cycle.motion_count==18
assert cycle.holders.snapshot()==plan['original_runtime_holder_references']
assert [value['actual_rate_rad_s'] for value in commands]==[.03,.25,.25,.25,.13,.03]
result=dict(status='PASS_WHOLE_SINGLE_TRANSPORT_DISPATCH_OFFLINE_NOT_PHYSICAL',
    actual_parent_plan=str(PLAN),actual_parent_plan_sha256=PLAN_SHA,
    original_five_stages=[stage['phase'] for stage in plan['stages']],actual_arm_commands=6,group_messages=18,
    initial_state='synthetic_at_actual_plan_first_point',publishers_and_feedback='mock',
    clearance_measurement='mock_nominal_only_not_collision_or_physical_evidence',
    actual_SequentialCycle_execute_stages_and_DualCycle_move=True,phase_not_manually_primed_for_setdown=True,
    original_holder_references_unchanged=True,new_physical_success_claimed=False)
with (ROOT/'whole_single_transport_qualification.json').open('x') as stream:
    json.dump(result,stream,indent=2)
print(result['status'])
