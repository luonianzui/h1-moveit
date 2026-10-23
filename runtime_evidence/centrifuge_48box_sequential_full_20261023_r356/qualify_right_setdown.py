import hashlib
import json
from pathlib import Path

from test_right_setdown import execute_actual_path
from paired_ramp import ramped_common_timing
from single_setdown import single_setdown_profile

ROOT=Path(__file__).resolve().parent
records=[]
for hand in ('left','right'):
    cycle,plan,path=execute_actual_path(hand)
    commands=[value for kind,value in cycle.events if kind=='command']
    assert len(commands)==6 and cycle.motion_count==18
    assert cycle.holders.snapshot()==plan['original_runtime_holder_references']
    coarse=commands[-2]
    expected_rate,ramp=single_setdown_profile(hand)
    old=ramped_common_timing(coarse['initial'],coarse['samples'],.13,.6)
    assert coarse['actual_rate_rad_s']==expected_rate and commands[-1]['actual_rate_rad_s']==.03
    records.append(dict(hand=hand,actual_parent_plan=str(path),plan_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        coarse_rate_rad_s=expected_rate,ramp_sim_s=ramp,coarse_duration_sim_s=coarse['times'][-1],
        previous_coarse_duration_sim_s=old['times'][-1],original_samples=len(coarse['samples']),
        group_messages=cycle.motion_count,original_holder_unchanged=True))
with (ROOT/'right_setdown_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_RIGHT_PATH_RETIMED_OFFLINE_NOT_PHYSICAL',records=records,
        mocks='publishers, feedback, clearance; synthetic start at actual qualified first point',
        new_physical_success_claimed=False),stream,indent=2)
print('PASS_ACTUAL_RIGHT_PATH_RETIMED_OFFLINE_NOT_PHYSICAL',records)
