import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import sequential_cycle
from dual_motion import rigid_reference
from paired_ramp import ramped_common_timing
from single_setdown import single_setdown_profile
from test_single_setdown import fixture
from test_right_setdown_inverse import undo_right_profile

ROOT=Path(__file__).resolve().parent
PARENT=ROOT.parent/'centrifuge_48box_sequential_full_20261020_r315'

def execute_actual_path(hand):
    plan_path=PARENT/'run'/('15_retrieval_transport_left_result.json' if hand=='left' else '19_retrieval_transport_right_result.json')
    plan=json.loads(plan_path.read_text())
    cycle,previous,samples,split=fixture()
    cycle.holders.rows={side:dict(payload=row['payload'],source_sim=row['source_sim'],
        native_reference=rigid_reference(row['native_reference']),fk_reference=rigid_reference(row['fk_reference']))
        for side,row in plan['original_runtime_holder_references'].items()}
    payload=plan['single_payload']
    cycle.order=SimpleNamespace(phase='retrieval',holder=dict(hand=hand,payload=payload),
        locations={payload:'held'},retrieval_assignment={hand:payload})
    cycle.positions.update(plan['stages'][0]['samples'][0])
    cycle.phase='seq_retrieval_single_pilot'
    cycle.poses={}
    cycle.contacts={payload:[]}
    cycle.bucket_rim_z={payload:0.}
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(sequential_cycle,'clearance_metrics',lambda *args: {'offline_nominal_fixture_not_physical':True})
        cycle.execute_stages(plan,'seq_retrieval')
    return cycle,plan,plan_path

@pytest.mark.parametrize('hand',['left','right'])
def test_both_actual_whole_paths_keep_all_points_guards_and_original_holder(hand):
    cycle,plan,path=execute_actual_path(hand)
    commands=[value for kind,value in cycle.events if kind=='command']
    assert len(commands)==6 and cycle.motion_count==18
    expected_rate,ramp=single_setdown_profile(hand)
    assert [value['actual_rate_rad_s'] for value in commands]==[.03,.25,.25,.25,expected_rate,.03]
    samples=plan['stages'][-1]['samples']
    split=max(1,len(samples)-8)
    assert [value['samples'] for value in commands]==[stage['samples'] for stage in plan['stages'][:-1]]+[samples[:split+1],samples[split:]]
    timing=ramped_common_timing(commands[-2]['initial'],commands[-2]['samples'],expected_rate,ramp)
    assert commands[-2]['times']==timing['times']
    assert cycle.holders.snapshot()==plan['original_runtime_holder_references']
    assert cycle.sequential_setdown_context is None
    if hand=='left':
        assert commands[-2]['times']==ramped_common_timing(commands[-2]['initial'],commands[-2]['samples'],.13,.6)['times']
    else:
        previous=ramped_common_timing(commands[-2]['initial'],commands[-2]['samples'],.13,.6)
        assert commands[-2]['times'][-1]>previous['times'][-1]
        assert timing['maximum_arc_acceleration_rad_s2']<previous['maximum_arc_acceleration_rad_s2']

def test_only_declared_right_timing_and_truthful_metadata_change():
    assert undo_right_profile((ROOT/'single_setdown.py').read_text())==(PARENT/'single_setdown.py').read_text()
    current=(ROOT/'sequential_cycle.py').read_text()
    current=current.replace('from bounded_release_alignment import ', 'from release_alignment import ')
    assert current.count("coarse_rate_rad_s=.06 if planned['single_hand']=='right' else .13")==1
    assert current.replace("coarse_rate_rad_s=.06 if planned['single_hand']=='right' else .13","coarse_rate_rad_s=.13")==(PARENT/'sequential_cycle.py').read_text()
    for name in ('execute_dual.py','sequential_planning.py','table_bucket_clearance.py','pilot_settle.py'):
        from test_return_height_inverse import undo_return_height
        source=(ROOT/name).read_text()
        if name=='sequential_planning.py':
            source=undo_return_height(source)
        assert source==(PARENT/name).read_text()

def test_unknown_hand_profile_rejected():
    with pytest.raises(ValueError):
        single_setdown_profile('both')
