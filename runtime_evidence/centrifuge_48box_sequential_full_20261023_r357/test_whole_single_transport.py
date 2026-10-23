import hashlib
import json
from pathlib import Path

import pytest

import sequential_cycle
from dual_motion import rigid_reference
from test_single_setdown import fixture

ROOT=Path(__file__).resolve().parent
PARENT=ROOT.parent/'centrifuge_48box_sequential_full_20261020_r314'
PLAN=PARENT/'run/16_retrieval_transport_left_result.json'
PLAN_SHA='40ca820562d5d10694c7fb14bfeefe43477e795ef8d1b793c3b75ab5c2dae374'

def actual_plan():
    assert hashlib.sha256(PLAN.read_bytes()).hexdigest()==PLAN_SHA
    return json.loads(PLAN.read_text())

def transport_fixture():
    cycle,old,samples,split=fixture()
    plan=actual_plan()
    cycle.holders.rows={hand:dict(payload=row['payload'],source_sim=row['source_sim'],
        native_reference=rigid_reference(row['native_reference']),fk_reference=rigid_reference(row['fk_reference']))
        for hand,row in plan['original_runtime_holder_references'].items()}
    cycle.positions.update(plan['stages'][0]['samples'][0])
    cycle.phase='seq_retrieval_single_pilot'
    cycle.poses={}
    cycle.contacts={'left':[]}
    cycle.bucket_rim_z={'left':0.}
    return cycle,plan

def execute_transport():
    cycle,plan=transport_fixture()
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(sequential_cycle,'clearance_metrics',lambda *args: {'offline_nominal_fixture_not_physical':True})
        cycle.execute_stages(plan,'seq_retrieval')
    return cycle,plan

def test_whole_actual_stage_dispatch_reaches_coarse_fine_without_manual_phase_priming():
    cycle,plan=execute_transport()
    commands=[value for kind,value in cycle.events if kind=='command']
    assert cycle.motion_count==18 and len(commands)==6
    assert [row['actual_rate_rad_s'] for row in commands]==[.03,.25,.25,.25,.13,.03]
    expected=[stage['samples'] for stage in plan['stages'][:-1]]
    samples=plan['stages'][-1]['samples']
    split=max(1,len(samples)-8)
    expected.extend([samples[:split+1],samples[split:]])
    assert [row['samples'] for row in commands]==expected
    assert cycle.holders.snapshot()==plan['original_runtime_holder_references']
    assert cycle.sequential_setdown_context is None
    hold=next(value for kind,value in cycle.events if kind=='retrieval_static_grip_hold_pass')
    assert hold['duration_sim_s']>=10

def test_actual_parent_plan_predecessor_reproduces_old_rejection():
    cycle,plan=transport_fixture()
    samples=plan['stages'][-1]['samples']
    cycle.phase='seq_retrieval_'+plan['stages'][-2]['phase']
    original={}
    exec(compile((PARENT/'single_setdown.py').read_text(),str(PARENT/'single_setdown.py'),'exec'),original)
    with pytest.raises(ValueError,match='Actual original single'):
        original['execute_single_setdown'](cycle,plan,samples,max(1,len(samples)-8))
    assert cycle.motion_count==0

def test_exact_predecessor_literal_is_only_runtime_change():
    from test_right_setdown_inverse import undo_right_profile
    current=undo_right_profile((ROOT/'single_setdown.py').read_text())
    assert current.count("'seq_retrieval_retrieve_transfer'")==1
    assert current.replace("'seq_retrieval_retrieve_transfer'","'seq_retrieval_retrieve_transport'")==(PARENT/'single_setdown.py').read_text()
    for name in ('sequential_cycle.py','table_bucket_clearance.py','execute_dual.py','coarse_ramp_cycle.py'):
        current=(ROOT/name).read_text().replace("coarse_rate_rad_s=.06 if planned['single_hand']=='right' else .13", "coarse_rate_rad_s=.13")
        current=current.replace('from bounded_release_alignment import ', 'from release_alignment import ')
        assert current==(PARENT/name).read_text()

@pytest.mark.parametrize('phase',['seq_retrieval_retrieve_transport','seq_retrieval_retrieve_outboard','seq_load_load_lower'])
def test_other_predecessors_remain_rejected(phase):
    from single_setdown import execute_single_setdown
    cycle,plan=transport_fixture()
    cycle.phase=phase
    samples=plan['stages'][-1]['samples']
    with pytest.raises(ValueError):
        execute_single_setdown(cycle,plan,samples,max(1,len(samples)-8))
    assert cycle.motion_count==0
