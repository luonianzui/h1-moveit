import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from coarse_ramp_cycle import CoarseRampCycle
from dual_motion import rigid_reference
from paired_ramp import ramped_common_timing
from sequential_cycle import SequentialCycle
from single_setdown import execute_single_setdown, single_setdown_timing
from test_complete_rate_profile import actual_publisher_cycle
from test_single_setdown_inverse import undo_single_setdown_integration

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent/'centrifuge_48box_sequential_full_20261020_r313'
PLAN = PARENT/'run/15_retrieval_transport_left_result.json'
PLAN_SHA = '749f3dd83f9c124229c61c22bb0d9e57bdfb06e31a845835e3a2c3332d983078'

def fixture():
    assert hashlib.sha256(PLAN.read_bytes()).hexdigest() == PLAN_SHA
    planned = json.loads(PLAN.read_text())
    stage = next(stage for stage in planned['stages'] if stage['phase']=='retrieve_setdown')
    samples, split = stage['samples'], max(1,len(stage['samples'])-8)
    cycle = actual_publisher_cycle()
    cycle.__class__ = SequentialCycle
    cycle.current_damping_empty_approach_completed = False
    cycle.holders.rows = {hand:dict(payload=row['payload'],source_sim=row['source_sim'],
        native_reference=rigid_reference(row['native_reference']),fk_reference=rigid_reference(row['fk_reference']))
        for hand,row in planned['original_runtime_holder_references'].items()}
    hand,payload = planned['single_hand'],planned['single_payload']
    cycle.order = SimpleNamespace(phase='retrieval',holder=dict(hand=hand,payload=payload),
        locations={payload:'held'},retrieval_assignment={hand:payload})
    prior = planned['stages'][planned['stages'].index(stage)-1]['phase']
    cycle.phase = 'seq_retrieval_' + prior
    cycle.sequential_setdown_context = None
    cycle.positions.update(samples[0])
    return cycle,planned,samples,split

def test_real_single_class_dispatch_keeps_every_point_six_group_messages_and_low_rate():
    cycle,planned,samples,split = fixture()
    initial = dict(cycle.positions)
    expected = ramped_common_timing(initial,samples[:split+1],.13,.6)
    before = cycle.holders.snapshot()
    execute_single_setdown(cycle,planned,samples,split)
    assert cycle.sequential_setdown_context is None and cycle.holders.snapshot()==before
    assert cycle.motion_count==6
    commands=[value for kind,value in cycle.events if kind=='command']
    assert len(commands)==2
    assert commands[0]['actual_rate_rad_s']==.13 and commands[1]['actual_rate_rad_s']==.03
    assert commands[0]['samples']==samples[:split+1] and commands[1]['samples']==samples[split:]
    assert commands[0]['times']==pytest.approx(expected['times'])
    for publisher in cycle.publishers.values():
        assert len(publisher.messages)==2
        for message in publisher.messages:
            times=[point.time_from_start.sec+point.time_from_start.nanosec*1e-9 for point in message.points]
            assert min(np.diff(times))>=.020-1e-9
            for point in message.points:
                assert not point.velocities and not point.accelerations
    assert any(kind=='sequential_single_setdown_timing' for kind,value in cycle.events)

def test_original_paired_guard_still_rejects_this_single_holder():
    cycle,planned,samples,split=fixture()
    cycle.phase='paired_retrieval_setdown'
    with pytest.raises(ValueError,match='Current original-native'):
        CoarseRampCycle.common_timing(cycle,cycle.positions,samples[:split+1],.13)
    assert cycle.motion_count==0

@pytest.mark.parametrize('change',['wrong_phase','wrong_mode','bad_status','bad_split','changed_path','changed_anchor',
                                  'wrong_order','wrong_assignment','two_holders','existing_context'])
def test_single_dispatch_cannot_accept_unqualified_plan_or_holder(change):
    cycle,planned,samples,split=fixture()
    if change=='wrong_phase': cycle.phase='seq_load_load_lower'
    elif change=='wrong_mode': planned['mode']='load_transport'
    elif change=='bad_status': planned['status']='FAIL'
    elif change=='bad_split': split-=1
    elif change=='changed_path':
        samples=copy.deepcopy(samples)
        samples[0]['torso_joint']+=.001
    elif change=='changed_anchor': planned['original_runtime_holder_references']['left']['source_sim']+=1
    elif change=='wrong_order': cycle.order.phase='loading'
    elif change=='wrong_assignment': cycle.order.retrieval_assignment={'left':'right'}
    elif change=='two_holders': cycle.holders.rows['right']=copy.deepcopy(cycle.holders.rows['left'])
    else: cycle.sequential_setdown_context={}
    with pytest.raises(ValueError):
        execute_single_setdown(cycle,planned,samples,split)
    assert cycle.motion_count==0

@pytest.mark.parametrize('change',['rate','point','missing_context'])
def test_coarse_timing_requires_exact_bound_context(change):
    cycle,planned,samples,split=fixture()
    def attempt(points,phase,rate):
        cycle.phase=phase
        if change=='rate': rate=.3
        elif change=='point':
            points=copy.deepcopy(points)
            points[0]['torso_joint']+=.001
        else: cycle.sequential_setdown_context=None
        single_setdown_timing(cycle,cycle.positions,points,rate)
    cycle.move=attempt
    with pytest.raises(ValueError):
        execute_single_setdown(cycle,planned,samples,split)
    assert cycle.sequential_setdown_context is None and cycle.motion_count==0

def test_only_single_setdown_dispatch_changes_parent_runtime():
    restored=undo_single_setdown_integration((ROOT/'sequential_cycle.py').read_text())
    assert restored==(PARENT/'sequential_cycle.py').read_text()
    for name in ('coarse_ramp_cycle.py','complete_rate_profile.py','execute_dual.py','table_bucket_clearance.py',
                 'sequential_planning.py','pilot_settle.py','preload_floor_frame.py'):
        from test_return_height_inverse import undo_return_height
        source=(ROOT/name).read_text()
        if name == 'preload_floor_frame.py':
            from floor_time_source import restore_floor_time
            source = restore_floor_time(source)
        if name=='sequential_planning.py':
            source=undo_return_height(source)
        assert source==(PARENT/name).read_text()
