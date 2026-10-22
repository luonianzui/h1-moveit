import inspect
import math

import pytest

from execute_cycle import FullCycle
from execute_loading import LoadingCycle
from free_carry_rate import ORIGINAL_RATES,carry_rate,carry_floor


@pytest.mark.parametrize('phase',sorted(ORIGINAL_RATES))
def test_only_verified_free_carry_gets_bounded_profile(phase,monkeypatch):
    side=phase.split('_')[0];payload='left' if '_retrieval_' in phase else None
    original=ORIGINAL_RATES[phase]
    rate=carry_rate(phase,original,side,payload,{side:16.})
    assert rate==pytest.approx(min(original*1.15,.36))
    assert rate*.035<=.0126+1e-12
    assert carry_floor(phase,side,payload,{side:16.})==.020
    cycle=FullCycle.__new__(FullCycle);cycle.holding=side;cycle.held_payload=payload
    cycle.accepted_gripper_efforts={side:16.};cycle.record=lambda kind,data:None
    captured=[]
    monkeypatch.setattr(LoadingCycle,'move',lambda self,path,label,speed:captured.append((path,label,speed)))
    samples=[{'torso_joint':0.},{'torso_joint':.1}]
    requested=.13 if phase=='left_transfer' else .20 if '_retrieval_' in phase else original
    cycle.move(samples,phase,requested)
    assert captured[0][0] is samples and captured[0][1]==phase and captured[0][2]==pytest.approx(rate)


@pytest.mark.parametrize('holding,payload,efforts',[(None,None,{}),('right',None,{'right':16.}),
    ('left',None,{'left':0.}),('left',None,{'left':math.nan}),('left','right',{'left':16.})])
def test_bad_or_unknown_loaded_identity_uses_original(holding,payload,efforts):
    assert carry_rate('left_lift',.312,holding,payload,efforts)==.312
    assert carry_floor('left_lift',holding,payload,efforts)==.020


@pytest.mark.parametrize('phase',['left_pilot_lift','right_lower','left_retrieval_lift',
    'right_retrieval_descend','left_retrieval_setdown','machine_close','left_approach'])
def test_contact_empty_extraction_and_buttons_keep_original_timing(phase):
    assert carry_rate(phase,.12,'left','left',{'left':16.})==.12
    assert carry_floor(phase,'left','left',{'left':16.})==.020


def test_unknown_original_cap_refused():
    with pytest.raises(ValueError):
        carry_rate('left_transfer',.4,'left',None,{'left':16.})


def test_retrieval_needs_actual_payload_identity():
    assert carry_floor('left_retrieval_transfer','left',None,{'left':16.})==.020
    assert carry_floor('left_retrieval_transfer','left','right',{'left':16.})==.020


def test_original_movement_tracking_guard_not_relaxed():
    source=inspect.getsource(LoadingCycle.move)
    assert 'if abs(self.positions[name] - reference) > .015:' in source
    assert '.0003' in source and 'settled_since > .2' in source
