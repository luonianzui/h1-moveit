import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from pilot_settle import RETRIEVAL_PILOT_SETTLE_SIM_S, settle_retrieval_pilot
from test_pilot_settle_inverse import undo_pilot_settle_integration

def fixture():
    row = {'left': dict(payload='left', native_reference=[1.,2.,3.], fk_reference=[4.,5.,6.], source_sim=1.)}
    cycle = SimpleNamespace(sim=5., phase='seq_retrieval_single_pilot', events=[], held=row, waits=[])
    cycle.holders = SimpleNamespace(snapshot=lambda: copy.deepcopy(cycle.held))
    def dwell(duration):
        cycle.waits.append(duration)
        cycle.sim += duration
    cycle.dwell = dwell
    cycle.record = lambda kind,payload: cycle.events.append((kind,payload))
    return cycle

def test_original_guarded_dwell_then_fresh_plan_without_changing_holder():
    cycle = fixture()
    before = copy.deepcopy(cycle.held)
    settle_retrieval_pilot(cycle,'left','left')
    assert cycle.waits == [1.] and RETRIEVAL_PILOT_SETTLE_SIM_S == 1.
    assert cycle.held == before and cycle.phase == 'seq_retrieval_single_pilot'
    assert cycle.events[0][0] == 'sequential_retrieval_pilot_settle_started'
    assert cycle.events[-1][0] == 'sequential_retrieval_pilot_settle_pass'
    assert cycle.events[-1][1]['original_runtime_holder_references'] == before

@pytest.mark.parametrize('change',['wrong_phase','missing_holder','other_payload','two_holders','nan','rebase','short_wait','guard_fault'])
def test_settle_fails_closed_without_accepting_stale_or_changed_grip(change):
    cycle = fixture()
    if change=='wrong_phase': cycle.phase='seq_load_single_pilot'
    elif change=='missing_holder': cycle.held={}
    elif change=='other_payload': cycle.held['left']['payload']='right'
    elif change=='two_holders': cycle.held['right']=dict(payload='right')
    elif change=='nan': cycle.sim=float('nan')
    else:
        original=cycle.dwell
        def changed(duration):
            if change=='guard_fault':
                raise RuntimeError('original physical guard')
            original(.99 if change=='short_wait' else duration)
            if change=='rebase':
                cycle.held['left']['source_sim']=cycle.sim
        cycle.dwell=changed
    with pytest.raises(RuntimeError):
        settle_retrieval_pilot(cycle,'left','left')
    assert not any(kind == 'sequential_retrieval_pilot_settle_pass' for kind, payload in cycle.events)

def test_only_retrieval_between_pilot_and_original_planning_changed():
    root=Path(__file__).resolve().parent
    parent=root.parent/'centrifuge_48box_sequential_full_20261019_r312'
    source=(root/'sequential_cycle.py').read_text()
    original=(parent/'sequential_cycle.py').read_bytes()
    assert hashlib.sha256(original).hexdigest()==json.loads((parent/'identity.json').read_text())['sha256']['sequential_cycle.py']
    assert undo_pilot_settle_integration(source).encode()==original
    anchor="""            self.execute_stages(self.single_plan('retrieval_pilot', hand, payload), 'seq_retrieval')
            settle_retrieval_pilot(self, hand, payload)
            planned = self.single_plan('retrieval_transport', hand, payload)
"""
    assert source.count(anchor)==1
