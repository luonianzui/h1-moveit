import ast
import copy
import hashlib
import json
from pathlib import Path

import pytest

from endpoint_overlap_cycle import EndpointOverlapCycle
from preload_floor_frame import FloorContactHistory, single_missing_floor_frame
from sequential_cycle import SequentialCycle
from sequential_support import actual_preload_support
from support_continuity import SupportContinuity
from test_preload_floor_frame import actual_fixture
from test_sequential_runtime import guard_fixture

ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261019_r311'
DIAGNOSIS = ROOT.parent / 'centrifuge_48box_sequential_review_20261019_r14/r311_basket_transition_fault.json'
EARLY_RECORD = """    def record(self, kind, payload):
        if kind == 'contact':
            self.closing_floor_history.observe(payload)
        super().record(kind, payload)

"""
HOOK = """        if kind == 'contact':
            self.closing_floor_history.observe(payload)
"""

def r311_fixture():
    record = json.loads(DIAGNOSIS.read_text())['records'][0]
    cycle, unused = actual_fixture()
    cycle.sim = record['fault']['sim']
    cycle.joint_time = record['last_joint']['payload']['source_sim']
    cycle.positions = record['last_joint']['payload']['positions']
    cycle.poses = record['fault']['payload']['poses']
    cycle.contacts = {'left': record['fault']['payload']['contacts']}
    cycle.preload_grasp_claim = record['last_claim']['payload']
    cycle.closing_floor_history = FloorContactHistory()
    return cycle, copy.deepcopy(record['fault']['payload']['metrics']), record['recent']

def runtime_class(source):
    return next(node for node in ast.parse(source).body if isinstance(node, ast.ClassDef) and node.name == 'SequentialCycle')

def test_one_record_method_preserves_unchanged_runtime_except_hook_merge():
    from test_pilot_settle_inverse import undo_pilot_settle_integration
    source = undo_pilot_settle_integration((ROOT/'sequential_cycle.py').read_text())
    parent = (PARENT/'sequential_cycle.py').read_text()
    identity = json.loads((PARENT/'identity.json').read_text())
    assert hashlib.sha256(parent.encode()).hexdigest() == identity['sha256']['sequential_cycle.py']
    assert parent.count(EARLY_RECORD) == 1
    expected = parent.replace(EARLY_RECORD, '', 1).replace(
        "    def record(self, kind, payload):\n        if kind == 'MACHINE_CYCLE_PASS':",
        "    def record(self, kind, payload):\n" + HOOK + "        if kind == 'MACHINE_CYCLE_PASS':", 1)
    assert source == expected
    names = [node.name for node in runtime_class(source).body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]
    assert len(names) == len(set(names))
    from floor_time_source import restore_floor_time
    assert restore_floor_time((ROOT/'preload_floor_frame.py').read_text()) == (PARENT/'preload_floor_frame.py').read_text()

def test_original_duplicate_record_reproduces_empty_history_without_changing_parent():
    original = runtime_class((PARENT/'sequential_cycle.py').read_text())
    original.body = [node for node in original.body if isinstance(node, ast.FunctionDef) and node.name == 'record']
    assert len(original.body) == 2
    original.bases = [ast.Name(id='Parent', ctx=ast.Load())]
    class Parent:
        def record(self, kind, payload):
            self.events.append((kind, payload))
    namespace = dict(Parent=Parent, copy=copy)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[original], type_ignores=[])), '<sealed-r311-record-binding>', 'exec'), namespace)
    cycle = namespace['SequentialCycle']()
    cycle.events = []
    cycle.closing_floor_history = FloorContactHistory()
    fixture, metrics, records = r311_fixture()
    for record in records:
        cycle.record(record['kind'], record['payload'])
    assert not cycle.closing_floor_history.rows
    fixture.closing_floor_history = cycle.closing_floor_history
    assert not single_missing_floor_frame(fixture, 'left', metrics)['valid']
    assert len(cycle.events) == len(records)

@pytest.mark.parametrize('source', ['r310', 'r311'])
def test_actual_record_binding_feeds_history_and_original_safety(monkeypatch, source):
    import sequential_cycle
    cycle = guard_fixture(monkeypatch)
    del cycle.record
    if source == 'r311':
        fixture, metrics, records = r311_fixture()
    else:
        fixture, metrics = actual_fixture()
        path = ROOT.parent/'centrifuge_48box_sequential_review_20261019_r13/r310_basket_transition_fault.json'
        records = json.loads(path.read_text())['records'][0]['recent']
        fixture.closing_floor_history = FloorContactHistory()
    for name, value in vars(fixture).items():
        setattr(cycle, name, value)
    events = []
    monkeypatch.setattr(EndpointOverlapCycle, 'record', lambda self, kind, payload: events.append((kind, payload)))
    for record in records:
        cycle.record(record['kind'], record['payload'])
    assert len(events) == len(records)
    assert cycle.closing_floor_history.rows['left']['positive'] is not None
    assert not actual_preload_support(cycle, 'left', retrieval=True)['valid']
    cycle.contacts['right'] = []
    cycle.pose_time = cycle.sim
    cycle.order.locations = dict(left='basket', right='basket')
    cycle.support_continuity = {payload:SupportContinuity(.02) for payload in ('left','right')}
    cycle.support_continuity['left'].last_positive_sim = cycle.sim-.0005
    original = sequential_cycle.retention_metrics
    def measured(poses, pairs, payload):
        if payload == 'right':
            return dict(supported=True, contacts_valid=True, sidewall_certified=True, vertical_inside=True)
        return original(poses,pairs,payload)
    monkeypatch.setattr(sequential_cycle, 'retention_metrics', measured)
    cycle.safety()
    classifications = [payload for kind,payload in events if kind == 'sequential_single_frame_floor_continuity']
    assert len(classifications) == 1
    assert classifications[0]['certificate']['observed_fingers'] == [1]
    assert classifications[0]['certificate']['not_a_grip_certificate']
    assert classifications[0]['actual_holders'] == {}
    assert not cycle.holders.rows

def test_existing_machine_result_recording_and_parent_return_preserved(monkeypatch):
    cycle = SequentialCycle.__new__(SequentialCycle)
    cycle.closing_floor_history = FloorContactHistory()
    events = []
    def parent(self, kind, payload):
        events.append((kind,payload))
        return 'parent-result'
    monkeypatch.setattr(EndpointOverlapCycle, 'record', parent)
    payload = dict(status='PASS', nested=dict(value=1))
    assert cycle.record('MACHINE_CYCLE_PASS', payload) == 'parent-result'
    payload['nested']['value'] = 2
    assert cycle.machine_cycle_result['nested']['value'] == 1
    assert cycle.record('unrelated', {}) == 'parent-result'
    assert len(events) == 2 and not cycle.closing_floor_history.rows
