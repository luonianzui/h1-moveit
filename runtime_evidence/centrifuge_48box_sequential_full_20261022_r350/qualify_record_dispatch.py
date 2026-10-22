import ast
import hashlib
import json
from pathlib import Path
from unittest.mock import patch

from endpoint_overlap_cycle import EndpointOverlapCycle
from preload_floor_frame import single_missing_floor_frame
from sequential_cycle import SequentialCycle
from test_record_dispatch import PARENT, r311_fixture, runtime_class

ROOT = Path(__file__).resolve().parent
source = (ROOT/'sequential_cycle.py').read_text()
parent = (PARENT/'sequential_cycle.py').read_text()
assert len([node for node in runtime_class(parent).body if isinstance(node, ast.FunctionDef) and node.name == 'record']) == 2
assert len([node for node in runtime_class(source).body if isinstance(node, ast.FunctionDef) and node.name == 'record']) == 1
assert (ROOT/'preload_floor_frame.py').read_bytes() == (PARENT/'preload_floor_frame.py').read_bytes()
fixture, metrics, events = r311_fixture()
cycle = SequentialCycle.__new__(SequentialCycle)
for name,value in vars(fixture).items():
    setattr(cycle,name,value)
assert not cycle.closing_floor_history.rows
before = single_missing_floor_frame(cycle,'left',metrics)
forwarded = []
with patch.object(EndpointOverlapCycle, 'record', lambda self,kind,payload: forwarded.append((kind,payload))):
    for event in events:
        cycle.record(event['kind'],event['payload'])
after = single_missing_floor_frame(cycle,'left',metrics)
assert not before['valid'] and after['valid'] and after['not_a_grip_certificate']
assert len(events) == len(forwarded) and not cycle.holders.rows
result = dict(status='PASS_ACTUAL_RECORD_BINDING_OFFLINE_NOT_TASK',
    original_duplicate_methods=2,current_record_methods=1, original_history_empty=before, classified_actual_history=after,
    original_parent_source_sha256=hashlib.sha256(parent.encode()).hexdigest(),
    current_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
    thresholds_and_commands_unchanged=True, no_new_physical_execution=True,
    parent_IO_only_mocked_real_SequentialCycle_record_executed=True)
with (ROOT/'record_dispatch_qualification.json').open('x') as stream:
    json.dump(result,stream,indent=2)
print(result['status'])
