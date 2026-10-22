import hashlib
import json
from pathlib import Path

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
SOURCE = ROOT / 'runtime_evidence/centrifuge_48box_full_cycle_20261009_r178'
records = {}
names = [path.name for path in SOURCE.iterdir() if path.is_file() and
         (path.name.startswith(('offline_', 'predicted_v2_', 'actual_stopped_')) or
          path.name in ('dense_audit_retry1.json', 'retrieval_endpoint_audit.json'))]
names += ['inputs/' + name for name in ('plan.json', 'retention_fault_fixture_r163.json',
          'retention_fault_fixture_r165.json', 'retention_fault_fixture_r166.json', 'robot_joint_state_relay_05ms.py')]
for name in names:
    original = SOURCE / name
    content = original.read_bytes()
    with (EVIDENCE / name).open('xb') as stream:
        stream.write(content)
    records[name] = dict(source=str(original), sha256=hashlib.sha256(content).hexdigest())
library = ROOT / 'install_centrifuge_brake_20261010_r189/h12_centrifuge_brake_r189/lib/libCentrifugeInterlockBrakeR189.so'
with (EVIDENCE / 'inputs/libCentrifugeInterlockBrakeR189.so').open('xb') as stream:
    stream.write(library.read_bytes())
records['inputs/libCentrifugeInterlockBrakeR189.so'] = dict(source=str(library), sha256=hashlib.sha256(library.read_bytes()).hexdigest())
with (EVIDENCE / 'prior_offline_proof.json').open('x') as stream:
    json.dump(dict(scope='identical geometry feasibility only, no new physical claim', records=records), stream, indent=2)

