import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'centrifuge_48box_dart_response_20261022_r1'
WORKSPACE = ROOT.parents[1]
library = WORKSPACE / 'install_lab_scene_48box_dart_response_20261022_r1/h12_dart_response_r1/lib/libDartContactResponse.so'
assert (SOURCE / 'conda_six_before.txt').read_bytes() == (SOURCE / 'conda_six_after.txt').read_bytes()
records = [json.loads(path.read_text().splitlines()[-1]) for path in sorted(SOURCE.glob('qualification*.log'))]
assert len(records) == 6
assert all(row['status'] == 'PASS_EXACT_CONTACT_RESPONSE_NOT_FULL_TASK' and row['world_steps'] == 2400
           and row['contact_steps'] > 1000 and row['response_cases'] >= 96
           and row['component_speedup'] > 1. for row in records)
destination = ROOT / 'inputs/libDartContactResponse.so'
with destination.open('xb') as stream:
    stream.write(library.read_bytes())
dependencies = {}
for source in [library.resolve(), destination, SOURCE / 'plugin_src/ContactResponse.cc',
               SOURCE / 'plugin_src/QualifyResponse.cc', SOURCE / 'UpstreamContactConstraint.cpp',
               *sorted(SOURCE.glob('qualification*.log'))]:
    dependencies[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
with (ROOT / 'dart_response_dependencies.json').open('x') as stream:
    json.dump(dict(status='PASS_EXACT_RESPONSE_COMPONENT_NOT_FULL_ROBOT',
        records=records, median_component_speedup=statistics.median(row['component_speedup'] for row in records),
        sha256=dependencies, original_arithmetic_and_constraints_unchanged=True), stream, indent=2)
print('DART_RESPONSE_COMPONENT_BOUND_NOT_FULL_TASK', flush=True)
