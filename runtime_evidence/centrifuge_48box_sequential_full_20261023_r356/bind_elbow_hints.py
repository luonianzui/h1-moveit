import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_minimum_turn_full_20261015_r285'
manifest_path = PARENT / 'SHA256SUMS'
assert hashlib.sha256(manifest_path.read_bytes()).hexdigest() == '424038061100b6fa80df89bb325416ae25970e90258576dfa7d7ba6b90373d76'
manifest = dict((name, checksum) for checksum, name in (line.split('  ', 1) for line in manifest_path.read_text().splitlines()))
hints = {}
for prefix in ('1_load_pickup', '7_retrieval_pickup'):
    relative = 'run/' + prefix + '_result.json'
    content = (PARENT / relative).read_bytes()
    assert hashlib.sha256(content).hexdigest() == manifest[relative]
    data = json.loads(content)
    assert data['status'] == 'PASS'
    mode = 'retrieval_pickup' if prefix.startswith('7_') else 'load_pickup'
    hints[mode] = dict(source=str(PARENT / relative), source_sha256=manifest[relative],
                       approach=data['stages'][0]['samples'], grip=data['stages'][-1]['samples'][-1])
with (ROOT / 'inputs/elbow_seed_hints.json').open('x') as stream:
    json.dump(dict(scope='PLANNING_HINTS_ONLY_REQUIRE_CURRENT_FULL_DENSE_FCL', hints=hints,
                   no_current_scene_or_physical_certificate_inherited=True), stream, indent=2)
print('BOUND_FROZEN_HINTS_NOT_CURRENT_TRAJECTORY_QUALIFICATION', flush=True)
