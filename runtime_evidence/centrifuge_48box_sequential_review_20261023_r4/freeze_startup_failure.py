import ast
import json
from pathlib import Path
import sys

REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261023_r357'
tree = ast.parse((REVIEW / 'freeze_results.py').read_text())
definitions = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom, ast.FunctionDef))]
namespace = {}
exec(compile(ast.Module(body=definitions, type_ignores=[]), '<same_sealer>', 'exec'), namespace)
def read(folder, name):
    return json.loads((folder / name).read_text())
assert read(ROOT, 'run/leg_initialization.json')['status'] == 'FAIL'
assert read(ROOT, 'run/leg_initialization.json')['samples'] == []
assert not (ROOT / 'run/execution.claim').exists()
assert not (ROOT / 'run/full_cycle.mp4').exists()
assert read(ROOT, 'run/retirement.json')['remaining'] == []
for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected', 'resumed_protected',
               'additional_protected', 'dual_protected', 'current_protected', 'startup_protected',
               'r353_protection', 'r354_protection'):
    assert read(ROOT, prefix + '_before.json') == read(ROOT, prefix + '_after.json')
for relative, checksum in read(ROOT, 'identity.json')['sha256'].items():
    assert namespace['digest'](ROOT / relative) == checksum
if sys.argv[1:] == ['core']:
    core = namespace['seal'](ROOT)
    with (REVIEW / 'failure_seal.json').open('x') as stream:
        json.dump(dict(status='SEALED_STARTUP_FAILURE_NO_TASK_VIDEO', core=core,
            source_identity_sha256=namespace['digest'](ROOT / 'identity.json'),
            original_r356_preserved=True), stream, indent=2)
else:
    assert sys.argv[1:] == ['review']
    assert read(REVIEW, 'documentation_validation.json')['status'] == 'PASS'
    namespace['seal'](REVIEW)
