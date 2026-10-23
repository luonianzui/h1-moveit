import ast
import hashlib
import json
from pathlib import Path

REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261023_r355'
def read(folder, name):
    return json.loads((folder / name).read_text())

tree = ast.parse((REVIEW / 'freeze_results.py').read_text())
definitions = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom, ast.FunctionDef))]
namespace = {}
exec(compile(ast.Module(body=definitions, type_ignores=[]), '<same_sealer>', 'exec'), namespace)
assert read(ROOT, 'run/result.json')['status'] == 'FAIL'
assert not read(ROOT, 'run/result.json')['actual_holder_registry']
assert not read(ROOT, 'run/retirement.json')['remaining']
assert read(ROOT, 'run/completion_finalization.json')['status'] == 'PASS'
assert read(ROOT, 'video_audit.json')['full_decode_returncode'] == 0
for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected',
    'resumed_protected', 'additional_protected', 'dual_protected', 'current_protected', 'startup_protected',
    'r353_protection', 'r354_protection'):
    assert read(ROOT, prefix + '_before.json') == read(ROOT, prefix + '_after.json'), prefix
for relative, checksum in read(ROOT, 'identity.json')['sha256'].items():
    assert namespace['digest'](ROOT / relative) == checksum
core = namespace['seal'](ROOT)
with (REVIEW / 'failure_seal.json').open('x') as stream:
    json.dump(dict(status='SEALED_FAILED_FULL_ATTEMPT_NOT_SUCCESS',
        source_identity_sha256=namespace['digest'](ROOT / 'identity.json'), core=core,
        video_sha256=namespace['digest'](ROOT / 'run/full_cycle.mp4'),
        original_success_r354_unchanged=True, no_remaining_owned_processes=True), stream, indent=2)
namespace['seal'](REVIEW)
