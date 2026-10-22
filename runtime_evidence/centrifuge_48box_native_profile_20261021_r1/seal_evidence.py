import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
REVIEW = ROOT.parent / 'centrifuge_48box_sequential_review_20261021_r10'
TASK = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r342'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


assert json.loads((REVIEW / 'final_review.json').read_text())['status'] == 'PASS_COMPLETE_SEQUENTIAL_TASK'
assert json.loads((REVIEW / 'native_symbol_audit.json').read_text())['status'] == 'PASS_EXACT_EXPORTED_SYMBOL_SIZE_AND_PROCESS_MAP_AUDIT'
dependencies = json.loads((TASK / 'native_profiler_dependencies.json').read_text())
for name, expected in dependencies['sha256'].items():
    assert digest(Path(name)) == expected
members = sorted(path for path in ROOT.rglob('*') if path.is_file() and not path.is_symlink()
                 and not {'build', 'colcon_log', '__pycache__'}.intersection(path.relative_to(ROOT).parts))
assert not (ROOT / 'SHA256SUMS').exists()
with (ROOT / 'SHA256SUMS').open('x') as stream:
    stream.write(''.join(digest(path) + '  ' + str(path.relative_to(ROOT)) + '\n' for path in members))
for path in members + [ROOT / 'SHA256SUMS']:
    path.chmod(0o444)
result = dict(status='SEALED_NATIVE_OBSERVER_WITH_REAL_TASK_VALIDATION', members=len(members),
              manifest_sha256=digest(ROOT / 'SHA256SUMS'), dependencies=dependencies)
with (REVIEW / 'native_plugin_seal.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
