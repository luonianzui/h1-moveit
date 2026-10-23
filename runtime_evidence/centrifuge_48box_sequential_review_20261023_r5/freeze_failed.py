import ast
import json
from pathlib import Path
import sys

REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261023_r358'
tree = ast.parse((REVIEW / 'freeze_results.py').read_text())
definitions = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom, ast.FunctionDef))]
namespace = {}
exec(compile(ast.Module(body=definitions, type_ignores=[]), '<same_sealer>', 'exec'), namespace)

def read(folder, name):
    return json.loads((folder / name).read_text())

result = read(ROOT, 'run/result.json')
assert result['status'] == 'FAIL' and result['holding'] == 'left'
assert result['order']['returned'] == [] and result['order']['loaded'] == ['left', 'right']
assert read(ROOT, 'run/retirement.json')['remaining'] == []
assert read(ROOT, 'run/completion_finalization.json')['status'] == 'PASS'
assert read(ROOT, 'video_audit.json')['full_decode_returncode'] == 0
assert read(ROOT, 'trace_audit.json')['status'] == 'PASS'
assert read(ROOT, 'speed_physical_audit.json')['status'] == 'FAIL'
for name in ('both_loaded', 'both_drop_stable', 'machine_cycle', 'steady_30_seconds'):
    assert read(ROOT, 'speed_physical_audit.json')['gates'][name]
grip = read(ROOT, 'independent_speed_grip.json')
assert grip['anchors'] == 3 and grip['observed_ends'] == 2 and grip['still_held'] == ['left']
assert read(REVIEW, 'axis_exit_failure_probe.json')['status'] == 'PASS_REPRODUCED_R358_GEOMETRIC_FAILURE_NOT_FIXED'
assert read(REVIEW, 'image_review.json')['actual_tool_inspection']
assert not read(REVIEW, 'image_review.json')['complete_task']
for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected', 'resumed_protected',
               'additional_protected', 'dual_protected', 'current_protected', 'startup_protected',
               'r353_protection', 'r354_protection', 'r357_protection'):
    assert read(ROOT, prefix + '_before.json') == read(ROOT, prefix + '_after.json')
for relative, checksum in read(ROOT, 'identity.json')['sha256'].items():
    assert namespace['digest'](ROOT / relative) == checksum
if sys.argv[1:] == ['core']:
    core = namespace['seal'](ROOT)
    with (REVIEW / 'failure_seal.json').open('x') as stream:
        json.dump(dict(status='SEALED_FAILED_FULL_ATTEMPT_NOT_SUCCESS',
            source_identity_sha256=namespace['digest'](ROOT / 'identity.json'), core=core,
            video_sha256=namespace['digest'](ROOT / 'run/full_cycle.mp4'),
            original_r356_preserved=True, task_returned_boxes=0,
            no_remaining_owned_processes=True), stream, indent=2)
else:
    assert sys.argv[1:] == ['review']
    assert read(REVIEW, 'documentation_validation.json')['status'] == 'PASS'
    for name, expected in read(REVIEW, 'documentation_after.json').items():
        assert namespace['digest'](REVIEW.parents[1] / name) == expected['sha256']
    namespace['seal'](REVIEW)
