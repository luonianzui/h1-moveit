import hashlib
import json
from pathlib import Path

from preload_floor_frame import single_missing_floor_frame
from sequential_support import actual_preload_support
from test_preload_floor_frame import actual_fixture


ROOT = Path(__file__).resolve().parent
source = ROOT.parent / 'centrifuge_48box_sequential_review_20261019_r13/r310_contact_gap.json'
assert hashlib.sha256(source.read_bytes()).hexdigest() == 'ca17d4f2a2ce027b313a36bdbfc087c60a5b13cf4dd4e10ed8fbd591278078a1'
observed = json.loads(source.read_text())
assert abs(observed['first_positive_after']['stamp'] - observed['last_positive_before']['stamp'] - .001) < 1e-10
cycle, metrics = actual_fixture()
original = actual_preload_support(cycle, 'left', retrieval=True)
assert not original['valid']
candidate = single_missing_floor_frame(cycle, 'left', metrics)
assert candidate['valid'] and candidate['observed_fingers'] == [1] and candidate['not_a_grip_certificate']
cycle.closing_floor_history.rows['left']['positive'] -= .0005
assert not single_missing_floor_frame(cycle, 'left', metrics)['valid']
with (ROOT / 'floor_frame_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_SINGLE_FRAME_CLASSIFICATION_NOT_GRIP_AND_NOT_TASK',
        source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        original_preload_still_rejected=original, candidate=candidate,
        actual_next_frame_restored_basket_support=observed['first_positive_after'],
        second_missing_frame_rejected=True, no_new_holder_or_motion=True), stream, indent=2, allow_nan=False)
print('ONE_MISSING_NATIVE_FRAME_QUALIFIED_NOT_GRIP_NOT_TASK', candidate, flush=True)
