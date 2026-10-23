import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
REVISION = ROOT / 'revision2'
manifest = json.loads((REVISION / 'edit_manifest.json').read_text())
qa = json.loads((REVISION / 'technical_qa.json').read_text())
source = Path(manifest['output'])
destination = Path('/home/liu/视频/H1-2_离心机双盒流程_汇报版_20261023_r356.mp4')
assert qa['full_decode_returncode'] == 0 and qa['source_unchanged']
assert destination.parent.is_dir()
with source.open('rb') as incoming, destination.open('xb') as outgoing:
    shutil.copyfileobj(incoming, outgoing)
with destination.open('rb') as stream:
    actual = hashlib.file_digest(stream, 'sha256').hexdigest()
assert actual == manifest['output_sha256']
receipt = dict(status='PASS_DELIVERED_EDITED_BRIEFING', file=str(destination), sha256=actual,
    duration_s=qa['duration_s'], size_bytes=qa['size_bytes'],
    original_untouched=True, full_decode_returncode=0, actual_visual_review=True,
    visually_reviewed_revision2_seconds=[1,44,65,76,97,139,153],
    visual_observations='Chinese/Latin/digits render correctly; robot, manipulation and stage labels clear; final overhead shows both boxes on separate table slots and empty baskets/grippers. Selected frames inspected, not every frame.',
    actual_source_execution_wall_s=manifest['source_task_wall_s'],
    edited_duration_is_not_actual_execution_time=True)
with (ROOT / 'DELIVERY.json').open('x') as stream:
    json.dump(receipt, stream, ensure_ascii=False, indent=2)
print(json.dumps(receipt, ensure_ascii=False))
