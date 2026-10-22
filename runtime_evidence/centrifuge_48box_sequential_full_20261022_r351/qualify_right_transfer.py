import hashlib
import json
from pathlib import Path

from speed_planning import plan
from speed_policy import configure_simulation_profile
from audit_speed_paths import review_path, verify_return_height
from pipeline_source import restore_pipeline_source


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r334'
name = 'speed_retrieval_geometry.py'
previous = (PARENT / name).read_text().replace(PARENT.name, ROOT.name)
assert restore_pipeline_source(name, (ROOT / name).read_text()) == previous
source = PARENT / 'run/20_retrieval_transport_right_request.json'
content = source.read_bytes()
request = json.loads(content)
configure_simulation_profile()
result = plan(request)
assert result['maximum_joint_step_rad'] == .002
assert result['original_runtime_holder_references'] == request['original_runtime_holder_references']
assert result['maximum_ik_position_error_m'] <= .00002
assert result['maximum_ik_angle_error_deg'] <= .01
verify_return_height(request, result)
review = review_path(request, result)
with (ROOT / 'right_transfer_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ACTUAL_R334_FAILED_SNAPSHOT_DENSE_GEOMETRY_NOT_PHYSICAL',
        source=str(source), source_sha256=hashlib.sha256(content).hexdigest(),
        previous_height_m=1.13475, candidate_height_m=1.12475,
        final_table_target_unchanged=True, original_ik_precision_and_dense_collision_limits_unchanged=True,
        plan=result, independent_dense_review=review), stream, indent=2)
print('RIGHT_TRANSFER_ACTUAL_FAILURE_QUALIFIED_NOT_PHYSICAL', review, flush=True)
