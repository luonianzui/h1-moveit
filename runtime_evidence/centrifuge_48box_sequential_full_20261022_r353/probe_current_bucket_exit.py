from inherited_source_identity import restore_inherited_source
import hashlib
import json
from pathlib import Path
import time

import dual_worker
from dual_motion import common_arm_timing
from lift_provider_source import restore_lift_provider


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_dense_endpoint_full_20261015_r281'
EXPECTED = 'b9e88fbc1c433e0fc6042bbc40061016bf8bd3b4f02090eac8dee122a3f489ad'
assert hashlib.sha256((PARENT/'SHA256SUMS').read_bytes()).hexdigest() == EXPECTED
manifest = {relative.lstrip('*'):checksum for checksum,relative in
            (line.split(maxsplit=1) for line in (PARENT/'SHA256SUMS').read_text().splitlines())}
for name in ('run/9_retrieval_transport_request.json','run/9_retrieval_transport_result.json'):
    assert hashlib.sha256((PARENT/name).read_bytes()).hexdigest() == manifest[name]
assert restore_lift_provider(restore_inherited_source('dual_worker.py',(ROOT/'dual_worker.py').read_text())) == (PARENT/'dual_worker.py').read_text()
request = json.loads((PARENT/'run/9_retrieval_transport_request.json').read_text())
assert json.loads((PARENT/'run/9_retrieval_transport_result.json').read_text())['status'] == 'FAIL'
began = time.monotonic()
result = dual_worker.plan_transport(request)
assert result['status'] == 'PASS' and result['maximum_joint_step_rad'] == .002
stage = next(stage for stage in result['stages'] if stage['phase']=='lift')
assert len(stage['samples']) == 82
timing = common_arm_timing(request['positions'],stage['samples'],.4)
with (ROOT/'current_bucket_exit_probe.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_CURRENT_PROVIDER_NOT_PHYSICAL',result=result,
        original_failed_request_sha256=manifest['run/9_retrieval_transport_request.json'],
        parent_manifest_sha256=EXPECTED,current_worker_sha256=hashlib.sha256((ROOT/'dual_worker.py').read_bytes()).hexdigest(),
        current_provider_sha256=hashlib.sha256((ROOT/'lift_target_provider.py').read_bytes()).hexdigest(),
        original_cartesian_targets=81,original_lift_joint_points=82,lift_command_duration_sim_s=timing['duration_sim_s'],
        original_fcl_joint_margin_tilt_holder_reference_and_endpoint_goals_preserved=True,
        elapsed_wall_s=time.monotonic()-began,no_launch_command_attachment_rebase_or_collision_exemption=True),stream,indent=2)
print('CURRENT_BUCKET_EXIT_PROVIDER_PASS',result['checked_samples'],timing['duration_sim_s'],time.monotonic()-began,flush=True)
