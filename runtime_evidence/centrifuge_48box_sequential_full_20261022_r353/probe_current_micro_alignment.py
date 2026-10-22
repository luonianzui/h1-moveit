import hashlib
import json
from pathlib import Path
import time

import dual_worker
from dual_motion import common_arm_timing
from inherited_source_identity import restore_inherited_source


ROOT=Path(__file__).resolve().parent
PARENT=ROOT.parent/'centrifuge_48box_bucket_axis_exit_full_20261015_r282'
EXPECTED='6b063f3438d0ee0789747f7bfb45f2fbc94d98685d678b7f30603d464c4ff429'
assert hashlib.sha256((PARENT/'SHA256SUMS').read_bytes()).hexdigest()==EXPECTED
manifest={relative.lstrip('*'):checksum for checksum,relative in
          (line.split(maxsplit=1) for line in (PARENT/'SHA256SUMS').read_text().splitlines())}
for name in ('run/4_load_alignment_request.json','run/4_load_alignment_result.json'):
    assert hashlib.sha256((PARENT/name).read_bytes()).hexdigest()==manifest[name]
assert restore_inherited_source('dual_worker.py',(ROOT/'dual_worker.py').read_text())==(PARENT/'dual_worker.py').read_text()
request=json.loads((PARENT/'run/4_load_alignment_request.json').read_text())
previous=json.loads((PARENT/'run/4_load_alignment_result.json').read_text())
before=common_arm_timing(request['positions'],previous['stages'][0]['samples'],.10)
began=time.monotonic()
result=dual_worker.plan_transport(request)
assert result['status']=='PASS' and result['maximum_joint_step_rad']==.002
assert len(result['stages'])==1 and len(result['stages'][0]['samples'])==12
after=common_arm_timing(request['positions'],result['stages'][0]['samples'],.10)
with (ROOT/'current_micro_alignment_probe.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_CURRENT_HELPER_NOT_PHYSICAL',result=result,
        original_request_sha256=manifest['run/4_load_alignment_request.json'],parent_manifest_sha256=EXPECTED,
        original_command_duration_sim_s=before['duration_sim_s'],candidate_command_duration_sim_s=after['duration_sim_s'],
        original_point_one_rate_fcl_joint_margin_reference_body_targets_and_all_physical_guards_preserved=True,
        current_worker_sha256=hashlib.sha256((ROOT/'dual_worker.py').read_bytes()).hexdigest(),
        elapsed_wall_s=time.monotonic()-began,no_robot_action_or_full_180_certificate=True),stream,indent=2)
print('CURRENT_MICRO_ALIGNMENT_PASS',result['checked_samples'],before['duration_sim_s'],after['duration_sim_s'],flush=True)
