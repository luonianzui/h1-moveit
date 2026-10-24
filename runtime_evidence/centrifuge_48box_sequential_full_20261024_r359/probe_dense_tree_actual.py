import hashlib
import json
from pathlib import Path
import time

import dense_tree
import dual_worker
from dual_motion import common_arm_timing


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_empty_protocol_full_20261015_r280'
EXPECTED = '0348672ca949bb209fe074b8841380676ec8c9b2297fba5f43dd6384d1a4d5f1'
assert hashlib.sha256((PARENT/'SHA256SUMS').read_bytes()).hexdigest() == EXPECTED
manifest = {relative.lstrip('*'):checksum for checksum,relative in
            (line.split(maxsplit=1) for line in (PARENT/'SHA256SUMS').read_text().splitlines())}
for name in ('run/7_retrieval_pickup_request.json','run/7_retrieval_pickup_result.json'):
    assert hashlib.sha256((PARENT/name).read_bytes()).hexdigest() == manifest[name]
request = json.loads((PARENT/'run/7_retrieval_pickup_request.json').read_text())
assert request['retrieval'] is True and request['original_runtime_holder_references'] == {}
assert dual_worker.select_free_path is dense_tree.select_free_path
assert dual_worker.select_free_path(request,object()) is dense_tree.free_path
began = time.monotonic()
result = dual_worker.plan_pickup(request)
assert result['status'] == 'PASS' and result['maximum_joint_step_rad'] == .002
assert result['prospective_retrieval_lookahead']['actual_post_grasp_plan_and_guards_still_required']
samples = next(stage['samples'] for stage in result['stages'] if stage['phase'] == 'approach')
timing = common_arm_timing(request['positions'],samples,.35)
with (ROOT/'dense_tree_current_probe.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_CURRENT_SELECTOR_NOT_PHYSICAL',result=result,
        request_sha256=manifest['run/7_retrieval_pickup_request.json'],parent_manifest_sha256=EXPECTED,
        current_worker_sha256=hashlib.sha256((ROOT/'dual_worker.py').read_bytes()).hexdigest(),
        current_dense_tree_sha256=hashlib.sha256((ROOT/'dense_tree.py').read_bytes()).hexdigest(),
        tree_and_final_full_state_sampling_rad=.002,approach_rate_rad_s=.35,
        approach_command_duration_sim_s=timing['duration_sim_s'],elapsed_wall_s=time.monotonic()-began,
        no_parent_failure_relabelled_or_new_physical_certificate=True),stream,indent=2)
print('CURRENT_DENSE_TREE_SELECTOR_OFFLINE_PASS',len(samples),timing['duration_sim_s'],time.monotonic()-began,flush=True)
