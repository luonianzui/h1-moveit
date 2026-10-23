import hashlib
import json
from pathlib import Path

from dual_motion import common_arm_timing
from sequential_geometry import SinglePlanner, pose_matrix


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261019_r306'
request_path = PARENT / 'run/15_retrieval_transport_left_request.json'
result_path = PARENT / 'run/15_retrieval_transport_left_result.json'
assert hashlib.sha256(request_path.read_bytes()).hexdigest() == '330885a77acef3cc03cddd6bec1f4bfc06f2e805f716fba762ee7f4433b25f89'
assert hashlib.sha256(result_path.read_bytes()).hexdigest() == '34c74fcb4d7e193c4b592a267afdc4992adcaec9b1ff972e9dcbfa81f1a43681'
request, planned = json.loads(request_path.read_text()), json.loads(result_path.read_text())
stage = planned['stages'][0]
assert stage['phase'] == 'retrieve_axis_exit'
planner = SinglePlanner(request, planned['planning_reference'])
first = {name: request['positions'][name] for name in planner.names}
assert stage['samples'][0] == first
planner.attach(pose_matrix(request['poses']['box48_' + request['payload']]), first)
for index, (initial, target) in enumerate(zip(stage['samples'], stage['samples'][1:])):
    planner.check_edge(initial, target, 'independent_actual_axis_exit',
                       initial_support='basket' if index <= 1 else None)
old = common_arm_timing(request['positions'], stage['samples'], .25)
new = common_arm_timing(request['positions'], stage['samples'], .03)
assert new['points'] == old['points'] and new['groups'] == old['groups']
assert new['duration_sim_s'] > old['duration_sim_s'] * 4
record = dict(status='PASS_ACTUAL_FAILED_PATH_RETIMED_NOT_PHYSICAL',
              source_request_sha256=hashlib.sha256(request_path.read_bytes()).hexdigest(),
              source_result_sha256=hashlib.sha256(result_path.read_bytes()).hexdigest(),
              independent_dense_step_rad=.002, independent_dense_checks=planner.checked,
              original_starting_basket_contact_exception_unchanged=True,
              old_rate_rad_s=.25, new_rate_rad_s=.03, tracking_limit_rad=.002,
              old_duration_sim_s=old['duration_sim_s'], new_duration_sim_s=new['duration_sim_s'],
              exact_geometry_points_and_holders_unchanged=True,
              force_40N_output_18_01N_and_observation_guards_unchanged=True,
              no_new_physical_pass_or_three_minute_claim=True)
with (ROOT / 'axis_exit_qualification.json').open('x') as stream:
    json.dump(record, stream, indent=2)
print(json.dumps(record), flush=True)
