import copy
import hashlib
import json
from pathlib import Path

from dual_worker import PairedPlanner
from gripper_return_geometry import FINGERS, MODES, plan_post_retreat_gripper
from post_retreat_contract import RETREAT_PHASES


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_pickup_approach_full_20261014_r276'
manifest = {relative.lstrip('*'): checksum for checksum, relative in
            (line.split(maxsplit=1) for line in (PARENT / 'SHA256SUMS').read_text().splitlines())}
path = PARENT / 'run/exact_velocity_window.json'
assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest[str(path.relative_to(PARENT))]
window = json.loads(path.read_text())
command = next(row['payload'] for row in window['commands']
               if row['phase'] == 'machine_paired_empty_return')
path = PARENT / 'run/4_empty_return_request.json'
assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest[str(path.relative_to(PARENT))]
request = json.loads(path.read_text())
request['positions'].update(copy.deepcopy(command['initial']))
native_fingers = next(row['payload']['positions'] for row in window['window']
                      if row['kind'] == 'raw_gripper_velocity')
request['positions'].update(native_fingers)
request.update(mode=MODES[0],
               current_run_guide_identity=ROOT.name, completed_empty_retreat_stages=list(RETREAT_PHASES),
               accepted_gripper_efforts=dict(left=0., right=0.))
stroke = plan_post_retreat_gripper(PairedPlanner, request, {}, ROOT.name)
request['mode'] = MODES[1]
request['positions'].update(dict.fromkeys(FINGERS, -.019))
neutral = json.loads((ROOT / 'inputs/plan.json').read_text())['initial_positions']
remaining = plan_post_retreat_gripper(PairedPlanner, request, neutral, ROOT.name)
with (ROOT / 'post_retreat_gripper_probe.json').open('x') as stream:
    json.dump(dict(status='PASS', stroke=stroke, remaining=remaining,
                   historical_current_scene_geometry_only_not_actual_fresh_new_run=True,
                   historical_arm_dispatch_seed_and_nearby_finger_stamp_not_coherent_new_actual_frame=True,
                   hypothetical_finger_endpoint_not_substituted_in_runtime=True,
                   original_full_scene_and_both_loose_boxes_no_touch_exemptions=True,
                   actual_retreat_actual_stroke_and_after_endpoint_FCL_still_required=True), stream, indent=2)
print('POST_RETREAT_GRIPPER_GEOMETRY_PASS', stroke['checked_samples'], remaining['checked_samples'], flush=True)
