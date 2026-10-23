import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent/'centrifuge_48box_sequential_review_20261019_r15/r312_settling_analysis.json'
expected='7c2056db000589781e0cb4f1ba2cc168a3893e246ef17b9a5e82b3b0cfdf7867'
assert hashlib.sha256(SOURCE.read_bytes()).hexdigest()==expected
data=json.loads(SOURCE.read_text())
check=data['actual_snapshot_checks'][0]['payload']
assert check['pose_drift_m']>.0003 and check['finger_drift_m']>.0003
later=[row for row in data['half_second_windows'] if row['delta_sim']>=1.]
assert later and all(row['original_0_3mm_0_1deg_bounds_pass'] for row in later)
result=dict(status='PASS_ACTUAL_SETTLING_TREND_NOT_NEW_PHYSICAL_RESULT',source=str(SOURCE),
    source_sha256=expected,original_failed_snapshot=check,observed_later_half_second_windows=len(later),
    maximum_later_position_drift_m=max(row['position_drift_m'] for row in later),
    maximum_later_angle_drift_deg=max(row['angle_drift_deg'] for row in later),
    proposed_guarded_wait_sim_s=1.,original_fresh_snapshot_check_still_required=True,
    original_path_force_contact_grip_and_observation_gates_unchanged=True,no_new_task_success_claim=True)
with (ROOT/'pilot_settle_qualification.json').open('x') as stream:
    json.dump(result,stream,indent=2,allow_nan=False)
print(json.dumps(result,indent=2))
