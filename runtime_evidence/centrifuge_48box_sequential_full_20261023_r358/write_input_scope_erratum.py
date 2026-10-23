import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
source = ROOT / 'inherited_inputs_audit.json'
assert json.loads(source.read_text())['status'] == 'PASS'
with (ROOT / 'inherited_inputs_runtime_scope_erratum.json').open('x') as stream:
    json.dump(dict(status='CURRENT_RUNTIME_SCOPE_CORRECTION', original_audit_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                   original_geometric_and_control_input_verification_unchanged=True,
                   inherited_twenty_N_label_is_stale_not_current_runtime=True,
                   current_retrieval_feedforward_N=18., loading_feedforward_N=16.,
                   original_software_output_guard_N=18.01, current_empty_pickup_approach_cap_rad_s=.35,
                   only_new_runtime_hypothesis='exact current actual optional alignment reverse contract; small optional alignment eleven targets/current warm seed; inherited axis exit and all physics gates preserved',
                   prior_geometry_proofs_not_fresh_physics_or_180_certificate=True), stream, indent=2)
