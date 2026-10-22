import hashlib
import json
import math

from leg_initialization_contract import LEG_JOINTS, endpoint_error


def verify_initialization(root, original, actual):
    report = json.loads((root / 'run/leg_initialization.json').read_text())
    if report.get('status') != 'PASS_STATIONARY_INITIALIZATION_ONLY' or report.get('action_error_code') != 0:
        raise ValueError('Fresh successful stationary initialization required')
    if report['original_plan_sha256'] != hashlib.sha256((root / 'inputs/plan.json').read_bytes()).hexdigest():
        raise ValueError('Initialization must use this identity original unchanged plan')
    if report['original_targets'] != [original[name] for name in LEG_JOINTS]:
        raise ValueError('Initialization cannot rebase original targets')
    if not math.isfinite(report['hold_sim_seconds']) or report['hold_sim_seconds'] < 5.:
        raise ValueError('Five actual simulation seconds required before manipulation')
    if not math.isfinite(report['hold_peak_error_rad']) or report['hold_peak_error_rad'] >= .005:
        raise ValueError('Original endpoint must hold continuously')
    if endpoint_error(original, report['after']) >= .005 or endpoint_error(original, actual) >= .010:
        raise ValueError('Original target fresh feedback no longer valid')
    return report
