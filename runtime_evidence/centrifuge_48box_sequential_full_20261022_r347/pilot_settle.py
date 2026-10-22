import copy
import math

RETRIEVAL_PILOT_SETTLE_SIM_S = 1.0

def settle_retrieval_pilot(cycle, hand, payload):
    before = copy.deepcopy(cycle.holders.snapshot())
    if (hand not in ('left', 'right') or payload not in ('left', 'right') or
            set(before) != {hand} or before[hand]['payload'] != payload or
            cycle.phase != 'seq_retrieval_single_pilot' or not math.isfinite(cycle.sim)):
        raise RuntimeError('Exact completed retrieval pilot and original single holder required')
    began = cycle.sim
    cycle.record('sequential_retrieval_pilot_settle_started', dict(hand=hand, payload=payload,
        requested_sim_s=RETRIEVAL_PILOT_SETTLE_SIM_S, original_runtime_holder_references=before))
    cycle.dwell(RETRIEVAL_PILOT_SETTLE_SIM_S)
    after = cycle.holders.snapshot()
    if (after != before or cycle.phase != 'seq_retrieval_single_pilot' or not math.isfinite(cycle.sim) or
            cycle.sim - began < RETRIEVAL_PILOT_SETTLE_SIM_S):
        raise RuntimeError('Original guarded pilot settling did not complete unchanged')
    cycle.record('sequential_retrieval_pilot_settle_pass', dict(hand=hand, payload=payload,
        duration_sim_s=cycle.sim-began, original_runtime_holder_references=after,
        no_rebase_or_new_gripper_command=True, fresh_planning_still_required=True,
        original_snapshot_bounds_unchanged=True))
