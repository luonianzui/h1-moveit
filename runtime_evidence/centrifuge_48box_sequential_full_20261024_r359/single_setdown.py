import copy

from paired_ramp import ramped_common_timing
from sequential_geometry import CONTRACT

COARSE_PHASE = 'paired_retrieval_setdown'
FINE_PHASE = 'paired_retrieval_setdown_fine'

def single_setdown_profile(hand):
    if hand not in ('left', 'right'):
        raise ValueError('Known original single holder required')
    return (.06, 1.2) if hand == 'right' else (.13, .6)

def require_single_setdown_holder(cycle, context):
    hand, payload = context['hand'], context['payload']
    actual = cycle.holders.snapshot()
    if (set(actual) != {hand} or actual[hand]['payload'] != payload or
            actual != context['original_holders'] or cycle.order.phase != 'retrieval' or
            cycle.order.holder != dict(hand=hand, payload=payload) or
            cycle.order.locations[payload] != 'held' or cycle.order.retrieval_assignment.get(hand) != payload):
        raise ValueError('Original single retrieval holder and assignment required for setdown')
    return actual

def single_setdown_timing(cycle, actual, samples, rate):
    context = getattr(cycle, 'sequential_setdown_context', None)
    if (not isinstance(context, dict) or cycle.phase != COARSE_PHASE or
            rate != single_setdown_profile(context['hand'])[0] or
            samples != context['coarse_samples']):
        raise ValueError('Exact qualified single coarse setdown context and point-thirteen rate required')
    holders = require_single_setdown_holder(cycle, context)
    expected_rate, ramp = single_setdown_profile(context['hand'])
    timing = ramped_common_timing(actual, samples, expected_rate, ramp_s=ramp)
    cycle.record('sequential_single_setdown_timing', dict(hand=context['hand'], payload=context['payload'],
        source_snapshot_sha256=context['source_snapshot_sha256'], original_runtime_holder_references=holders,
        actual_rate_rad_s=expected_rate, ramp_sim_s=ramp, unchanged_samples=True,
        legacy_phase_retained_for_original_table_clearance=True, paired_profile_not_claimed=True))
    return timing

def execute_single_setdown(cycle, planned, samples, split):
    matches = [stage for stage in planned['stages'] if stage['phase'] == 'retrieve_setdown']
    if (getattr(cycle, 'sequential_setdown_context', None) is not None or
            cycle.phase != 'seq_retrieval_retrieve_transfer' or
            planned['status'] != 'PASS_GEOMETRY_ONLY' or planned['mode'] != 'retrieval_transport' or
            planned['sequential_contract'] != CONTRACT or planned['maximum_joint_step_rad'] != .002 or
            len(matches) != 1 or matches[0]['samples'] != samples or
            split != max(1,len(samples)-8)):
        raise ValueError('Actual original single retrieval plan and coarse/fine split required')
    context = dict(hand=planned['single_hand'],payload=planned['single_payload'],
        coarse_samples=copy.deepcopy(samples[:split+1]),
        original_holders=copy.deepcopy(planned['original_runtime_holder_references']),
        source_snapshot_sha256=planned['source_snapshot_sha256'])
    require_single_setdown_holder(cycle, context)
    cycle.sequential_setdown_context = context
    try:
        cycle.move(samples[:split+1], COARSE_PHASE, single_setdown_profile(context['hand'])[0])
        require_single_setdown_holder(cycle, context)
        cycle.move(samples[split:], FINE_PHASE, .03)
        require_single_setdown_holder(cycle, context)
    finally:
        cycle.sequential_setdown_context = None
