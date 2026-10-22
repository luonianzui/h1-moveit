import math

from bucket_axis_exit import bucket_axis_exit_targets
from extraction_clearance import bucket_rim_bounds
from execute_loading import pose_matrix


def stage_body_targets(request, phase, first, last, count, guides, legacy, model_path):
    if request.get('mode') != 'retrieval_transport' or phase != 'lift':
        return legacy(first, last, count)
    holders = request.get('original_runtime_holder_references')
    if not holders:
        return legacy(first, last, count)
    if (request.get('retrieval') is not True or not isinstance(holders, dict)
            or set(holders) != {'left', 'right'} or count != 81 or guides is not None):
        raise ValueError('Only original actual paired retrieval lift can select the new target provider')
    assignment = request.get('assignment', {})
    if set(assignment) != set(holders) or set(assignment.values()) != {'left', 'right'}:
        raise ValueError('Two distinct actual payloads required')
    source = request.get('source_sim')
    if source is None or not math.isfinite(source):
        raise ValueError('Finite actual transport snapshot required')
    for hand, holder in holders.items():
        if (not isinstance(holder, dict) or holder.get('payload') != assignment[hand]
                or not math.isfinite(holder.get('source_sim', math.nan))
                or holder['source_sim'] > source):
            raise ValueError('Original physical holder identity and frozen source epoch required')
    return bucket_axis_exit_targets(request, first, last, count, bucket_rim_bounds(model_path), pose_matrix, legacy)
