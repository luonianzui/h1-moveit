import copy


def declare_prospective_empty_holder_scope(request):
    if request.get('mode') != 'retrieval_pickup' or request.get('retrieval') is not True:
        raise ValueError('Only prospective retrieval pickup fixtures may declare this scope')
    if request.get('nominal_phase_fixture_is_prospective_not_actual_new_stop') is not True:
        raise ValueError('An explicitly prospective nominal fixture is required, not an actual runtime request')
    if 'original_runtime_holder_references' in request and request['original_runtime_holder_references'] != {}:
        raise ValueError('A declared nonempty or unknown holder registry cannot be cleared')
    result = copy.deepcopy(request)
    result['original_runtime_holder_references'] = {}
    result['prospective_empty_holder_registry_not_measured'] = True
    result['not_current_fresh_execution_permission'] = True
    return result
