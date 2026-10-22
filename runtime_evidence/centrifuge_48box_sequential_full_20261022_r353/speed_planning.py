import copy

from sequential_geometry import validate_request
from sequential_worker import loaded_reference, plan as original_plan
from axial_transport import retrieve_transport
from axial_pilot import retrieval_pilot
from table_alignment import MODE, plan_table_alignment


def plan(request):
    if request.get('mode') not in ('retrieval_transport', 'retrieval_pilot', MODE):
        return original_plan(request)
    validate_request(request)
    original = copy.deepcopy(request)
    reference = loaded_reference(request)
    result = (plan_table_alignment(request, reference) if request['mode'] == MODE
              else retrieval_pilot(request, reference) if request['mode'] == 'retrieval_pilot'
              else retrieve_transport(request, reference))
    if request != original:
        raise RuntimeError('Planner mutated actual snapshot or original holder anchors')
    result.update(sequential_contract=request['sequential_contract'], mode=request['mode'],
                  original_runtime_holder_references=copy.deepcopy(request['original_runtime_holder_references']),
                  execution_dispatch_not_qualified=True)
    return result
