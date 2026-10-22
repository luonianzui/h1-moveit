import hashlib
import json
from pathlib import Path

from sequential_worker import plan

ROOT=Path(__file__).resolve().parent
records=[]
for identity,prefix in [('centrifuge_48box_sequential_full_20261020_r316','19_retrieval_transport_right'),
                        ('centrifuge_48box_sequential_full_20261020_r315','19_retrieval_transport_right'),
                        ('centrifuge_48box_sequential_full_20261020_r315','15_retrieval_transport_left')]:
    request_path=ROOT.parent/identity/'run'/(prefix+'_request.json')
    request=json.loads(request_path.read_text())
    result=plan(request)
    assert result['status']=='PASS_GEOMETRY_ONLY' and result['maximum_joint_step_rad']==.002
    assert result['original_runtime_holder_references']==request['original_runtime_holder_references']
    transfer=next(stage for stage in result['stages'] if stage['phase']=='retrieve_transfer')
    down=next(stage for stage in result['stages'] if stage['phase']=='retrieve_setdown')
    expected=1.13475 if request['hand']=='right' else 1.14475
    assert transfer['last_body'][2][3]==expected and down['last_body'][2][3]==.84475
    output_name=identity+'_'+prefix+'_height_result.json'
    with (ROOT/output_name).open('x') as output:
        json.dump(result,output,indent=2,allow_nan=False)
    records.append(dict(actual_request=str(request_path),request_sha256=hashlib.sha256(request_path.read_bytes()).hexdigest(),
        result=output_name,hand=request['hand'],height_m=expected,dense_checks=result['checked_samples'],
        maximum_ik_position_error_m=result['maximum_ik_position_error_m'],maximum_ik_angle_error_deg=result['maximum_ik_angle_error_deg']))
with (ROOT/'return_height_qualification.json').open('x') as output:
    json.dump(dict(status='PASS_ACTUAL_RETURN_HEIGHT_DENSE_GEOMETRY_NOT_PHYSICAL',records=records,
        unchanged_original_bounds_accuracy_collision_and_runtime_holder=True),output,indent=2)
print('PASS_ACTUAL_RETURN_HEIGHT_DENSE_GEOMETRY_NOT_PHYSICAL',records)
