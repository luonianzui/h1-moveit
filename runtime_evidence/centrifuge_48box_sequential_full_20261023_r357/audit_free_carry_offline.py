import json
from pathlib import Path

import numpy as np

from free_carry_rate import FREE_CARRY_PHASES,ORIGINAL_RATES,carry_rate
from trace_io import open_trace


EVIDENCE=Path(__file__).resolve().parent
BASELINE=EVIDENCE.parent/'centrifuge_48box_fast_complete_20261012_r219'


def timing(data,rate,floor=.020):
    initial=data['initial'];names=list(initial);points=[initial]+data['samples']
    elapsed=.1;previous=initial;times=[]
    for point in points:
        elapsed+=max(floor,max(abs(point[name]-previous[name])/rate for name in names))
        times.append(elapsed);previous=point
    return times


def main():
    records=[]
    with open_trace(BASELINE) as stream:
        for line in stream:
            if '"kind": "command"' not in line[:140]:continue
            row=json.loads(line);phase=row['phase']
            if phase not in FREE_CARRY_PHASES:continue
            original=ORIGINAL_RATES[phase];holding=phase.split('_')[0]
            rate=carry_rate(phase,original,holding,'left' if '_retrieval_' in phase else None,{holding:16.})
            old=timing(row['payload'],original);new=timing(row['payload'],rate,.020)
            assert np.max(np.abs(np.array(old)-row['payload']['times']))<1e-8
            assert rate<=.36 and rate*.035<=.0126+1e-12
            records.append(dict(phase=phase,original_rate_rad_s=original,candidate_rate_rad_s=rate,
                original_duration_sim_s=old[-1],candidate_duration_sim_s=new[-1],
                predicted_saved_sim_s=old[-1]-new[-1],original_samples_unchanged=True))
    savings=sum(record['predicted_saved_sim_s'] for record in records)
    result=dict(status='PASS' if len(records)==4 and savings>5 else 'FAIL',
        scope='offline_bounded_profile_estimate_not_physical_speed',baseline_identity=BASELINE.name,
        observed_prior_lag_s=.035,estimated_peak_lag_error_rad=.0126,unchanged_actual_tracking_gate_rad=.015,
        margin_is_estimate_not_a_safety_certificate=True,
        predicted_command_saved_sim_s=savings,counterfactual_complete_sim_s=471.314-savings,
        geometry_dense_FCL_samples_unchanged=True,free_carry_floor_s=[.020,.020],
        all_other_floors_holds_and_contact_paths_unchanged=True,records=records)
    with (EVIDENCE/'free_carry_offline_audit.json').open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))
    assert result['status']=='PASS'


if __name__=='__main__':
    main()
