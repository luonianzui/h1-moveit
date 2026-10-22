import copy
import json
import time

import numpy as np

from sequential_geometry import EVIDENCE, SinglePlanner
from probe_sequential_loading import pose_dictionary


def main():
    results = []
    for hand in ('left', 'right'):
        previous = json.loads((EVIDENCE / ('sequential_loading_' + hand + '.json')).read_text())
        request = copy.deepcopy(previous['request'])
        seed = previous['stages'][-1]['samples'][-1]
        request['positions'].update(seed)
        body = np.array(previous['stages'][-1]['last_body'])
        request['poses']['box48_' + hand] = pose_dictionary(body)
        planner = SinglePlanner(request, previous['reference'])
        planner.attach(body, seed)
        began = time.monotonic()
        result = dict(hand=hand, status='REJECTED_OFFLINE_GUIDED_SINGLE_LOAD', stages=[])
        try:
            for index, old in enumerate(reversed(previous['stages'])):
                guides = list(reversed(old['samples'][1:]))
                target = np.array(old['first_body'])
                stage = planner.guided_stage(body, target, seed, 'guided_load_' + str(index), guides,
                                             support_start='table' if index == 0 else None)
                result['stages'].append(stage)
                body, seed = target, stage['samples'][-1]
                print('GUIDED_LOAD_STAGE', hand, index, planner.checked, planner.elbow_metrics(seed)['from_down_deg'], flush=True)
            result.update(status='PASS_OFFLINE_GUIDED_SINGLE_LOAD', checked=planner.checked, elbow=planner.elbow_metrics(seed),
                          max_position_error_m=planner.maximum_position_error, max_angle_error_deg=planner.maximum_angle_error)
        except (RuntimeError, ValueError, AssertionError) as error:
            result['error'] = str(error)
        result['wall_s'] = time.monotonic() - began
        with (EVIDENCE / ('sequential_loading_guided_' + hand + '.json')).open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        results.append(result)
        print('GUIDED_LOAD_RESULT', hand, result['status'], result.get('error'), flush=True)
    with (EVIDENCE / 'sequential_loading_guided_summary.json').open('x') as stream:
        json.dump(results, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
