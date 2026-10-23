import copy
import json
import time

import numpy as np

from sequential_geometry import EVIDENCE, SinglePlanner
from probe_sequential_loading import pose_dictionary


def main():
    records = []
    for hand in ('left', 'right'):
        reverse = json.loads((EVIDENCE / ('sequential_loading_' + hand + '.json')).read_text())
        request = copy.deepcopy(reverse['request'])
        stages = reverse['stages']
        seed = stages[-1]['samples'][-1]
        request['positions'].update(seed)
        body = np.array(stages[-1]['last_body'])
        request['poses']['box48_' + hand] = pose_dictionary(body)
        planner = SinglePlanner(request, reverse['reference'])
        planner.attach(body, seed)
        result = dict(hand=hand, status='REJECTED_OFFLINE_FORWARD_SINGLE_LOAD', stages=[])
        began = time.monotonic()
        try:
            for index, old in enumerate(reversed(stages)):
                target = np.array(old['first_body'])
                phase = 'forward_' + str(index)
                stage = planner.stage(body, target, seed, phase, support_start='table' if index == 0 else None)
                body, seed = target, stage['samples'][-1]
                result['stages'].append(stage)
                print('FORWARD_SINGLE', hand, phase, planner.checked, planner.elbow_metrics(seed)['from_down_deg'], flush=True)
            result.update(status='PASS_OFFLINE_FORWARD_SINGLE_LOAD', elbow=planner.elbow_metrics(seed), checked=planner.checked)
        except (ValueError, RuntimeError, AssertionError) as error:
            result['error'] = str(error)
        result['wall_s'] = time.monotonic() - began
        records.append(result)
        with (EVIDENCE / ('sequential_loading_forward_' + hand + '.json')).open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print('FORWARD_SINGLE_RESULT', hand, result['status'], result.get('error'), flush=True)
    with (EVIDENCE / 'sequential_loading_forward_summary.json').open('x') as stream:
        json.dump(records, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
