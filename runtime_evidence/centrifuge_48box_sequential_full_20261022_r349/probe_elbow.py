import hashlib
import json
from pathlib import Path
import time
import sys

import dual_worker
from elbow_posture import ELBOW_CONTRACT
from minimum_turn import CONTRACT


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_minimum_turn_full_20261015_r285'


def main():
    revision = sys.argv[1] if len(sys.argv) == 2 else 'revision2'
    records = []
    for prefix in ('1_load_pickup', '7_retrieval_pickup'):
        path = PARENT / 'run' / (prefix + '_request.json')
        content = path.read_bytes()
        request = json.loads(content)
        request['minimum_turn_contract'] = CONTRACT
        request['elbow_posture_contract'] = ELBOW_CONTRACT
        started = time.monotonic()
        try:
            result = dual_worker.plan_pickup(request)
        except (ValueError, RuntimeError) as error:
            result = dict(status='FAIL', error=str(error))
        record = dict(mode=request['mode'], request_sha256=hashlib.sha256(content).hexdigest(),
                      wall_s=time.monotonic() - started, result=result, offline_only=True)
        records.append(record)
        with (ROOT / (prefix + '_elbow_' + revision + '.json')).open('x') as stream:
            json.dump(record, stream, indent=2)
        print('ELBOW_PROBE', request['mode'], result['status'], result.get('elbow_posture_selection'),
              result.get('error', '')[-300:], record['wall_s'], flush=True)
    with (ROOT / ('elbow_offline_' + revision + '.json')).open('x') as stream:
        json.dump(dict(status='PASS_OFFLINE_NOT_PHYSICAL' if all(row['result']['status'] == 'PASS' for row in records)
                       else 'FAIL_OFFLINE', records=records), stream, indent=2)


if __name__ == '__main__':
    main()
