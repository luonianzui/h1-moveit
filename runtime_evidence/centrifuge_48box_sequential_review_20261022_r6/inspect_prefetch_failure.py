import copy
import gzip
import json
import math
from pathlib import Path

import numpy as np

from sequential_geometry import pose_matrix
from speed_planning import plan
import planner_resources
import sequential_geometry
from alignment_scene import make_alignment_scene
from speed_policy import configure_simulation_profile


REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r348'


def main():
    configure_simulation_profile()
    source = ROOT / 'run/17_retrieval_transport_left_request.json'
    request = json.loads(source.read_text())
    body = pose_matrix(request['poses']['box48_left'])
    selected = []
    samples = {}
    with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
        for line in stream:
            prefix, separator, unused = line.partition(', "payload": ')
            header = json.loads(prefix + '}') if separator else json.loads(line)
            if header['sim'] is None or header['sim'] < request['source_sim'] - .2:
                continue
            kind = header['kind']
            if kind in ('sequential_retrieval_pilot_settle_started', 'sequential_retrieval_pilot_settle_pass',
                        'sequential_actual_planning_started', 'fault', 'machine_guard_fault'):
                selected.append(json.loads(line))
            if kind in ('pose', 'joints', 'joint', 'poses'):
                samples[kind] = json.loads(line)
    runs = []
    for label, factory in (('original_fresh', make_alignment_scene),
                           ('resource_cache', planner_resources.make_resource_scene)):
        sequential_geometry.make_alignment_scene = factory
        try:
            result = plan(copy.deepcopy(request))
            outcome = dict(status='PASS_GEOMETRY', maximum_tilt=result.get('maximum_planned_payload_tilt_deg'))
        except (RuntimeError, ValueError) as error:
            outcome = dict(status='GEOMETRY_REJECTED', error=str(error))
        runs.append(dict(factory=label, **outcome))
    result = dict(status='FAILED_SNAPSHOT_DIAGNOSTIC_NOT_TASK_SUCCESS',
        request_source_sim=request['source_sim'],
        initial_payload_tilt_deg=math.degrees(math.acos(float(np.clip(body[2, 2], -1, 1)))),
        compared=runs, timeline=selected, final_sample_kinds=samples)
    with (REVIEW / 'prefetch_failure_diagnostic.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(dict(initial_tilt=result['initial_payload_tilt_deg'], compared=runs,
        sample_keys=list(samples), timeline=selected)), flush=True)


if __name__ == '__main__':
    main()
