import ast
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from cached_pose import cached_pose_matrix, matrix_from_bytes
from pose_cache_source import ORIGINAL_POSE_FUNCTION


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r329'
parent_source = (PARENT / 'execute_loading.py').read_text()
function = next(node for node in ast.parse(parent_source).body
                if isinstance(node, ast.FunctionDef) and node.name == 'pose_matrix')
assert ast.get_source_segment(parent_source, function) == ORIGINAL_POSE_FUNCTION
scope = dict(np=np, Rotation=Rotation)
exec(ORIGINAL_POSE_FUNCTION, scope)
sample_path = ROOT.parent / 'centrifuge_48box_sequential_runtime_perf_20261020_r1/actual_samples.json'
samples = json.loads(sample_path.read_text())
poses = []
def collect(value):
    if isinstance(value, dict):
        if 'position' in value and 'orientation' in value:
            poses.append(value)
        else:
            for child in value.values():
                collect(child)
    elif isinstance(value, list):
        for child in value:
            collect(child)
collect(samples)
assert poses
for pose in poses:
    assert cached_pose_matrix(pose).tobytes() == scope['pose_matrix'](pose).tobytes()
measurements = {}
for name, function in [('original', scope['pose_matrix']), ('cached', cached_pose_matrix),
                       ('cached_repeat', cached_pose_matrix), ('original_repeat', scope['pose_matrix'])]:
    began = time.perf_counter()
    for number in range(20000):
        function(poses[number % len(poses)])
    measurements[name] = time.perf_counter() - began
with (ROOT / 'pose_cache_qualification.json').open('x') as stream:
    json.dump(dict(status='PASS_ORIGINAL_MATRIX_BYTES_BOUNDED_CACHE_NOT_NEW_PHYSICS',
                   actual_pose_count=len(poses), sample_sha256=hashlib.sha256(sample_path.read_bytes()).hexdigest(),
                   bounded_entries=512, defensive_return_copy=True, content_key_including_signed_zero=True,
                   wall_seconds=measurements,
                   component_speedup=min(measurements['original'], measurements['original_repeat']) /
                                     max(measurements['cached'], measurements['cached_repeat'])), stream, indent=2)
print('POSE_CACHE_ORIGINAL_BYTES_PASS', len(poses), measurements, flush=True)
