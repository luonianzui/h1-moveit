import hashlib
import importlib.util
import json
from pathlib import Path
import statistics
import time

from cached_bucket import cached_bucket_corners, bucket_from_keys
from cached_pose import cached_pose_matrix


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r337'


def load_parent(name):
    path = PARENT / (name + '.py')
    identity = json.loads((PARENT / 'identity.json').read_text())
    assert hashlib.sha256(path.read_bytes()).hexdigest() == identity['sha256'][path.name]
    spec = importlib.util.spec_from_file_location('parent_' + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    old_pose, old_bucket = load_parent('cached_pose'), load_parent('cached_bucket')
    fixture = ROOT.parent / 'centrifuge_48box_sequential_runtime_perf_20261020_r1/actual_samples.json'
    poses = json.loads(fixture.read_text())['loaded_retention']['payload']['poses']
    values = [value for value in poses.values() if isinstance(value, dict) and 'position' in value and 'orientation' in value]
    assert values
    for value in values:
        assert old_pose.cached_pose_matrix(value).tobytes() == cached_pose_matrix(value).tobytes()
    for side in ('left', 'right'):
        assert old_bucket.cached_bucket_corners(poses, side).tobytes() == cached_bucket_corners(poses, side).tobytes()
    records = {}
    for category in ('pose', 'corners'):
        rows = dict(parent=[], candidate=[])
        for repetition in range(6):
            order = ('parent', 'candidate') if repetition % 2 == 0 else ('candidate', 'parent')
            for version in order:
                function = ((old_pose.cached_pose_matrix if version == 'parent' else cached_pose_matrix)
                            if category == 'pose' else
                            (old_bucket.cached_bucket_corners if version == 'parent' else cached_bucket_corners))
                began = time.perf_counter()
                if category == 'pose':
                    for index in range(100000):
                        function(values[index % len(values)])
                else:
                    for index in range(50000):
                        function(poses, 'left')
                        function(poses, 'right')
                rows[version].append(time.perf_counter() - began)
        rows['median_component_speedup'] = statistics.median(rows['parent']) / statistics.median(rows['candidate'])
        records[category] = rows
    result = dict(status='PASS_EXACT_POSE_GEOMETRY_COMPONENT_ONLY_NOT_REALTIME_PROOF',
                  actual_fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),
                  source_pose_sha256=hashlib.sha256((ROOT/'cached_pose.py').read_bytes()).hexdigest(),
                  source_bucket_sha256=hashlib.sha256((ROOT/'cached_bucket.py').read_bytes()).hexdigest(),
                  actual_fixture_poses=len(values), measurements=records,
                  minimum_component_speedup=min(row['median_component_speedup'] for row in records.values()),
                  bucket_transform_cache=bucket_from_keys.cache_info()._asdict(),
                  dynamics_contact_sampling_guards_and_all_stamps_unchanged=True)
    with (ROOT/'geometry_fastpath_qualification.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result), flush=True)
    assert result['minimum_component_speedup'] > 1.


if __name__ == '__main__':
    main()
