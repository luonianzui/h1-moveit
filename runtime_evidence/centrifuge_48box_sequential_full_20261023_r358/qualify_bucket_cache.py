import ast
import hashlib
import itertools
import json
from pathlib import Path
import time

import numpy as np

from cached_bucket import cached_bucket_corners, corners_from_keys
from bucket_cache_source import ORIGINAL_BUCKET_FUNCTION as ORIGINAL_FUNCTION, CACHED_BUCKET_FUNCTION
from execute_loading import pose_matrix


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r331'


def original_function():
    scope = dict(np=np, pose_matrix=pose_matrix, itertools=itertools)
    exec(ORIGINAL_FUNCTION, scope)
    return scope['bucket_relative_corners']


def rebind(content):
    content = content.replace(str(PARENT), str(ROOT)).replace(PARENT.name, ROOT.name)
    content = content.replace('install_lab_scene_48box_sequential_full_20261020_r331', 'install_lab_scene_48box_sequential_full_20261023_r358')
    content = content.replace('h1_seq_full_r331', 'h1_seq_full_r337').replace('centrifuge_48box_sequential_full_r331', 'centrifuge_48box_sequential_full_r349')
    return content.replace('ROS_DOMAIN_ID=139', 'ROS_DOMAIN_ID=166')


def verify_source():
    parent_source = (PARENT / 'execute_loading.py').read_text()
    candidate_source = (ROOT / 'execute_loading.py').read_text()
    parent_function = next(node for node in ast.parse(parent_source).body
                           if isinstance(node, ast.FunctionDef) and node.name == 'bucket_relative_corners')
    assert ast.get_source_segment(parent_source, parent_function) == ORIGINAL_FUNCTION
    expected = rebind(parent_source)
    assert expected.replace(ORIGINAL_FUNCTION, CACHED_BUCKET_FUNCTION, 1) == candidate_source
    unchanged = {}
    for name in ('sequential_cycle.py', 'loaded_retention.py', 'preload_observation.py',
                 'preload_floor_frame.py', 'sequential_support.py', 'sequential_speed_cycle.py',
                 'speed_retrieval_geometry.py', 'speed_policy.py'):
        expected = rebind((PARENT / name).read_text())
        from pipeline_source import restore_pipeline_source
        assert restore_pipeline_source(name, (ROOT / name).read_text()) == expected, name
        unchanged[name] = hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
    for source in (PARENT / 'inputs').iterdir():
        if not source.is_file() or source.name == 'actual_release_snapshot.json':
            continue
        data = source.read_bytes()
        if source.suffix in ('.py', '.sh', '.json', '.yaml', '.sdf', '.urdf', '.config'):
            data = rebind(data.decode()).encode()
        current = (ROOT / 'inputs' / source.name).read_bytes()
        if source.name == 'lab_dual.launch.py':
            from localization_source import restore_localizer_launch
            current = restore_localizer_launch(current.decode()).encode()
        if source.name == 'gazebo_dual.launch.py':
            current = current.decode().replace("PythonLaunchDescriptionSource(str(Path(__file__).with_name('gz_solver.launch.py')))", "PythonLaunchDescriptionSource(os.path.join(\n            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))").encode()
        assert current == data, source.name
    return unchanged


def main():
    unchanged = verify_source()
    source = ROOT.parent / 'centrifuge_48box_sequential_runtime_perf_20261020_r1/actual_samples.json'
    actual = json.loads(source.read_text())['loaded_retention']['payload']['poses']
    original = original_function()
    for side in ('left', 'right'):
        assert cached_bucket_corners(actual, side).tobytes() == original(actual, side).tobytes()
    timings = {}
    for name, function in (('original', original), ('candidate', cached_bucket_corners),
                           ('candidate_repeat', cached_bucket_corners), ('original_repeat', original)):
        began = time.perf_counter()
        for iteration in range(20000):
            function(actual, 'left')
            function(actual, 'right')
        timings[name] = time.perf_counter()-began
    with (ROOT / 'bucket_cache_qualification.json').open('x') as stream:
        json.dump(dict(
            status='PASS_EXACT_BUCKET_CORNERS_ONLY_NOT_NEW_PHYSICS_OR_WHOLE_TASK_SPEED',
            source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            cache_max_entries=512, defensive_copy=True, signed_zero_key_preserved=True,
            guard_models_and_rates_unchanged=unchanged, actual_payloads=2,
            wall_seconds=timings, component_speedup=min(timings['original'], timings['original_repeat']) /
            max(timings['candidate'], timings['candidate_repeat']),
            cache_info=corners_from_keys.cache_info()._asdict()), stream, indent=2)
    print('BUCKET_CACHE_ORIGINAL_BYTES_PASS', timings, flush=True)


if __name__ == '__main__':
    main()
