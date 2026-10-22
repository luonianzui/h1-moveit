import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as element_tree

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from scene_contract import classify_regression, SCOPE, verify_declared_nodes, verify_fixed_pelvis
from scene_contract import verify_scope, verify_ctest_result
from scene_contract import verify_hand_imports


EVIDENCE = Path(__file__).resolve().parent
WORKSPACE = EVIDENCE.parents[1]


def digest(path):
    hasher = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def record(name, payload):
    with (EVIDENCE / name).open('x') as stream:
        json.dump(payload, stream, indent=2)
        stream.write('\n')


def protect(phase):
    previous_argv = sys.argv[:]
    try:
        for name in ('audit_protected.py', 'audit_source_1009.py'):
            path = EVIDENCE / name
            sys.argv = [str(path), phase]
            exec(compile(path.read_text(), str(path), 'exec'),
                 {'__name__':'__main__', '__file__':str(path)})
    finally:
        sys.argv = previous_argv
    failures = []
    counts = {}
    for name in ('centrifuge_48box_skip_empty_return_20261010_r216',
                 'centrifuge_48box_scene_resume_20261012_r217',
                 'centrifuge_48box_fast_complete_20261012_r218',
                 'centrifuge_48box_fast_complete_20261012_r219',
                 'centrifuge_48box_empty_quintic_20261012_r220',
                 'centrifuge_48box_empty_shortcut_20261012_r221',
                 'centrifuge_48box_loaded_grid_20261012_r222',
                 'centrifuge_48box_loaded_rate_20261012_r223',
                 'centrifuge_48box_retrieval_context_20261012_r224',
                 'rl_source_recovery_20261010_r1', 'rl_official_download_20261010_r1',
                 'rl_official_redownload_20261012_r1'):
        folder = WORKSPACE / 'runtime_evidence' / name
        lines = (folder / 'SHA256SUMS').read_text().splitlines()
        counts[name] = len(lines)
        for line in lines:
            expected, relative = line.split(maxsplit=1)
            path = folder / relative.lstrip('*')
            if not path.is_file() or digest(path) != expected:
                failures.append(str(path))
    record('later_protected_' + phase + '.json', {'status':'FAIL' if failures else 'PASS',
                                               'counts':counts, 'failures':failures})
    assert not failures


def audit():
    verify_scope(SCOPE, False, False)
    verify_fixed_pelvis((EVIDENCE / 'inputs/robot_dual.urdf').read_text())
    launch_paths = [EVIDENCE / 'inputs' / name for name in ('lab_dual.launch.py', 'gazebo_dual.launch.py')]
    nodes = []
    for path in launch_paths:
        nodes.extend(verify_declared_nodes(path.read_text()))
    resource_paths = set(launch_paths)
    resources = []
    for row in nodes:
        resource_paths.add(Path(get_package_share_directory(row['package'])) / 'package.xml')
        executable = Path(get_package_prefix(row['package'])) / 'lib' / row['package'] / row['executable']
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise ValueError('Missing executable: ' + str(executable))
        resource_paths.add(executable)
    gz_launch = Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py'
    resource_paths.add(gz_launch)
    for name in ('gz', 'ros2', 'ffmpeg', 'ffprobe', 'gst-launch-1.0', 'xwininfo'):
        executable = shutil.which(name)
        if not executable:
            raise ValueError('Missing command: ' + name)
        resource_paths.add(Path(executable))
    robot = element_tree.parse(EVIDENCE / 'inputs/robot_dual.urdf').getroot()
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename')
        if not filename.startswith('package://'):
            raise ValueError('Unreviewed mesh URI')
        package, relative = filename.removeprefix('package://').split('/', 1)
        resource_paths.add(Path(get_package_share_directory(package)) / relative)
    for name in ('libCentrifugeButton.so', 'libCentrifugeInterlock.so'):
        resource_paths.add(Path(get_package_prefix('h1_2_moveit_config')) / 'lib' / name)
    navigation = WORKSPACE / 'src/h1_2_navigation/h1_2_navigation'
    for name in ('__init__.py', 'mid360_square_localizer.py', 'square_localization.py'):
        source = navigation / name
        contents = source.read_text()
        if any(marker in contents for marker in ('torch.load(', 'IsaacLab-3.0-beta2', 'v12_policy', 'unitree_h12_policy')):
            raise ValueError('Localizer imports an unreviewed policy dependency')
        resource_paths.add(source)
    resource_paths.add(Path(get_package_share_directory('h1_2_navigation')) / 'config/mid360_localization.yaml')
    resource_paths.update((EVIDENCE / 'inputs').iterdir())
    initialization = json.loads((EVIDENCE / 'initialization_compatibility.json').read_text())
    assert initialization['status'] == 'PASS_ACTUAL_STATIONARY_LEG_COMPATIBILITY_NOT_LOADED_TASK'
    for relative, expected in initialization['source_reports'].items():
        source = Path(initialization['source']) / relative
        assert digest(source) == expected
        resource_paths.add(source)
    for relative in (
        'centrifuge_48box_coupled_path_20261013_r240/coupled_path.py',
        'centrifuge_48box_empty_sampling_20261013_r237/button_sampling.py',
        'centrifuge_48box_alignment_cap_20261013_r236/command_sampling.py',
        'centrifuge_48box_alignment_cap_20261013_r236/identity.json',
        'centrifuge_48box_control_resample_20261012_r232/parallel_station_ik_probe.json'):
        resource_paths.add(WORKSPACE / 'runtime_evidence' / relative)
    hand_paths = [path for path in EVIDENCE.glob('*.py') if not path.name.startswith('test_')]
    hand_paths += [WORKSPACE / 'runtime_evidence/centrifuge_48box_loading_20260923_r121' / name
                   for name in ('plan_candidate.py', 'probe_loading_collision.py')]
    hand_paths += [WORKSPACE / 'src/h1_2_moveit_config/scripts/centrifuge_press_plan.py']
    for path in hand_paths:
        verify_hand_imports(path.read_text())
        resource_paths.add(path)
    for relative in ('src/h1_2_moveit_config/config/h1_2.srdf',
                     'src/h1_2_moveit_config/config/initial_positions.yaml',
                     'runtime_evidence/centrifuge_robot_press_20260922_r119/robot.urdf',
                     'runtime_evidence/centrifuge_robot_press_20260922_r119/passive_leg_baseline.json',
                     'runtime_evidence/centrifuge_robot_press_20260922_r119/press_plan.json'):
        resource_paths.add(WORKSPACE / relative)
    for model in ('centrifuge', 'lab_48box', 'table'):
        resource_paths.update(path for path in (WORKSPACE / 'src/h1_2_description/models' / model).rglob('*') if path.is_file())
    import importlib.util
    for module in ('moveit.core', 'rclpy', 'numpy', 'scipy', 'ros_gz_interfaces.msg',
                   'trajectory_msgs.msg', 'control_msgs.action', 'PIL.Image', 'ctypes'):
        spec = importlib.util.find_spec(module)
        if spec is None or not spec.origin:
            raise ValueError('Missing hand execution module: ' + module)
        resource_paths.add(Path(spec.origin))
    resource_paths.add(Path('/usr/lib/x86_64-linux-gnu/libX11.so.6').resolve())
    for path in sorted(resource_paths):
        if not path.is_file():
            raise ValueError('Missing selected scene dependency: ' + str(path))
        resources.append({'path':str(path), 'sha256':digest(path), 'bytes':path.stat().st_size})
    regression = classify_regression((EVIDENCE / 'navigation_tests.xml').read_text())
    assert json.loads((EVIDENCE / 'codec_audit.json').read_text())['status'] == 'PASS'
    assert json.loads((EVIDENCE / 'protected_before.json').read_text())['status'] == 'PASS'
    assert (EVIDENCE / 'conda_six_before.txt').read_bytes() == (EVIDENCE / 'conda_six_after.txt').read_bytes()
    for name in ('tests_speed_final.log', 'package_tests.log', 'loading_regression_tests_final.log'):
        contents = (EVIDENCE / name).read_text()
        assert ' passed' in contents and ' failed' not in contents and 'ERROR' not in contents
    package_tests = verify_ctest_result((EVIDENCE / 'ctest.log').read_text())
    record('scene_dependency_audit_revision2.json', {
        'status':'PASS', 'scope':SCOPE, 'walking_enabled':False, 'reconstructed_checkpoint':False,
        'user_authorization':'继续双盒装篮、关盖、旋转、开盖、取回全流程，运行尽量快；仅Gazebo',
        'fixed_pelvis':True, 'declared_nodes':nodes, 'selected_resources':resources,
        'launch_include_chain':[str(path) for path in launch_paths] + [str(gz_launch)],
        'localizer_python_dependencies':['mid360_square_localizer', 'square_localization'],
        'regression':regression, 'private_training_provenance_required_for_this_scope':False,
        'package_tests_passed':package_tests,
        'private_training_provenance_required_for_original_training_scope':True,
        'old_runtime_contract_replaced':False, 'full_cycle_execution_certified':False,
        'full_hand_dependency_reviewed':True,
        'static_leg_controller_not_RL_walking':True,
        'hand_python_dependency_chain':[str(path) for path in hand_paths],
        'hand_execution_requires_fresh_health_video_and_one_time_claim':True,
    })
    print('SCENE_DEPENDENCIES_PASS', len(resources), regression)


if __name__ == '__main__':
    if sys.argv[1] in ('before', 'after'):
        protect(sys.argv[1])
    elif sys.argv[1] == 'audit':
        audit()
    else:
        raise ValueError(sys.argv[1])
