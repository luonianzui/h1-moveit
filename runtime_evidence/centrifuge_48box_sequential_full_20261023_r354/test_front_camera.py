from acceleration_source import restore_acceleration, restore_acceleration_bytes
import json
from pathlib import Path
from minimum_turn_source import restore_minimum_turn


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_optional_micro_full_20261015_r283'


def rebound(contents):
    return contents.replace(str(PARENT).encode(), str(ROOT).encode()).replace(
        b'install_lab_scene_48box_optional_micro_full_20261015_r283',
        b'install_lab_scene_48box_sequential_full_20261023_r354').replace(
        b'centrifuge_48box_optional_micro_full_r283', b'centrifuge_48box_sequential_full_r349')


def test_camera_is_in_front_not_robot_physical_camera():
    contract = json.loads((ROOT / 'front_camera_contract.json').read_text())
    assert contract['position'][0] > 1.5
    assert contract['position'][2] > 1.8
    assert abs(contract['target'][1]) < .001
    assert contract['external_observer_only']
    assert contract['gui_pose'] in (ROOT / 'inputs/centrifuge_run_gui.config').read_text()


def test_task_physics_geometry_and_all_holds_unchanged():
    for name in ('execute_cycle.py', 'execute_dual.py', 'dual_worker.py', 'soft_grasp.py',
                 'inputs/robot_dual.urdf', 'inputs/centrifuge_brake.sdf', 'inputs/48box_friction.sdf',
                 'inputs/lab_fast_world.sdf', 'inputs/controllers_servo.yaml', 'inputs/lab_dual.launch.py'):
        content = restore_acceleration_bytes(name, (ROOT / name).read_bytes())
        if name == 'inputs/lab_dual.launch.py':
            from localization_source import restore_localizer_launch
            content = restore_localizer_launch(content.decode()).encode()
        if name == 'dual_worker.py':
            content = restore_minimum_turn(name, content.decode()).encode()
        assert content == rebound((PARENT / name).read_bytes()), name
    assert 'LD_PRELOAD' not in (ROOT / 'inputs/gazebo_dual.launch.py').read_text()


def test_initial_recording_request_and_guard_share_front_pose():
    contract = json.loads((ROOT / 'front_camera_contract.json').read_text())
    source = (ROOT / 'start_full_window_video.py').read_text()
    assert f"position = {contract['position']!r}" in source
    assert f"target = {contract['target']!r}" in source
    assert 'camera_request.json' in (ROOT / 'camera_guard.py').read_text()
