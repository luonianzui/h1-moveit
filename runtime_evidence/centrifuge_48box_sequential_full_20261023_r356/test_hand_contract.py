import json
from pathlib import Path

import pytest

from scene_contract import SCOPE, read_contract, verify_hand_imports, verify_scope


def test_full_hand_scope_is_distinct_from_startup():
    assert SCOPE == 'two_box_load_close_spin_open_retrieve'
    with pytest.raises(ValueError):
        verify_scope('fixed_pelvis_scene_startup_only', False, False)


@pytest.mark.parametrize('contents', ('import unitree_h12_runtime', 'from v12_policy import Policy',
                                    'from nav2_simple_commander import BasicNavigator',
                                    'import locomotion_bridge'))
def test_policy_and_navigation_imports_rejected(contents):
    with pytest.raises(ValueError):
        verify_hand_imports(contents)


def test_hand_and_static_leg_imports_pass():
    verify_hand_imports('from execute_loading import LoadingCycle\nfrom leg_posture import LEG_JOINTS\nimport rclpy')


def test_startup_audit_does_not_authorize_hand_execution(tmp_path):
    (tmp_path / 'scene_dependency_audit_revision2.json').write_text(json.dumps(
        dict(status='PASS', scope=SCOPE, walking_enabled=False, reconstructed_checkpoint=False)))
    with pytest.raises(ValueError):
        read_contract(tmp_path)


def test_unrelated_navigation_failures_remain_sealed_raw():
    from identity_members import VALIDATION_FILES
    assert {'navigation_tests.log', 'navigation_tests.xml'} <= VALIDATION_FILES


def test_runtime_and_sealer_require_full_contract():
    root = Path(__file__).resolve().parent
    for name in ('launch_scene.py', 'execute_cycle.py', 'seal_identity.py'):
        assert 'read_contract(EVIDENCE)' in (root / name).read_text() or 'read_contract(evidence)' in (root / name).read_text()


def test_gui_render_scaling_does_not_change_physics():
    root = Path(__file__).resolve().parent
    assert "QT_SCALE_FACTOR='1'" in (root / 'launch_scene.py').read_text()
    assert '<max_step_size>0.0005</max_step_size>' in (root / 'inputs/lab_fast_world.sdf').read_text()
