"""Fail-closed regressions for the hardened right-handle offline tools."""

from pathlib import Path
import subprocess


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
IK_SOURCE = (PACKAGE / 'src/right_handle_ik_branch_search.cpp').read_text(
    encoding='utf-8')
SPLICE_SOURCE = (PACKAGE / 'src/right_handle_splice_search.cpp').read_text(
    encoding='utf-8')
AUDITOR_SOURCE = (PACKAGE / 'src/right_handle_path_exact_auditor.cpp').read_text(
    encoding='utf-8')
MIDDLE_SOURCE = (PACKAGE / 'src/candidate3_middle_exact_auditor.cpp').read_text(
    encoding='utf-8')

Q0_PASS_ARM = (
    '-0.14590890409780025,0.15022053092544321,0.35893300624450669,'
    '0.581998803037025,0.026528288913425842,-0.3486319286613882,'
    '-1.1506581032340752')
Q0_FAIL_ARM = (
    '-0.14233229392322147,0.17448695125740113,0.3109309801652427,'
    '0.5894133761521352,-0.01110471422383302,-0.34996782670167187,'
    '-1.1123836344854279')


def _command(torso_deg: str, arm: str) -> list[str]:
    return [
        str(WORKSPACE / 'build/h1_2_moveit_demo/right_handle_ik_branch_search'),
        '--scene', str(WORKSPACE / (
            'runtime_evidence/right_natural_828_table_xplus100_'
            'incubator_original_20260828/middle_v1/scene.cdr')),
        '--path', str(WORKSPACE / (
            'runtime_assets/right_handle_8263_natural/'
            'right_handle_q0_q90_fixed_torso_natural_v2_path.json')),
        '--urdf', str(WORKSPACE / (
            'runtime_evidence/826_apriltag_plan_only_20260826/'
            'stage_h_nominal_pass/clearance_audit_robot.urdf')),
        '--srdf', str(WORKSPACE / (
            'runtime_evidence/826_apriltag_plan_only_20260826/'
            'stage_h_nominal_pass/clearance_audit_robot.srdf')),
        '--park-left-roll', '0.316', '--handle-angle-deg', '0',
        '--torso-deg', torso_deg, '--samples', '1',
        '--maximum-results', '1', '--seed', '826745',
        '--report-best-failed', '--geometry-id', 'A0',
        '--phase-deg', '-180', '--z-mm', '95',
        '--palm-down-tilt-deg', '7', '--side-tilt-deg', '0',
        '--outward-mm', '16', '--local-x-mm', '0',
        '--q1-mm', '15.3', '--q2-mm', '15.3', '--test-arm', arm,
    ]


def test_hardening_source_contracts_are_fail_closed():
    """Geometry must exist, arbitrary door angles are bounded, and moves are safe."""
    for source in (IK_SOURCE, SPLICE_SOURCE):
        assert 'verify_collision_geometry_fail_closed(scene)' in source
    assert 'verify_collision_geometry_fail_closed(physical_scene)' in AUDITOR_SOURCE
    assert 'handle_angle > kPi / 2.0 + 1e-12' in IK_SOURCE
    assert 'handle_angle > kPi / 2.0 + 1e-12' in SPLICE_SOURCE
    assert 'ARGUMENT_VECTOR:--test-arm' in IK_SOURCE
    move_function = MIDDLE_SOURCE.split('void set_single_shape_pose(', 1)[1]
    move_function = move_function.split('DynamicGeometry prepare_dynamic_geometry(', 1)[0]
    assert 'const auto object = scene.getWorld()->getObject(id);' in move_function
    assert '}\n  if (!scene.getWorldNonConst()->moveShapeInObject' in move_function


def test_q0_a0_exact_torso_45_is_positive_boundary():
    completed = subprocess.run(
        _command('45', Q0_PASS_ARM), capture_output=True, text=True,
        check=False, timeout=30)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert 'IK_BRANCH_SEARCH status=PASS mode=TEST_ARM accepted=1' in completed.stdout
    assert 'margin_deg=6.5241599089970315' in completed.stdout
    assert 'shoulder_torso_signed_mm=1.4227048649033665' in completed.stdout


def test_q0_a0_exact_torso_44_5_is_negative_boundary():
    completed = subprocess.run(
        _command('44.5', Q0_FAIL_ARM), capture_output=True, text=True,
        check=False, timeout=30)
    assert completed.returncode == 2, completed.stdout + completed.stderr
    assert 'IK_BRANCH_SEARCH status=NO_VALID_BRANCH mode=TEST_ARM' in completed.stdout
    assert 'pair=right_shoulder_yaw_link:torso_link count=1' in completed.stdout
    assert 'shoulder_torso_signed_mm=-17.438728315788911' in completed.stdout
