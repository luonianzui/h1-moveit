import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent

def test_exact_solver_qualification():
    document = json.loads((ROOT / 'dart_cache_dependencies.json').read_text())
    assert document['qualification']['matrix_cases'] == 105
    assert document['qualification']['real_contact_steps'] > 100
    assert document['physical_timestep_contacts_friction_mass_solver_bounds_unchanged']

def test_only_owned_gazebo_process_environment():
    text = (ROOT / 'inputs/gz_solver.launch.py').read_text()
    assert 'additional_env=env' in text
    assert 'env.update(LD_PRELOAD=' in text
    assert "environ.get('LD_PRELOAD')" in text
    assert "'h1_seq_full_r349'" in text and "'157'" in text
    assert "SetEnvironmentVariable(" not in text
    assert "os.environ[" not in text

def test_solver_preserves_original_policy():
    text = (ROOT / 'inputs/CachedSolver.cc').read_text()
    for token in ('mABackup = mA;', 'mXBackup = mX;', 'mFIndexBackup = mFIndex;',
                  'mSecondaryBoxedLcpSolver->solve(', 'success && mX.hasNaN()',
                  'mX = mXBackup;', 'mX.setZero();', '1.0 / mTimeStep'):
        assert token in text
    assert 'setTimeStep' not in text and 'setFriction' not in text

def test_runtime_binding_before_robot_initialization():
    text = (ROOT / 'start_front_recording.sh').read_text()
    assert text.index('verify_dart_runtime.py') < text.index('initialize_stationary_legs.py')

def test_gazebo_vendor_launch_only_adds_scoped_environment():
    vendor = Path('/opt/ros/jazzy/share/ros_gz_sim/launch/gz_sim.launch.py').read_text()
    candidate = (ROOT / 'inputs/gz_solver.launch.py').read_text()
    start = candidate.index('    import hashlib\n')
    finish = candidate.index("    gz_args = LaunchConfiguration('gz_args').perform(context)")
    assert candidate[:start] + candidate[finish:] == vendor

def test_all_physical_assets_identical_to_r342_after_identity_rebind():
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r342'
    checked = []
    for source in (parent / 'inputs').iterdir():
        if source.suffix not in ('.sdf', '.urdf', '.stl', '.STL', '.yaml'):
            continue
        expected = source.read_bytes()
        if source.suffix not in ('.stl', '.STL'):
            expected = expected.decode().replace(parent.name, ROOT.name).replace(
                'install_lab_scene_48box_sequential_full_20261021_r342',
                'install_lab_scene_48box_sequential_full_20261022_r349').encode()
        assert (ROOT / 'inputs' / source.name).read_bytes() == expected, source.name
        checked.append(source.name)
    assert len(checked) == 8
    assert {'robot_dual.urdf', 'centrifuge_brake.sdf', '48box_friction.sdf', 'lab_fast_world.sdf'} <= set(checked)
