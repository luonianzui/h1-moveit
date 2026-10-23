from pathlib import Path


def test_launch_and_qualification_use_new_runtime_partition():
    root = Path(__file__).resolve().parent
    for name in ('start_front_recording.sh', 'qualify_current_pipeline.sh'):
        source = (root / name).read_text()
        assert 'ROS_DOMAIN_ID=164' in source
        assert 'GZ_PARTITION=h1_seq_full_r356' in source
        assert 'GZ_PARTITION=h1_seq_full_r337' not in source
    for name in ('build_scene.sh', 'run_dev.sh', 'start_front_recording.sh', 'launch_scene.py'):
        source = (root / name).read_text()
        assert 'install_lab_scene_48box_sequential_full_20261023_r356' in source
        assert 'install_lab_scene_48box_sequential_full_20261021_r339' not in source
