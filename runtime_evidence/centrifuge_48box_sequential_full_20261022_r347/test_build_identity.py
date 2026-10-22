from pathlib import Path


EVIDENCE = Path(__file__).resolve().parent
PREFIX = 'install_lab_scene_48box_sequential_full_20261022_r347'


def test_build_uses_exclusive_identity_prefix():
    source = (EVIDENCE / 'build_scene.sh').read_text()
    assert '--install-base "${workspace}/' + PREFIX + '"' in source
    assert 'test ! -e "${workspace}/' + PREFIX + '"' in source
    assert '--build-base "${evidence}/build/unique_prefix"' in source
    assert 'install_lab_scene_48box_full_cycle_20261010_r193' not in source


def test_launch_uses_same_identity_prefix():
    source = (EVIDENCE / 'launch_scene.py').read_text()
    assert 'source ' + PREFIX + '/local_setup.bash' in source
    assert 'install_lab_scene_48box_full_cycle_20261010_r193' not in source

