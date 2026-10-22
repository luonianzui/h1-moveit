from pathlib import Path


def test_pipeline_output_root_is_current_identity_not_readonly_parent():
    evidence = Path(__file__).resolve().parent
    source = (evidence / 'prepare_candidate.sh').read_text()
    assert 'evidence="${workspace}/runtime_evidence/' + evidence.name + '"' in source
    assert 'source install_lab_scene_48box_sequential_full_20261022_r351/local_setup.bash' in source
