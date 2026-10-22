from pathlib import Path

from h1_2_vision.tag_config import load_tag_map
import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
CONFIG = WORKSPACE / 'src/h1_2_vision/config/tag_map.yaml'
TEXTURES = [
    WORKSPACE / 'src/h1_2_description/models/incubator/materials/textures',
    WORKSPACE / 'src/h1_2_description/models/green_cube/materials/textures',
]


def test_complete_unique_versioned_tag_map():
    value = load_tag_map(CONFIG, TEXTURES)
    assert value['schema'] == 'h1_2_apriltag_map/v1'
    assert value['family'] == 'tag36h11'
    assert set(value['objects']) == {'cabinet', 'door', 'cube'}
    assert set(value['tags_by_id']) == {10, 11, 12, 20, 21, 22, 23}
    assert len(value['config_hash']) == 64
    assert value['stability']['consecutive_frames'] >= 5
    assert all(
        value['objects'][name]['object_pose_correction'].shape == (4, 4)
        for name in ('cabinet', 'door', 'cube'))


def test_texture_content_participates_in_hash(tmp_path):
    copied = []
    for source_root in TEXTURES:
        destination = tmp_path / source_root.name / source_root.parent.parent.name
        destination.mkdir(parents=True)
        for source in source_root.glob('*.png'):
            target = destination / source.name
            target.write_bytes(source.read_bytes())
        copied.append(destination)
    first = load_tag_map(CONFIG, copied)['config_hash']
    target = next(copied[0].glob('*.png'))
    target.write_bytes(target.read_bytes() + b'changed')
    second = load_tag_map(CONFIG, copied)['config_hash']
    assert first != second


def test_duplicate_id_is_rejected(tmp_path):
    text = CONFIG.read_text()
    damaged = text.replace('id: 12', 'id: 10')
    path = tmp_path / 'map.yaml'
    path.write_text(damaged)
    with pytest.raises(ValueError, match='duplicate'):
        load_tag_map(path)


def test_wrong_physical_tag_size_is_rejected(tmp_path):
    text = CONFIG.read_text()
    damaged = text.replace('size_m: 0.080', 'size_m: 0.060', 1)
    path = tmp_path / 'map.yaml'
    path.write_text(damaged)
    with pytest.raises(ValueError, match='physical/visual size'):
        load_tag_map(path)
