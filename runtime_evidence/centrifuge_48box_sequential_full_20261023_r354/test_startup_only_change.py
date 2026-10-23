from acceleration_source import restore_acceleration, restore_acceleration_bytes
import hashlib
from test_return_height_inverse import undo_return_height
import json
from pathlib import Path
from test_frame_source_inverse import undo_frame_integration


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261019_r309'


def test_all_r309_action_and_grasp_modules_remain_byte_identical():
    identity_file = PARENT / 'identity.json'
    assert hashlib.sha256(identity_file.read_bytes()).hexdigest() == '5fb2f25dd8da483794adf13b170d2b22ba26ec78b68e3aae65e41f1d99acff38'
    manifest = json.loads(identity_file.read_text())['sha256']
    for name in ('sequential_cycle.py', 'sequential_planning.py', 'sequential_geometry.py',
                 'sequential_worker.py', 'sequential_force.py', 'sequential_support.py',
                 'load_bound_reserve.py', 'release_alignment.py', 'pilot_centering.py', 'execute_dual.py'):
        source = restore_acceleration(name, (ROOT / name).read_text())
        if name == 'sequential_planning.py':
            source = undo_return_height(source)
        if name == 'sequential_cycle.py':
            source = undo_frame_integration(source)
        assert hashlib.sha256(source.encode()).hexdigest() == manifest[name]


def test_startup_reader_changes_only_statistics_parsing():
    source = (ROOT / 'startup_review_hold.py').read_text().replace(', current_fresh_window_sim_s=45', '')
    restored = source.replace('from startup_stats_stream import latest_world_stats\n', '')
    restored = restored.replace('return latest_world_stats(response.stdout)', 'return json.loads(response.stdout)')
    assert restored == (PARENT / 'startup_review_hold.py').read_text()
