from types import SimpleNamespace

import pytest

from execute_lower_release import LowerReleaseCycle
from execute_loading import LoadingCycle
from release_descent import LOWER_DISTANCE_M, plan_descent
from trace_io import LosslessTrace, verify_trace


@pytest.mark.parametrize('side,distance', [('middle', .045), ('left', 0), ('right', .056), ('left', float('nan'))])
def test_invalid_descent_fails_before_loading_scene(side, distance):
    with pytest.raises(ValueError, match='Invalid'):
        plan_descent({'side':side}, distance)


def test_selected_height_is_less_than_original_gap_without_touching_floor():
    assert LOWER_DISTANCE_M == .045
    assert .068 - LOWER_DISTANCE_M == pytest.approx(.023)


def test_new_descent_keeps_original_guarded_move_and_20ms_floor(monkeypatch):
    calls = []
    instance = object.__new__(LowerReleaseCycle)
    instance.record = lambda *args:None
    monkeypatch.setattr(LoadingCycle, 'move', lambda self, samples, phase, rate:calls.append((samples, phase, rate)))
    samples = [{'torso_joint':0}, {'torso_joint':.01}]
    instance.move(samples, 'left_release_lowering', .8)
    assert calls == [(samples, 'left_release_lowering', .08)]


def test_fast_lossless_compression_keeps_exact_bytes_rows_hash_and_crc(tmp_path):
    import json
    (tmp_path / 'run').mkdir()
    writer = LosslessTrace(tmp_path / 'run')
    for index in range(100):
        writer.write(json.dumps(dict(kind='state', phase='test', sim=index, wall=index, payload='摇篮')) + '\n')
    writer.close()
    assert verify_trace(tmp_path)['status'] == 'PASS'
    assert json.loads((tmp_path / 'run/trace_metadata.json').read_text())['compression'] == 'gzip_level_1'
