import pytest

from camera_guard import should_restore


@pytest.mark.parametrize('last,phase,age,result,expected', [
    (None, 'preflight', 0., False, True), ('load', 'load', 3.9, False, False),
    ('load', 'load', 4., False, True), ('load', 'drop', .1, False, True),
    ('load', 'PASS', 10., True, False), ('hold', 'FAIL', 10., True, False)])
def test_camera_restore_is_periodic_phase_aware_and_does_not_override_terminal_video(last, phase, age, result, expected):
    assert should_restore(last, phase, age, result) is expected
