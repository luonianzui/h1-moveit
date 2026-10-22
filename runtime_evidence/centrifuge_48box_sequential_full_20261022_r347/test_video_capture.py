from pathlib import Path

import pytest

from video_capture import capture_command, recorder_alive


def test_capture_is_only_explicit_window_and_fragmented_eos_not_full_desktop(tmp_path):
    command = capture_command('0x123', tmp_path / 'movie.mp4', frames=72)
    assert command[:4] == ['gst-launch-1.0', '-e', 'ximagesrc', 'display-name=:0']
    assert 'xid=291' in command and 'num-buffers=72' in command
    assert 'fragment-duration=2000' in command and 'key-int-max=24' in command
    assert 'location=' + str(tmp_path / 'movie.mp4') in command
    assert 'x11grab' not in command and 'startx=0' not in command


def test_video_output_is_never_overwritten(tmp_path):
    path = tmp_path / 'movie.mp4'
    path.touch()
    with pytest.raises(FileExistsError):
        capture_command('0x123', path)


@pytest.mark.parametrize('window', ['0x0', '-1', 'invalid'])
def test_no_generic_root_window_capture(window, tmp_path):
    with pytest.raises(ValueError):
        capture_command(window, tmp_path / 'movie.mp4')


def test_missing_recorder_never_looks_healthy():
    assert not recorder_alive(dict(pid=99999999, created=0, command=[]))
