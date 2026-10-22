from pathlib import Path

from video_capture import capture_command


def test_avc_access_units_directly_mux_without_missing_parser(tmp_path):
    command = capture_command('0x123', tmp_path / 'film.mp4')
    assert 'video/x-h264,stream-format=avc,alignment=au' in command
    assert 'h264parse' not in command
    assert command.index('x264enc') < command.index('video/x-h264,stream-format=avc,alignment=au') < command.index('mp4mux')


def test_seal_requires_real_72_frame_codec_preflight_before_gazebo():
    source = Path(__file__).with_name('seal_identity.py').read_text()
    assert "evidence / 'codec_audit.json'" in source
    source = Path(__file__).with_name('codec_preflight.py').read_text()
    assert 'frames == 72' in source and 'decode.returncode' in source
    assert 'no_desktop_capture=True' in source
