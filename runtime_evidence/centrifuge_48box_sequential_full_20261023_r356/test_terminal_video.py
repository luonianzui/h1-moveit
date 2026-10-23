from pathlib import Path


def test_terminal_inventory_view_is_recorded_before_normal_video_sigint():
    source = Path(__file__).with_name('supervise_completion.py').read_text()
    assert source.index("str(EVIDENCE / 'run/final_overhead.png')") < source.index("str(EVIDENCE / 'finish_video.py')")
    assert source.index("str(EVIDENCE / 'finish_video.py')") < source.index("str(EVIDENCE / 'retire_scene.py')")
    assert 'before_video_sigint=True' in source and 'no_robot_command=True' in source
