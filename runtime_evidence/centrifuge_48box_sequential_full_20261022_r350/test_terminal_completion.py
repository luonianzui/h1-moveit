import pytest

from terminal_completion import finish_before_retirement


@pytest.mark.parametrize('failed_step', [None, 'inspection', 'video', 'retirement'])
def test_failures_are_preserved_and_video_always_precedes_retirement(failed_step):
    calls = []

    def operation(name):
        calls.append(name)
        if name == failed_step:
            raise RuntimeError(name + ' failure')

    result = finish_before_retirement(lambda: operation('inspection'), lambda: operation('video'),
                                      lambda: operation('retirement'), lambda: False)
    assert calls == ['inspection', 'video', 'retirement']
    assert result['status'] == ('PASS' if failed_step is None else 'FAIL')
    if failed_step:
        assert result['steps'][failed_step]['status'] == 'FAIL'


def test_live_recorder_blocks_scene_retirement_without_forced_kill():
    calls = []
    result = finish_before_retirement(lambda: calls.append('inspection'), lambda: calls.append('video'),
                                      lambda: calls.append('retirement'), lambda: True)
    assert calls == ['inspection', 'video']
    assert result['status'] == 'FAIL'
    assert result['steps']['retirement']['status'] == 'HELD'


def test_unknown_recorder_identity_fails_closed_and_keeps_scene():
    calls = []

    def unavailable():
        raise RuntimeError('Identity inspection failed')

    result = finish_before_retirement(lambda: calls.append('inspection'), lambda: calls.append('video'),
                                      lambda: calls.append('retirement'), unavailable)
    assert calls == ['inspection', 'video']
    assert result['status'] == 'FAIL'
    assert result['steps']['recorder_exit']['alive_unknown']
