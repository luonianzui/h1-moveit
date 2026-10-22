import pytest

from controller_health import active_controllers, command_health, EXPECTED_CONTROLLERS


def output(statuses):
    return '\n'.join(name + ' controller/type ' + status for name, status in zip(EXPECTED_CONTROLLERS, statuses))


def test_all_six_exact_active_pass():
    assert active_controllers(output(['active'] * 6))['status'] == 'PASS'


def test_service_error_never_passes_even_with_positive_looking_stdout():
    assert command_health(1, output(['active'] * 6))['status'] == 'FAIL'
    assert command_health(0, output(['active'] * 6))['status'] == 'PASS'


@pytest.mark.parametrize('status', ['inactive', 'unconfigured', 'finalized', 'activating', 'unknown'])
def test_inactive_substring_and_other_states_never_pass(status):
    assert active_controllers(output(['active'] * 5 + [status]))['status'] == 'FAIL'


def test_missing_or_duplicate_controller_fails_closed():
    assert active_controllers(output(['active'] * 5))['status'] == 'FAIL'
    duplicate = output(['active'] * 6) + '\n' + EXPECTED_CONTROLLERS[0] + ' controller/type active'
    assert active_controllers(duplicate)['status'] == 'FAIL'


def test_ansi_and_service_wait_notice_do_not_change_exact_state():
    text = '[INFO] waiting for list_controllers service\n\x1b[32m' + output(['active'] * 6) + '\x1b[0m'
    assert active_controllers(text)['status'] == 'PASS'

