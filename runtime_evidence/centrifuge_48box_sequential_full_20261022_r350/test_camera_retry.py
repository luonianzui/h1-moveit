from types import SimpleNamespace
import subprocess

import pytest

from camera_guard import restore_with_retries


def responses(sequence):
    calls = []

    def run(command, **options):
        calls.append((command, options))
        item = sequence[len(calls) - 1]
        if isinstance(item, Exception):
            raise item
        return item

    return run, calls


def response(accepted):
    return SimpleNamespace(returncode=0, stdout='data: true\n' if accepted else '',
                           stderr='' if accepted else 'Service call timed out\n')


def test_successful_first_ack_never_generates_extra_service_requests():
    run, calls = responses([response(True)])
    accepted, rows, error = restore_with_retries('camera', 'phase', run, lambda duration: None)
    assert accepted and error is None and len(rows) == len(calls) == 1


def test_r243_single_timeout_can_recover_without_stopping_the_encoder():
    run, calls = responses([response(False), response(True)])
    delays = []
    accepted, rows, error = restore_with_retries('camera', 'phase', run, delays.append)
    assert accepted and error is None and len(calls) == 2
    assert [row['accepted'] for row in rows] == [False, True]
    assert delays == [.2]
    assert all(call[1]['timeout'] == 5 for call in calls)
    assert all('/gui/move_to/pose' in call[0] and '3000' in call[0] for call in calls)


def test_two_timeouts_can_recover_on_the_last_bounded_attempt():
    run, calls = responses([response(False), response(False), response(True)])
    accepted, rows, error = restore_with_retries('camera', 'phase', run, lambda duration: None)
    assert accepted and error is None and len(rows) == len(calls) == 3


def test_persistent_ack_failure_still_stops_after_exactly_three_attempts():
    run, calls = responses([response(False)] * 3)
    accepted, rows, error = restore_with_retries('camera', 'phase', run, lambda duration: None)
    assert not accepted and error is not None and len(rows) == len(calls) == 3


@pytest.mark.parametrize('exception', [subprocess.TimeoutExpired('gz', 5), OSError('unavailable')])
def test_exception_is_recorded_and_cannot_turn_into_an_ack(exception):
    run, calls = responses([exception] * 3)
    accepted, rows, error = restore_with_retries('camera', 'phase', run, lambda duration: None)
    assert not accepted and error is not None and len(calls) == 3
    assert all(not row['accepted'] and row['returncode'] is None for row in rows)
