import json
from types import SimpleNamespace

import pytest

from startup_stats_stream import latest_world_stats
from startup_review_freeze import measured_clock
import startup_review_hold


def stats(seconds, nanoseconds=0):
    return dict(simTime=dict(sec=str(seconds), nsec=nanoseconds), iterations='10')


@pytest.mark.parametrize('separator', ['\n', ' ', '\n\n', ''])
def test_concatenated_valid_records_use_latest_without_ignoring_data(separator):
    first, last = stats(3, 1), stats(3, 2)
    assert latest_world_stats(json.dumps(first) + separator + json.dumps(last)) == last


def test_single_pretty_printed_json_and_duplicate_stamp_are_valid():
    value = stats(3)
    assert latest_world_stats('\n' + json.dumps(value, indent=2) + '\n') == value
    assert latest_world_stats(json.dumps(value) * 2) == value


@pytest.mark.parametrize('contents', ['', '{}', '[]', '{"simTime":{}}', 'garbage',
    '{"simTime":{"sec":"NaN"}}', '{"simTime":{"sec":true}}',
    '{"simTime":{"sec":-1}}', '{"simTime":{"nsec":1000000000}}',
    '{"simTime":{"sec":1.5}}'])
def test_invalid_statistics_rejected(contents):
    with pytest.raises(ValueError):
        latest_world_stats(contents)


def test_malformed_trailer_or_time_regression_cannot_be_silently_discarded():
    for contents in (json.dumps(stats(3)) + 'warning', json.dumps(stats(3)) + json.dumps(stats(2))):
        with pytest.raises(ValueError):
            latest_world_stats(contents)


def test_last_record_does_not_bypass_original_fresh_window():
    latest = latest_world_stats(json.dumps(stats(2)) + '\n' + json.dumps(stats(301)))
    with pytest.raises(ValueError):
        measured_clock(latest)


def test_unbounded_stream_rejected():
    with pytest.raises(ValueError):
        latest_world_stats(json.dumps(stats(2)) * 65)


def test_original_readonly_command_timeout_and_checked_exit_unchanged(monkeypatch):
    observed = []
    def run(command, **keywords):
        observed.append((command, keywords))
        return SimpleNamespace(stdout=json.dumps(stats(3)) + '\n' + json.dumps(stats(4)))
    monkeypatch.setattr(startup_review_hold.subprocess, 'run', run)
    assert startup_review_hold.world_stats() == stats(4)
    assert observed == [(['gz', 'topic', '-e', '-t', '/world/h1_world/stats', '--json-output', '-n', '1'],
                        dict(capture_output=True, text=True, timeout=8, check=True))]
