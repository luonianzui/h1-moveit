import inspect
import textwrap
from pathlib import Path

import pytest

from inputs.exact_rate_localizer import ExactRateLocalizer, localization_due
from inputs.localizer_runtime import original
from localization_freshness import localization_freshness

ROOT = Path(__file__).resolve().parent
OLD = "        period = 1.0 / self.localization_rate\n        if (\n                self.last_localization_stamp is not None\n                and stamp_seconds >= self.last_localization_stamp\n                and stamp_seconds - self.last_localization_stamp < period):\n            return\n        self.last_localization_stamp = stamp_seconds"
NEW = "        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec\n        last_ns = getattr(self, 'last_localization_stamp_ns', None) if self.last_localization_stamp is not None else None\n        if not localization_due(stamp_ns, last_ns, self.localization_rate):\n            return\n        self.last_localization_stamp_ns = stamp_ns\n        self.last_localization_stamp = stamp_seconds"


def test_callback_changes_only_rate_gate():
    previous = inspect.getsource(original.Mid360SquareLocalizer._point_cloud_callback).strip()
    current = inspect.getsource(ExactRateLocalizer._point_cloud_callback).strip()
    assert previous.count(OLD) == 1
    assert current == previous.replace(OLD, NEW)


@pytest.mark.parametrize('start', [0, 100000000, 113900000000, 86400000000000])
@pytest.mark.parametrize('offset,expected', [(0, False), (199999999, False), (200000000, True), (200000001, True)])
def test_exact_period_boundary(start, offset, expected):
    assert localization_due(start + offset, start, 5.) is expected


def test_observed_float_drop_reproduced_and_fixed():
    assert 114.1 - 113.9 < .2
    assert localization_due(114100000000, 113900000000, 5.)


def test_reset_and_first_observation():
    assert localization_due(0, 1000000000, 5.)
    assert localization_due(0, None, 5.)


@pytest.mark.parametrize('rate', [0., -1., float('nan'), float('inf')])
def test_invalid_rate_rejected(rate):
    with pytest.raises(ValueError):
        localization_due(1, 0, rate)


def test_stale_data_still_rejected():
    assert not localization_freshness(114.9005, 113.9, 5., 1., 1., True)['valid']
    assert not localization_freshness(115.1005, 114.1, 5., 1., 1., True)['valid']


def test_both_inputs_keep_formal_stamps_and_no_gate_override():
    source = inspect.getsource(ExactRateLocalizer)
    assert 'message.header.stamp =' not in source
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r344'
    assert (ROOT / 'localization_freshness.py').read_bytes() == (parent / 'localization_freshness.py').read_bytes()
