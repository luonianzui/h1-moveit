import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import psutil
import pytest

from inputs.localizer_runtime import InstrumentedLocalizer, ORIGINAL_SHA256, PipelineLog, original
from inputs.exact_rate_localizer import ExactRateLocalizer
from localization_source import NEW_NODE, restore_localizer_launch
from retirement_race import active_process, describe_and_interrupt


ROOT = Path(__file__).resolve().parent


def test_formal_localizer_unchanged():
    assert hashlib.sha256(Path(original.__file__).read_bytes()).hexdigest() == ORIGINAL_SHA256


def test_launch_only_declared_rate_and_observer_change():
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r336'
    source = (ROOT / 'inputs/lab_dual.launch.py').read_text()
    assert restore_localizer_launch(source) == (parent / 'inputs/lab_dual.launch.py').read_text().replace(str(parent), str(ROOT))
    with pytest.raises(ValueError):
        restore_localizer_launch(source.replace(NEW_NODE, NEW_NODE + NEW_NODE))


def test_pipeline_log_is_exclusive(tmp_path):
    path = tmp_path / 'diagnostic.jsonl'
    recorder = PipelineLog(path)
    recorder.record('cloud', stamp=1.2)
    recorder.close()
    assert json.loads(path.read_text())['stamp'] == 1.2
    with pytest.raises(FileExistsError):
        PipelineLog(path)


def test_pose_observer_preserves_source_and_false_tf_return():
    source = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=100)))
    node = object.__new__(InstrumentedLocalizer)
    node.recorder = Mock()
    with patch.object(original.Mid360SquareLocalizer, '_publish_pose', return_value=False) as method:
        assert node._publish_pose(source, .01) is False
        method.assert_called_once_with(source, .01)
    assert node.recorder.record.call_args.kwargs['stamp'] == 12.0000001


def test_callback_logs_failure_without_suppressing_it():
    source = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=0)))
    node = object.__new__(InstrumentedLocalizer)
    node.recorder = Mock()
    with patch.object(ExactRateLocalizer, '_point_cloud_callback', side_effect=ValueError('test')):
        with pytest.raises(ValueError):
            node._point_cloud_callback(source)
    assert node.active_stamp is None
    assert [row.args[0] for row in node.recorder.record.call_args_list] == ['cloud_enter', 'cloud_exit']


@pytest.mark.parametrize('method', ['create_time', 'cmdline', 'send_signal'])
def test_retirement_tolerates_only_disappeared_owned_process(method):
    process = Mock(pid=123)
    process.create_time.return_value = 1.
    process.cmdline.return_value = ['owned']
    process.status.return_value = psutil.STATUS_RUNNING
    getattr(process, method).side_effect = psutil.NoSuchProcess(123)
    assert describe_and_interrupt(process)['disappeared_before_sigint']


def test_retirement_does_not_hide_permission_failure():
    process = Mock(pid=123)
    process.create_time.side_effect = psutil.AccessDenied(123)
    with pytest.raises(psutil.AccessDenied):
        describe_and_interrupt(process)


def test_retirement_active_race():
    process = Mock(pid=123)
    process.status.side_effect = psutil.NoSuchProcess(123)
    assert not active_process(process)
