import time

import pytest

from indexed_timeline import IndexedJointTimeline
from runtime_performance import RuntimePerformanceMixin
from runtime_stack_sampler import RuntimeStackSampler
from coherent_time import JointTimeline


def test_mixin_preserves_history():
    class Parent:
        def __init__(self, plan, buttons):
            self.joint_timeline = JointTimeline(plan)
            self.joint_timeline.add(1., plan)
            from test_localization_delivery import FakeNode
            self.node = FakeNode()
            self.record = lambda *args: None

    class Candidate(RuntimePerformanceMixin, Parent):
        pass

    candidate = Candidate({'joint': 3.}, {})
    assert isinstance(candidate.joint_timeline, IndexedJointTimeline)
    assert candidate.joint_timeline.at(1.)['positions'] == {'joint': 3.}
    assert len(candidate.localization_delivery_subscriptions) == 2


def test_no_guard_or_motion_overrides():
    assert not set(('spin', 'safety', 'check_coherent_grip', 'latest_native_guard', 'move',
                    'single_plan', 'retrieve', 'capture_pair')).intersection(RuntimePerformanceMixin.__dict__)
    from sequential_speed_cycle import SpeedPriorityCycle
    from speed_feedback_cycle import SpeedFeedbackMixin
    assert SpeedPriorityCycle.spin is SpeedFeedbackMixin.spin
    assert RuntimePerformanceMixin in SpeedPriorityCycle.__mro__


def test_sampler_lifecycle_and_no_overwrite(tmp_path):
    output = tmp_path / 'profile.json'
    sampler = RuntimeStackSampler(output, lambda: 'test')
    sampler.start()
    time.sleep(.09)
    result = sampler.stop()
    assert result['samples'] >= 1
    assert not result['errors']
    assert any(row['thread'] == 'MainThread' and row['phase'] == 'test' for row in result['stacks'])
    assert not sampler.thread.is_alive()
    with pytest.raises(RuntimeError):
        sampler.start()
    with pytest.raises(RuntimeError):
        RuntimeStackSampler(output, lambda: 'test').start()
    with pytest.raises(FileExistsError):
        sampler.stop()


@pytest.mark.parametrize('interval', [0, .001, float('nan'), float('inf')])
def test_sampler_interval_is_bounded(tmp_path, interval):
    with pytest.raises(ValueError):
        RuntimeStackSampler(tmp_path / 'profile.json', lambda: '', interval)
