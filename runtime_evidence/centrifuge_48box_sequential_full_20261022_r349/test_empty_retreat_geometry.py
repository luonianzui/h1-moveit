from types import SimpleNamespace

import numpy as np
import pytest

from empty_retreat_geometry import plan_empty_retreat, plan_empty_guided_retreat


class State:
    def __init__(self):
        self.joint_positions = {}

    def update(self):
        pass

    def get_global_link_transform(self, name):
        body = np.eye(4)
        body[1, 3] = .085 if name.startswith('left') else -.085
        body[2, 3] = 1.27
        return body


class Planner:
    def __init__(self, request, references):
        assert all(np.array_equal(matrix, np.eye(4)) for matrix in references.values())
        self.names = ['torso_joint', 'left_elbow_joint', 'right_elbow_joint']
        self.planner = SimpleNamespace(state=State())
        self.planner.initial = dict(request['positions'])
        self.planner.check = lambda joints, **options: dict(collision=False)
        self.lower = np.array([-10.] * len(self.names))
        self.upper = np.array([10.] * len(self.names))
        self.checked = 0
        self.targets = []

    def checked_stage(self, first, last, seed, phase, count):
        self.checked += count
        self.targets.append(last)
        return dict(phase=phase, samples=[dict(seed)])

    def empty_connector(self, first, target):
        self.checked += 2
        return [dict(first), dict(target)]


def request():
    return dict(mode='empty_return', original_runtime_holder_references={},
                positions=dict(torso_joint=0., left_elbow_joint=0., right_elbow_joint=0.))


def test_clearance_sequence_never_attaches_or_moves_the_released_boxes():
    actual = request()
    result = plan_empty_retreat(Planner, actual, actual['positions'])
    assert [stage['phase'] for stage in result['stages']] == ['empty_vertical_clearance', 'empty_outboard_clearance', 'empty_return']
    assert result['maximum_joint_step_rad'] == .002
    assert result['no_physical_attachment'] and not result['planning_only_attachment']
    assert result['original_loose_payload_collision_guards_unchanged']
    assert actual == request()


def test_held_payload_cannot_be_disguised_as_empty_retreat():
    actual = request()
    actual['original_runtime_holder_references'] = dict(left=dict(payload='left'))
    with pytest.raises(ValueError, match='empty holder'):
        plan_empty_retreat(Planner, actual, actual['positions'])


@pytest.mark.parametrize('height, width', [(.01, .30), (.20, .30), (.12, .10), (.12, .50)])
def test_unreviewed_corridor_dimensions_fail_closed(height, width):
    actual = request()
    with pytest.raises(ValueError):
        plan_empty_retreat(Planner, actual, actual['positions'], height, width)


def test_guided_empty_retreat_keeps_measured_start_and_omits_unneeded_table_lowering():
    actual = request()
    guide = [{name: value + .01 for name, value in actual['positions'].items()}]
    result = plan_empty_guided_retreat(Planner, actual, actual['positions'],
                                     [dict(phase='table_lift', samples=guide), dict(phase='above_bucket', samples=guide)])
    assert result['stages'][0]['samples'][0] == actual['positions']
    assert [stage['phase'] for stage in result['stages']] == ['empty_reverse_above_bucket', 'empty_return']
    assert result['no_physical_attachment'] and not result['planning_only_attachment']
    assert result['actual_measured_start_and_static_released_boxes_unchanged']
    assert actual == request()


def test_unknown_or_held_guide_cannot_override_actual_empty_state():
    actual = request()
    with pytest.raises(ValueError, match='qualified loading'):
        plan_empty_guided_retreat(Planner, actual, actual['positions'], [])
    actual['original_runtime_holder_references'] = dict(left=dict(payload='left'))
    with pytest.raises(ValueError, match='empty holder'):
        plan_empty_guided_retreat(Planner, actual, actual['positions'], [{}, {}])


def test_collision_free_home_proposal_cannot_cross_original_joint_margin():
    class BadHomePlanner(Planner):
        def empty_connector(self, first, target):
            return [dict(first), dict(target, torso_joint=11.)]

    actual = request()
    guides = [dict(phase='table_lift', samples=[actual['positions']]),
              dict(phase='above_bucket', samples=[actual['positions']])]
    with pytest.raises(RuntimeError, match='joint margin'):
        plan_empty_guided_retreat(BadHomePlanner, actual, actual['positions'], guides)
