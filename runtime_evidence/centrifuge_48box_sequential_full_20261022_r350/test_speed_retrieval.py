import inspect
import math
from types import SimpleNamespace

import numpy as np
import pytest

from sequential_geometry import SinglePlanner
from dual_motion import TASK_JOINTS
from speed_policy import PROFILE
from speed_retrieval_geometry import SpeedRetrievalPlanner
import speed_planning


def test_dense_collision_joint_bounds_and_support_logic_are_unchanged():
    original = inspect.getsource(SinglePlanner.check_edge)
    current = inspect.getsource(SpeedRetrievalPlanner.check_edge)
    current = current.replace("math.radians(PROFILE['retrieval_commanded_tilt_deg'])", 'math.radians(2)')
    current = current.replace('Explicit five-degree simulation retrieval tilt exceeded', 'Original two-degree commanded payload tilt exceeded')
    current = current.replace("                tilt = math.degrees(math.acos(float(np.clip(body[2, 2], -1., 1.))))\n", '')
    current = current.replace("                self.maximum_payload_tilt_deg = max(getattr(self, 'maximum_payload_tilt_deg', 0.), tilt)\n", '')
    assert current == original


@pytest.mark.parametrize('tilt,accepted', [(0., True), (2.1, True), (4.9, True), (5.1, False)])
def test_explicit_five_degree_boundary_still_rejects_larger_tilt(tilt, accepted):
    planner = object.__new__(SpeedRetrievalPlanner)
    planner.names = sorted(TASK_JOINTS)
    planner.lower, planner.upper = np.full(len(planner.names), -1.), np.full(len(planner.names), 1.)
    first = dict.fromkeys(planner.names, 0.)
    last = dict(first, left_elbow_joint=.001)
    planner.hand, planner.payload, planner.attached, planner.checked = 'left', 'left', True, 0
    planner.reference = np.eye(4)
    body = np.eye(4)
    body[2, 2] = math.cos(math.radians(tilt))
    planner.planner = SimpleNamespace(
        scene=SimpleNamespace(allowed_collision_matrix=SimpleNamespace(set_entry=lambda *args: None)),
        state=SimpleNamespace(get_global_link_transform=lambda name: body),
        check=lambda *args, **kwargs: dict(collision=False))
    if accepted:
        planner.check_edge(first, last, 'test')
        assert planner.checked > 0 and planner.maximum_payload_tilt_deg == pytest.approx(tilt)
    else:
        with pytest.raises(RuntimeError, match='five-degree'):
            planner.check_edge(first, last, 'test')
    assert PROFILE['retrieval_commanded_tilt_deg'] == 5.


def test_other_modes_still_call_original_planner(monkeypatch):
    request = dict(mode='load_transport')
    monkeypatch.setattr(speed_planning, 'original_plan', lambda actual: ('original', actual))
    assert speed_planning.plan(request) == ('original', request)
