from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from retrieval_rrt import free_path


class NarrowObstacle:
    def __init__(self):
        self.urdf = ET.fromstring('<robot><joint name="torso_joint"><limit lower="-1" upper="1"/></joint>'
                                 '<joint name="arm_joint"><limit lower="-1" upper="1"/></joint></robot>')
        self.queries = []

    def check(self, joints):
        self.queries.append(joints)
        return dict(collision=self.collision(joints))

    def collision(self, joints):
        return .043 < joints['torso_joint'] < .057 and abs(joints['arm_joint']) < .020


def test_rrt_cannot_shortcut_through_narrow_collision_missed_by_coarse_edge():
    planner = NarrowObstacle()
    first = dict(torso_joint=0.0, arm_joint=0.0)
    last = dict(torso_joint=.1, arm_joint=0.0)
    assert not any(planner.collision(dict(torso_joint=float(value), arm_joint=0.0))
                   for value in np.linspace(0, .1, 4))
    samples = free_path(planner, first, last)
    assert samples[0] == first
    assert samples[-1] == last
    assert any(planner.collision(query) for query in planner.queries)
    for start, end in zip(samples, samples[1:]):
        count = max(2, int(np.ceil(max(abs(end[name] - start[name]) for name in start) / .002)) + 1)
        for fraction in np.linspace(0, 1, count):
            assert not planner.collision({name: start[name] + fraction * (end[name] - start[name]) for name in start})


@pytest.mark.parametrize('position', [.049, .051])
def test_rrt_endpoint_collision_is_not_ignored(position):
    planner = NarrowObstacle()
    with pytest.raises(ValueError, match='endpoint in collision'):
        free_path(planner, dict(torso_joint=position, arm_joint=0.0), dict(torso_joint=.1, arm_joint=0.0))


def test_all_rrt_edge_checks_use_dense_original_bound():
    source = Path(__file__).with_name('retrieval_rrt.py').read_text()
    assert '/ .035' not in source
    assert '/ .002' in source

