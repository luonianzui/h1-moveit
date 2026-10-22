import json
from types import SimpleNamespace

import pytest
from scipy.spatial.transform import Rotation

import execute_cycle
from execute_cycle import FullCycle


@pytest.mark.parametrize('drift_m,drift_deg,accepted', [(0, 0, True), (.000299, 0, True), (.000301, 0, False), (0, .099, True), (0, .101, False)])
def test_planner_compares_immutable_before_pose_after_live_updates(monkeypatch, tmp_path, drift_m, drift_deg, accepted):
    (tmp_path / 'run').mkdir()
    monkeypatch.setattr(execute_cycle, 'EVIDENCE', tmp_path)
    cycle = FullCycle.__new__(FullCycle)
    cycle.positions = {'torso_joint': 0.0}
    cycle.poses = {'box48_left': {'position': {'x': .6, 'y': 0, 'z': 1.05}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}}
    cycle.dwell = lambda duration: None
    cycle.spin = lambda: None
    cycle.record = lambda kind, value: None
    launched = []
    def launch(command, **kwargs):
        launched.append(command)
        return SimpleNamespace()
    monkeypatch.setattr(execute_cycle.subprocess, 'Popen', launch)
    def finish(process, spin, timeout_s):
        result = {'status': 'PASS', 'maximum_joint_sampling_rad': .002, 'stages': [{'samples': [{'torso_joint': 0.0}]}]}
        with open(launched[0][3], 'x') as stream: json.dump(result, stream)
        cycle.poses['box48_left']['position']['x'] += drift_m
        quaternion = Rotation.from_euler('z', drift_deg, degrees=True).as_quat()
        cycle.poses['box48_left']['orientation'].update(zip('xyzw', quaternion))
    monkeypatch.setattr(execute_cycle, 'wait_for_planner', finish)
    if accepted:
        assert cycle.plan_retrieval('right', 'left', 'pickup')['status'] == 'PASS'
    else:
        with pytest.raises(AssertionError): cycle.plan_retrieval('right', 'left', 'pickup')
    saved = json.loads((tmp_path / 'run/left_right_pickup_request.json').read_text())
    assert saved['poses']['box48_left']['position']['x'] == .6
    assert saved['poses']['box48_left']['orientation']['w'] == 1

