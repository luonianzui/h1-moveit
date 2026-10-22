import json
import math
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation

from alignment_wait import wait_for_planner
from execute_coherent import CoherentCycle
from execute_loading import LoadingCycle, pose_matrix
from release_descent import LOWER_DISTANCE_M


EVIDENCE = Path(__file__).resolve().parent


class LowerReleaseCycle(CoherentCycle):
    def __init__(self, plan, buttons):
        self.lowered_release = set()
        super().__init__(plan, buttons)

    def move(self, samples, phase, rate):
        if phase.endswith('_release_lowering'):
            self.record('release_descent_motion', dict(rate_rad_s=.08,
                original_floor_sim_s=.020, actual_same_time_grip_guards=True,
                original_tracking_endpoint_and_contact_guards=True))
            return LoadingCycle.move(self, samples, phase, .08)
        return super().move(samples, phase, rate)

    def check_insertion_alignment(self, side):
        super().check_insertion_alignment(side)
        if side in self.lowered_release:
            return
        assert self.holding == side and self.held_payload is None
        self.phase = side + '_release_descent_planning'
        request = dict(side=side, positions=dict(self.positions), poses=self.poses,
                       source_sim=self.sim, pose_sim=self.pose_time, joint_sim=self.joint_time)
        request_path = EVIDENCE / 'run' / (side + '_release_descent_request.json')
        result_path = EVIDENCE / 'run' / (side + '_release_descent_result.json')
        with request_path.open('x') as stream:
            json.dump(request, stream, indent=2)
        self.record('release_descent_planning_started', request)
        with (EVIDENCE / 'run' / (side + '_release_descent_worker.log')).open('x') as stream:
            process = subprocess.Popen(['/usr/bin/python3', str(EVIDENCE / 'release_worker.py'),
                                        str(request_path), str(result_path)], stdin=subprocess.DEVNULL,
                                       stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            wait_for_planner(process, self.spin, timeout_s=60)
        result = json.loads(result_path.read_text())
        assert result['status'] == 'PASS' and result['descent_m'] == LOWER_DISTANCE_M
        assert result['maximum_joint_step_rad'] == .002 and result['gripper_opening_dense_checked']
        before = pose_matrix(request['poses']['box48_' + side])
        current = pose_matrix(self.poses['box48_' + side])
        drift_m = float(np.linalg.norm(current[:3, 3] - before[:3, 3]))
        drift_deg = math.degrees(Rotation.from_matrix(before[:3, :3].T @ current[:3, :3]).magnitude())
        joint_drift = max(abs(self.positions[name] - result['samples'][0][name]) for name in result['samples'][0])
        self.record('release_descent_snapshot_checked', dict(drift_m=drift_m,
                    drift_deg=drift_deg, joint_drift_rad=joint_drift))
        assert drift_m <= .0003 and drift_deg <= .1 and joint_drift <= .0003
        self.record('release_descent_plan', result)
        self.move(result['samples'], side + '_release_lowering', .08)
        super().check_insertion_alignment(side)
        target = np.asarray(result['target_body'])
        current = pose_matrix(self.poses['box48_' + side])
        assert np.linalg.norm(current[:3, 3] - target[:3, 3]) <= .0005
        assert Rotation.from_matrix(target[:3, :3].T @ current[:3, :3]).magnitude() <= math.radians(.25)
        self.record('lower_release_gate_pass', dict(side=side, poses=self.poses,
                    nominal_floor_gap_m=.068 - LOWER_DISTANCE_M, descent_m=LOWER_DISTANCE_M,
                    no_physical_attachment=True, original_containment_and_three_seconds_required=True))
        self.lowered_release.add(side)
