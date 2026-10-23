from collections import deque
import json
import math
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from coherent_time import JointTimeline, newest_pair
from execute_fast import FastCycle
from execute_loading import pose_matrix
from grip_limits import grip_exceeds_limits


class CoherentCycle(FastCycle):
    def __init__(self, plan, buttons):
        self.joint_timeline = JointTimeline(plan['initial_positions'])
        self.native_frames = deque(maxlen=256)
        self.native_lock = threading.Lock()
        self.native_stream_fault = None
        self.coherent_reference_key = None
        self.coherent_reference = None
        self.reference_native_bytes = None
        self.last_coherent_record = -1.
        self.maximum_legacy_unaligned_slip = 0.
        super().__init__(plan, buttons)

    def receive_joints(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        actual = dict(zip(message.name, message.position))
        self.joint_timeline.add(stamp, actual)
        super().receive_joints(message)
        self.record('actual_full_joint_frame', dict(source_sim=stamp, positions=actual,
                    commanded_values_not_used=True))

    def read_poses(self):
        names = frozenset(('box48_left', 'box48_right', 'bucket_left', 'bucket_right',
            'rotor', 'lid', 'centrifuge', 'left_wrist_yaw_link', 'right_wrist_yaw_link',
            'left_dex1_finger_1_slider_link', 'left_dex1_finger_2_slider_link',
            'right_dex1_finger_1_slider_link', 'right_dex1_finger_2_slider_link'))
        for line in self.pose_process.stdout:
            try:
                value = json.loads(line)
                poses = {pose['name']:pose for pose in value['pose'] if pose.get('name') in names}
                if poses.keys() != names:
                    continue
                header = value['header']['stamp']
                stamp = int(header.get('sec', 0)) + int(header.get('nsec', 0)) * 1e-9
                if not math.isfinite(stamp):
                    self.native_stream_fault = 'Nonfinite native source time'
                    continue
                snapshot = (stamp, time.monotonic(), poses)
                with self.native_lock:
                    if self.native_frames and stamp < self.native_frames[-1][0]:
                        self.native_stream_fault = 'Native time reversal'
                        continue
                    if self.native_frames and stamp == self.native_frames[-1][0]:
                        self.native_frames[-1] = snapshot
                    else:
                        self.native_frames.append(snapshot)
                    self.pose_snapshot = snapshot
            except (ValueError, KeyError):
                continue

    def run_complete(self):
        self.record('coherent_grip_measurement_declared', dict(
            real_full_joint_history=True, maximum_bracket_sim_s=.012,
            maximum_pair_to_latest_native_lag_sim_s=.012,
            no_extrapolation=True, no_commanded_state_substitution=True,
            fk_native_wrist_agreement_m=.001, fk_native_wrist_agreement_deg=.2,
            latest_native_independent_guard=True, slip_m=.002, angle_deg=5.,
            other_physical_checks_use_original_latest_pose=True,
            raw_unaligned_FK_diagnostic_not_original_strict_certificate=True,
            frozen_native_reference_never_rebased_from_current_payload=True))
        super().run_complete()

    def safety(self):
        if self.native_stream_fault:
            raise RuntimeError(self.native_stream_fault)
        hand, held = self.holding, self.held_payload
        if hand is None:
            if self.coherent_reference_key is not None:
                self.record('coherent_guard_scope_end', dict(previous_key=self.coherent_reference_key,
                    source='original_holder_state_end', physical_empty_not_preclaimed=True))
            self.coherent_reference_key = None
            self.coherent_reference = None
            self.reference_native_bytes = None
            return super().safety()
        self.holding, self.held_payload = None, None
        try:
            super().safety()
        finally:
            self.holding, self.held_payload = hand, held
        self.check_coherent_grip(hand, held if held is not None else hand, held is not None)

    def check_coherent_grip(self, hand, payload, retrieval):
        with self.native_lock:
            frames = list(self.native_frames)
        pair = newest_pair(frames, self.joint_timeline, self.pose_time)
        if pair is None or time.monotonic() - pair['pose_wall'] > 2:
            raise RuntimeError('No recent actual joint/native bracket; no extrapolation permitted')
        positions = pair['joints']['positions']
        self.state.joint_positions = positions
        self.state.update()
        fk = self.state.get_global_link_transform(hand + '_wrist_yaw_link').copy()
        paired_native = pose_matrix(pair['poses'][hand + '_wrist_yaw_link'])
        agreement_m = float(np.linalg.norm(fk[:3, 3] - paired_native[:3, 3]))
        agreement_deg = math.degrees(Rotation.from_matrix(fk[:3, :3].T @ paired_native[:3, :3]).magnitude())
        if agreement_m > .001 or agreement_deg > .2:
            self.record('coherent_wrist_agreement_fault', dict(pair=pair,
                        position_m=agreement_m, angle_deg=agreement_deg))
            raise RuntimeError('Same-time FK/native wrist agreement exceeded 1mm/0.2deg')
        key = (hand, payload)
        native_bytes = self.native_grasp_transform.tobytes()
        if self.coherent_reference_key is None:
            self.coherent_reference_key = key
            self.reference_native_bytes = native_bytes
            self.coherent_reference = np.linalg.inv(fk) @ paired_native @ self.native_grasp_transform
            self.record('coherent_grip_reference_frozen', dict(hand=hand, payload=payload,
                pose_sim=pair['pose_stamp'], actual_joint_pair=pair['joints'],
                native_reference=self.native_grasp_transform.tolist(),
                fk_reference=self.coherent_reference.tolist(), paired_poses=pair['poses'],
                current_payload_not_used_to_rebase=True))
        elif key != self.coherent_reference_key or native_bytes != self.reference_native_bytes:
            raise RuntimeError('Held reference changed without an observed empty-hand interval')
        body_pair = pose_matrix(pair['poses']['box48_' + payload])
        body_latest = pose_matrix(self.poses['box48_' + payload])
        native_latest = pose_matrix(self.poses[hand + '_wrist_yaw_link'])
        expected_fk = fk @ self.coherent_reference
        expected_pair_native = paired_native @ self.native_grasp_transform
        expected_latest_native = native_latest @ self.native_grasp_transform

        def error(reference, actual):
            return (float(np.linalg.norm(reference[:3, 3] - actual[:3, 3])),
                    math.degrees(Rotation.from_matrix(reference[:3, :3].T @ actual[:3, :3]).magnitude()))

        slip, angle = error(expected_fk, body_pair)
        pair_native_slip, pair_native_angle = error(expected_pair_native, body_pair)
        latest_native_slip, latest_native_angle = error(expected_latest_native, body_latest)
        unaligned_slip, unaligned_angle = error(self.wrist(hand) @ self.grasp_transform, body_latest)
        self.max_slip = max(self.max_slip, slip)
        self.max_native_slip = max(self.max_native_slip, pair_native_slip, latest_native_slip)
        self.maximum_legacy_unaligned_slip = max(self.maximum_legacy_unaligned_slip, unaligned_slip)
        metrics = dict(hand=hand, side=hand, payload=payload,
            fk_slip_m=slip, fk_angle_deg=angle,
            native_slip_m=max(pair_native_slip, latest_native_slip),
            native_angle_deg=max(pair_native_angle, latest_native_angle),
            latest_native_slip_m=latest_native_slip, latest_native_angle_deg=latest_native_angle,
            legacy_unaligned_fk_slip_m=unaligned_slip, legacy_unaligned_fk_angle_deg=unaligned_angle,
            raw_joint_sim=self.joint_time, pose_sim=self.pose_time, coherent_pose_sim=pair['pose_stamp'],
            actual_joint_bracket=pair['joints'], paired_poses=pair['poses'],
            latest_poses=self.poses,
            native_lag_s=pair['native_lag_s'], fk_native_agreement_m=agreement_m,
            fk_native_agreement_deg=agreement_deg, interpolation_is_measurement_estimate=True,
            latest_native_not_delayed_or_ignored=True)
        if self.pose_time - self.last_coherent_record > .02:
            self.last_coherent_record = self.pose_time
            self.record('retrieval_grip' if retrieval else 'synchronized_grip', metrics)
        if any(grip_exceeds_limits(distance, degrees) for distance, degrees in (
                (slip, angle), (pair_native_slip, pair_native_angle),
                (latest_native_slip, latest_native_angle))):
            self.record('coherent_grip_fault', metrics)
            raise RuntimeError('Coherent FK or immediate native grip exceeded unchanged 2mm/5deg')
