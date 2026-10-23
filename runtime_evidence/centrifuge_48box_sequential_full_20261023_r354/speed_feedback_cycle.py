import math
import time

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from execute_loading import pose_matrix
from grip_limits import grip_exceeds_limits
from speed_coherent import select_coherent_pair
from speed_policy import PROFILE


class SpeedFeedbackMixin:
    def spin(self, guarded=True):
        self.refresh_native()
        rclpy.spin_once(self.node, timeout_sec=.01)
        self.refresh_native()
        if guarded and self.holders.rows:
            deadline = time.monotonic() + .010
            for attempt in range(64):
                self.latest_native_guard()
                with self.native_lock:
                    pair = select_coherent_pair(list(self.native_frames), self.joint_timeline, self.pose_time)
                if pair is not None or time.monotonic() >= deadline:
                    break
                rclpy.spin_once(self.node, timeout_sec=0)
                self.refresh_native()
        if guarded:
            self.safety()

    def check_coherent_grip(self, hand, payload, retrieval):
        try:
            return self._check_speed_coherent_grip(hand, payload, retrieval)
        except RuntimeError as error:
            if 'No recent actual joint/native bracket' in str(error):
                with self.native_lock:
                    frames = list(self.native_frames)
                    pair = select_coherent_pair(frames, self.joint_timeline, self.pose_time)
                self.record('paired_feedback_pair_fault', dict(hand=hand, payload=payload,
                    exact_requested_native_stamp=self.pose_time, joint_stamp=self.joint_time,
                    clock_sim=self.sim, history_length=len(self.joint_timeline.rows),
                    history_first=self.joint_timeline.rows[0][0] if self.joint_timeline.rows else None,
                    history_last=self.joint_timeline.rows[-1][0] if self.joint_timeline.rows else None,
                    last_native_stamps=[frame[0] for frame in frames[-8:]],
                    no_actual_bracket=pair is None,
                    actual_pair_wall_age_s=None if pair is None else time.monotonic() - pair['pose_wall'],
                    original_maximum_bracket_and_lag_sim_s=.012, current_maximum_native_lag_sim_s=PROFILE['coherent_native_lag_sim_s'], original_pair_age_wall_s=2,
                    no_extrapolation_command_substitution_or_reference_rebase=True))
            raise

    def measured_pair(self):
        deadline = time.monotonic() + .010
        for attempt in range(65):
            if self.holders.rows:
                self.spin()
            else:
                rclpy.spin_once(self.node, timeout_sec=0)
                self.refresh_native()
                self.safety()
            with self.native_lock:
                pair = select_coherent_pair(list(self.native_frames), self.joint_timeline, self.pose_time)
            if pair is not None and time.monotonic() - pair['pose_wall'] <= 2:
                return pair
            if attempt == 64 or time.monotonic() >= deadline:
                break
            rclpy.spin_once(self.node, timeout_sec=0)
        self.record('paired_snapshot_pair_fault', dict(requested_native_stamp=self.pose_time,
                    last_joint_stamp=self.joint_time, no_extrapolation=True,
                    no_actual_bracket=pair is None,
                    actual_pair_wall_age_s=None if pair is None else time.monotonic() - pair['pose_wall'],
                    original_bracket_and_lag_limit_sim_s=.012, bounded_actual_callbacks=64))
        raise RuntimeError('No actual bracket for paired snapshot after bounded real callbacks')

    def _check_speed_coherent_grip(self, hand, payload, retrieval):
        with self.native_lock:
            frames = list(self.native_frames)
        pair = select_coherent_pair(frames, self.joint_timeline, self.pose_time)
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
