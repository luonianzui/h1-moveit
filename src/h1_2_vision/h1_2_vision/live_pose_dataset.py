"""Collect fail-closed live AprilTag/depth accuracy evidence."""

from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import time

from h1_2_perception_msgs.msg import TrackedObjectState
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


SCHEMA = 'h1_apriltag_live_pose_dataset/v1'


def stamp_ns(stamp) -> int:
    """Convert a builtin time value to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def expected_quaternion(yaw_rad: float) -> np.ndarray:
    """Return the world-frame quaternion for a level yaw pose."""
    return np.asarray(
        [0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad)],
        dtype=float,
    )


def pose_errors(position, quaternion, expected_position, expected_orientation):
    """Return translation and shortest quaternion-angle errors."""
    observed_position = np.asarray(position, dtype=float)
    expected_position = np.asarray(expected_position, dtype=float)
    observed_orientation = np.asarray(quaternion, dtype=float)
    expected_orientation = np.asarray(expected_orientation, dtype=float)
    if (observed_position.shape != (3,) or expected_position.shape != (3,) or
            observed_orientation.shape != (4,) or
            expected_orientation.shape != (4,) or
            not np.isfinite(observed_position).all() or
            not np.isfinite(expected_position).all() or
            not np.isfinite(observed_orientation).all() or
            not np.isfinite(expected_orientation).all()):
        raise ValueError('pose values must be finite vectors')
    observed_norm = float(np.linalg.norm(observed_orientation))
    expected_norm = float(np.linalg.norm(expected_orientation))
    if observed_norm < 1e-12 or expected_norm < 1e-12:
        raise ValueError('pose quaternion has zero norm')
    dot = abs(float(np.dot(
        observed_orientation / observed_norm,
        expected_orientation / expected_norm,
    )))
    dot = min(1.0, max(0.0, dot))
    return (
        float(np.linalg.norm(observed_position - expected_position)),
        2.0 * math.acos(dot),
    )


def summarize(samples):
    """Build deterministic aggregate metrics for non-empty samples."""
    if not samples:
        raise ValueError('dataset has no samples')
    translation = np.asarray(
        [value['translation_error_m'] for value in samples], dtype=float)
    rotation = np.asarray(
        [value['rotation_error_rad'] for value in samples], dtype=float)
    depth = np.asarray(
        [value['maximum_depth_residual_m'] for value in samples], dtype=float)
    reprojection = np.asarray(
        [value['maximum_reprojection_error_px'] for value in samples],
        dtype=float,
    )
    if not all(np.isfinite(values).all() for values in (
            translation, rotation, depth, reprojection)):
        raise ValueError('dataset metrics are non-finite')
    return {
        'sample_count': len(samples),
        'translation_error_max_m': float(np.max(translation)),
        'translation_error_rms_m': float(np.sqrt(np.mean(translation ** 2))),
        'rotation_error_max_rad': float(np.max(rotation)),
        'rotation_error_rms_rad': float(np.sqrt(np.mean(rotation ** 2))),
        'depth_residual_max_m': float(np.max(depth)),
        'reprojection_error_max_px': float(np.max(reprojection)),
        'minimum_consecutive_frames': int(min(
            value['consecutive_frames'] for value in samples)),
        'maximum_nearest_depth_skew_ns': int(max(
            value['nearest_depth_skew_ns'] for value in samples)),
        'maximum_nearest_cloud_skew_ns': int(max(
            value['nearest_cloud_skew_ns'] for value in samples)),
    }


class LivePoseDataset(Node):
    """Collect one immutable stable-pose dataset without commanding motion."""

    def __init__(self):
        super().__init__('h1_2_live_pose_dataset')
        self.declare_parameter('object_name', 'cube')
        self.declare_parameter('output', '')
        self.declare_parameter('sample_count', 50)
        self.declare_parameter('timeout_s', 30.0)
        self.declare_parameter('expected_x', 0.0)
        self.declare_parameter('expected_y', 0.0)
        self.declare_parameter('expected_z', 0.0)
        self.declare_parameter('expected_yaw_rad', 0.0)
        self.declare_parameter('maximum_translation_error_m', 0.005)
        self.declare_parameter(
            'maximum_rotation_error_rad', math.radians(1.0))
        self.declare_parameter('maximum_depth_residual_m', 0.010)
        self.declare_parameter('maximum_sensor_skew_s', 0.100)
        self.declare_parameter('maximum_cloud_skew_s', 1.500)
        self.declare_parameter('record_failure', False)

        self.object_name = str(
            self.get_parameter('object_name').value).strip()
        self.output = Path(str(self.get_parameter('output').value).strip())
        self.sample_count = int(self.get_parameter('sample_count').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.expected_position = np.asarray([
            float(self.get_parameter('expected_x').value),
            float(self.get_parameter('expected_y').value),
            float(self.get_parameter('expected_z').value),
        ])
        self.expected_yaw = float(
            self.get_parameter('expected_yaw_rad').value)
        self.expected_orientation = expected_quaternion(self.expected_yaw)
        self.maximum_translation_error = float(
            self.get_parameter('maximum_translation_error_m').value)
        self.maximum_rotation_error = float(
            self.get_parameter('maximum_rotation_error_rad').value)
        self.maximum_depth_residual = float(
            self.get_parameter('maximum_depth_residual_m').value)
        self.maximum_sensor_skew_ns = int(float(
            self.get_parameter('maximum_sensor_skew_s').value) * 1e9)
        self.maximum_cloud_skew_ns = int(float(
            self.get_parameter('maximum_cloud_skew_s').value) * 1e9)
        self.record_failure = bool(
            self.get_parameter('record_failure').value)
        if (not self.object_name or str(self.output) in ('', '.') or
                self.sample_count < 2 or self.timeout_s <= 0.0 or
                not np.isfinite(self.expected_position).all() or
                not math.isfinite(self.expected_yaw) or
                min(self.maximum_translation_error,
                    self.maximum_rotation_error,
                    self.maximum_depth_residual,
                    self.maximum_sensor_skew_ns,
                    self.maximum_cloud_skew_ns) <= 0):
            raise ValueError('invalid live-pose dataset configuration')
        if self.output.exists() or self.output.is_symlink():
            raise FileExistsError(f'output already exists: {self.output}')

        self.depth_stamps = deque(maxlen=512)
        self.cloud_stamps = deque(maxlen=512)
        self.cloud_width = {}
        self.samples = []
        self.seen_stamps = set()
        self.started = time.monotonic()
        self.failed = ''
        self.finished = False
        self.config_hash = ''

        self.create_subscription(
            Image, '/realsense_camera/depth/image', self.depth_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/realsense_camera/depth/points',
            self.cloud_callback, qos_profile_sensor_data)
        self.create_subscription(
            TrackedObjectState, f'/tag_pose/{self.object_name}',
            self.object_callback, 10)
        self.create_timer(0.1, self.timeout_callback)

    def depth_callback(self, message):
        self.depth_stamps.append(stamp_ns(message.header.stamp))

    def cloud_callback(self, message):
        value = stamp_ns(message.header.stamp)
        self.cloud_stamps.append(value)
        self.cloud_width[value] = int(message.width)
        if len(self.cloud_width) > 1024:
            retained = set(self.cloud_stamps)
            self.cloud_width = {
                key: item for key, item in self.cloud_width.items()
                if key in retained
            }

    @staticmethod
    def nearest(stamps, target):
        if not stamps:
            return None
        return min(stamps, key=lambda value: abs(value - target))

    def object_callback(self, message):
        if self.finished or self.failed:
            return
        value_stamp = stamp_ns(message.header.stamp)
        if value_stamp in self.seen_stamps:
            return
        if (message.object_name != self.object_name or not message.valid or
                not message.stable or message.status != 'STABLE' or
                not message.config_hash):
            return
        if self.config_hash and message.config_hash != self.config_hash:
            self.fail('TAG_CONFIG_CHANGED')
            return
        self.config_hash = message.config_hash
        depth_stamp = self.nearest(self.depth_stamps, value_stamp)
        cloud_stamp = self.nearest(self.cloud_stamps, value_stamp)
        if depth_stamp is None or cloud_stamp is None:
            return
        depth_skew = abs(depth_stamp - value_stamp)
        cloud_skew = abs(cloud_stamp - value_stamp)
        if (depth_skew > self.maximum_sensor_skew_ns or
                cloud_skew > self.maximum_cloud_skew_ns):
            return
        pose = message.pose_world.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        quaternion = [
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ]
        translation_error, rotation_error = pose_errors(
            position, quaternion,
            self.expected_position, self.expected_orientation)
        if translation_error > self.maximum_translation_error:
            if self.samples:
                self.fail(f'TRANSLATION_ERROR:{translation_error:.9f}')
            return
        if rotation_error > self.maximum_rotation_error:
            if self.samples:
                self.fail(f'ROTATION_ERROR:{rotation_error:.9f}')
            return
        if (not math.isfinite(message.maximum_depth_residual_m) or
                message.maximum_depth_residual_m > self.maximum_depth_residual):
            self.fail(
                f'DEPTH_RESIDUAL:{message.maximum_depth_residual_m:.9f}')
            return
        self.seen_stamps.add(value_stamp)
        self.samples.append({
            'stamp_ns': value_stamp,
            'position': position,
            'quaternion_xyzw': quaternion,
            'translation_error_m': translation_error,
            'rotation_error_rad': rotation_error,
            'maximum_reprojection_error_px': float(
                message.maximum_reprojection_error_px),
            'maximum_depth_residual_m': float(
                message.maximum_depth_residual_m),
            'consecutive_frames': int(message.consecutive_frames),
            'contributing_tag_ids': list(message.contributing_tag_ids),
            'source_mode': message.source_mode,
            'nearest_depth_stamp_ns': depth_stamp,
            'nearest_depth_skew_ns': depth_skew,
            'nearest_cloud_stamp_ns': cloud_stamp,
            'nearest_cloud_skew_ns': cloud_skew,
            'pointcloud_width': int(self.cloud_width.get(cloud_stamp, 0)),
        })
        if len(self.samples) == self.sample_count:
            self.finish()

    def timeout_callback(self):
        if (not self.finished and not self.failed and
                time.monotonic() - self.started > self.timeout_s):
            self.fail(f'TIMEOUT:{len(self.samples)}/{self.sample_count}')

    def fail(self, reason):
        self.failed = reason
        if self.record_failure:
            self.write_document({
                'schema': SCHEMA,
                'valid': False,
                'status': reason,
                'object_name': self.object_name,
                'config_hash': self.config_hash,
                'ground_truth': {
                    'position': self.expected_position.tolist(),
                    'yaw_rad': self.expected_yaw,
                    'quaternion_xyzw': self.expected_orientation.tolist(),
                },
                'collected_sample_count': len(self.samples),
                'elapsed_wall_s': time.monotonic() - self.started,
            })
        self.get_logger().error(f'LIVE_POSE_DATASET=FAIL reason={reason}')
        rclpy.shutdown()

    def write_document(self, document):
        """Publish one non-overwriting read-only evidence document."""
        self.output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        descriptor = os.open(
            self.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o400)
        with os.fdopen(descriptor, 'w', encoding='utf-8') as output_file:
            json.dump(
                document, output_file, sort_keys=True,
                separators=(',', ':'), allow_nan=False)
            output_file.write('\n')
            output_file.flush()
            os.fsync(output_file.fileno())

    def finish(self):
        summary = summarize(self.samples)
        document = {
            'schema': SCHEMA,
            'valid': True,
            'status': 'PASS',
            'object_name': self.object_name,
            'config_hash': self.config_hash,
            'ground_truth': {
                'position': self.expected_position.tolist(),
                'yaw_rad': self.expected_yaw,
                'quaternion_xyzw': self.expected_orientation.tolist(),
            },
            'limits': {
                'maximum_translation_error_m': self.maximum_translation_error,
                'maximum_rotation_error_rad': self.maximum_rotation_error,
                'maximum_depth_residual_m': self.maximum_depth_residual,
                'maximum_sensor_skew_ns': self.maximum_sensor_skew_ns,
                'maximum_cloud_skew_ns': self.maximum_cloud_skew_ns,
            },
            'summary': summary,
            'samples': self.samples,
        }
        self.write_document(document)
        self.finished = True
        self.get_logger().info(
            'LIVE_POSE_DATASET=PASS '
            f'object={self.object_name} samples={self.sample_count} '
            f'translation_max_m={summary["translation_error_max_m"]:.9f} '
            f'rotation_max_rad={summary["rotation_error_max_rad"]:.9f} '
            f'depth_max_m={summary["depth_residual_max_m"]:.9f} '
            f'output={self.output}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LivePoseDataset()
    try:
        rclpy.spin(node)
    finally:
        failed = node.failed
        finished = node.finished
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise RuntimeError(failed)
    if not finished:
        raise RuntimeError('dataset collection ended without a result')


if __name__ == '__main__':
    main()
