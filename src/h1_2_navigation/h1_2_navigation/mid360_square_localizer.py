#!/usr/bin/env python3

"""Localize the fixed H1-2 from Mid-360 returns on the square walls."""

import json
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException
from tf2_ros import TransformListener

from .square_localization import build_square_occupancy
from .square_localization import evaluate_square_pose
from .square_localization import filter_navigation_points
from .square_localization import map_to_odom_pose
from .square_localization import normalize_angle
from .square_localization import points_to_laser_scan
from .square_localization import refine_square_pose
from .square_localization import select_square_wall_candidates
from .square_localization import square_quality_passes
from .square_localization import transform_points


def _point_cloud_xyz(message):
    fields = {field.name: field for field in message.fields}
    for name in ('x', 'y', 'z'):
        if name not in fields:
            raise ValueError(f'PointCloud2 field {name!r} is missing')
        if fields[name].datatype != PointField.FLOAT32:
            raise ValueError(f'PointCloud2 field {name!r} must be FLOAT32')
    endian = '>' if message.is_bigendian else '<'
    data_type = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + 'f4'] * 3,
        'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
        'itemsize': message.point_step,
    })
    organized = np.ndarray(
        shape=(message.height, message.width),
        dtype=data_type,
        buffer=memoryview(message.data),
        strides=(message.row_step, message.point_step),
    )
    points = np.column_stack((
        organized['x'].reshape(-1),
        organized['y'].reshape(-1),
        organized['z'].reshape(-1),
    )).astype(np.float64)
    return points[np.isfinite(points).all(axis=1)]


def _yaw_from_quaternion(quaternion):
    sine = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y)
    cosine = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z)
    return math.atan2(sine, cosine)


class Mid360SquareLocalizer(Node):
    """Project Mid-360 points and match them against the known square walls."""

    def __init__(self):
        super().__init__('h1_2_mid360_square_localizer')
        self.declare_parameter('point_cloud_topic', '/livox/mid360/points')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter(
            'pose_topic', '/mid360_localization/pose')
        self.declare_parameter(
            'status_topic', '/mid360_localization/status')
        self.declare_parameter('base_frame', 'pelvis')
        self.declare_parameter('scan_frame', 'pelvis')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter(
            'localized_frame', 'mid360_localized_pelvis')
        self.declare_parameter('publish_map_to_odom', False)
        self.declare_parameter('map_to_odom_transform_tolerance', 2.0)
        self.declare_parameter('map_to_odom_publish_rate', 50.0)
        self.declare_parameter('minimum_height', -0.35)
        self.declare_parameter('maximum_height', 1.25)
        self.declare_parameter('scan_minimum_range', 0.10)
        self.declare_parameter('minimum_range', 0.80)
        self.declare_parameter('maximum_range', 4.00)
        self.declare_parameter('scan_bin_count', 720)
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('transform_timeout', 1.0)
        self.declare_parameter('localization_rate', 2.0)
        self.declare_parameter('maximum_localization_points', 720)
        self.declare_parameter('wall_half_extent', 2.50)
        self.declare_parameter('wall_thickness', 0.10)
        self.declare_parameter('map_outside_margin', 0.50)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('minimum_point_count', 200)
        self.declare_parameter('inlier_threshold', 0.08)
        self.declare_parameter('minimum_inlier_fraction', 0.80)
        self.declare_parameter('maximum_inlier_rms', 0.05)
        self.declare_parameter('minimum_points_per_wall', 20)
        self.declare_parameter('minimum_visible_walls', 3)
        self.declare_parameter('localization_candidate_band', 0.50)
        self.declare_parameter('static_obstacle_enabled', False)
        self.declare_parameter('static_obstacle_minimum_x', 0.20)
        self.declare_parameter('static_obstacle_maximum_x', 1.00)
        self.declare_parameter('static_obstacle_minimum_y', -0.60)
        self.declare_parameter('static_obstacle_maximum_y', 0.60)

        self.point_cloud_topic = self.get_parameter(
            'point_cloud_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.localized_frame = self.get_parameter('localized_frame').value
        self.publish_map_to_odom = bool(self.get_parameter(
            'publish_map_to_odom').value)
        self.map_to_odom_transform_tolerance = float(self.get_parameter(
            'map_to_odom_transform_tolerance').value)
        self.map_to_odom_publish_rate = float(self.get_parameter(
            'map_to_odom_publish_rate').value)
        self.minimum_height = float(self.get_parameter(
            'minimum_height').value)
        self.maximum_height = float(self.get_parameter(
            'maximum_height').value)
        self.scan_minimum_range = float(self.get_parameter(
            'scan_minimum_range').value)
        self.minimum_range = float(self.get_parameter(
            'minimum_range').value)
        self.maximum_range = float(self.get_parameter(
            'maximum_range').value)
        self.scan_bin_count = int(self.get_parameter(
            'scan_bin_count').value)
        self.scan_rate = float(self.get_parameter('scan_rate').value)
        self.transform_timeout = float(self.get_parameter(
            'transform_timeout').value)
        self.localization_rate = float(self.get_parameter(
            'localization_rate').value)
        self.maximum_localization_points = int(self.get_parameter(
            'maximum_localization_points').value)
        self.wall_half_extent = float(self.get_parameter(
            'wall_half_extent').value)
        self.wall_thickness = float(self.get_parameter(
            'wall_thickness').value)
        self.map_outside_margin = float(self.get_parameter(
            'map_outside_margin').value)
        self.map_resolution = float(self.get_parameter(
            'map_resolution').value)
        self.minimum_point_count = int(self.get_parameter(
            'minimum_point_count').value)
        self.inlier_threshold = float(self.get_parameter(
            'inlier_threshold').value)
        self.minimum_inlier_fraction = float(self.get_parameter(
            'minimum_inlier_fraction').value)
        self.maximum_inlier_rms = float(self.get_parameter(
            'maximum_inlier_rms').value)
        self.minimum_points_per_wall = int(self.get_parameter(
            'minimum_points_per_wall').value)
        self.minimum_visible_walls = int(self.get_parameter(
            'minimum_visible_walls').value)
        self.localization_candidate_band = float(self.get_parameter(
            'localization_candidate_band').value)
        self.static_obstacle_enabled = bool(self.get_parameter(
            'static_obstacle_enabled').value)
        self.static_obstacle_bounds = tuple(float(self.get_parameter(
            name).value) for name in (
                'static_obstacle_minimum_x',
                'static_obstacle_maximum_x',
                'static_obstacle_minimum_y',
                'static_obstacle_maximum_y',
            ))
        if (
                self.scan_rate <= 0.0
                or self.localization_rate <= 0.0
                or self.transform_timeout <= 0.0
                or self.map_to_odom_publish_rate <= 0.0
                or self.map_to_odom_transform_tolerance < 0.0
                or self.scan_minimum_range < 0.0
                or self.scan_minimum_range > self.minimum_range
                or self.localization_candidate_band <= 0.0):
            raise ValueError(
                'scan and localization rates and transform timeout must be '
                'positive, map-to-odom publish rate must be positive, and '
                'scan/localization range and candidate gates must be valid')
        if self.maximum_localization_points < self.minimum_point_count:
            raise ValueError(
                'maximum_localization_points must cover minimum_point_count')
        if not self.scan_frame:
            raise ValueError('scan_frame must not be empty')

        self.pose = np.asarray([
            float(self.get_parameter('initial_x').value),
            float(self.get_parameter('initial_y').value),
            normalize_angle(float(self.get_parameter('initial_yaw').value)),
        ])
        self.last_localization_stamp = None
        self.last_valid = None
        self.map_to_odom = None

        self.transform_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.transform_listener = TransformListener(
            self.transform_buffer, self)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.scan_publisher = self.create_publisher(
            LaserScan, self.scan_topic, qos_profile_sensor_data)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.pose_topic, 10)
        self.status_publisher = self.create_publisher(
            String, self.status_topic, 10)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.map_publisher = self.create_publisher(
            OccupancyGrid, '/map', map_qos)
        self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self._point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initial_pose_callback,
            10,
        )
        self.map_message = self._build_map_message()
        self.map_timer = self.create_timer(2.0, self._publish_map)
        self.map_to_odom_timer = None
        if self.publish_map_to_odom:
            self.map_to_odom_timer = self.create_timer(
                1.0 / self.map_to_odom_publish_rate,
                self._publish_map_to_odom,
            )
        self._publish_map()
        self.get_logger().info(
            'Mid-360 square localizer ready; no cmd_vel, odom or robot '
            'trajectory publisher is created')

    def _build_map_message(self):
        occupied_rectangles = (
            (self.static_obstacle_bounds,)
            if self.static_obstacle_enabled else ())
        occupancy, origin = build_square_occupancy(
            self.wall_half_extent,
            self.wall_thickness,
            self.map_outside_margin,
            self.map_resolution,
            occupied_rectangles,
        )
        message = OccupancyGrid()
        message.header.frame_id = self.map_frame
        message.info.resolution = self.map_resolution
        message.info.width = occupancy.shape[1]
        message.info.height = occupancy.shape[0]
        message.info.origin.position.x = origin
        message.info.origin.position.y = origin
        message.info.origin.orientation.w = 1.0
        message.data = occupancy.reshape(-1).astype(int).tolist()
        return message

    def _publish_map(self):
        self.map_message.header.stamp = self.get_clock().now().to_msg()
        self.map_publisher.publish(self.map_message)

    def _initial_pose_callback(self, message):
        if message.header.frame_id not in ('', self.map_frame):
            self.get_logger().error(
                'Rejecting initial pose outside the configured map frame')
            return
        values = np.asarray([
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            _yaw_from_quaternion(message.pose.pose.orientation),
        ])
        if not np.isfinite(values).all():
            self.get_logger().error('Rejecting non-finite initial pose')
            return
        values[2] = normalize_angle(values[2])
        self.pose = values
        self.last_localization_stamp = None
        self.get_logger().info('Accepted a new Mid-360 initial pose')

    def _point_cloud_callback(self, message):
        try:
            transform = self.transform_buffer.lookup_transform(
                self.base_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
            points = _point_cloud_xyz(message)
        except (TransformException, ValueError) as error:
            self._publish_status(False, f'input_error:{error}', 0)
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        points = transform_points(
            points,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        scan_points = filter_navigation_points(
            points,
            self.minimum_height,
            self.maximum_height,
            self.scan_minimum_range,
            self.maximum_range,
        )
        self._publish_scan(message, scan_points)
        localization_points = filter_navigation_points(
            scan_points,
            self.minimum_height,
            self.maximum_height,
            self.minimum_range,
            self.maximum_range,
        )[:, :2]
        localization_points = select_square_wall_candidates(
            localization_points,
            self.pose,
            self.wall_half_extent,
            self.localization_candidate_band,
        )
        if len(localization_points) < self.minimum_point_count:
            self._publish_status(
                False, 'insufficient_wall_candidates',
                len(localization_points))
            return
        stamp_seconds = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9)
        period = 1.0 / self.localization_rate
        if (
                self.last_localization_stamp is not None
                and stamp_seconds >= self.last_localization_stamp
                and stamp_seconds - self.last_localization_stamp < period):
            return
        self.last_localization_stamp = stamp_seconds
        if len(localization_points) > self.maximum_localization_points:
            indices = np.linspace(
                0,
                len(localization_points) - 1,
                self.maximum_localization_points,
                dtype=np.int64,
            )
            localization_points = localization_points[indices]
        estimated_pose, robust_rms = refine_square_pose(
            localization_points,
            self.pose,
            self.wall_half_extent,
        )
        inlier_fraction, inlier_rms, side_counts = evaluate_square_pose(
            localization_points,
            estimated_pose,
            self.wall_half_extent,
            self.inlier_threshold,
        )
        valid, visible_wall_count = square_quality_passes(
            inlier_fraction,
            inlier_rms,
            side_counts,
            self.minimum_inlier_fraction,
            self.maximum_inlier_rms,
            self.minimum_points_per_wall,
            self.minimum_visible_walls,
        )
        reason = 'pass' if valid else 'quality_gate_failed'
        map_to_odom_published = False
        if valid:
            self.pose = estimated_pose
            try:
                map_to_odom_published = self._publish_pose(
                    message, inlier_rms)
            except TransformException as error:
                valid = False
                reason = f'odom_transform_unavailable:{error}'
        self._publish_status(
            valid,
            reason,
            len(localization_points),
            robust_rms,
            inlier_fraction,
            inlier_rms,
            side_counts,
            estimated_pose,
            visible_wall_count,
            map_to_odom_published,
        )

    def _publish_scan(self, source, points):
        angle_minimum = -math.pi
        angle_maximum = math.pi
        ranges, angle_increment = points_to_laser_scan(
            points,
            angle_minimum,
            angle_maximum,
            self.scan_bin_count,
            self.scan_minimum_range,
            self.maximum_range,
        )
        message = LaserScan()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self.scan_frame
        message.angle_min = angle_minimum
        message.angle_increment = angle_increment
        message.angle_max = angle_minimum + angle_increment * (
            self.scan_bin_count - 1)
        message.time_increment = 0.0
        message.scan_time = 1.0 / self.scan_rate
        message.range_min = self.scan_minimum_range
        message.range_max = self.maximum_range
        message.ranges = ranges.astype(np.float32).tolist()
        self.scan_publisher.publish(message)

    def _publish_pose(self, source, inlier_rms):
        map_to_odom = None
        if self.publish_map_to_odom:
            odom_to_base = self.transform_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time.from_msg(source.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
            translation = odom_to_base.transform.translation
            odom_pose = np.asarray([
                translation.x,
                translation.y,
                _yaw_from_quaternion(odom_to_base.transform.rotation),
            ])
            map_to_odom = map_to_odom_pose(self.pose, odom_pose)
        message = PoseWithCovarianceStamped()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = float(self.pose[0])
        message.pose.pose.position.y = float(self.pose[1])
        message.pose.pose.orientation.z = math.sin(0.5 * self.pose[2])
        message.pose.pose.orientation.w = math.cos(0.5 * self.pose[2])
        position_variance = max(inlier_rms * inlier_rms, 2.5e-5)
        yaw_variance = max(
            (0.5 * inlier_rms / self.wall_half_extent) ** 2,
            math.radians(0.1) ** 2,
        )
        covariance = [0.0] * 36
        covariance[0] = position_variance
        covariance[7] = position_variance
        covariance[14] = 1.0e6
        covariance[21] = 1.0e6
        covariance[28] = 1.0e6
        covariance[35] = yaw_variance
        message.pose.covariance = covariance
        self.pose_publisher.publish(message)

        localized_transform = TransformStamped()
        localized_transform.header = message.header
        localized_transform.child_frame_id = self.localized_frame
        localized_transform.transform.translation.x = float(self.pose[0])
        localized_transform.transform.translation.y = float(self.pose[1])
        localized_transform.transform.rotation.z = (
            message.pose.pose.orientation.z)
        localized_transform.transform.rotation.w = (
            message.pose.pose.orientation.w)
        self.transform_broadcaster.sendTransform(localized_transform)
        if map_to_odom is not None:
            self.map_to_odom = map_to_odom
            self._publish_map_to_odom()
        return map_to_odom is not None

    def _publish_map_to_odom(self):
        if self.map_to_odom is None:
            return
        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now()
            + Duration(seconds=self.map_to_odom_transform_tolerance)
        ).to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = float(self.map_to_odom[0])
        transform.transform.translation.y = float(self.map_to_odom[1])
        transform.transform.rotation.z = math.sin(
            0.5 * self.map_to_odom[2])
        transform.transform.rotation.w = math.cos(
            0.5 * self.map_to_odom[2])
        self.transform_broadcaster.sendTransform(transform)

    def _publish_status(
            self, valid, reason, point_count, robust_rms=math.inf,
            inlier_fraction=0.0, inlier_rms=math.inf,
            side_counts=(0, 0, 0, 0), pose=None,
            visible_wall_count=0, map_to_odom_published=False):
        if pose is None:
            pose = self.pose
        payload = {
            'valid': bool(valid),
            'reason': str(reason),
            'point_count': int(point_count),
            'robust_rms_m': (
                float(robust_rms) if math.isfinite(robust_rms) else None),
            'inlier_fraction': float(inlier_fraction),
            'inlier_rms_m': (
                float(inlier_rms) if math.isfinite(inlier_rms) else None),
            'wall_point_counts': [int(value) for value in side_counts],
            'visible_wall_count': int(visible_wall_count),
            'map_to_odom_published': bool(map_to_odom_published),
            'pose': {
                'x': float(pose[0]),
                'y': float(pose[1]),
                'yaw': float(pose[2]),
            },
        }
        message = String()
        message.data = json.dumps(
            payload, sort_keys=True, separators=(',', ':'))
        self.status_publisher.publish(message)
        if self.last_valid is None or self.last_valid != bool(valid):
            if valid:
                self.get_logger().info(
                    'MID360_SQUARE_LOCALIZATION=PASS')
            else:
                self.get_logger().warning(
                    f'MID360_SQUARE_LOCALIZATION=WAIT:{reason}')
            self.last_valid = bool(valid)


def main(args=None):
    """Run the Mid-360 square-wall localizer."""
    rclpy.init(args=args)
    node = Mid360SquareLocalizer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
