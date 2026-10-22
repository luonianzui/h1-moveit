"""Synchronized, fail-closed AprilTag bundle localizer for the torso D435."""

from __future__ import annotations

from collections import defaultdict, deque
import json
import math
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovariance
from h1_2_perception_msgs.msg import TagObservation, TrackedObjectState
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException

from .tag_config import load_tag_map
from .tag_geometry import (
    matrix_quaternion,
    project_points,
    rotation_distance,
    solve_square_tag,
    transform_from_ros,
    weighted_transform_mean,
)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def duration_message(seconds: float):
    result = Duration(seconds=max(0.0, seconds)).to_msg()
    return result


class TagLocalizer(Node):
    def __init__(self):
        super().__init__('h1_2_tag_localizer')
        vision_share = Path(get_package_share_directory('h1_2_vision'))
        description_share = Path(get_package_share_directory('h1_2_description'))
        self.declare_parameter('tag_map', str(vision_share / 'config/tag_map.yaml'))
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('synchronizer_queue', 20)
        tag_map_path = Path(self.get_parameter('tag_map').value)
        self.config = load_tag_map(tag_map_path, [
            description_share / 'models/incubator/materials/textures',
            description_share / 'models/green_cube/materials/textures',
        ])
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.settings = self.config['stability']
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.history = defaultdict(lambda: deque(
            maxlen=int(self.settings['consecutive_frames'])))
        self.consecutive = defaultdict(int)
        self.last_stamp = {}
        self.frame_lock = threading.Lock()
        self.frame_queue = deque(maxlen=max(
            8, int(self.get_parameter('synchronizer_queue').value)))

        self.observation_pub = self.create_publisher(
            TagObservation, '/apriltag/detections', 20)
        self.object_publishers = {
            name: self.create_publisher(
                TrackedObjectState, f'/tag_pose/{name}', 10)
            for name in ('cabinet', 'door', 'cube')
        }
        self.status_pub = self.create_publisher(String, '/apriltag/status', 10)
        self.debug_pub = self.create_publisher(
            Image, '/apriltag/debug_image', qos_profile_sensor_data)

        queue = int(self.get_parameter('synchronizer_queue').value)
        self.color_sub = message_filters.Subscriber(
            self, Image, '/realsense_camera/color/image',
            qos_profile=qos_profile_sensor_data)
        self.color_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/realsense_camera/color/camera_info',
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/realsense_camera/depth/image',
            qos_profile=qos_profile_sensor_data)
        self.depth_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/realsense_camera/depth/camera_info',
            qos_profile=qos_profile_sensor_data)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.color_info_sub, self.depth_sub, self.depth_info_sub],
            queue_size=queue,
            slop=float(self.settings['rgb_depth_max_skew_s']),
            allow_headerless=False)
        self.synchronizer.registerCallback(self.synchronized_callback)
        self.process_timer = self.create_timer(0.02, self.process_ready_frame)
        self.get_logger().info(
            f'tag36h11 localizer ready; config={self.config["config_hash"]}')

    @staticmethod
    def camera_matrix(message: CameraInfo) -> np.ndarray:
        return np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    @staticmethod
    def distortion(message: CameraInfo) -> np.ndarray:
        return np.asarray(message.d, dtype=np.float64)

    @staticmethod
    def fill_pose(target: PoseWithCovariance, value: np.ndarray, sigma_p, sigma_r):
        target.pose.position.x, target.pose.position.y, target.pose.position.z = value[:3, 3]
        quaternion = matrix_quaternion(value[:3, :3])
        target.pose.orientation.x, target.pose.orientation.y = quaternion[:2]
        target.pose.orientation.z, target.pose.orientation.w = quaternion[2:]
        covariance = np.zeros((6, 6), dtype=np.float64)
        covariance[:3, :3] = np.eye(3) * sigma_p ** 2
        covariance[3:, 3:] = np.eye(3) * sigma_r ** 2
        target.covariance = covariance.reshape(-1).tolist()

    def lookup(self, target: str, source: str, stamp):
        return transform_from_ros(self.tf_buffer.lookup_transform(
            target, source, Time.from_msg(stamp),
            timeout=Duration(seconds=0.0)))

    def depth_check(self, depth, depth_info, color_frame, stamp,
                    color_from_tag, object_points):
        depth_frame = depth_info.header.frame_id
        depth_from_color = self.lookup(depth_frame, color_frame, stamp)
        depth_from_tag = depth_from_color @ color_from_tag
        points = (depth_from_tag[:3, :3] @ object_points.T).T + depth_from_tag[:3, 3]
        pixels = project_points(points, self.camera_matrix(depth_info))
        if pixels is None:
            return math.inf, 0
        polygon = np.rint(pixels).astype(np.int32)
        mask = np.zeros(depth.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, polygon, 255)
        values = np.asarray(depth[mask != 0], dtype=np.float64)
        values = values[np.isfinite(values) & (values > 0.0)]
        if values.size and float(np.median(values)) > 10.0:
            values *= 0.001
        if values.size < int(self.settings['minimum_depth_samples']):
            return math.inf, int(values.size)
        observed = float(np.median(values))
        expected = float(np.median(points[:, 2]))
        return abs(observed - expected), int(values.size)

    def stable(self, object_name, value, stamp):
        current_stamp = stamp_ns(stamp)
        if self.last_stamp.get(object_name) == current_stamp:
            return False, 0
        self.last_stamp[object_name] = current_stamp
        history = self.history[object_name]
        history.append(value.copy())
        required = int(self.settings['consecutive_frames'])
        if len(history) < required:
            self.consecutive[object_name] = len(history)
            return False, len(history)
        first = history[0]
        maximum_translation = max(
            float(np.linalg.norm(sample[:3, 3] - first[:3, 3])) for sample in history)
        maximum_rotation = max(rotation_distance(first, sample) for sample in history)
        object_settings = self.config['objects'][object_name]
        accepted = (
            maximum_translation <= float(object_settings['stable_translation_m']) and
            maximum_rotation <= float(object_settings['stable_rotation_rad']))
        self.consecutive[object_name] = (
            self.consecutive[object_name] + 1 if accepted else 0)
        if not accepted:
            history.clear()
            history.append(value.copy())
        return accepted, self.consecutive[object_name]

    def synchronized_callback(self, color_msg, color_info, depth_msg, depth_info):
        stamps = [stamp_ns(value.header.stamp) for value in
                  (color_msg, color_info, depth_msg, depth_info)]
        info_limit = int(float(self.settings['rgb_info_max_skew_s']) * 1e9)
        if (abs(stamps[0] - stamps[1]) > info_limit or
                abs(stamps[2] - stamps[3]) > info_limit or
                max(stamps) - min(stamps) > int(
                    float(self.settings['rgb_depth_max_skew_s']) * 1e9)):
            self.publish_status(color_msg.header, {}, 'SYNC_SKEW')
            return
        with self.frame_lock:
            self.frame_queue.append(
                (color_msg, color_info, depth_msg, depth_info))
            newest = stamp_ns(color_msg.header.stamp)
            maximum_age_ns = int(
                float(self.settings['tf_queue_max_age_s']) * 1e9)
            while (self.frame_queue and newest - stamp_ns(
                    self.frame_queue[0][0].header.stamp) > maximum_age_ns):
                self.frame_queue.popleft()

    def process_ready_frame(self):
        selected = None
        with self.frame_lock:
            for candidate in reversed(self.frame_queue):
                color_msg, _, _, depth_info = candidate
                stamp = Time.from_msg(color_msg.header.stamp)
                ready = (
                    self.tf_buffer.can_transform(
                        self.world_frame, color_msg.header.frame_id, stamp,
                        timeout=Duration(seconds=0.0)) and
                    self.tf_buffer.can_transform(
                        depth_info.header.frame_id, color_msg.header.frame_id,
                        stamp, timeout=Duration(seconds=0.0)))
                if ready:
                    selected = candidate
                    break
            if selected is not None:
                while self.frame_queue:
                    value = self.frame_queue.popleft()
                    if value is selected:
                        break
        if selected is not None:
            self.process_synchronized(*selected)

    def process_synchronized(self, color_msg, color_info, depth_msg, depth_info):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as error:
            self.publish_status(color_msg.header, {}, f'CV_BRIDGE:{error}')
            return
        camera = self.camera_matrix(color_info)
        distortion = self.distortion(color_info)
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_parameters)
        del rejected
        accepted = defaultdict(list)
        debug_corners, debug_ids = [], []
        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.reshape(-1)):
                record = self.config['tags_by_id'].get(int(marker_id))
                if record is None:
                    continue
                solved = solve_square_tag(
                    marker_corners, record['size_m'], camera, distortion)
                if solved is None:
                    continue
                color_from_tag, object_points, reprojection = solved
                try:
                    residual, samples = self.depth_check(
                        depth, depth_info, color_msg.header.frame_id,
                        color_msg.header.stamp, color_from_tag, object_points)
                    world_from_color = self.lookup(
                        self.world_frame, color_msg.header.frame_id,
                        color_msg.header.stamp)
                except TransformException as error:
                    self.get_logger().warning(
                        f'exact-stamp TF rejected tag {marker_id}: {error}',
                        throttle_duration_sec=2.0)
                    continue
                valid = (
                    reprojection <= float(self.settings['maximum_reprojection_error_px']) and
                    residual <= float(self.settings['maximum_depth_residual_m']) and
                    samples >= int(self.settings['minimum_depth_samples']))
                world_from_tag = world_from_color @ color_from_tag
                world_from_object = world_from_tag @ np.linalg.inv(
                    record['object_from_tag'])
                sigma_p = max(0.0005, 0.0005 * reprojection + 0.25 * residual)
                sigma_r = max(math.radians(0.15), math.radians(0.2) * reprojection)
                observation = TagObservation()
                observation.header = color_msg.header
                observation.id = int(marker_id)
                observation.family = self.config['family']
                observation.object_name = record['object_name']
                observation.camera_frame = color_msg.header.frame_id
                observation.world_frame = self.world_frame
                self.fill_pose(observation.pose_camera, color_from_tag, sigma_p, sigma_r)
                self.fill_pose(observation.pose_world, world_from_tag, sigma_p, sigma_r)
                observation.tag_size_m = record['size_m']
                observation.reprojection_error_px = reprojection
                observation.depth_residual_m = residual
                observation.depth_samples = samples
                observation.consecutive_frames = self.consecutive[record['object_name']]
                observation.observation_age = duration_message(0.0)
                observation.config_hash = self.config['config_hash']
                observation.valid = valid
                observation.status = 'VALID' if valid else 'QUALITY_GATE'
                self.observation_pub.publish(observation)
                if valid:
                    weight = 1.0 / max(1e-12, sigma_p * sigma_p)
                    accepted[record['object_name']].append((
                        world_from_object, weight, int(marker_id), reprojection,
                        residual, sigma_p, sigma_r))
                    debug_corners.append(marker_corners)
                    debug_ids.append([int(marker_id)])

        results = {}
        for object_name, values in accepted.items():
            fused = weighted_transform_mean(
                [value[0] for value in values], [value[1] for value in values])
            # Apply the versioned object-frame optical correction only after
            # bundle fusion. The correction is part of the tag-map hash.
            fused = fused @ self.config['objects'][object_name][
                'object_pose_correction']
            is_stable, consecutive = self.stable(
                object_name, fused, color_msg.header.stamp)
            message = TrackedObjectState()
            message.header.stamp = color_msg.header.stamp
            message.header.frame_id = self.world_frame
            message.object_name = object_name
            message.object_frame = self.config['objects'][object_name]['frame']
            sigma_p = max(value[5] for value in values)
            sigma_r = max(value[6] for value in values)
            self.fill_pose(message.pose_world, fused, sigma_p, sigma_r)
            message.contributing_tag_ids = sorted(value[2] for value in values)
            message.consecutive_frames = consecutive
            message.observation_age = duration_message(0.0)
            message.maximum_reprojection_error_px = max(value[3] for value in values)
            message.maximum_depth_residual_m = max(value[4] for value in values)
            message.source_mode = 'TAG_BUNDLE' if len(values) > 1 else 'SINGLE_TAG'
            message.config_hash = self.config['config_hash']
            message.stable = is_stable
            message.valid = True
            message.status = 'STABLE' if is_stable else 'STABILIZING'
            self.object_publishers[object_name].publish(message)
            results[object_name] = message.status
        self.publish_status(color_msg.header, results, 'OK')
        if bool(self.get_parameter('publish_debug_image').value):
            if debug_corners:
                cv2.aruco.drawDetectedMarkers(
                    color, debug_corners, np.asarray(debug_ids, dtype=np.int32))
            debug = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
            debug.header = color_msg.header
            self.debug_pub.publish(debug)

    def publish_status(self, header, objects, status):
        message = String()
        message.data = json.dumps({
            'stamp_ns': stamp_ns(header.stamp),
            'frame_id': header.frame_id,
            'config_hash': self.config['config_hash'],
            'status': status,
            'objects': objects,
        }, sort_keys=True, separators=(',', ':'))
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
