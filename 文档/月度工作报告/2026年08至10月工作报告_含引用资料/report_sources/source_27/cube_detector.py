#!/usr/bin/env python3
"""
绿色方块检测节点.

订阅 RealSense 的 RGB 和深度图像，检测绿色方块并发布其 3D 位置。
"""

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from tf2_ros import TransformException


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Mock 模式标志
        self.declare_parameter('use_mock_data', False)
        self.use_mock = self.get_parameter('use_mock_data').value
        # The depth image sees the nearest face.  The current tabletop block is
        # 30 mm tall, so move 15 mm along the camera ray to estimate its center.
        self.declare_parameter('cube_center_depth_offset', 0.015)
        self.cube_center_depth_offset = float(
            self.get_parameter('cube_center_depth_offset').value
        )
        self.declare_parameter('detection_period', 0.20)
        self.detection_period = float(
            self.get_parameter('detection_period').value
        )
        self.declare_parameter('max_rgb_depth_skew', 0.10)
        self.max_rgb_depth_skew = float(
            self.get_parameter('max_rgb_depth_skew').value
        )
        if not 0.10 <= self.detection_period <= 2.0:
            raise ValueError('detection_period must be in [0.10, 2.0] seconds')
        if not 0.0 < self.max_rgb_depth_skew <= 0.50:
            raise ValueError('max_rgb_depth_skew must be in (0, 0.50] seconds')
        self.camera_timeout_count = 0

        # 订阅相机话题
        self.color_sub = self.create_subscription(
            Image,
            '/realsense_camera/color/image',
            self.color_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/realsense_camera/depth/image',
            self.depth_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/realsense_camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.depth_camera_info_sub = self.create_subscription(
            CameraInfo,
            '/realsense_camera/depth/camera_info',
            self.depth_camera_info_callback,
            10
        )

        # 发布方块位置
        self.cube_pose_pub = self.create_publisher(
            PoseStamped,
            '/cube_pose',
            10
        )

        # 发布调试图像
        self.debug_image_pub = self.create_publisher(
            Image,
            '/cube_detection/debug_image',
            10
        )

        self.color_image = None
        self.color_stamp = None
        self.depth_image = None
        self.depth_stamp = None
        self.camera_info = None
        self.depth_camera_info = None
        self.last_processed_color_stamp_ns = None

        # Process close to the rendered camera rate while ensuring that each
        # color frame is consumed at most once.
        self.timer = self.create_timer(self.detection_period, self.detect_cube)

        if self.use_mock:
            self.get_logger().info(
                'Cube detector node started '
                '(MOCK MODE - will publish simulated cube pose)'
            )
        else:
            self.get_logger().info(
                'Cube detector node started '
                '(REAL MODE - waiting for camera data)'
            )

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.color_stamp = msg.header.stamp

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_stamp = msg.header.stamp

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f'Color camera info received: {msg.width}x{msg.height}')

    def depth_camera_info_callback(self, msg):
        if self.depth_camera_info is None:
            self.depth_camera_info = msg
            self.get_logger().info(f'Depth camera info received: {msg.width}x{msg.height}')

    @staticmethod
    def quaternion_to_matrix(x, y, z, w):
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm <= 1e-12:
            return np.eye(3)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        return np.array([
            [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w), 2.0*(x*z + y*w)],
            [2.0*(x*y + z*w), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)],
            [2.0*(x*z - y*w), 2.0*(y*z + x*w), 1.0 - 2.0*(x*x + y*y)],
        ])

    @staticmethod
    def matrix_to_quaternion(rotation):
        """Return an xyzw quaternion for a proper 3x3 rotation matrix."""
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = np.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (rotation[2, 1] - rotation[1, 2]) / scale
            y = (rotation[0, 2] - rotation[2, 0]) / scale
            z = (rotation[1, 0] - rotation[0, 1]) / scale
        else:
            diagonal = np.diag(rotation)
            index = int(np.argmax(diagonal))
            if index == 0:
                scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
                w = (rotation[2, 1] - rotation[1, 2]) / scale
                x = 0.25 * scale
                y = (rotation[0, 1] + rotation[1, 0]) / scale
                z = (rotation[0, 2] + rotation[2, 0]) / scale
            elif index == 1:
                scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
                w = (rotation[0, 2] - rotation[2, 0]) / scale
                x = (rotation[0, 1] + rotation[1, 0]) / scale
                y = 0.25 * scale
                z = (rotation[1, 2] + rotation[2, 1]) / scale
            else:
                scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
                w = (rotation[1, 0] - rotation[0, 1]) / scale
                x = (rotation[0, 2] + rotation[2, 0]) / scale
                y = (rotation[1, 2] + rotation[2, 1]) / scale
                z = 0.25 * scale
        quaternion = np.array([x, y, z, w], dtype=np.float64)
        quaternion /= np.linalg.norm(quaternion)
        return quaternion

    def estimate_object_orientation(
            self, contour, color_fx, color_fy, color_cx, color_cy,
            depth_fx, depth_fy, depth_cx, depth_cy):
        """Estimate the block's long-axis roll/yaw from real segmented depth."""
        contour_mask = np.zeros(self.color_image.shape[:2], dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
        color_v, color_u = np.nonzero(contour_mask)
        if color_u.size < 100:
            return None

        stride = max(1, color_u.size // 4000)
        color_u = color_u[::stride].astype(np.float64)
        color_v = color_v[::stride].astype(np.float64)
        ray_x = (color_u - color_cx) / color_fx
        ray_y = (color_v - color_cy) / color_fy
        depth_u = np.rint(depth_cx + ray_x * depth_fx).astype(np.int32)
        depth_v = np.rint(depth_cy + ray_y * depth_fy).astype(np.int32)
        inside = (
            (depth_u >= 0) & (depth_u < self.depth_image.shape[1]) &
            (depth_v >= 0) & (depth_v < self.depth_image.shape[0])
        )
        if np.count_nonzero(inside) < 100:
            return None
        ray_x, ray_y = ray_x[inside], ray_y[inside]
        depth_u, depth_v = depth_u[inside], depth_v[inside]
        depths = self.depth_image[depth_v, depth_u].astype(np.float64)
        valid = np.isfinite(depths) & (depths > 0.0)
        if np.count_nonzero(valid) < 100:
            return None
        ray_x, ray_y, depths = ray_x[valid], ray_y[valid], depths[valid]
        if np.median(depths) > 10.0:
            depths *= 0.001
        median_depth = np.median(depths)
        on_object = np.abs(depths - median_depth) <= 0.08
        if np.count_nonzero(on_object) < 100:
            return None
        ray_x, ray_y, depths = ray_x[on_object], ray_y[on_object], depths[on_object]
        points = np.column_stack((ray_x * depths, ray_y * depths, depths))
        centered = points - np.mean(points, axis=0)
        covariance = centered.T @ centered / max(1, points.shape[0] - 1)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[order]
        eigenvectors = eigenvectors[:, order]
        if eigenvalues[1] <= 1e-8:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'realsense_camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Cannot estimate cube orientation without camera/world TF: {ex}',
                throttle_duration_sec=2.0
            )
            return None
        rotation = transform.transform.rotation
        world_from_camera = self.quaternion_to_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )

        # The 100 mm object axis is nominally world +Y.  Select whichever of
        # the two broad PCA axes matches that task axis; this avoids swapping
        # the similar 80 and 100 mm dimensions under partial hand occlusion.
        candidate_world_axes = [
            world_from_camera @ eigenvectors[:, index] for index in (0, 1)
        ]
        scores = [abs(axis[1]) for axis in candidate_world_axes]
        selected = int(np.argmax(scores))
        object_y_world = candidate_world_axes[selected]
        object_y_world /= np.linalg.norm(object_y_world)
        if scores[selected] < 0.65:
            return None
        if object_y_world[1] < 0.0:
            object_y_world *= -1.0

        # The task permits roll/yaw correction while retaining a horizontal
        # X axis, which is observable and avoids inventing pitch from a partly
        # occluded face.  Columns are the block X/Y/Z axes in world.
        world_up = np.array([0.0, 0.0, 1.0])
        object_x_world = np.cross(object_y_world, world_up)
        object_x_world /= np.linalg.norm(object_x_world)
        object_z_world = np.cross(object_x_world, object_y_world)
        object_z_world /= np.linalg.norm(object_z_world)
        world_from_object = np.column_stack(
            (object_x_world, object_y_world, object_z_world)
        )
        camera_from_object = world_from_camera.T @ world_from_object
        quaternion = self.matrix_to_quaternion(camera_from_object)
        roll = np.degrees(np.arctan2(object_y_world[2], object_y_world[1]))
        yaw = np.degrees(np.arctan2(-object_y_world[0], object_y_world[1]))
        return quaternion, roll, yaw, points.shape[0], scores[selected]

    def detect_cube(self):
        """检测绿色方块并发布位置."""
        # Mock 模式：直接发布已知的方块位置
        if self.use_mock:
            if (self.color_image is None or self.depth_image is None or
                    self.camera_info is None or self.depth_camera_info is None):
                self.camera_timeout_count += 1
                if self.camera_timeout_count >= 3:
                    # 3秒后相机数据仍未到达，发布 mock 数据
                    self.publish_mock_cube_pose()
                    return
                else:
                    self.get_logger().warn(
                        'Waiting for camera data '
                        '(will use mock data if not available)...',
                        throttle_duration_sec=5.0
                    )
                    return

        # 真实模式：必须有相机数据才能继续
        if (self.color_image is None or self.depth_image is None or
                self.color_stamp is None or self.depth_stamp is None or
                self.camera_info is None or self.depth_camera_info is None):
            self.get_logger().warn('Waiting for camera data...', throttle_duration_sec=5.0)
            return

        color_stamp_ns = (
            int(self.color_stamp.sec) * 1_000_000_000
            + int(self.color_stamp.nanosec)
        )
        depth_stamp_ns = (
            int(self.depth_stamp.sec) * 1_000_000_000
            + int(self.depth_stamp.nanosec)
        )
        if color_stamp_ns == self.last_processed_color_stamp_ns:
            return

        frame_skew = abs(color_stamp_ns - depth_stamp_ns) / 1_000_000_000.0
        if frame_skew > self.max_rgb_depth_skew:
            self.get_logger().warn(
                f'RGB/depth frame skew {frame_skew * 1000.0:.1f}ms exceeds '
                f'{self.max_rgb_depth_skew * 1000.0:.1f}ms',
                throttle_duration_sec=2.0
            )
            return
        self.last_processed_color_stamp_ns = color_stamp_ns

        # 转换到 HSV 色彩空间
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)

        # 绿色范围（需要根据实际情况调整）
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        # 创建掩码
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            self.get_logger().warn('No green object detected', throttle_duration_sec=5.0)
            if self.use_mock:
                self.publish_mock_cube_pose()
            return

        # 找到最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area < 100:  # 过滤小区域
            if self.use_mock:
                self.publish_mock_cube_pose()
            return

        # Use the contour centroid at sub-pixel precision instead of the
        # integer bounding-box center; one rendered pixel is several mm at the
        # tabletop and directly affects TCP centering.
        x, y, w, h = cv2.boundingRect(largest_contour)
        moments = cv2.moments(largest_contour)
        if moments['m00'] > 0.0:
            cx = moments['m10'] / moments['m00']
            cy = moments['m01'] / moments['m00']
        else:
            cx = x + w / 2.0
            cy = y + h / 2.0

        # D435 的 RGB 与深度视场不同（本模型分别约 69° 与 87°）。两幅图虽然
        # 已共光心、共方向，但相同像素编号不是同一条射线。利用两套内参把
        # RGB 中心映射到对应深度像素，再取邻域中值。
        color_fx = self.camera_info.k[0]
        color_fy = self.camera_info.k[4]
        color_cx = self.camera_info.k[2]
        color_cy = self.camera_info.k[5]
        depth_fx = self.depth_camera_info.k[0]
        depth_fy = self.depth_camera_info.k[4]
        depth_cx = self.depth_camera_info.k[2]
        depth_cy = self.depth_camera_info.k[5]

        depth_u = int(round(depth_cx + (cx - color_cx) * depth_fx / color_fx))
        depth_v = int(round(depth_cy + (cy - color_cy) * depth_fy / color_fy))
        depth_roi = self.depth_image[
            max(0, depth_v-5):min(self.depth_image.shape[0], depth_v+5),
            max(0, depth_u-5):min(self.depth_image.shape[1], depth_u+5)
        ]

        if depth_roi.size == 0:
            if self.use_mock:
                self.publish_mock_cube_pose()
            return

        valid_depth = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]
        if valid_depth.size == 0:
            self.get_logger().warn('Invalid depth value', throttle_duration_sec=5.0)
            if self.use_mock:
                self.publish_mock_cube_pose()
            return

        depth = np.median(valid_depth)

        if np.isnan(depth) or depth <= 0:
            self.get_logger().warn('Invalid depth value', throttle_duration_sec=5.0)
            if self.use_mock:
                self.publish_mock_cube_pose()
            return

        # 相机内参
        fx = color_fx
        fy = color_fy
        cx_cam = color_cx
        cy_cam = color_cy

        # 像素坐标转换为相机坐标系
        # The rendered depth is the visible object surface. Shift the point
        # along the camera ray so the published pose represents its center.
        z = float(depth) + self.cube_center_depth_offset
        x_cam = float((cx - cx_cam) * z / fx)
        y_cam = float((cy - cy_cam) * z / fy)

        orientation_estimate = self.estimate_object_orientation(
            largest_contour,
            color_fx, color_fy, color_cx, color_cy,
            depth_fx, depth_fy, depth_cx, depth_cy
        )
        if orientation_estimate is None:
            self.get_logger().warn(
                'Green block position found, but real-depth orientation is not reliable',
                throttle_duration_sec=2.0
            )
            return
        orientation, roll, yaw, orientation_points, orientation_score = orientation_estimate

        # 发布方块位置和真实深度估计姿态（相机坐标系）
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.color_stamp
        pose_msg.header.frame_id = 'realsense_camera_depth_optical_frame'
        pose_msg.pose.position.x = x_cam
        pose_msg.pose.position.y = y_cam
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = float(orientation[0])
        pose_msg.pose.orientation.y = float(orientation[1])
        pose_msg.pose.orientation.z = float(orientation[2])
        pose_msg.pose.orientation.w = float(orientation[3])

        self.cube_pose_pub.publish(pose_msg)

        # 确认相机到世界坐标系的 TF 可用
        try:
            self.tf_buffer.lookup_transform(
                'world',
                'realsense_camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            self.get_logger().info(
                f'Cube detected at camera frame: ({x_cam:.3f}, {y_cam:.3f}, {z:.3f}), '
                f'world roll/yaw=({roll:.1f}, {yaw:.1f}) deg from '
                f'{orientation_points} depth points (axis score {orientation_score:.2f})',
                throttle_duration_sec=2.0
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform to world frame: {ex}',
                throttle_duration_sec=5.0
            )

        # 发布调试图像
        debug_img = self.color_image.copy()
        cv2.rectangle(debug_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.circle(
            debug_img,
            (int(round(cx)), int(round(cy))),
            5,
            (0, 0, 255),
            -1
        )
        cv2.putText(
            debug_img,
            f'Depth: {z:.2f}m  roll: {roll:.1f}deg',
            (x, y-10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        self.debug_image_pub.publish(debug_msg)

    def publish_mock_cube_pose(self):
        """发布模拟的方块位置（基于 Gazebo 中的实际位置）."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'

        # Gazebo 中绿色方块的实际位置
        pose_msg.pose.position.x = 0.30
        pose_msg.pose.position.y = -0.25
        pose_msg.pose.position.z = 0.890
        pose_msg.pose.orientation.w = 1.0

        self.cube_pose_pub.publish(pose_msg)

        self.get_logger().info(
            '[MOCK] Publishing cube pose at world frame: '
            '(0.300, -0.250, 0.890)',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()

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
