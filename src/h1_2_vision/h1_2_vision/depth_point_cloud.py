#!/usr/bin/env python3

"""Publish a bounded, downsampled PointCloud2 from the simulated D435 depth."""

from pathlib import Path
import struct

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from h1_2_perception_msgs.msg import TrackedObjectState
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from .tag_config import load_tag_map


def _points_inside_euclidean_aabb_margin(points, minimum, maximum, margin):
    """Return the rounded-AABB mask without enlarging its corners."""
    below = np.maximum(minimum - points, 0.0)
    above = np.maximum(points - maximum, 0.0)
    outside = np.maximum(below, above)
    squared_distance = np.einsum('ij,ij->i', outside, outside)
    return squared_distance <= margin * margin + np.finfo(np.float64).eps


def _load_binary_stl_triangles(path):
    """Load finite, non-degenerate triangles from the shipped binary STL."""
    payload = Path(path).read_bytes()
    if len(payload) < 84:
        raise ValueError(f'binary STL is truncated: {path}')
    triangle_count = struct.unpack_from('<I', payload, 80)[0]
    expected_size = 84 + triangle_count * 50
    if triangle_count == 0 or len(payload) != expected_size:
        raise ValueError(f'binary STL has an invalid triangle table: {path}')
    record_dtype = np.dtype([
        ('normal', '<f4', (3,)),
        ('vertices', '<f4', (3, 3)),
        ('attribute', '<u2'),
    ])
    records = np.frombuffer(
        payload, dtype=record_dtype, count=triangle_count, offset=84)
    triangles = np.array(records['vertices'], dtype=np.float64, copy=True)
    if not np.isfinite(triangles).all():
        raise ValueError(f'binary STL contains non-finite vertices: {path}')
    cross = np.cross(
        triangles[:, 1] - triangles[:, 0],
        triangles[:, 2] - triangles[:, 0])
    non_degenerate = np.einsum('ij,ij->i', cross, cross) > 1.0e-24
    triangles = triangles[non_degenerate]
    if not len(triangles):
        raise ValueError(f'binary STL contains no usable triangles: {path}')
    extent = np.ptp(triangles, axis=(0, 1))
    if np.any(extent <= 0.0) or np.any(extent > 1.0):
        raise ValueError(f'binary STL dimensions are invalid: {path}')
    return triangles


def _point_triangle_squared_distances(point, triangles):
    """Return exact squared distances from one point to many triangles."""
    first = triangles[:, 0]
    first_edge = triangles[:, 1] - first
    second_edge = triangles[:, 2] - first
    relative = point - first
    dot_00 = np.einsum('ij,ij->i', first_edge, first_edge)
    dot_01 = np.einsum('ij,ij->i', first_edge, second_edge)
    dot_11 = np.einsum('ij,ij->i', second_edge, second_edge)
    dot_20 = np.einsum('ij,ij->i', relative, first_edge)
    dot_21 = np.einsum('ij,ij->i', relative, second_edge)
    denominator = dot_00 * dot_11 - dot_01 * dot_01
    denominator = np.maximum(denominator, np.finfo(np.float64).tiny)
    first_fraction = (dot_11 * dot_20 - dot_01 * dot_21) / denominator
    second_fraction = (dot_00 * dot_21 - dot_01 * dot_20) / denominator
    projected = (
        first + first_fraction[:, None] * first_edge +
        second_fraction[:, None] * second_edge)
    projected_distance = np.einsum(
        'ij,ij->i', point - projected, point - projected)
    inside_projection = (
        (first_fraction >= 0.0) & (second_fraction >= 0.0) &
        (first_fraction + second_fraction <= 1.0))
    projected_distance[~inside_projection] = np.inf

    edge_distances = []
    for start, end in (
            (triangles[:, 0], triangles[:, 1]),
            (triangles[:, 1], triangles[:, 2]),
            (triangles[:, 2], triangles[:, 0])):
        edge = end - start
        length_squared = np.einsum('ij,ij->i', edge, edge)
        fraction = np.einsum('ij,ij->i', point - start, edge)
        fraction /= np.maximum(length_squared, np.finfo(np.float64).tiny)
        fraction = np.clip(fraction, 0.0, 1.0)
        nearest = start + fraction[:, None] * edge
        delta = point - nearest
        edge_distances.append(np.einsum('ij,ij->i', delta, delta))
    return np.minimum(projected_distance, np.minimum.reduce(edge_distances))


def _build_triangle_surface_index(triangles, margin, cell_size=0.010):
    """Index expanded triangle bounds without approximating final distance."""
    if not np.isfinite(cell_size) or cell_size <= 0.0:
        raise ValueError('triangle surface index cell size must be positive')
    origin = np.min(triangles, axis=(0, 1)) - margin - 1.0e-12
    cells = {}
    triangle_minimum = np.min(triangles, axis=1) - margin
    triangle_maximum = np.max(triangles, axis=1) + margin
    lower = np.floor((triangle_minimum - origin) / cell_size).astype(int)
    upper = np.floor((triangle_maximum - origin) / cell_size).astype(int)
    for triangle_index, (first, last) in enumerate(zip(lower, upper)):
        for x_index in range(first[0], last[0] + 1):
            for y_index in range(first[1], last[1] + 1):
                for z_index in range(first[2], last[2] + 1):
                    cells.setdefault(
                        (x_index, y_index, z_index), []).append(triangle_index)
    cells = {
        key: np.asarray(value, dtype=np.int32)
        for key, value in cells.items()
    }
    return origin, float(cell_size), cells


def _points_within_triangle_surface_margin(
        points, triangles, minimum, maximum, margin, surface_index=None):
    """Select points no farther than margin from the exact triangle surface."""
    near_bounds = _points_inside_euclidean_aabb_margin(
        points, minimum, maximum, margin)
    selected = np.zeros(points.shape[0], dtype=bool)
    threshold = margin * margin + np.finfo(np.float64).eps
    for index in np.flatnonzero(near_bounds):
        candidates = triangles
        if surface_index is not None:
            origin, cell_size, cells = surface_index
            cell = tuple(np.floor(
                (points[index] - origin) / cell_size).astype(int))
            triangle_indices = cells.get(cell)
            if triangle_indices is None:
                continue
            candidates = triangles[triangle_indices]
        distances = _point_triangle_squared_distances(
            np.asarray(points[index], dtype=np.float64), candidates)
        selected[index] = bool(np.min(distances) <= threshold)
    return selected


class DepthPointCloud(Node):
    """Convert aligned D435 depth pixels into an XYZ point cloud."""

    def __init__(self):
        super().__init__('h1_2_depth_point_cloud')
        self.declare_parameter('pixel_stride', 4)
        self.declare_parameter('minimum_range', 0.15)
        self.declare_parameter('maximum_range', 2.0)
        self.declare_parameter('maximum_update_rate', 5.0)
        self.declare_parameter(
            'depth_image_topic', '/realsense_camera/depth/image')
        self.declare_parameter(
            'camera_info_topic', '/realsense_camera/depth/camera_info')
        self.declare_parameter(
            'point_cloud_topic', '/realsense_camera/depth/points')
        self.declare_parameter('target_pose_topic', '/tag_pose/cube')
        self.declare_parameter('target_size_x', 0.080)
        self.declare_parameter('target_size_y', 0.160)
        self.declare_parameter('target_size_z', 0.030)
        self.declare_parameter('target_filter_margin', 0.005)
        self.declare_parameter('known_scene_filter_enabled', False)
        self.declare_parameter('known_scene_frame', 'pelvis')
        self.declare_parameter('known_scene_filter_margin', 0.005)
        self.declare_parameter('incubator_x', 0.590)
        self.declare_parameter('dynamic_door_filter_enabled', False)
        self.declare_parameter(
            'dynamic_door_joint_topic', '/incubator/joint_states')
        self.declare_parameter('dynamic_door_filter_margin', 0.005)
        self.declare_parameter('dynamic_door_state_max_age', 2.0)
        self.declare_parameter('dynamic_handle_filter_enabled', False)
        self.declare_parameter(
            'dynamic_handle_joint_topic', '/incubator/joint_states')
        self.declare_parameter('dynamic_handle_filter_margin', 0.007)
        self.declare_parameter('dynamic_handle_tip_extension', 0.003)
        self.declare_parameter('dynamic_handle_state_max_age', 2.0)
        self.declare_parameter('left_finger2_self_filter_enabled', False)
        self.declare_parameter('wrist_visual_self_filter_enabled', False)
        self.declare_parameter('wrist_visual_self_filter_margin', 0.0)
        self.declare_parameter(
            'right_finger2_slider_self_filter_enabled', False)
        self.declare_parameter(
            'right_finger2_slider_self_filter_margin', 0.0)
        self.declare_parameter('support_surface_filter_enabled', False)
        self.declare_parameter('support_surface_frame', 'pelvis')
        self.declare_parameter('support_surface_ground_z', -1.000)
        self.declare_parameter('support_surface_half_thickness', 0.002)
        self.declare_parameter('support_surface_x_min', -0.095)
        self.declare_parameter('support_surface_x_max', 0.195)
        self.declare_parameter('support_surface_left_y_min', 0.104)
        self.declare_parameter('support_surface_left_y_max', 0.220)
        self.declare_parameter('support_surface_right_y_min', -0.217)
        self.declare_parameter('support_surface_right_y_max', -0.104)
        # The arm occludes the table-supported target during the roughly
        # 14-second pregrasp-and-descent sequence.  Retain the last recognized
        # target pose across that short occlusion so the target itself cannot be
        # reinserted into OctoMap; the filtered volume remains tightly bound to
        # the measured 80 x 160 x 30 mm object plus its 5 mm perception margin.
        self.declare_parameter('target_pose_max_age', 20.0)

        self._stride = int(self.get_parameter('pixel_stride').value)
        self._minimum_range = float(self.get_parameter('minimum_range').value)
        self._maximum_range = float(self.get_parameter('maximum_range').value)
        self._maximum_update_rate = float(
            self.get_parameter('maximum_update_rate').value)
        self._target_size = np.array([
            float(self.get_parameter('target_size_x').value),
            float(self.get_parameter('target_size_y').value),
            float(self.get_parameter('target_size_z').value),
        ], dtype=np.float32)
        self._target_filter_margin = float(
            self.get_parameter('target_filter_margin').value)
        self._known_scene_filter_enabled = bool(
            self.get_parameter('known_scene_filter_enabled').value)
        self._known_scene_frame = str(
            self.get_parameter('known_scene_frame').value).lstrip('/')
        self._known_scene_filter_margin = float(
            self.get_parameter('known_scene_filter_margin').value)
        self._incubator_x = float(self.get_parameter('incubator_x').value)
        self._dynamic_door_filter_enabled = bool(
            self.get_parameter('dynamic_door_filter_enabled').value)
        self._dynamic_door_filter_margin = float(
            self.get_parameter('dynamic_door_filter_margin').value)
        self._dynamic_door_state_max_age_ns = int(
            float(self.get_parameter('dynamic_door_state_max_age').value) * 1e9)
        self._dynamic_handle_filter_enabled = bool(
            self.get_parameter('dynamic_handle_filter_enabled').value)
        self._dynamic_handle_filter_margin = float(
            self.get_parameter('dynamic_handle_filter_margin').value)
        self._dynamic_handle_tip_extension = float(
            self.get_parameter('dynamic_handle_tip_extension').value)
        self._dynamic_handle_state_max_age_ns = int(
            float(self.get_parameter('dynamic_handle_state_max_age').value) * 1e9)
        self._left_finger2_self_filter_enabled = bool(
            self.get_parameter('left_finger2_self_filter_enabled').value)
        self._wrist_visual_self_filter_enabled = bool(
            self.get_parameter('wrist_visual_self_filter_enabled').value)
        self._wrist_visual_self_filter_margin = float(
            self.get_parameter('wrist_visual_self_filter_margin').value)
        self._right_finger2_slider_self_filter_enabled = bool(
            self.get_parameter(
                'right_finger2_slider_self_filter_enabled').value)
        self._right_finger2_slider_self_filter_margin = float(
            self.get_parameter(
                'right_finger2_slider_self_filter_margin').value)
        self._support_surface_filter_enabled = bool(
            self.get_parameter('support_surface_filter_enabled').value)
        self._support_surface_frame = str(
            self.get_parameter('support_surface_frame').value).lstrip('/')
        self._support_surface_ground_z = float(
            self.get_parameter('support_surface_ground_z').value)
        self._support_surface_half_thickness = float(
            self.get_parameter('support_surface_half_thickness').value)
        self._support_surface_x = np.array([
            float(self.get_parameter('support_surface_x_min').value),
            float(self.get_parameter('support_surface_x_max').value),
        ], dtype=np.float32)
        self._support_surface_left_y = np.array([
            float(self.get_parameter('support_surface_left_y_min').value),
            float(self.get_parameter('support_surface_left_y_max').value),
        ], dtype=np.float32)
        self._support_surface_right_y = np.array([
            float(self.get_parameter('support_surface_right_y_min').value),
            float(self.get_parameter('support_surface_right_y_max').value),
        ], dtype=np.float32)
        self._target_pose_max_age_ns = int(
            float(self.get_parameter('target_pose_max_age').value) * 1e9)
        if not 1 <= self._stride <= 32:
            raise ValueError('pixel_stride must be in [1, 32]')
        if not 0.05 <= self._minimum_range < self._maximum_range <= 10.0:
            raise ValueError('depth range is invalid')
        if not 0.1 <= self._maximum_update_rate <= 30.0:
            raise ValueError('maximum_update_rate must be in [0.1, 30] Hz')
        if np.any(self._target_size <= 0.0) or np.any(self._target_size > 1.0):
            raise ValueError('target dimensions must be in (0, 1] m')
        if not 0.0 <= self._target_filter_margin <= 0.05:
            raise ValueError('target_filter_margin must be in [0, 0.05] m')
        if not self._known_scene_frame:
            raise ValueError('known_scene_frame must not be empty')
        if not 0.0 <= self._known_scene_filter_margin < 0.01:
            raise ValueError(
                'known_scene_filter_margin must be in [0, 0.01) m')
        if not np.isfinite(self._incubator_x):
            raise ValueError('incubator_x must be finite')
        if not 0.0 <= self._dynamic_door_filter_margin < 0.01:
            raise ValueError(
                'dynamic_door_filter_margin must be in [0, 0.01) m')
        if not 0.1e9 <= self._dynamic_door_state_max_age_ns <= 5.0e9:
            raise ValueError(
                'dynamic_door_state_max_age must be in [0.1, 5.0] s')
        if not 0.0 <= self._dynamic_handle_filter_margin < 0.01:
            raise ValueError(
                'dynamic_handle_filter_margin must be in [0, 0.01) m')
        if not 0.0 <= self._dynamic_handle_tip_extension <= 0.003:
            raise ValueError(
                'dynamic_handle_tip_extension must be in [0, 0.003] m')
        if not 0.1e9 <= self._dynamic_handle_state_max_age_ns <= 5.0e9:
            raise ValueError(
                'dynamic_handle_state_max_age must be in [0.1, 5.0] s')
        if (not np.isfinite(self._right_finger2_slider_self_filter_margin) or
                not 0.0 <=
                self._right_finger2_slider_self_filter_margin <= 0.005):
            raise ValueError(
                'right_finger2_slider_self_filter_margin must be finite and '
                'in [0, 0.005] m')
        if (not np.isfinite(self._wrist_visual_self_filter_margin) or
                not 0.0 <= self._wrist_visual_self_filter_margin <= 0.007):
            raise ValueError(
                'wrist_visual_self_filter_margin must be finite and '
                'in [0, 0.007] m')
        if not self._support_surface_frame:
            raise ValueError('support_surface_frame must not be empty')
        if (not np.isfinite(self._support_surface_ground_z) or
                not 0.0 < self._support_surface_half_thickness <= 0.005):
            raise ValueError(
                'support surface z must be finite and half-thickness in '
                '(0, 0.005] m')
        for name, bounds in (
                ('support_surface_x', self._support_surface_x),
                ('support_surface_left_y', self._support_surface_left_y),
                ('support_surface_right_y', self._support_surface_right_y)):
            if (not np.all(np.isfinite(bounds)) or bounds[0] >= bounds[1] or
                    bounds[1] - bounds[0] > 0.5):
                raise ValueError(f'{name} bounds are invalid')
        if not 0.05e9 <= self._target_pose_max_age_ns <= 30.0e9:
            raise ValueError('target_pose_max_age must be in [0.05, 30.0] s')

        self._minimum_period_ns = int(1e9 / self._maximum_update_rate)
        self._last_publish_stamp_ns = None
        self._camera_info = None
        self._target_pose = None
        self._dynamic_door_angle = None
        self._dynamic_door_stamp_ns = None
        self._dynamic_handle_angle = None
        self._dynamic_handle_door_angle = None
        self._dynamic_handle_stamp_ns = None
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        vision_share = Path(get_package_share_directory('h1_2_vision'))
        description_share = Path(
            get_package_share_directory('h1_2_description'))
        tag_config = load_tag_map(
            vision_share / 'config/tag_map.yaml', [
                description_share / 'models/incubator/materials/textures',
                description_share / 'models/green_cube/materials/textures',
            ])
        self._target_config_hash = tag_config['config_hash']
        self._known_scene_obbs = self._make_known_scene_obbs()
        self._dynamic_handle_pivot = np.array([
            0.330848293340 + (self._incubator_x - 0.490),
            0.129565720889,
            0.024999998514,
        ], dtype=np.float32)
        self._dynamic_handle_closed_rotation = self._rpy_matrix(
            0.0, 0.0, 1.613424341497)
        self._dynamic_handle_mesh_translation = np.array(
            [-0.009999668689, 0.000027284668, -0.119003896682],
            dtype=np.float32)
        self._dynamic_handle_mesh_fixed_rotation = self._rpy_matrix(
            0.0, 0.0, 0.0)
        self._dynamic_door_hinge = np.array([
            0.449954077853 + (self._incubator_x - 0.590),
            -0.247963042272,
            0.024999998514,
        ], dtype=np.float32)
        self._dynamic_door_closed_origin = np.array([
            0.455546220717 + (self._incubator_x - 0.590),
            0.160646463171,
            -0.165000000000,
        ], dtype=np.float32)
        self._dynamic_door_closed_rotation = self._rpy_matrix(
            1.570796326795, 0.0, -1.528168312093)
        self._dynamic_door_minimum = np.zeros(3, dtype=np.float32)
        self._dynamic_door_maximum = np.array(
            [0.410001, 0.380001, 0.035001], dtype=np.float32)
        self._dynamic_handle_mesh_minimum = np.array(
            [0.0, 0.0, 0.004112720490], dtype=np.float32)
        self._dynamic_handle_mesh_maximum = np.array(
            [0.020, 0.030, 0.133901916504], dtype=np.float32)
        self._dynamic_handle_mesh_maximum[2] += \
            self._dynamic_handle_tip_extension
        # Exact zero-margin AABB of the shipped Link2_2.STL.  This filter is
        # only a depth self-mask in the moving link frame; it does not add or
        # enlarge any PlanningScene collision geometry.
        self._left_finger2_minimum = np.array(
            [-0.013, -0.0068, 0.0], dtype=np.float32)
        self._left_finger2_maximum = np.array(
            [0.006, 0.075, 0.0304], dtype=np.float32)
        self._wrist_visual_surfaces = []
        if self._wrist_visual_self_filter_enabled:
            mesh_cache = {}
            for link_name, mesh_name in (
                    ('left_wrist_pitch_link',
                     'left_wrist_pitch_link.STL'),
                    ('left_wrist_yaw_link', 'wrist_yaw_link.STL'),
                    ('right_wrist_pitch_link',
                     'right_wrist_pitch_link.STL'),
                    ('right_wrist_yaw_link', 'wrist_yaw_link.STL')):
                if mesh_name not in mesh_cache:
                    triangles = _load_binary_stl_triangles(
                        description_share / 'meshes' / mesh_name)
                    minimum = np.min(triangles, axis=(0, 1))
                    maximum = np.max(triangles, axis=(0, 1))
                    surface_index = _build_triangle_surface_index(
                        triangles, self._wrist_visual_self_filter_margin)
                    mesh_cache[mesh_name] = (
                        triangles, minimum, maximum, surface_index)
                self._wrist_visual_surfaces.append(
                    (link_name, *mesh_cache[mesh_name]))
        # Exact AABB of the shipped Link2_1.STL on the right finger-2 slider.
        # Its separate, default-zero parameter may add at most 5.0 mm solely
        # as a Euclidean depth self-render/TF and 10-mm-OctoMap-voxel
        # tolerance.  A rounded Euclidean AABB is used so its corners do not
        # grow to 8.66 mm.  This changes no PlanningScene geometry, padding,
        # collision permission, or 10 mm safety gate.
        self._right_finger2_slider_minimum = np.array(
            [0.0, -0.005, -0.0065], dtype=np.float32)
        self._right_finger2_slider_maximum = np.array(
            [0.080, 0.009, 0.008], dtype=np.float32)
        self._reported_first_cloud = False
        self._reported_target_filter = False
        self._reported_target_tf_error = False
        self._reported_known_scene_filter = False
        self._reported_known_scene_tf_error = False
        self._reported_support_surface_filter = False
        self._reported_support_surface_tf_error = False
        self._reported_dynamic_door_filter = False
        self._reported_dynamic_door_stale = False
        self._reported_dynamic_handle_filter = False
        self._reported_dynamic_handle_stale = False
        self._reported_left_finger2_filter = False
        self._reported_wrist_visual_links = set()
        self._reported_wrist_visual_tf_errors = set()
        self._reported_right_finger2_slider_filter = False
        self._reported_right_finger2_slider_tf_error = False
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('point_cloud_topic').value),
            qos_profile_sensor_data)
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._camera_info_callback,
            qos_profile_sensor_data)
        self._depth_subscription = self.create_subscription(
            Image,
            str(self.get_parameter('depth_image_topic').value),
            self._depth_callback,
            qos_profile_sensor_data)
        self._target_pose_subscription = self.create_subscription(
            TrackedObjectState,
            str(self.get_parameter('target_pose_topic').value),
            self._target_pose_callback,
            10)
        self._dynamic_door_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter('dynamic_door_joint_topic').value),
            self._dynamic_door_callback,
            qos_profile_sensor_data)
        self._dynamic_handle_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter('dynamic_handle_joint_topic').value),
            self._dynamic_handle_callback,
            qos_profile_sensor_data)

    def _camera_info_callback(self, message):
        if message.width > 0 and message.height > 0:
            self._camera_info = message

    def _target_pose_callback(self, message):
        # Any malformed or wrong-version state invalidates the cached pose.
        # A simple absence of messages still permits the explicitly bounded
        # short-occlusion cache below.
        self._target_pose = None
        if (message.object_name != 'cube' or not message.valid or
                message.config_hash != self._target_config_hash or
                message.header.frame_id.lstrip('/') != 'world'):
            return
        self._target_pose = message

    def _dynamic_door_callback(self, message):
        # A malformed or incomplete feedback message invalidates the previous
        # pose immediately.  The cloud filter therefore fails open instead of
        # masking at a stale or assumed closed-door pose.
        self._dynamic_door_angle = None
        self._dynamic_door_stamp_ns = None
        if 'door_hinge_joint' not in message.name:
            return
        door_index = message.name.index('door_hinge_joint')
        if door_index >= len(message.position):
            return
        door_angle = float(message.position[door_index])
        if not np.isfinite(door_angle):
            return
        maximum_door_angle = 1.308996939
        endpoint_tolerance = 1.0e-6
        if (door_angle < -endpoint_tolerance or
                door_angle > maximum_door_angle + endpoint_tolerance):
            return
        self._dynamic_door_angle = float(
            np.clip(door_angle, 0.0, maximum_door_angle))
        self._dynamic_door_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000 +
            int(message.header.stamp.nanosec))

    def _dynamic_handle_callback(self, message):
        if ('handle_latch_joint' not in message.name or
                'door_hinge_joint' not in message.name):
            return
        handle_index = message.name.index('handle_latch_joint')
        door_index = message.name.index('door_hinge_joint')
        if (handle_index >= len(message.position) or
                door_index >= len(message.position)):
            return
        handle_angle = float(message.position[handle_index])
        door_angle = float(message.position[door_index])
        if not np.isfinite(handle_angle) or not np.isfinite(door_angle):
            return
        self._dynamic_handle_angle = float(np.clip(handle_angle, 0.0, np.pi))
        self._dynamic_handle_door_angle = float(
            np.clip(door_angle, 0.0, 1.308996939))
        self._dynamic_handle_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000 +
            int(message.header.stamp.nanosec))

    @staticmethod
    def _quaternion_matrix(orientation):
        quaternion = np.array([
            orientation.x, orientation.y,
            orientation.z, orientation.w,
        ], dtype=np.float64)
        norm = float(np.linalg.norm(quaternion))
        if not np.isfinite(norm) or norm < 1e-9:
            return None
        x, y, z, w = quaternion / norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z),
             2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w),
             1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w),
             2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float32)

    @staticmethod
    def _rpy_matrix(roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ], dtype=np.float32)
        ry = np.array([
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ], dtype=np.float32)
        rz = np.array([
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        return rz @ ry @ rx

    def _make_known_scene_obbs(self):
        """Return only stationary incubator OBBs in the planning frame."""
        delta_x = self._incubator_x - 0.590
        return [
            (
                np.array([
                    0.779890542898 + delta_x,
                    0.250472606667,
                    -0.185000000000,
                ], dtype=np.float32),
                self._rpy_matrix(1.570796326795, 0.0, -1.570796326795),
                np.array([0.0, 0.0, 0.0], dtype=np.float32),
                np.array([0.500000, 0.405001, 0.338001], dtype=np.float32),
            ),
            (
                np.array([
                    0.741890542898 + delta_x,
                    0.146472606667,
                    -0.024592942445,
                ], dtype=np.float32),
                self._rpy_matrix(1.570796326795, 0.0, -1.570796326795),
                np.array([0.0, 0.0, 0.0], dtype=np.float32),
                np.array([0.352001, 0.005001, 0.262001], dtype=np.float32),
            ),
        ]

    def _filter_known_scene(self, points, cloud_header):
        if not self._known_scene_filter_enabled or points.size == 0:
            return points
        cloud_frame = cloud_header.frame_id.lstrip('/')
        if not cloud_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self._known_scene_frame,
                cloud_frame,
                # The simulated camera is rigidly mounted to the torso.  Use
                # the latest complete TF sample: startup image stamps can
                # precede the first joint-state TF and would otherwise retain
                # every known-model point indefinitely.
                Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as error:
            if not self._reported_known_scene_tf_error:
                self.get_logger().warning(
                    f'Known-scene point filter is retaining all points until '
                    f'TF {cloud_frame}->{self._known_scene_frame} is ready: '
                    f'{error}')
                self._reported_known_scene_tf_error = True
            return points

        rotation = self._quaternion_matrix(transform.transform.rotation)
        if rotation is None:
            return points
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        scene_points = points @ rotation.T + translation
        margin = self._known_scene_filter_margin
        known = np.zeros(scene_points.shape[0], dtype=bool)
        for origin, obb_rotation, minimum, maximum in self._known_scene_obbs:
            local = (scene_points - origin) @ obb_rotation
            known |= np.all(
                (local >= minimum - margin) &
                (local <= maximum + margin), axis=1)

        removed = int(np.count_nonzero(known))
        if removed > 0 and not self._reported_known_scene_filter:
            self.get_logger().info(
                f'Filtering {removed} known incubator points from OctoMap '
                f'input in {self._known_scene_frame} with '
                f'{margin * 1000:.0f} mm surface margin; unknown points remain')
            self._reported_known_scene_filter = True
        return points[~known]

    def _filter_support_surfaces(self, points, cloud_header):
        """Remove only the known ground layer directly under the fixed feet."""
        if not self._support_surface_filter_enabled or points.size == 0:
            return points
        cloud_frame = cloud_header.frame_id.lstrip('/')
        if not cloud_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self._support_surface_frame,
                cloud_frame,
                Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as error:
            if not self._reported_support_surface_tf_error:
                self.get_logger().warning(
                    f'Support-surface filter is retaining all points until TF '
                    f'{cloud_frame}->{self._support_surface_frame} is ready: '
                    f'{error}')
                self._reported_support_surface_tf_error = True
            return points

        rotation = self._quaternion_matrix(transform.transform.rotation)
        if rotation is None:
            return points
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        support_points = points @ rotation.T + translation
        in_x = (
            (support_points[:, 0] >= self._support_surface_x[0]) &
            (support_points[:, 0] <= self._support_surface_x[1]))
        in_left_y = (
            (support_points[:, 1] >= self._support_surface_left_y[0]) &
            (support_points[:, 1] <= self._support_surface_left_y[1]))
        in_right_y = (
            (support_points[:, 1] >= self._support_surface_right_y[0]) &
            (support_points[:, 1] <= self._support_surface_right_y[1]))
        in_ground_layer = np.abs(
            support_points[:, 2] - self._support_surface_ground_z
        ) <= self._support_surface_half_thickness
        known_support = in_x & (in_left_y | in_right_y) & in_ground_layer
        removed = int(np.count_nonzero(known_support))
        if removed > 0 and not self._reported_support_surface_filter:
            self.get_logger().info(
                f'Filtering {removed} known ground-support points only beneath '
                f'the fixed feet in {self._support_surface_frame}: '
                f'X [{self._support_surface_x[0]:.3f}, '
                f'{self._support_surface_x[1]:.3f}] m, '
                f'left Y [{self._support_surface_left_y[0]:.3f}, '
                f'{self._support_surface_left_y[1]:.3f}] m, '
                f'right Y [{self._support_surface_right_y[0]:.3f}, '
                f'{self._support_surface_right_y[1]:.3f}] m, '
                f'Z {self._support_surface_ground_z:.3f} +/- '
                f'{self._support_surface_half_thickness * 1000:.1f} mm')
            self._reported_support_surface_filter = True
        return points[~known_support]

    def _filter_dynamic_door(self, points, cloud_header, stamp_ns):
        """Mask the door OBB only at its fresh measured hinge angle."""
        if not self._dynamic_door_filter_enabled or points.size == 0:
            return points
        if (self._dynamic_door_angle is None or
                self._dynamic_door_stamp_ns is None or
                abs(stamp_ns - self._dynamic_door_stamp_ns) >
                self._dynamic_door_state_max_age_ns):
            if not self._reported_dynamic_door_stale:
                self.get_logger().warning(
                    'Dynamic door filter is retaining all points until a '
                    'fresh finite door_hinge_joint sample is available')
                self._reported_dynamic_door_stale = True
            return points
        cloud_frame = cloud_header.frame_id.lstrip('/')
        if not cloud_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self._known_scene_frame,
                cloud_frame,
                Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return points
        cloud_rotation = self._quaternion_matrix(transform.transform.rotation)
        if cloud_rotation is None:
            return points
        cloud_translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        scene_points = points @ cloud_rotation.T + cloud_translation

        angle = self._dynamic_door_angle
        ca, sa = np.cos(angle), np.sin(angle)
        door_rotation = np.array([
            [ca, -sa, 0.0],
            [sa, ca, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        door_origin = (
            self._dynamic_door_hinge +
            door_rotation @
            (self._dynamic_door_closed_origin - self._dynamic_door_hinge))
        obb_rotation = (
            door_rotation @ self._dynamic_door_closed_rotation)
        local_points = (scene_points - door_origin) @ obb_rotation
        margin = self._dynamic_door_filter_margin
        inside = np.all(
            (local_points >= self._dynamic_door_minimum - margin) &
            (local_points <= self._dynamic_door_maximum + margin), axis=1)
        removed = int(np.count_nonzero(inside))
        if removed > 0 and not self._reported_dynamic_door_filter:
            self.get_logger().info(
                f'Filtering {removed} points inside the measured-angle door '
                f'OBB at q={angle * 180.0 / np.pi:.2f} deg with '
                f'{margin * 1000.0:.1f} mm margin; no closed-pose fallback '
                'or swept volume used')
            self._reported_dynamic_door_filter = True
        return points[~inside]

    def _filter_dynamic_handle(self, points, cloud_header, stamp_ns):
        """Mask only the detailed handle mesh OBB at its measured joint angle."""
        if not self._dynamic_handle_filter_enabled or points.size == 0:
            return points
        if (self._dynamic_handle_angle is None or
                self._dynamic_handle_door_angle is None or
                self._dynamic_handle_stamp_ns is None or
                abs(stamp_ns - self._dynamic_handle_stamp_ns) >
                self._dynamic_handle_state_max_age_ns):
            if not self._reported_dynamic_handle_stale:
                self.get_logger().warning(
                    'Dynamic handle filter is retaining all points until a '
                    'fresh handle_latch_joint and door_hinge_joint sample is '
                    'available')
                self._reported_dynamic_handle_stale = True
            return points
        cloud_frame = cloud_header.frame_id.lstrip('/')
        if not cloud_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                self._known_scene_frame,
                cloud_frame,
                Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return points
        cloud_rotation = self._quaternion_matrix(transform.transform.rotation)
        if cloud_rotation is None:
            return points
        cloud_translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        scene_points = points @ cloud_rotation.T + cloud_translation

        # First move the closed-door pivot with the measured door hinge, then
        # apply the handle's local +Y counterclockwise joint rotation.  The
        # SolidWorks mesh was rigidly zeroed about this same pivot so q=0 is
        # vertical down.  This tracks the exact STL through both mechanism
        # joints without a swept-volume approximation.
        door_angle = self._dynamic_handle_door_angle
        cd, sd = np.cos(door_angle), np.sin(door_angle)
        door_rotation = np.array([
            [cd, -sd, 0.0],
            [sd, cd, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        handle_pivot = (
            self._dynamic_door_hinge +
            door_rotation @
            (self._dynamic_handle_pivot - self._dynamic_door_hinge))
        angle = self._dynamic_handle_angle
        ca, sa = np.cos(angle), np.sin(angle)
        local_joint_rotation = np.array([
            [ca, 0.0, sa],
            [0.0, 1.0, 0.0],
            [-sa, 0.0, ca],
        ], dtype=np.float32)
        link_rotation = (
            door_rotation @ self._dynamic_handle_closed_rotation @
            local_joint_rotation)
        mesh_rotation = (
            link_rotation @ self._dynamic_handle_mesh_fixed_rotation)
        mesh_origin = (
            handle_pivot + link_rotation @ self._dynamic_handle_mesh_translation)
        mesh_points = (scene_points - mesh_origin) @ mesh_rotation
        margin = self._dynamic_handle_filter_margin
        inside = np.all(
            (mesh_points >= self._dynamic_handle_mesh_minimum - margin) &
            (mesh_points <= self._dynamic_handle_mesh_maximum + margin), axis=1)
        removed = int(np.count_nonzero(inside))
        if removed > 0 and not self._reported_dynamic_handle_filter:
            self.get_logger().info(
                f'Filtering {removed} points inside the measured-angle handle '
                f'mesh OBB at q={self._dynamic_handle_angle * 180.0 / np.pi:.2f} '
                f'deg and door='
                f'{self._dynamic_handle_door_angle * 180.0 / np.pi:.2f} '
                f'deg with {margin * 1000.0:.1f} mm margin and only a '
                f'{self._dynamic_handle_tip_extension * 1000.0:.1f} mm free-tip '
                f'extension; no swept volume used')
            self._reported_dynamic_handle_filter = True
        return points[~inside]

    def _filter_left_finger2_self(self, points, cloud_header):
        """Remove only points inside the exact Link2_2.STL local AABB."""
        if not self._left_finger2_self_filter_enabled or points.size == 0:
            return points
        cloud_frame = cloud_header.frame_id.lstrip('/')
        if not cloud_frame:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                'left_dex1_finger_2_link', cloud_frame, Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            return points
        rotation = self._quaternion_matrix(transform.transform.rotation)
        if rotation is None:
            return points
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        link_points = points @ rotation.T + translation
        inside = np.all(
            (link_points >= self._left_finger2_minimum) &
            (link_points <= self._left_finger2_maximum), axis=1)
        removed = int(np.count_nonzero(inside))
        if removed > 0 and not self._reported_left_finger2_filter:
            self.get_logger().info(
                f'Filtering {removed} left finger-2 self points inside the '
                'zero-margin Link2_2.STL local AABB; no PlanningScene '
                'geometry or environment clearance is changed')
            self._reported_left_finger2_filter = True
        return points[~inside]

    def _filter_wrist_visual_self(self, points, cloud_header):
        """Remove exact wrist visual surface bands at the image timestamp."""
        if (not self._wrist_visual_self_filter_enabled or
                points.size == 0):
            return points
        cloud_frame = cloud_header.frame_id.lstrip('/')
        if not cloud_frame:
            return None
        filtered = points
        margin = self._wrist_visual_self_filter_margin
        for (link_name, triangles, minimum, maximum,
             surface_index) in self._wrist_visual_surfaces:
            try:
                transform = self._tf_buffer.lookup_transform(
                    link_name, cloud_frame,
                    Time.from_msg(cloud_header.stamp),
                    timeout=Duration(seconds=0.10))
            except TransformException as error:
                if link_name not in self._reported_wrist_visual_tf_errors:
                    self.get_logger().warning(
                        'Dropping a depth frame because its exact-timestamp '
                        f'{link_name} transform is unavailable: {error}')
                    self._reported_wrist_visual_tf_errors.add(link_name)
                return None
            rotation = self._quaternion_matrix(transform.transform.rotation)
            if rotation is None:
                return None
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ], dtype=np.float64)
            link_points = np.asarray(filtered, dtype=np.float64) @ rotation.T
            link_points += translation
            inside = _points_within_triangle_surface_margin(
                link_points, triangles, minimum, maximum, margin,
                surface_index)
            removed = int(np.count_nonzero(inside))
            if (removed > 0 and
                    link_name not in self._reported_wrist_visual_links):
                self.get_logger().info(
                    f'Filtering {removed} {link_name} self points within '
                    f'{margin * 1000.0:.1f} mm of the exact shipped STL '
                    'surface at the image timestamp; depth self-mask only, '
                    'with no PlanningScene geometry, ACM, or 10 mm '
                    'environment-clearance change')
                self._reported_wrist_visual_links.add(link_name)
            filtered = filtered[~inside]
            if filtered.size == 0:
                break
        return filtered

    def _filter_right_finger2_slider_self(self, points, cloud_header):
        """Remove points in the bounded Euclidean Link2_1 self-mask."""
        if (not self._right_finger2_slider_self_filter_enabled or
                points.size == 0):
            return points
        cloud_frame = cloud_header.frame_id.lstrip('/')
        if not cloud_frame:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                'right_dex1_finger_2_slider_link', cloud_frame,
                Time.from_msg(cloud_header.stamp),
                timeout=Duration(seconds=0.10))
        except TransformException as error:
            if not self._reported_right_finger2_slider_tf_error:
                self.get_logger().warning(
                    'Dropping a depth frame because its exact-timestamp '
                    'right finger-2 slider transform is unavailable: '
                    f'{error}')
                self._reported_right_finger2_slider_tf_error = True
            return None
        rotation = self._quaternion_matrix(transform.transform.rotation)
        if rotation is None:
            return None
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        link_points = points @ rotation.T + translation
        margin = self._right_finger2_slider_self_filter_margin
        inside = _points_inside_euclidean_aabb_margin(
            link_points, self._right_finger2_slider_minimum,
            self._right_finger2_slider_maximum, margin)
        removed = int(np.count_nonzero(inside))
        if (removed > 0 and
                not self._reported_right_finger2_slider_filter):
            self.get_logger().info(
                f'Filtering {removed} right finger-2 slider self points '
                'inside the Link2_1.STL local AABB plus a rounded Euclidean '
                f'{margin * 1000.0:.1f} mm self-render/TF and OctoMap-voxel '
                'tolerance at the image timestamp; depth self-mask only, '
                'with no PlanningScene geometry or 10 mm '
                'environment-clearance change')
            self._reported_right_finger2_slider_filter = True
        return points[~inside]

    def _filter_recognized_target(self, points, cloud_header, stamp_ns):
        """Mask only the tag-measured cube volume at the image timestamp."""
        target = self._target_pose
        if target is None:
            return points
        cloud_frame = cloud_header.frame_id.lstrip('/')
        target_frame = target.header.frame_id.lstrip('/')
        if not cloud_frame or target_frame != 'world':
            return points
        target_stamp_ns = int(target.header.stamp.sec) * 1_000_000_000
        target_stamp_ns += int(target.header.stamp.nanosec)
        if abs(stamp_ns - target_stamp_ns) > self._target_pose_max_age_ns:
            return points
        object_rotation = self._quaternion_matrix(
            target.pose_world.pose.orientation)
        if object_rotation is None:
            return points
        center = np.array([
            target.pose_world.pose.position.x,
            target.pose_world.pose.position.y,
            target.pose_world.pose.position.z,
        ], dtype=np.float32)
        if not np.all(np.isfinite(center)):
            return points
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame, cloud_frame,
                Time.from_msg(cloud_header.stamp),
                timeout=Duration(seconds=0.10))
        except TransformException as error:
            if not self._reported_target_tf_error:
                self.get_logger().warning(
                    'Dropping a depth frame because its exact-timestamp '
                    f'{cloud_frame}->world transform for the recognized cube '
                    f'mask is unavailable: {error}')
                self._reported_target_tf_error = True
            return None
        cloud_rotation = self._quaternion_matrix(
            transform.transform.rotation)
        if cloud_rotation is None:
            return None
        cloud_translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        world_points = points @ cloud_rotation.T + cloud_translation
        # The tracked orientation maps object coordinates into world.  For
        # row-vector points, right multiplication maps world deltas back into
        # the measured cube frame.
        object_points = (world_points - center) @ object_rotation
        half_extents = self._target_size * 0.5 + self._target_filter_margin
        inside = np.all(np.abs(object_points) <= half_extents, axis=1)
        removed = int(np.count_nonzero(inside))
        if removed > 0 and not self._reported_target_filter:
            self.get_logger().info(
                f'Filtering {removed} AprilTag-measured cube points from '
                'OctoMap input at the image timestamp '
                f'using {self._target_size[0] * 1000:.0f}x'
                f'{self._target_size[1] * 1000:.0f}x'
                f'{self._target_size[2] * 1000:.0f} mm geometry plus '
                f'{self._target_filter_margin * 1000:.0f} mm margin')
            self._reported_target_filter = True
        return points[~inside]

    def _depth_callback(self, message):
        if self._camera_info is None:
            return
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000
        stamp_ns += int(message.header.stamp.nanosec)
        if self._last_publish_stamp_ns is not None:
            elapsed = stamp_ns - self._last_publish_stamp_ns
            if 0 <= elapsed < self._minimum_period_ns:
                return

        depth = self._bridge.imgmsg_to_cv2(
            message, desired_encoding='passthrough').astype(np.float32, copy=False)
        rows = np.arange(0, depth.shape[0], self._stride, dtype=np.int32)
        columns = np.arange(0, depth.shape[1], self._stride, dtype=np.int32)
        column_grid, row_grid = np.meshgrid(columns, rows)
        ranges = depth[row_grid, column_grid].astype(np.float32, copy=True)
        finite = np.isfinite(ranges) & (ranges > 0.0)
        if np.any(finite) and float(np.median(ranges[finite])) > 10.0:
            ranges *= 0.001
        valid = (
            np.isfinite(ranges) &
            (ranges >= self._minimum_range) &
            (ranges <= self._maximum_range)
        )
        if not np.any(valid):
            return

        camera = self._camera_info
        fx = float(camera.k[0])
        fy = float(camera.k[4])
        cx = float(camera.k[2])
        cy = float(camera.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return
        z = ranges[valid]
        u = column_grid[valid].astype(np.float32)
        v = row_grid[valid].astype(np.float32)
        points = np.empty((z.size, 3), dtype='<f4')
        points[:, 0] = (u - cx) * z / fx
        points[:, 1] = (v - cy) * z / fy
        points[:, 2] = z
        points = self._filter_support_surfaces(points, message.header)
        points = self._filter_dynamic_door(
            points, message.header, stamp_ns)
        points = self._filter_dynamic_handle(points, message.header, stamp_ns)
        points = self._filter_left_finger2_self(points, message.header)
        points = self._filter_wrist_visual_self(points, message.header)
        if points is None:
            return
        points = self._filter_right_finger2_slider_self(
            points, message.header)
        if points is None:
            return
        points = self._filter_known_scene(points, message.header)
        points = self._filter_recognized_target(points, message.header, stamp_ns)
        if points is None or points.size == 0:
            return

        cloud = PointCloud2()
        cloud.header = message.header
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = points.tobytes()
        cloud.is_dense = True
        self._publisher.publish(cloud)
        self._last_publish_stamp_ns = stamp_ns
        if not self._reported_first_cloud:
            self.get_logger().info(
                f'Publishing D435 XYZ cloud with {cloud.width} points, '
                f'stride {self._stride}, range '
                f'{self._minimum_range:.2f}..{self._maximum_range:.2f} m')
            self._reported_first_cloud = True


def main(args=None):
    rclpy.init(args=args)
    node = DepthPointCloud()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
