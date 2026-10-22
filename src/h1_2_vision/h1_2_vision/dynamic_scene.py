"""Atomic, immutable planning-scene epochs built from stable tag observations."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import struct
import threading

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from h1_2_perception_msgs.msg import SceneEpoch, TrackedObjectState
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException

from .candidate3_alignment import (
    handle_pose_from_cabinet,
    validate_door_tag_cross_check,
)
from .candidate3_post_nav_contract import (
    build_alignment_certificate,
    validate_navigation_hold_status,
    validate_tag_roles,
)
from .tag_config import load_tag_map
from .tag_geometry import (
    matrix_quaternion,
    rpy_matrix,
    transform,
    transform_from_ros,
)


PLANNER_VERSION = 'h1_candidate3_scene/826-c1'
DYNAMIC_OBJECT_IDS = (
    'incubator_body_exact', 'incubator_door_exact',
    'incubator_handle_exact', 'green_cube_dynamic_exact',
    'green_cube_10mm_guard',
)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def object_freshness_error(now_ns, objects, maximum_age_s):
    """Return a fail-closed status for future or expired tag snapshots."""
    maximum_age_ns = int(maximum_age_s * 1e9)
    for name, value in objects.items():
        object_stamp = stamp_ns(value.header.stamp)
        if object_stamp > now_ns or now_ns - object_stamp > maximum_age_ns:
            return f'STALE_OBJECT:{name}'
    return ''


def pose_transform(message) -> np.ndarray:
    position = message.position
    orientation = message.orientation
    from .tag_geometry import quaternion_matrix
    return transform(
        quaternion_matrix(
            orientation.x, orientation.y, orientation.z, orientation.w),
        [position.x, position.y, position.z],
    )


def transform_pose(value: np.ndarray) -> Pose:
    message = Pose()
    message.position.x, message.position.y, message.position.z = value[:3, 3]
    q = matrix_quaternion(value[:3, :3])
    message.orientation.x, message.orientation.y = q[:2]
    message.orientation.z, message.orientation.w = q[2:]
    return message


def load_binary_stl(path: Path) -> Mesh:
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f'invalid STL {path}')
    count = struct.unpack_from('<I', payload, 80)[0]
    if len(payload) != 84 + count * 50:
        raise ValueError(f'only exact binary STL is accepted: {path}')
    result = Mesh()
    result.vertices = []
    result.triangles = []
    for index in range(count):
        values = struct.unpack_from('<12fH', payload, 84 + index * 50)
        base = len(result.vertices)
        for offset in range(3):
            vertex = Point()
            vertex.x = float(values[3 + offset * 3]) * 0.001
            vertex.y = float(values[4 + offset * 3]) * 0.001
            vertex.z = float(values[5 + offset * 3]) * 0.001
            result.vertices.append(vertex)
        triangle = MeshTriangle()
        triangle.vertex_indices = [base, base + 1, base + 2]
        result.triangles.append(triangle)
    return result


def combine_hash(paths) -> str:
    digest = hashlib.sha256()
    for path in sorted(Path(value) for value in paths):
        digest.update(path.name.encode('utf-8') + b'\0')
        digest.update(path.read_bytes())
    return digest.hexdigest()


class DynamicScene(Node):
    def __init__(self):
        super().__init__('h1_2_dynamic_scene')
        self.group = ReentrantCallbackGroup()
        vision_share = Path(get_package_share_directory('h1_2_vision'))
        description_share = Path(get_package_share_directory('h1_2_description'))
        mesh_root = description_share / 'models/incubator/meshes'
        self.declare_parameter('tag_map', str(vision_share / 'config/tag_map.yaml'))
        self.declare_parameter('planning_frame', 'pelvis')
        self.declare_parameter('maximum_input_age_s', 0.50)
        self.declare_parameter('maximum_sensor_skew_s', 0.10)
        self.declare_parameter('require_pointcloud', True)
        self.declare_parameter('require_latch', True)
        self.declare_parameter('freeze_once', False)
        self.declare_parameter('require_post_navigation_hold', False)
        self.declare_parameter('navigation_status_maximum_age_s', 0.50)
        self.declare_parameter('post_navigation_goal_x', 0.0)
        self.declare_parameter('post_navigation_goal_y', 0.0)
        self.declare_parameter('post_navigation_goal_yaw', 0.0)
        self.planning_frame = str(self.get_parameter('planning_frame').value)
        self.maximum_age = float(self.get_parameter('maximum_input_age_s').value)
        self.maximum_skew = float(self.get_parameter('maximum_sensor_skew_s').value)
        self.config = load_tag_map(
            Path(self.get_parameter('tag_map').value), [
                description_share / 'models/incubator/materials/textures',
                description_share / 'models/green_cube/materials/textures',
            ])
        self.mesh_paths = {
            name: mesh_root / f'incubator_{name}.STL'
            for name in ('body', 'tray', 'door', 'handle')
        }
        self.meshes = {name: load_binary_stl(path)
                       for name, path in self.mesh_paths.items()}
        self.stl_hash = combine_hash(self.mesh_paths.values())
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.objects = {}
        self.joint_state = None
        self.mechanism_state = None
        self.cloud_revision = 0
        self.cloud_stamp_ns = 0
        self.latch_state = ''
        self.latch_receive_ns = 0
        self.right_support_state = 'DETACHED_NO_ENDPOINT_PLAN_ONLY'
        self.left_support_state = 'DETACHED_NO_ENDPOINT_PLAN_ONLY'
        self.robot_description = ''
        self.navigation_status = None
        self.navigation_status_receive_ns = 0
        self.pending = False
        self.last_signature = None
        self.epoch_id = 0
        self.frozen_once = False

        transient = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.epoch_pub = self.create_publisher(
            SceneEpoch, '/dynamic_scene/epoch', transient)
        self.frozen_scene_pub = self.create_publisher(
            PlanningScene, '/dynamic_scene/frozen_scene', transient)
        self.status_pub = self.create_publisher(
            String, '/dynamic_scene/status', 10)
        self.alignment_pub = self.create_publisher(
            String, '/dynamic_scene/post_nav_alignment', transient)
        for object_name in ('cabinet', 'door', 'cube'):
            self.create_subscription(
                TrackedObjectState, f'/tag_pose/{object_name}',
                lambda message, name=object_name: self.object_callback(name, message),
                10, callback_group=self.group)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback,
            qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(
            JointState, '/incubator/joint_states', self.mechanism_callback,
            qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(
            PointCloud2, '/realsense_camera/depth/points', self.cloud_callback,
            qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(
            String, '/incubator/latch/state', self.latch_callback, 10,
            callback_group=self.group)
        self.create_subscription(
            String, '/h1_2/right_door_support/state', self.right_support_callback,
            10, callback_group=self.group)
        self.create_subscription(
            String, '/h1_2/left_pose_grasp/state', self.left_support_callback,
            10, callback_group=self.group)
        self.create_subscription(
            String, '/robot_description', self.robot_description_callback,
            transient, callback_group=self.group)
        self.create_subscription(
            String, '/h1_2_navigation/incubator_navigation/status',
            self.navigation_status_callback, transient,
            callback_group=self.group)
        self.apply_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self.group)
        self.get_client = self.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=self.group)
        self.timer = self.create_timer(
            0.1, self.try_build_epoch, callback_group=self.group)

    def object_callback(self, name, message):
        with self.lock:
            self.objects[name] = message

    def joint_callback(self, message):
        with self.lock:
            self.joint_state = message

    def mechanism_callback(self, message):
        with self.lock:
            self.mechanism_state = message

    def cloud_callback(self, message):
        with self.lock:
            self.cloud_revision += 1
            self.cloud_stamp_ns = stamp_ns(message.header.stamp)

    def latch_callback(self, message):
        with self.lock:
            self.latch_state = message.data.strip()
            self.latch_receive_ns = self.get_clock().now().nanoseconds

    def right_support_callback(self, message):
        with self.lock:
            self.right_support_state = message.data.strip().upper()

    def left_support_callback(self, message):
        with self.lock:
            self.left_support_state = message.data.strip().upper()

    def robot_description_callback(self, message):
        with self.lock:
            self.robot_description = message.data

    def navigation_status_callback(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            value = None
        with self.lock:
            self.navigation_status = value
            self.navigation_status_receive_ns = self.get_clock().now().nanoseconds

    @staticmethod
    def joint_position(message, name):
        try:
            return float(message.position[message.name.index(name)])
        except (ValueError, IndexError):
            return None

    def validate_inputs(self, now_ns, objects, joint_state, mechanism_state,
                        cloud_revision, cloud_stamp_ns, latch, latch_receive_ns,
                        robot_description, navigation_status=None,
                        navigation_status_receive_ns=0):
        missing = [name for name in ('cabinet', 'door', 'cube') if name not in objects]
        if missing:
            return f'MISSING_OBJECT:{",".join(missing)}'
        for name, value in objects.items():
            if not value.valid or not value.stable:
                return f'UNSTABLE_OBJECT:{name}'
            if value.config_hash != self.config['config_hash']:
                return f'TAG_CONFIG_HASH:{name}'
        freshness_error = object_freshness_error(
            now_ns, objects, self.maximum_age)
        if freshness_error:
            return freshness_error
        stamps = [stamp_ns(objects[name].header.stamp)
                  for name in ('cabinet', 'door', 'cube')]
        if max(stamps) - min(stamps) > int(self.maximum_skew * 1e9):
            return 'OBJECT_SENSOR_SKEW'
        if joint_state is None:
            return 'MISSING_JOINT_STATE'
        if mechanism_state is None:
            return 'MISSING_MECHANISM_STATE'
        joint_stamp = stamp_ns(joint_state.header.stamp)
        if abs(joint_stamp - max(stamps)) > int(self.maximum_skew * 1e9):
            return 'JOINT_SENSOR_SKEW'
        mechanism_stamp = stamp_ns(mechanism_state.header.stamp)
        if abs(mechanism_stamp - max(stamps)) > int(self.maximum_skew * 1e9):
            return 'MECHANISM_SENSOR_SKEW'
        if bool(self.get_parameter('require_pointcloud').value):
            if cloud_revision == 0 or abs(cloud_stamp_ns - max(stamps)) > int(
                    self.maximum_age * 1e9):
                return 'STALE_POINTCLOUD'
        if bool(self.get_parameter('require_latch').value):
            if not latch or now_ns - latch_receive_ns > int(self.maximum_age * 1e9):
                return 'STALE_LATCH'
        if not robot_description:
            return 'MISSING_ROBOT_DESCRIPTION'
        if bool(self.get_parameter('require_post_navigation_hold').value):
            if navigation_status is None:
                return 'MISSING_NAVIGATION_HOLD'
            maximum_navigation_age = float(self.get_parameter(
                'navigation_status_maximum_age_s').value)
            if (maximum_navigation_age <= 0.0 or
                    navigation_status_receive_ns > now_ns or
                    now_ns - navigation_status_receive_ns >
                    int(maximum_navigation_age * 1.0e9)):
                return 'STALE_NAVIGATION_HOLD'
            try:
                validate_navigation_hold_status(
                    navigation_status,
                    expected_goal_pose=(
                        float(self.get_parameter(
                            'post_navigation_goal_x').value),
                        float(self.get_parameter(
                            'post_navigation_goal_y').value),
                        float(self.get_parameter(
                            'post_navigation_goal_yaw').value)))
                validate_tag_roles(
                    objects['cabinet'], objects['door'],
                    self.config['config_hash'])
            except (TypeError, ValueError) as error:
                return f'POST_NAVIGATION_ALIGNMENT:{error}'
        return ''

    def mechanism_gate(self, objects, mechanism_state):
        world_body = pose_transform(objects['cabinet'].pose_world.pose)
        world_door = pose_transform(objects['door'].pose_world.pose)
        angle = self.joint_position(mechanism_state, 'door_hinge_joint')
        if angle is None:
            return False, 'MISSING_DOOR_JOINT', None, None, None
        try:
            door_check = validate_door_tag_cross_check(
                world_body, world_door, angle)
        except ValueError as error:
            return False, f'HINGE_GATE:{error}', None, None, None
        handle = self.joint_position(mechanism_state, 'handle_latch_joint')
        if handle is None:
            return False, 'MISSING_HANDLE_JOINT', None, None, None
        return True, '', handle, angle, door_check

    def collision_objects(
            self, planning_from_world, objects, door_hinge_angle, handle_angle,
            authoritative_world_door):
        world_body = pose_transform(objects['cabinet'].pose_world.pose)
        world_door = authoritative_world_door
        world_cube = pose_transform(objects['cube'].pose_world.pose)
        body_visual = transform(rpy_matrix(1.570796326795, 0, -1.570796326795),
                                [0.189890542898, 0.250472606667, 0.0])
        tray_visual = transform(rpy_matrix(1.570796326795, 0, -1.570796326795),
                                [0.151890542898, 0.146472606667, 0.160407057555])
        door_visual = transform(rpy_matrix(1.570796326795, 0, -1.528168312093),
                                [0.005592142864, 0.408609505443, 0.0])
        handle_visual = transform(np.eye(3),
                                  [-0.009999668689, 0.000027284668, -0.119003896682])
        world_handle = handle_pose_from_cabinet(
            world_body, door_hinge_angle, handle_angle)

        body = CollisionObject()
        body.header.frame_id = self.planning_frame
        body.id = 'incubator_body_exact'
        body.operation = CollisionObject.ADD
        body.meshes = [self.meshes['body'], self.meshes['tray']]
        body.mesh_poses = [
            transform_pose(planning_from_world @ world_body @ body_visual),
            transform_pose(planning_from_world @ world_body @ tray_visual),
        ]
        door = CollisionObject()
        door.header.frame_id = self.planning_frame
        door.id = 'incubator_door_exact'
        door.operation = CollisionObject.ADD
        door.meshes = [self.meshes['door']]
        door.mesh_poses = [transform_pose(
            planning_from_world @ world_door @ door_visual)]
        handle = CollisionObject()
        handle.header.frame_id = self.planning_frame
        handle.id = 'incubator_handle_exact'
        handle.operation = CollisionObject.ADD
        handle.meshes = [self.meshes['handle']]
        handle.mesh_poses = [transform_pose(
            planning_from_world @ world_handle @ handle_visual)]

        cube = CollisionObject()
        cube.header.frame_id = self.planning_frame
        cube.id = 'green_cube_dynamic_exact'
        cube.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.080, 0.160, 0.030]
        cube.primitives = [primitive]
        cube.primitive_poses = [transform_pose(planning_from_world @ world_cube)]
        guard = CollisionObject()
        guard.header.frame_id = self.planning_frame
        guard.id = 'green_cube_10mm_guard'
        guard.operation = CollisionObject.ADD
        expanded = SolidPrimitive()
        expanded.type = SolidPrimitive.BOX
        expanded.dimensions = [0.100, 0.180, 0.050]
        guard.primitives = [expanded]
        guard.primitive_poses = [transform_pose(planning_from_world @ world_cube)]
        return [body, door, handle, cube, guard]

    def try_build_epoch(self):
        now_ns = self.get_clock().now().nanoseconds
        with self.lock:
            if self.pending or (
                    self.frozen_once and
                    bool(self.get_parameter('freeze_once').value)):
                return
            objects = dict(self.objects)
            joint_state = self.joint_state
            mechanism_state = self.mechanism_state
            cloud_revision = self.cloud_revision
            cloud_stamp_ns = self.cloud_stamp_ns
            latch = self.latch_state
            latch_receive_ns = self.latch_receive_ns
            robot_description = self.robot_description
            right_support_state = self.right_support_state
            left_support_state = self.left_support_state
            navigation_status = copy.deepcopy(self.navigation_status)
            navigation_status_receive_ns = self.navigation_status_receive_ns
        error = self.validate_inputs(
            now_ns, objects, joint_state, mechanism_state, cloud_revision,
            cloud_stamp_ns, latch, latch_receive_ns, robot_description,
            navigation_status, navigation_status_receive_ns)
        if error:
            self.publish_status(False, error)
            return
        gate, error, handle_angle, door_hinge_angle, door_check = (
            self.mechanism_gate(objects, mechanism_state))
        if not gate:
            self.publish_status(False, error)
            return
        stamps = tuple(stamp_ns(objects[name].header.stamp)
                       for name in ('cabinet', 'door', 'cube'))
        signature = stamps + (
            stamp_ns(joint_state.header.stamp),
            stamp_ns(mechanism_state.header.stamp), cloud_revision)
        if signature == self.last_signature:
            return
        try:
            planning_from_world = transform_from_ros(
                self.tf_buffer.lookup_transform(
                    self.planning_frame, objects['cabinet'].header.frame_id,
                    Time.from_msg(objects['cabinet'].header.stamp),
                    timeout=Duration(seconds=0.05)))
        except TransformException as error:
            self.publish_status(False, f'PLANNING_FRAME_TF:{error}')
            return
        alignment_payload = None
        alignment_hash = ''
        if bool(self.get_parameter('require_post_navigation_hold').value):
            try:
                alignment_payload, alignment_hash = (
                    build_alignment_certificate(
                        navigation_status,
                        pose_transform(objects['cabinet'].pose_world.pose),
                        pose_transform(objects['door'].pose_world.pose),
                        door_hinge_angle, handle_angle,
                        planning_from_world, self.config['config_hash'],
                        stamp_ns(objects['cabinet'].header.stamp),
                        stamp_ns(objects['door'].header.stamp)))
            except (TypeError, ValueError) as error:
                self.publish_status(
                    False, f'POST_NAVIGATION_CERTIFICATE:{error}')
                return
            signature += (alignment_hash,)
            if signature == self.last_signature:
                return
        if not self.apply_client.service_is_ready():
            self.apply_client.wait_for_service(timeout_sec=0.0)
            self.publish_status(False, 'APPLY_SCENE_SERVICE')
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = self.collision_objects(
            planning_from_world, objects, door_hinge_angle, handle_angle,
            door_check['predicted_world_door'])
        request = ApplyPlanningScene.Request()
        request.scene = scene
        with self.lock:
            self.pending = True
        future = self.apply_client.call_async(request)
        authoritative_objects = dict(objects)
        authoritative_door = copy.deepcopy(objects['cabinet'])
        authoritative_door.object_name = 'door'
        authoritative_door.object_frame = objects['door'].object_frame
        authoritative_door.pose_world.pose = transform_pose(
            door_check['predicted_world_door'])
        authoritative_door.contributing_tag_ids = list(
            objects['cabinet'].contributing_tag_ids)
        authoritative_door.consecutive_frames = min(
            objects['cabinet'].consecutive_frames,
            objects['door'].consecutive_frames)
        authoritative_door.maximum_reprojection_error_px = max(
            objects['cabinet'].maximum_reprojection_error_px,
            objects['door'].maximum_reprojection_error_px)
        authoritative_door.maximum_depth_residual_m = max(
            objects['cabinet'].maximum_depth_residual_m,
            objects['door'].maximum_depth_residual_m)
        authoritative_door.source_mode = (
            'CABINET_TAGS_10_12_RIGID_MODEL_TAG11_CHECKED')
        authoritative_door.status = (
            'VALID:TAG11_TRANSLATION='
            f'{door_check["translation_error_m"]:.9g}:'
            'TAG11_ROTATION='
            f'{door_check["rotation_error_rad"]:.9g}')
        authoritative_objects['door'] = authoritative_door
        context = (
            signature, authoritative_objects, joint_state, mechanism_state,
            cloud_revision, latch, right_support_state, left_support_state,
            robot_description, alignment_payload, alignment_hash)
        future.add_done_callback(lambda value: self.apply_done(value, context))

    def apply_done(self, future, context):
        try:
            accepted = bool(future.result().success)
        except Exception as error:
            accepted = False
            self.publish_status(False, f'APPLY_SCENE_EXCEPTION:{error}')
        if not accepted:
            with self.lock:
                self.pending = False
            self.publish_status(False, 'APPLY_SCENE_REJECTED')
            return
        if not self.get_client.service_is_ready():
            with self.lock:
                self.pending = False
            self.publish_status(False, 'GET_SCENE_SERVICE')
            return
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.SCENE_SETTINGS |
            PlanningSceneComponents.ROBOT_STATE |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
            PlanningSceneComponents.WORLD_OBJECT_NAMES |
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.OCTOMAP |
            PlanningSceneComponents.TRANSFORMS |
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
            PlanningSceneComponents.LINK_PADDING_AND_SCALING)
        result = self.get_client.call_async(request)
        result.add_done_callback(lambda value: self.freeze_done(value, context))

    def freeze_done(self, future, context):
        (signature, objects, joint_state, mechanism_state, cloud_revision,
         latch, right_support_state, left_support_state,
         robot_description, alignment_payload, alignment_hash) = context
        try:
            scene = future.result().scene
        except Exception as error:
            with self.lock:
                self.pending = False
            self.publish_status(False, f'GET_SCENE_EXCEPTION:{error}')
            return
        ids = {value.id for value in scene.world.collision_objects}
        missing = sorted(set(DYNAMIC_OBJECT_IDS) - ids)
        if missing:
            with self.lock:
                self.pending = False
            self.publish_status(False, f'FROZEN_SCENE_MISSING:{",".join(missing)}')
            return
        # Bind the frozen PlanningScene to the exact joint-state sample that
        # passed the epoch skew/freshness gate.  MoveIt's get-scene service
        # can briefly return its all-zero default before the current-state
        # monitor catches up, which would otherwise make uncontrolled joints
        # differ between SceneEpoch and the C++ clearance replay.
        scene.robot_state.joint_state = joint_state
        scene.robot_state.is_diff = False
        self.epoch_id += 1
        epoch = SceneEpoch()
        epoch.header = objects['cabinet'].header
        epoch.header.frame_id = objects['cabinet'].header.frame_id
        epoch.epoch_id = self.epoch_id
        epoch.cabinet = objects['cabinet']
        epoch.door = objects['door']
        epoch.cube = objects['cube']
        epoch.robot_state = joint_state
        epoch.mechanism_state = mechanism_state
        epoch.pointcloud_revision = cloud_revision
        epoch.latch_state = latch
        epoch.right_door_support_state = right_support_state
        epoch.left_cube_support_state = left_support_state
        epoch.robot_model_hash = hashlib.sha256(
            robot_description.encode('utf-8')).hexdigest()
        epoch.stl_hash = self.stl_hash
        epoch.tag_config_hash = self.config['config_hash']
        epoch.planner_version = PLANNER_VERSION
        epoch.stable = True
        epoch.valid = True
        epoch.status = (
            f'FROZEN_POST_NAV_ALIGNMENT:{alignment_hash}'
            if alignment_hash else 'FROZEN_PLAN_ONLY')
        canonical = json.dumps({
            'epoch_id': epoch.epoch_id,
            'tag_config_hash': epoch.tag_config_hash,
            'stl_hash': epoch.stl_hash,
            'robot_model_hash': epoch.robot_model_hash,
            'pointcloud_revision': epoch.pointcloud_revision,
            'latch': epoch.latch_state,
            'right_support': epoch.right_door_support_state,
            'left_support': epoch.left_cube_support_state,
            'epoch_status': epoch.status,
        }, sort_keys=True, separators=(',', ':')).encode('utf-8')
        digest = hashlib.sha256()
        digest.update(canonical)
        digest.update(serialize_message(epoch.robot_state))
        digest.update(serialize_message(epoch.mechanism_state))
        digest.update(serialize_message(scene))
        epoch.scene_hash = digest.hexdigest()
        scene.name = f'h1_scene_epoch:{epoch.epoch_id}:{epoch.scene_hash}'
        scene.is_diff = False
        self.frozen_scene_pub.publish(scene)
        self.epoch_pub.publish(epoch)
        if alignment_payload is not None:
            alignment_message = String()
            alignment_message.data = json.dumps({
                'alignment_hash': alignment_hash,
                'epoch_id': int(epoch.epoch_id),
                'scene_hash': epoch.scene_hash,
                'payload': alignment_payload,
            }, sort_keys=True, separators=(',', ':'), allow_nan=False)
            self.alignment_pub.publish(alignment_message)
        with self.lock:
            self.last_signature = signature
            self.frozen_once = True
            self.pending = False
        self.publish_status(
            True, 'FROZEN_POST_NAV_ALIGNMENT'
            if alignment_hash else 'FROZEN', epoch,
            alignment_hash=alignment_hash)

    def publish_status(
            self, valid, status, epoch=None, alignment_hash=''):
        message = String()
        message.data = json.dumps({
            'valid': bool(valid),
            'status': status,
            'epoch_id': int(epoch.epoch_id) if epoch else 0,
            'scene_hash': epoch.scene_hash if epoch else '',
            'tag_config_hash': self.config['config_hash'],
            'planner_version': PLANNER_VERSION,
            'alignment_hash': alignment_hash,
        }, sort_keys=True, separators=(',', ':'))
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicScene()
    executor = MultiThreadedExecutor(num_threads=4)
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
