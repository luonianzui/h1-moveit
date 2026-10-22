"""Gazebo-only controller for the official Unitree H1_2 recurrent actor."""

import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import MultiDOFCommand
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .locomotion_contract import EXPECTED_LEG_JOINTS
from .motion_arming import MotionArming
from .unitree_h12_contract import load_contract
from .unitree_h12_contract import verify_file_identity
from .unitree_h12_contract import verify_source_assets
from .unitree_h12_policy import UnitreeH12Policy
from .unitree_h12_runtime import projected_gravity
from .unitree_h12_runtime import UnitreeH12Runtime
from .unitree_h12_runtime import UnitreeH12SensorState


def _seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _quaternion_rotation_matrix(quaternion):
    x_value, y_value, z_value, w_value = quaternion
    norm = math.sqrt(
        x_value * x_value + y_value * y_value
        + z_value * z_value + w_value * w_value)
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError('invalid IMU quaternion')
    x_value /= norm
    y_value /= norm
    z_value /= norm
    w_value /= norm
    return np.asarray([
        [1.0 - 2.0 * (y_value * y_value + z_value * z_value),
         2.0 * (x_value * y_value - z_value * w_value),
         2.0 * (x_value * z_value + y_value * w_value)],
        [2.0 * (x_value * y_value + z_value * w_value),
         1.0 - 2.0 * (x_value * x_value + z_value * z_value),
         2.0 * (y_value * z_value - x_value * w_value)],
        [2.0 * (x_value * z_value - y_value * w_value),
         2.0 * (y_value * z_value + x_value * w_value),
         1.0 - 2.0 * (x_value * x_value + y_value * y_value)],
    ], dtype=np.float64)


def _up_z(quaternion):
    return float((_quaternion_rotation_matrix(quaternion) @ np.asarray(
        [0.0, 0.0, 1.0], dtype=np.float64))[2])


def _yaw_from_quaternion(quaternion):
    x_value, y_value, z_value, w_value = quaternion
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value))


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _pose_tracking_command(
        reference, position, yaw, feedforward,
        position_gain, yaw_gain, limits):
    reference = np.asarray(reference, dtype=np.float64).reshape(-1)
    position = np.asarray(position, dtype=np.float64).reshape(-1)
    feedforward = np.asarray(feedforward, dtype=np.float64).reshape(-1)
    limits = np.asarray(limits, dtype=np.float64).reshape(-1)
    if (
            reference.size != 3
            or position.size != 2
            or feedforward.size != 3
            or limits.size != 3):
        raise ValueError('invalid pose-tracking vector size')
    values = np.concatenate((
        reference, position, feedforward, limits,
        (float(yaw), float(position_gain), float(yaw_gain))))
    if not np.isfinite(values).all() or np.any(limits <= 0.0):
        raise ValueError('invalid pose-tracking value')
    world_error = reference[:2] - position
    cosine_yaw = math.cos(yaw)
    sine_yaw = math.sin(yaw)
    body_error = np.asarray([
        cosine_yaw * world_error[0] + sine_yaw * world_error[1],
        -sine_yaw * world_error[0] + cosine_yaw * world_error[1],
    ], dtype=np.float64)
    yaw_error = _normalize_angle(reference[2] - yaw)
    command = feedforward + np.asarray([
        position_gain * body_error[0],
        position_gain * body_error[1],
        yaw_gain * yaw_error,
    ], dtype=np.float64)
    return (
        np.clip(command, -limits, limits).astype(np.float32),
        world_error,
        yaw_error,
    )


def _station_keeping_command(
        anchor, position, yaw, position_gain, yaw_gain, limits,
        body_linear_velocity=(0.0, 0.0), yaw_rate=0.0,
        linear_velocity_gain=0.0, yaw_rate_gain=0.0):
    anchor = np.asarray(anchor, dtype=np.float64).reshape(-1)
    if anchor.size != 3:
        raise ValueError('invalid station-keeping vector size')
    body_linear_velocity = np.asarray(
        body_linear_velocity, dtype=np.float64).reshape(-1)
    damping_values = np.concatenate((
        body_linear_velocity,
        (float(yaw_rate), float(linear_velocity_gain),
         float(yaw_rate_gain))))
    if (
            body_linear_velocity.size != 2
            or not np.isfinite(damping_values).all()
            or linear_velocity_gain < 0.0
            or yaw_rate_gain < 0.0):
        raise ValueError('invalid station-keeping velocity damping')
    damping = np.asarray((
        -linear_velocity_gain * body_linear_velocity[0],
        -linear_velocity_gain * body_linear_velocity[1],
        -yaw_rate_gain * yaw_rate,
    ), dtype=np.float64)
    return _pose_tracking_command(
        anchor, position, yaw, damping,
        position_gain, yaw_gain, limits)


def _capture_station_anchor(motion_reference, position, yaw):
    position = np.asarray(position, dtype=np.float64).reshape(-1)
    if position.size != 2 or not np.isfinite(position).all():
        raise ValueError('invalid station-anchor position')
    if not math.isfinite(float(yaw)):
        raise ValueError('invalid station-anchor yaw')
    if motion_reference is None:
        return np.asarray((position[0], position[1], yaw), dtype=np.float64)
    reference = np.asarray(
        motion_reference, dtype=np.float64).reshape(-1).copy()
    if reference.size != 3 or not np.isfinite(reference).all():
        raise ValueError('invalid station-anchor motion reference')
    return reference


def _advance_motion_reference(reference, command, period):
    reference = np.asarray(reference, dtype=np.float64).reshape(-1)
    command = np.asarray(command, dtype=np.float64).reshape(-1)
    period = float(period)
    if reference.size != 3 or command.size != 3:
        raise ValueError('invalid motion-reference vector size')
    if (
            not np.isfinite(reference).all()
            or not np.isfinite(command).all()
            or not math.isfinite(period)
            or period <= 0.0):
        raise ValueError('invalid motion-reference value')
    cosine_yaw = math.cos(reference[2])
    sine_yaw = math.sin(reference[2])
    return np.asarray((
        reference[0] + period * (
            cosine_yaw * command[0] - sine_yaw * command[1]),
        reference[1] + period * (
            sine_yaw * command[0] + cosine_yaw * command[1]),
        _normalize_angle(reference[2] + period * command[2]),
    ), dtype=np.float64)


def _leash_motion_reference(
        reference, position, yaw,
        maximum_position_error, maximum_yaw_error):
    reference = np.asarray(reference, dtype=np.float64).reshape(-1).copy()
    position = np.asarray(position, dtype=np.float64).reshape(-1)
    values = np.concatenate((reference, position, (
        float(yaw), float(maximum_position_error),
        float(maximum_yaw_error))))
    if reference.size != 3 or position.size != 2:
        raise ValueError('invalid motion-reference leash vector size')
    if (
            not np.isfinite(values).all()
            or maximum_position_error <= 0.0
            or maximum_yaw_error <= 0.0):
        raise ValueError('invalid motion-reference leash value')
    world_error = reference[:2] - position
    error_norm = float(np.linalg.norm(world_error))
    if error_norm > maximum_position_error:
        reference[:2] = (
            position
            + world_error * (maximum_position_error / error_norm))
    yaw_error = _normalize_angle(reference[2] - yaw)
    reference[2] = _normalize_angle(
        yaw + float(np.clip(
            yaw_error, -maximum_yaw_error, maximum_yaw_error)))
    return reference


def _quaternion_multiply(left, right):
    left_x, left_y, left_z, left_w = left
    right_x, right_y, right_z, right_w = right
    return (
        left_w * right_x + left_x * right_w
        + left_y * right_z - left_z * right_y,
        left_w * right_y - left_x * right_z
        + left_y * right_w + left_z * right_x,
        left_w * right_z + left_x * right_y
        - left_y * right_x + left_z * right_w,
        left_w * right_w - left_x * right_x
        - left_y * right_y - left_z * right_z,
    )


def _wrench_force(message):
    force = message.wrench.force
    return float(np.linalg.norm((force.x, force.y, force.z)))


class UnitreeH12GazeboController(Node):
    """Run the reviewed Unitree actor continuously at 50 Hz."""

    def __init__(self):
        super().__init__('h1_2_unitree_h12_gazebo_controller')
        package_share = get_package_share_directory('h1_2_navigation')
        default_contract = os.path.join(
            package_share, 'config', 'unitree_h12_locomotion.yaml')
        default_policy = os.path.join(
            package_share, 'policy', 'unitree_h12_motion_policy.npz')
        self.declare_parameter('contract_file', default_contract)
        self.declare_parameter('policy_file', default_policy)
        self.declare_parameter('enabled', False)
        self.declare_parameter('movement_authorized', False)
        self.contract = load_contract(
            self.get_parameter('contract_file').value)
        verify_source_assets(self.contract)
        policy_path = self.get_parameter('policy_file').value
        verify_file_identity(self.contract['policy']['export'], policy_path)
        self.policy = UnitreeH12Policy.load(policy_path)
        self.runtime = UnitreeH12Runtime(self.contract, self.policy)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.movement_authorized = bool(
            self.get_parameter('movement_authorized').value)
        self.runtime_config = self.contract['gazebo_runtime']
        actuator = self.contract['training_actuator']
        self.joints = tuple(actuator['controlled_joints'])
        if self.joints != EXPECTED_LEG_JOINTS:
            raise RuntimeError('controlled joint order changed')
        self.default_positions = np.asarray(
            actuator['default_positions'], dtype=np.float64)
        self.last_commanded_targets = self.default_positions.copy()
        self.command = np.zeros(3, dtype=np.float32)
        self.motion_arming = MotionArming(
            self.contract['control']['motion_onset_threshold'])
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        self.positions = None
        self.velocities = None
        self.imu = None
        self.odom = None
        self.contact_forces = [0.0, 0.0]
        self.received = {}
        self.control_mode = 'STANDBY_POSITION'
        self.switch_future = None
        self.switch_requested_at = None
        self.effort_settle_step = 0
        self.step_index = 0
        self.station_anchor = None
        self.station_anchor_source = None
        self.station_mode_active = False
        self.station_recurrent_reset_count = 0
        self.motion_reference = None
        self.fault = None
        self.fault_targets = None
        self.last_status = None
        self.standby_publisher = self.create_publisher(
            Float64MultiArray,
            self.runtime_config['standby_controller_topic'], 10)
        self.command_publisher = self.create_publisher(
            MultiDOFCommand, self.runtime_config['controller_topic'], 10)
        self.switch_client = self.create_client(
            SwitchController, self.runtime_config['switch_service'])
        self.odom_publisher = self.create_publisher(
            Odometry, self.runtime_config['odom_topic'], 10)
        self.status_publisher = self.create_publisher(
            String, self.runtime_config['status_topic'], 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            JointState, self.runtime_config['joint_state_topic'],
            self._joint_state_callback, qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.runtime_config['imu_topic'], self._imu_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            WrenchStamped, self.runtime_config['left_load_topic'],
            lambda message: self._contact_callback(0, message),
            qos_profile_sensor_data)
        self.create_subscription(
            WrenchStamped, self.runtime_config['right_load_topic'],
            lambda message: self._contact_callback(1, message),
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.runtime_config['ground_truth_odom_topic'],
            self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            Twist, self.contract['deployment']['output_topic'],
            self._command_callback, 10,
            callback_group=self.control_callback_group)
        self.timer = self.create_timer(
            self.contract['control']['control_period'], self._control_step,
            callback_group=self.control_callback_group)
        self.status_timer = self.create_timer(1.0, self._republish_status)
        self.get_logger().warning(
            'Gazebo-only Unitree H1_2 controller initialized: '
            f'enabled={self.enabled}, '
            f'movement_authorized={self.movement_authorized}, '
            'direct_targets=true, odometry_mode=truth_aided_simulation')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _mark(self, name, stamp):
        value = _seconds(stamp)
        self.received[name] = value if value > 0.0 else self._now()

    def _joint_state_callback(self, message):
        index = {name: offset for offset, name in enumerate(message.name)}
        if any(name not in index for name in self.joints):
            return
        positions = np.asarray([
            message.position[index[name]] for name in self.joints
        ], dtype=np.float64)
        velocities = np.asarray([
            message.velocity[index[name]] for name in self.joints
        ], dtype=np.float64)
        if np.isfinite(positions).all() and np.isfinite(velocities).all():
            self.positions = positions
            self.velocities = velocities
            self._mark('joints', message.header.stamp)

    def _imu_callback(self, message):
        quaternion = (
            message.orientation.x, message.orientation.y,
            message.orientation.z, message.orientation.w)
        angular = (
            message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z)
        if np.isfinite(quaternion).all() and np.isfinite(angular).all():
            self.imu = (quaternion, angular)
            self._mark('imu', message.header.stamp)

    def _contact_callback(self, side, message):
        self.contact_forces[side] = _wrench_force(message)
        self._mark(f'contact_{side}', message.header.stamp)

    def _odom_callback(self, message):
        self.odom = message
        self._mark('odom', message.header.stamp)
        pose = message.pose.pose
        quaternion = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        rotation = _quaternion_rotation_matrix(quaternion)
        yaw = _yaw_from_quaternion(quaternion)
        yaw_quaternion = (
            0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))
        pelvis_relative_quaternion = _quaternion_multiply(
            (0.0, 0.0, -yaw_quaternion[2], yaw_quaternion[3]),
            quaternion)
        world_linear = rotation @ np.asarray([
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z])
        world_angular = rotation @ np.asarray([
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z])
        cosine_yaw = math.cos(yaw)
        sine_yaw = math.sin(yaw)
        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.runtime_config['odom_frame']
        output.child_frame_id = self.runtime_config['base_frame']
        output.pose.pose.position.x = pose.position.x
        output.pose.pose.position.y = pose.position.y
        output.pose.pose.orientation.z = yaw_quaternion[2]
        output.pose.pose.orientation.w = yaw_quaternion[3]
        output.pose.covariance = message.pose.covariance
        output.twist.twist.linear.x = (
            cosine_yaw * world_linear[0] + sine_yaw * world_linear[1])
        output.twist.twist.linear.y = (
            -sine_yaw * world_linear[0] + cosine_yaw * world_linear[1])
        output.twist.twist.angular.z = world_angular[2]
        output.twist.covariance = message.twist.covariance
        self.odom_publisher.publish(output)
        odom_transform = TransformStamped()
        odom_transform.header.stamp = output.header.stamp
        odom_transform.header.frame_id = self.runtime_config['odom_frame']
        odom_transform.child_frame_id = output.child_frame_id
        odom_transform.transform.translation.x = pose.position.x
        odom_transform.transform.translation.y = pose.position.y
        odom_transform.transform.rotation.z = yaw_quaternion[2]
        odom_transform.transform.rotation.w = yaw_quaternion[3]
        pelvis_transform = TransformStamped()
        pelvis_transform.header.stamp = output.header.stamp
        pelvis_transform.header.frame_id = self.runtime_config['base_frame']
        pelvis_transform.child_frame_id = self.runtime_config['body_frame']
        pelvis_transform.transform.translation.z = pose.position.z
        pelvis_transform.transform.rotation.x = pelvis_relative_quaternion[0]
        pelvis_transform.transform.rotation.y = pelvis_relative_quaternion[1]
        pelvis_transform.transform.rotation.z = pelvis_relative_quaternion[2]
        pelvis_transform.transform.rotation.w = pelvis_relative_quaternion[3]
        self.tf_broadcaster.sendTransform([
            odom_transform, pelvis_transform])

    def _command_callback(self, message):
        command = np.asarray([
            message.linear.x, message.linear.y, message.angular.z
        ], dtype=np.float32)
        if not np.isfinite(command).all():
            self.command.fill(0.0)
            return
        self.command[:] = command if self.movement_authorized else 0.0

    def _fresh(self):
        required = ('joints', 'imu', 'contact_0', 'contact_1', 'odom')
        if any(name not in self.received for name in required):
            return False
        now = self._now()
        timeout = self.runtime_config['state_timeout']
        return all(
            abs(now - self.received[name]) <= timeout
            for name in required)

    def _contact_values(self):
        threshold = self.runtime_config['load_contact_threshold']
        return [
            force if force >= threshold else 0.0
            for force in self.contact_forces]

    def _root_diagnostics(self):
        position = self.odom.pose.pose.position
        linear = self.odom.twist.twist.linear
        angular = self.odom.twist.twist.angular
        return {
            'root_position': [position.x, position.y, position.z],
            'root_linear_velocity': [linear.x, linear.y, linear.z],
            'root_angular_velocity': [angular.x, angular.y, angular.z],
        }

    def _publish_standby_targets(self, targets):
        values = np.asarray(targets, dtype=np.float64)
        self.standby_publisher.publish(Float64MultiArray(data=values.tolist()))
        self.last_commanded_targets[:] = values

    def _publish_effort_targets(self, targets):
        values = np.asarray(targets, dtype=np.float64)
        message = MultiDOFCommand()
        message.dof_names = list(self.joints)
        message.values = values.tolist()
        message.values_dot = [0.0] * len(self.joints)
        self.command_publisher.publish(message)
        self.last_commanded_targets[:] = values

    def _publish_active_targets(self, targets):
        if self.control_mode == 'EFFORT_PID':
            self._publish_effort_targets(targets)
        else:
            self._publish_standby_targets(targets)

    def _latch_fault(self, name, detail):
        self.fault = name
        self.fault_targets = self.positions.copy()
        self.command.fill(0.0)
        self.get_logger().error(detail)

    def _effort_mode_ready(self):
        if self.control_mode == 'EFFORT_PID':
            if self.effort_settle_step < self.runtime_config[
                    'effort_settle_steps']:
                self._publish_effort_targets(self.default_positions)
                self.effort_settle_step += 1
                self._status(
                    'EFFORT_PID_SETTLING',
                    settle_step=self.effort_settle_step,
                    settle_steps=self.runtime_config['effort_settle_steps'])
                return False
            return True
        if self.switch_future is None:
            if not self.switch_client.service_is_ready():
                self._status('WAITING_FOR_CONTROLLER_SWITCH_SERVICE')
                return False
            request = SwitchController.Request()
            request.activate_controllers = [
                self.runtime_config['effort_controller_name']]
            request.deactivate_controllers = [
                self.runtime_config['standby_controller_name']]
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            timeout = self.runtime_config['switch_timeout']
            request.timeout.sec = int(timeout)
            request.timeout.nanosec = int(
                round((timeout - int(timeout)) * 1.0e9))
            self.switch_requested_at = self._now()
            self.switch_future = self.switch_client.call_async(request)
            self.control_mode = 'SWITCHING_TO_EFFORT_PID'
            self._status('ARMING_EFFORT_PID')
            return False
        if self.switch_future.done():
            try:
                response = self.switch_future.result()
            except Exception as error:
                self._latch_fault(
                    'CONTROLLER_SWITCH_EXCEPTION',
                    f'controller switch raised: {error}')
                return False
            if not response.ok:
                self._latch_fault(
                    'CONTROLLER_SWITCH_REJECTED',
                    f'controller switch rejected: {response.message}')
                return False
            self.control_mode = 'EFFORT_PID'
            self.effort_settle_step = 0
            self.switch_future = None
            self.switch_requested_at = None
            self.runtime.reset()
            self.step_index = 0
            self._publish_effort_targets(self.positions)
            self._status('EFFORT_PID_READY')
            return False
        if self._now() - self.switch_requested_at > self.runtime_config[
                'switch_timeout']:
            self._latch_fault(
                'CONTROLLER_SWITCH_TIMEOUT',
                'controller switch timed out')
        else:
            self._status('ARMING_EFFORT_PID')
        return False

    def _policy_step(
            self, state, requested_command, policy_command, quaternion,
            angular, up_z_value, contact_forces, **diagnostics):
        self.step_index += 1
        sensor_state = UnitreeH12SensorState(
            base_quaternion_xyzw=quaternion,
            base_angular_velocity=tuple(angular),
            leg_positions=self.positions,
            leg_velocities=self.velocities)
        output = self.runtime.step(sensor_state, policy_command)
        self._publish_effort_targets(output.leg_position_targets)
        position = self.odom.pose.pose.position
        self._status(
            state,
            capture_latched=False,
            contact_forces=contact_forces,
            raw_load_forces=self.contact_forces,
            command=np.asarray(
                requested_command, dtype=np.float32).tolist(),
            policy_command=np.asarray(
                policy_command, dtype=np.float32).tolist(),
            up_z=up_z_value,
            height=position.z,
            phase=output.phase,
            projected_gravity=output.observation[3:6].tolist(),
            policy_action=output.policy_action.tolist(),
            leg_positions=self.positions.tolist(),
            leg_velocities=self.velocities.tolist(),
            leg_targets=output.leg_position_targets.tolist(),
            leg_target_error=(
                self.positions - output.leg_position_targets).tolist(),
            zero_command_behavior=(
                self.contract['zero_command']['mode']
                if state == 'STATION_KEEPING_GAIT' else None),
            **diagnostics,
            **self._root_diagnostics())
        return output

    def _station_command(self):
        pose = self.odom.pose.pose
        quaternion = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        yaw = _yaw_from_quaternion(quaternion)
        if self.station_anchor is None:
            self.station_anchor = _capture_station_anchor(
                None, (pose.position.x, pose.position.y), yaw)
            self.station_anchor_source = 'current_pose'
        zero_command = self.contract['zero_command']
        linear = self.odom.twist.twist.linear
        angular = self.odom.twist.twist.angular
        command, world_error, yaw_error = _station_keeping_command(
            self.station_anchor,
            (pose.position.x, pose.position.y),
            yaw,
            zero_command['position_gain'],
            zero_command['yaw_gain'],
            zero_command['policy_command_limits'],
            (linear.x, linear.y),
            angular.z,
            zero_command['linear_velocity_gain'],
            zero_command['yaw_rate_gain'])
        return command, {
            'station_anchor': self.station_anchor.tolist(),
            'station_anchor_source': self.station_anchor_source,
            'station_world_error': world_error.tolist(),
            'station_error': float(np.linalg.norm(world_error)),
            'station_yaw_error': yaw_error,
            'station_body_velocity': [linear.x, linear.y],
            'station_yaw_rate': angular.z,
        }

    def _motion_command(self):
        pose = self.odom.pose.pose
        quaternion = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        yaw = _yaw_from_quaternion(quaternion)
        if self.motion_reference is None:
            self.motion_reference = np.asarray(
                [pose.position.x, pose.position.y, yaw], dtype=np.float64)
        self.motion_reference = _advance_motion_reference(
            self.motion_reference,
            self.command,
            self.contract['control']['control_period'])
        tracking = self.contract['motion_tracking']
        self.motion_reference = _leash_motion_reference(
            self.motion_reference,
            (pose.position.x, pose.position.y),
            yaw,
            tracking['maximum_position_error'],
            tracking['maximum_yaw_error'])
        command, world_error, yaw_error = _pose_tracking_command(
            self.motion_reference,
            (pose.position.x, pose.position.y),
            yaw,
            self.command,
            tracking['position_gain'],
            tracking['yaw_gain'],
            tracking['policy_command_limits'])
        return command, {
            'motion_tracking_mode': tracking['mode'],
            'motion_reference': self.motion_reference.tolist(),
            'tracking_world_error': world_error.tolist(),
            'tracking_error': float(np.linalg.norm(world_error)),
            'tracking_yaw_error': yaw_error,
        }

    def _status(self, state, **extra):
        payload = {
            'state': state,
            'profile': 'unitree_h12_official_motion',
            'enabled': self.enabled,
            'movement_authorized': self.movement_authorized,
            'fresh': self._fresh(),
            'fault': self.fault,
            'controller_mode': self.control_mode,
            'step_index': self.step_index,
            'direct_position_targets': True,
            'odometry_mode': 'truth_aided_simulation',
        }
        if 'odom' in self.received:
            payload.update(self._root_diagnostics())
        payload.update(extra)
        serialized = json.dumps(payload, sort_keys=True)
        if serialized != self.last_status:
            self.status_publisher.publish(String(data=serialized))
            self.last_status = serialized

    def _republish_status(self):
        if self.last_status is not None:
            self.status_publisher.publish(String(data=self.last_status))

    def _control_step(self):
        if not self.enabled:
            self._status('DISABLED')
            return
        if not self._fresh():
            hold = (
                self.fault_targets if self.fault_targets is not None
                else self.last_commanded_targets)
            self._publish_active_targets(hold)
            self._status('WAITING_FOR_FRESH_STATE')
            return
        quaternion, angular_values = self.imu
        angular = np.asarray(angular_values, dtype=np.float32)
        up_z_value = _up_z(quaternion)
        position = self.odom.pose.pose.position
        if (
                up_z_value < self.runtime_config['fall_minimum_up_z']
                or position.z < self.runtime_config['fall_minimum_height']):
            if self.fault is None:
                self._latch_fault(
                    'FALL_ENVELOPE',
                    'Unitree H1_2 fall envelope breached; hold latched')
            self._publish_active_targets(self.fault_targets)
            self._status(
                'FAULT_HOLD', up_z=up_z_value, height=position.z)
            return
        if self.fault is not None:
            self._publish_active_targets(self.fault_targets)
            self._status('FAULT_HOLD')
            return
        contact_forces = self._contact_values()
        if self.motion_arming.standby(self.command):
            self._publish_standby_targets(self.default_positions)
            self._status(
                'STANDBY_HOLD',
                capture_latched=False,
                contact_forces=contact_forces,
                command=self.command.tolist(),
                up_z=up_z_value,
                height=position.z,
                **self._root_diagnostics())
            return
        if not self._effort_mode_ready():
            if self.control_mode != 'EFFORT_PID':
                self._publish_standby_targets(self.default_positions)
            return
        exact_zero = bool(np.all(self.command == 0.0))
        if exact_zero:
            station_entry = not self.station_mode_active
            self.station_mode_active = True
            if self.station_anchor is None and self.motion_reference is not None:
                self.station_anchor = _capture_station_anchor(
                    self.motion_reference,
                    (position.x, position.y),
                    _yaw_from_quaternion(quaternion))
                self.station_anchor_source = 'motion_reference'
            self.motion_reference = None
            if (
                    station_entry
                    and self.contract['zero_command'][
                        'reset_recurrent_state_on_entry']):
                self.policy.reset()
                self.station_recurrent_reset_count += 1
            policy_command, diagnostics = self._station_command()
            diagnostics.update({
                'station_entry_recurrent_reset': station_entry,
                'station_recurrent_reset_count': (
                    self.station_recurrent_reset_count),
            })
            self._policy_step(
                'STATION_KEEPING_GAIT', self.command, policy_command,
                quaternion, angular, up_z_value, contact_forces,
                **diagnostics)
            return
        self.station_mode_active = False
        self.station_anchor = None
        self.station_anchor_source = None
        policy_command, diagnostics = self._motion_command()
        self._policy_step(
            'MOVE', self.command, policy_command,
            quaternion, angular, up_z_value, contact_forces,
            **diagnostics)


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeH12GazeboController()
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
