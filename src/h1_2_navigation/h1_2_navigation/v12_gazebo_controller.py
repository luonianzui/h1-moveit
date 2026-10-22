"""Gazebo-only, fail-closed ROS adapter for the frozen V12 actor."""

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

from .locomotion_contract import EXPECTED_CONTROLLED_JOINTS
from .locomotion_contract import EXPECTED_LEG_JOINTS
from .locomotion_contract import load_contract
from .motion_arming import MotionArming
from .v12_policy import V12Policy
from .v12_runtime import V12LocomotionRuntime
from .v12_runtime import V12SensorState


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


def _imu_up_world(quaternion):
    """Return the IMU local +Z axis expressed in world coordinates."""
    return _quaternion_rotation_matrix(quaternion) @ np.asarray(
        [0.0, 0.0, 1.0], dtype=np.float64)


def _yaw_from_quaternion(quaternion):
    x_value, y_value, z_value, w_value = quaternion
    return math.atan2(
        2.0 * (w_value * z_value + x_value * y_value),
        1.0 - 2.0 * (y_value * y_value + z_value * z_value),
    )


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


def _contact_force(message):
    maximum = 0.0
    for contact in message.contacts:
        for wrench in contact.wrenches:
            for body_wrench in (
                    wrench.body_1_wrench, wrench.body_2_wrench):
                force = body_wrench.force
                maximum = max(maximum, float(np.linalg.norm(
                    (force.x, force.y, force.z))))
    return maximum


def _wrench_force(message):
    force = message.wrench.force
    return float(np.linalg.norm((force.x, force.y, force.z)))


def _rate_limit_targets(previous, desired, maximum_delta):
    previous = np.asarray(previous, dtype=np.float64)
    desired = np.asarray(desired, dtype=np.float64)
    if previous.shape != desired.shape:
        raise ValueError('target shapes differ')
    return previous + np.clip(
        desired - previous, -maximum_delta, maximum_delta)


class V12GazeboController(Node):
    """Run V12 at 50 Hz and publish bounded Gazebo position targets."""

    def __init__(self):
        super().__init__('h1_2_v12_gazebo_controller')
        package_share = get_package_share_directory('h1_2_navigation')
        default_contract = os.path.join(
            package_share, 'config', 'v12_locomotion.yaml')
        default_policy = os.path.join(
            package_share, 'policy', 'v12_model_17250_policy.npz')
        self.declare_parameter('contract_file', default_contract)
        self.declare_parameter('policy_file', default_policy)
        self.declare_parameter('enabled', False)
        self.declare_parameter('movement_authorized', False)
        self.contract = load_contract(
            self.get_parameter('contract_file').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.movement_authorized = bool(
            self.get_parameter('movement_authorized').value)
        self.policy = V12Policy.load(self.get_parameter('policy_file').value)
        self.runtime = V12LocomotionRuntime(self.contract, self.policy)
        self.runtime_config = self.contract['gazebo_runtime']
        actuator = self.contract['training_actuator']
        self.joints = tuple(actuator['controlled_joints'])
        if self.joints != EXPECTED_CONTROLLED_JOINTS:
            raise RuntimeError('controlled joint order changed')
        self.default_positions = np.asarray(
            actuator['default_positions'], dtype=np.float64)
        self.last_commanded_targets = self.default_positions.copy()
        self.leg_indices = np.asarray([
            self.joints.index(name) for name in EXPECTED_LEG_JOINTS])
        shoulder_names = self.contract['shoulder_pitch']['joints']
        self.shoulder_indices = np.asarray([
            self.joints.index(name) for name in shoulder_names])
        self.command = np.zeros(3, dtype=np.float32)
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        self.motion_arming = MotionArming(
            self.contract['control']['motion_onset_threshold'])
        self.positions = None
        self.velocities = None
        self.imu = None
        self.odom = None
        self.contact_forces = [0.0, 0.0]
        self.received = {}
        self.step_index = 0
        self.last_status = None
        self.fault = None
        self.fault_targets = None
        self.gazebo_stop_active = False
        self.gazebo_stop_complete = False
        self.gazebo_stop_stable_steps = 0
        self.gazebo_stop_targets = self.default_positions.copy()
        self.position_hold_targets = None
        self.burst_braking = False
        self.burst_motion_steps = 0
        self.burst_stable_steps = 0
        self.captured_effort_hold_active = False
        self.captured_effort_hold_targets = self.default_positions.copy()
        self.effort_settle_step = 0
        if self.runtime_config['controller_mode'] == 'position':
            self.control_mode = 'POSITION'
        else:
            self.control_mode = 'STANDBY_POSITION'
        self.switch_future = None
        self.switch_requested_at = None
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
            'Gazebo-only V12 controller initialized: '
            f'enabled={self.enabled}, '
            f'movement_authorized={self.movement_authorized}, '
            'odometry_mode=truth_aided_simulation')

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
            message.position[index[name]] for name in self.joints],
            dtype=np.float64)
        velocities = np.asarray([
            message.velocity[index[name]] for name in self.joints],
            dtype=np.float64)
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
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        rotation = _quaternion_rotation_matrix(quaternion)
        yaw = _yaw_from_quaternion(quaternion)
        yaw_quaternion = (
            0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))
        inverse_yaw_quaternion = (
            0.0, 0.0, -yaw_quaternion[2], yaw_quaternion[3])
        pelvis_relative_quaternion = _quaternion_multiply(
            inverse_yaw_quaternion, quaternion)
        world_linear = rotation @ np.asarray([
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        ])
        world_angular = rotation @ np.asarray([
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        ])
        cosine_yaw = math.cos(yaw)
        sine_yaw = math.sin(yaw)
        output = Odometry()
        output.header = message.header
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
            message.linear.x, message.linear.y, message.angular.z],
            dtype=np.float32)
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

    def _current_contact_forces(self):
        threshold = self.runtime_config['load_contact_threshold']
        return [
            force if force >= threshold else 0.0
            for force in self.contact_forces
        ]

    def _root_diagnostics(self):
        position = self.odom.pose.pose.position
        linear = self.odom.twist.twist.linear
        angular = self.odom.twist.twist.angular
        return {
            'root_position': [position.x, position.y, position.z],
            'root_linear_velocity': [linear.x, linear.y, linear.z],
            'root_angular_velocity': [angular.x, angular.y, angular.z],
        }

    def _gazebo_stop_step(self, imu_up_world, angular, contact_forces):
        if not self.gazebo_stop_active:
            self.gazebo_stop_active = True
            self.gazebo_stop_complete = False
            self.gazebo_stop_stable_steps = 0
            self.gazebo_stop_targets[:] = self.last_commanded_targets
            self.position_hold_targets = None
        rates = np.full(len(self.joints), 0.02, dtype=np.float64)
        rates[self.leg_indices] = np.asarray(
            self.contract['stop_controller']['stop_target_rate_limit'],
            dtype=np.float64)
        rates[self.shoulder_indices] = self.contract['stop_controller'][
            'stop_shoulder_rate_limit']
        if self.control_mode == 'EFFORT_PID':
            rates[:] = np.minimum(
                rates, self.runtime_config['effort_target_rate_limit'])
        else:
            rates[:] = np.minimum(
                rates, self.runtime_config['position_stop_target_rate_limit'])
        self.gazebo_stop_targets[:] = _rate_limit_targets(
            self.gazebo_stop_targets, self.default_positions, rates)
        self._publish_hold_targets(self.gazebo_stop_targets)
        self.step_index += 1

        linear = self.odom.twist.twist.linear
        stop = self.contract['stop_controller']
        target_ready = bool(np.allclose(
            self.gazebo_stop_targets, self.default_positions,
            rtol=0.0, atol=1.0e-8))
        stable = bool(
            target_ready
            and np.linalg.norm((linear.x, linear.y))
            < stop['capture_planar_speed_threshold']
            and abs(angular[2]) < stop['capture_yaw_speed_threshold']
            and imu_up_world[2] > stop['capture_minimum_up_z']
            and np.linalg.norm(angular[:2])
            < stop['capture_angular_velocity_xy_threshold']
            and abs(linear.z) < stop['capture_vertical_velocity_threshold']
            and all(force > 0.0 for force in contact_forces))
        self.gazebo_stop_stable_steps = (
            self.gazebo_stop_stable_steps + 1 if stable else 0)
        if (
                not self.gazebo_stop_complete
                and self.gazebo_stop_stable_steps
                >= self.runtime_config['stop_capture_dwell_steps']):
            self.gazebo_stop_complete = True
            self.runtime.reset()
        state = (
            'GAZEBO_HOLD' if self.gazebo_stop_complete
            else 'GAZEBO_BRAKE')
        position = self.odom.pose.pose.position
        self._status(
            state,
            capture_latched=self.gazebo_stop_complete,
            capture_stable_steps=self.gazebo_stop_stable_steps,
            contact_forces=contact_forces,
            raw_load_forces=self.contact_forces,
            command=self.command.tolist(),
            up_z=float(imu_up_world[2]),
            height=position.z,
            leg_positions=self.positions[self.leg_indices].tolist(),
            leg_velocities=self.velocities[self.leg_indices].tolist(),
            leg_targets=self.gazebo_stop_targets[self.leg_indices].tolist(),
            leg_target_error=(
                self.positions[self.leg_indices]
                - self.gazebo_stop_targets[self.leg_indices]).tolist(),
            **self._root_diagnostics())

    def _publish_standby_targets(self, targets):
        message = Float64MultiArray()
        values = np.asarray(targets, dtype=np.float64)
        message.data = values.tolist()
        self.standby_publisher.publish(message)
        self.last_commanded_targets[:] = values

    def _publish_effort_targets(self, targets):
        message = MultiDOFCommand()
        values = np.asarray(targets, dtype=np.float64)
        message.dof_names = list(self.joints)
        message.values = values.tolist()
        message.values_dot = [0.0] * len(self.joints)
        self.command_publisher.publish(message)
        self.last_commanded_targets[:] = values

    def _publish_hold_targets(self, targets):
        if self.control_mode == 'EFFORT_PID':
            self._publish_effort_targets(targets)
        else:
            self._publish_standby_targets(targets)

    def _publish_policy_targets(self, targets):
        if self.runtime_config['controller_mode'] == 'effort_pid':
            self._publish_effort_targets(targets)
        else:
            self._publish_standby_targets(targets)

    def _latch_switch_fault(self, fault, detail):
        self.fault = fault
        self.fault_targets = self.positions.copy()
        self.command.fill(0.0)
        self.get_logger().error(detail)

    def _effort_mode_ready(self):
        if self.control_mode == 'EFFORT_PID':
            if (
                    self.effort_settle_step
                    < self.runtime_config['effort_settle_steps']):
                self._publish_effort_targets(self.default_positions)
                self.effort_settle_step += 1
                self._status(
                    'EFFORT_PID_SETTLING',
                    settle_step=self.effort_settle_step,
                    settle_steps=self.runtime_config['effort_settle_steps'],
                    **self._root_diagnostics())
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
                self._latch_switch_fault(
                    'CONTROLLER_SWITCH_EXCEPTION',
                    f'controller switch raised: {error}')
                return False
            if not response.ok:
                self._latch_switch_fault(
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
        elapsed = self._now() - self.switch_requested_at
        if elapsed > self.runtime_config['switch_timeout']:
            self._latch_switch_fault(
                'CONTROLLER_SWITCH_TIMEOUT',
                'controller switch timed out')
            return False
        self._status('ARMING_EFFORT_PID')
        return False

    def _position_mode_ready(self):
        if self.control_mode == 'POSITION':
            return True
        if self.position_hold_targets is None:
            self.position_hold_targets = self.positions.copy()
        self._publish_standby_targets(self.position_hold_targets)
        if self.control_mode == 'EFFORT_PID':
            if not self.switch_client.service_is_ready():
                self._status('WAITING_FOR_CONTROLLER_SWITCH_SERVICE')
                return False
            request = SwitchController.Request()
            request.activate_controllers = [
                self.runtime_config['standby_controller_name']]
            request.deactivate_controllers = [
                self.runtime_config['effort_controller_name']]
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            timeout = self.runtime_config['switch_timeout']
            request.timeout.sec = int(timeout)
            request.timeout.nanosec = int(
                round((timeout - int(timeout)) * 1.0e9))
            self.switch_requested_at = self._now()
            self.switch_future = self.switch_client.call_async(request)
            self.control_mode = 'SWITCHING_TO_POSITION'
            self._status('ARMING_POSITION_HOLD')
            return False
        if self.control_mode != 'SWITCHING_TO_POSITION':
            self._latch_switch_fault(
                'INVALID_CONTROLLER_SWITCH_STATE',
                f'cannot switch to position from {self.control_mode}')
            return False
        if self.switch_future.done():
            try:
                response = self.switch_future.result()
            except Exception as error:
                self._latch_switch_fault(
                    'POSITION_SWITCH_EXCEPTION',
                    f'position controller switch raised: {error}')
                return False
            if not response.ok:
                self._latch_switch_fault(
                    'POSITION_SWITCH_REJECTED',
                    f'position controller switch rejected: {response.message}')
                return False
            self.control_mode = 'POSITION'
            self.switch_future = None
            self.switch_requested_at = None
            self._publish_standby_targets(self.position_hold_targets)
            self._status('POSITION_HOLD_READY')
            return True
        elapsed = self._now() - self.switch_requested_at
        if elapsed > self.runtime_config['switch_timeout']:
            self._latch_switch_fault(
                'POSITION_SWITCH_TIMEOUT',
                'position controller switch timed out')
            return False
        self._status('ARMING_POSITION_HOLD')
        return False

    def _status(self, state, **extra):
        payload = {
            'state': state,
            'enabled': self.enabled,
            'movement_authorized': self.movement_authorized,
            'fresh': self._fresh(),
            'fault': self.fault,
            'controller_mode': self.control_mode,
            'step_index': self.step_index,
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
            hold_targets = (
                self.fault_targets if self.fault_targets is not None
                else self.last_commanded_targets)
            self._publish_hold_targets(hold_targets)
            self._status('WAITING_FOR_FRESH_STATE')
            return
        quaternion, angular = self.imu
        imu_up_world = _imu_up_world(quaternion)
        position = self.odom.pose.pose.position
        if (
                imu_up_world[2] < self.runtime_config['fall_minimum_up_z']
                or position.z < self.runtime_config['fall_minimum_height']):
            if self.fault is None:
                self.fault = 'FALL_ENVELOPE'
                self.fault_targets = self.positions.copy()
                self.get_logger().error(
                    'V12 fall envelope breached; controller hold latched')
            self.command.fill(0.0)
            self._publish_hold_targets(self.fault_targets)
            self._status(
                'FAULT_HOLD', up_z=float(imu_up_world[2]),
                height=position.z)
            return
        if self.fault is not None:
            self._publish_hold_targets(self.fault_targets)
            self._status('FAULT_HOLD')
            return
        if self.motion_arming.standby(self.command):
            contact_forces = self._current_contact_forces()
            self._publish_standby_targets(self.default_positions)
            self._status(
                'STANDBY_HOLD',
                contact_forces=contact_forces,
                command=self.command.tolist(),
                up_z=float(imu_up_world[2]),
                height=position.z,
                **self._root_diagnostics(),
            )
            return
        if self.runtime_config['deterministic_stop_adapter']:
            exact_zero = bool(np.all(self.command == 0.0))
            restart_ready = self.motion_arming.reaches_onset(self.command)
            if (
                    self.gazebo_stop_active
                    and self.gazebo_stop_complete
                    and not exact_zero
                    and restart_ready):
                self.gazebo_stop_active = False
                self.gazebo_stop_complete = False
                self.gazebo_stop_stable_steps = 0
                self.runtime.reset()
                self.step_index = 0
            elif self.gazebo_stop_active or exact_zero:
                if not self._position_mode_ready():
                    return
                self._gazebo_stop_step(
                    imu_up_world, angular,
                    self._current_contact_forces())
                return
        if (
                self.runtime_config['controller_mode'] == 'effort_pid'
                and not self._effort_mode_ready()):
            if self.control_mode != 'EFFORT_PID':
                self._publish_standby_targets(self.default_positions)
            return
        linear = self.odom.twist.twist.linear
        contact_forces = self._current_contact_forces()
        requested_command = self.command.copy()
        if self.captured_effort_hold_active:
            root_linear_velocity = np.asarray(
                [linear.x, linear.y, linear.z], dtype=np.float32)
            self.captured_effort_hold_targets[:] = _rate_limit_targets(
                self.captured_effort_hold_targets,
                self.default_positions,
                self.runtime_config['captured_hold_return_rate'])
            stop = self.contract['stop_controller']
            captured_breach = bool(
                np.linalg.norm(root_linear_velocity[:2])
                > stop['recovery_planar_speed_threshold']
                or abs(angular[2]) > stop['recovery_yaw_speed_threshold']
                or np.linalg.norm(angular[:2])
                > stop['recovery_angular_velocity_xy_threshold']
                or abs(root_linear_velocity[2])
                > stop['recovery_vertical_velocity_threshold']
                or float(imu_up_world[2]) < stop['recovery_minimum_up_z']
                or not all(force > 0.0 for force in contact_forces))
            if captured_breach:
                self.captured_effort_hold_active = False
                self.burst_stable_steps = 0
            else:
                captured_stable = bool(
                    np.linalg.norm(root_linear_velocity[:2])
                    < self.runtime_config['burst_restart_planar_speed']
                    and np.linalg.norm(angular[:2])
                    < self.runtime_config['captured_hold_angular_speed']
                    and abs(root_linear_velocity[2])
                    < self.runtime_config['captured_hold_vertical_speed']
                    and float(imu_up_world[2])
                    > self.runtime_config['burst_restart_minimum_up_z'])
                self.burst_stable_steps = (
                    self.burst_stable_steps + 1 if captured_stable else 0)
                restart_ready = bool(
                    self.burst_stable_steps
                    >= self.runtime_config['burst_restart_dwell_steps']
                    and self.motion_arming.reaches_onset(requested_command))
                if restart_ready:
                    self.captured_effort_hold_active = False
                    self.burst_braking = False
                    self.burst_motion_steps = 0
                    self.burst_stable_steps = 0
                    self.runtime.reset()
                    self.step_index = 0
                else:
                    self._publish_hold_targets(
                        self.captured_effort_hold_targets)
                    self._status(
                        'CAPTURED_HOLD',
                        capture_latched=True,
                        contact_forces=contact_forces,
                        raw_load_forces=self.contact_forces,
                        command=[0.0, 0.0, 0.0],
                        requested_command=requested_command.tolist(),
                        burst_braking=True,
                        burst_motion_steps=0,
                        burst_stable_steps=self.burst_stable_steps,
                        up_z=float(imu_up_world[2]),
                        height=position.z,
                        leg_positions=self.positions[
                            self.leg_indices].tolist(),
                        leg_velocities=self.velocities[
                            self.leg_indices].tolist(),
                        leg_targets=self.captured_effort_hold_targets[
                            self.leg_indices].tolist(),
                        leg_target_error=(
                            self.positions[self.leg_indices]
                            - self.captured_effort_hold_targets[
                                self.leg_indices]).tolist(),
                        **self._root_diagnostics(),
                    )
                    return
        policy_command = requested_command.copy()
        if not self.runtime_config['deterministic_stop_adapter']:
            if (
                    self.runtime.capture_latched
                    and not self.motion_arming.reaches_onset(
                        requested_command)):
                policy_command.fill(0.0)
            if (
                    self.burst_braking
                    or (
                        self.runtime.stop_state == self.runtime.MOVE
                        and (
                            self.burst_motion_steps
                            >= self.runtime_config['motion_burst_steps']
                            or np.linalg.norm((linear.x, linear.y))
                            >= self.runtime_config[
                                'motion_burst_planar_speed']))):
                self.burst_braking = True
                self.burst_motion_steps = 0
                policy_command.fill(0.0)
        root_linear_velocity = np.asarray(
            [linear.x, linear.y, linear.z], dtype=np.float32)
        self.step_index += 1
        sample = V12SensorState(
            step_index=self.step_index,
            imu_gyro=angular,
            imu_up_vector=imu_up_world,
            root_linear_velocity_body=root_linear_velocity,
            root_angular_velocity_body=angular,
            torso_up_z=float(imu_up_world[2]),
            leg_positions=self.positions[self.leg_indices],
            leg_velocities=self.velocities[self.leg_indices],
            shoulder_pitch_positions=self.positions[self.shoulder_indices],
            foot_force_magnitudes=contact_forces,
        )
        output = self.runtime.step(sample, policy_command)
        targets = self.default_positions.copy()
        leg_targets = output.leg_position_targets
        if self.runtime_config['controller_mode'] == 'effort_pid':
            target_rate_limit = (
                self.runtime_config['effort_target_rate_limit']
                if output.mode == 'MOVE'
                else np.asarray(
                    self.contract['stop_controller'][
                        'stop_target_rate_limit'],
                    dtype=np.float64))
        else:
            target_rate_limit = self.runtime_config[
                'position_target_rate_limit']
        leg_targets = _rate_limit_targets(
            self.last_commanded_targets[self.leg_indices], leg_targets,
            target_rate_limit)
        self.runtime.motor_targets[:] = leg_targets
        targets[self.leg_indices] = leg_targets
        targets[self.shoulder_indices] = output.shoulder_pitch_targets
        self._publish_policy_targets(targets)
        if (
                self.runtime_config['captured_effort_hold']
                and self.burst_braking
                and output.capture_latched
                and np.linalg.norm(root_linear_velocity[:2])
                < self.runtime_config['burst_restart_planar_speed']
                and np.linalg.norm(angular[:2])
                < self.runtime_config['captured_hold_angular_speed']
                and abs(root_linear_velocity[2])
                < self.runtime_config['captured_hold_vertical_speed']
                and float(imu_up_world[2])
                > self.runtime_config['burst_restart_minimum_up_z']
                and all(force > 0.0 for force in contact_forces)):
            self.captured_effort_hold_active = True
            self.captured_effort_hold_targets[:] = targets
        if output.mode == 'MOVE' and not self.burst_braking:
            self.burst_motion_steps += 1
        if self.burst_braking:
            burst_stable = bool(
                output.capture_latched
                and np.linalg.norm(root_linear_velocity[:2])
                < self.runtime_config['burst_restart_planar_speed']
                and float(imu_up_world[2])
                > self.runtime_config['burst_restart_minimum_up_z']
                and all(force > 0.0 for force in contact_forces))
            self.burst_stable_steps = (
                self.burst_stable_steps + 1 if burst_stable else 0)
            if (
                    self.burst_stable_steps
                    >= self.runtime_config['burst_restart_dwell_steps']
                    and self.motion_arming.reaches_onset(requested_command)):
                self.burst_braking = False
                self.burst_motion_steps = 0
                self.burst_stable_steps = 0
        else:
            self.burst_stable_steps = 0
        self._status(
            output.mode,
            capture_latched=output.capture_latched,
            contact_forces=contact_forces,
            raw_load_forces=self.contact_forces,
            command=policy_command.tolist(),
            requested_command=requested_command.tolist(),
            burst_braking=self.burst_braking,
            burst_motion_steps=self.burst_motion_steps,
            burst_stable_steps=self.burst_stable_steps,
            up_z=float(imu_up_world[2]),
            height=position.z,
            imu_gyro=list(angular),
            imu_up_vector=imu_up_world.tolist(),
            policy_action=output.policy_action.tolist(),
            leg_positions=self.positions[self.leg_indices].tolist(),
            leg_velocities=self.velocities[self.leg_indices].tolist(),
            leg_targets=leg_targets.tolist(),
            raw_leg_targets=output.leg_position_targets.tolist(),
            leg_target_error=(
                self.positions[self.leg_indices]
                - leg_targets).tolist(),
            **self._root_diagnostics(),
        )


def main(args=None):
    """Run the Gazebo-only V12 controller."""
    rclpy.init(args=args)
    node = V12GazeboController()
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
