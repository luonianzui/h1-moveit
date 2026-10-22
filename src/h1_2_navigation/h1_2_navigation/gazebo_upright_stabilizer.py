"""Gazebo-only upright assistance with no planar propulsion."""

from dataclasses import replace
import json

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.msg import EntityWrench
from std_msgs.msg import String

from .upright_assistance import compute_upright_assistance
from .upright_assistance import UprightAssistanceConfig


class GazeboUprightStabilizer(Node):
    def __init__(self):
        super().__init__('h1_2_gazebo_upright_stabilizer')
        defaults = {
            'enabled': False,
            'movement_authorized': False,
            'entity_name': 'h1_2_navigation::pelvis',
            'odom_topic': '/h1_2_navigation/ground_truth/odom',
            'wrench_topic': '/world/h1_world/wrench',
            'status_topic': '/h1_2_navigation/upright_assistance/status',
            'v12_status_topic': '/h1_2_navigation/v12/status',
            'nominal_height': 1.08912,
            'mass': 67.249,
            'gravity': 9.8,
            'vertical_stiffness': 2500.0,
            'vertical_damping': 350.0,
            'maximum_vertical_force_fraction': 0.30,
            'planar_damping': 300.0,
            'captured_planar_damping': 900.0,
            'maximum_planar_damping_force': 300.0,
            'attitude_stiffness': 350.0,
            'attitude_damping': 60.0,
            'maximum_attitude_torque': 120.0,
            'minimum_up_z': 0.60,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.movement_authorized = bool(
            self.get_parameter('movement_authorized').value)
        if not self.enabled or not self.movement_authorized:
            raise RuntimeError(
                'Gazebo upright assistance requires enabled=true and '
                'movement_authorized=true')
        self.entity_name = str(self.get_parameter('entity_name').value)
        self.config = UprightAssistanceConfig(**{
            name: self.get_parameter(name).value
            for name in (
                'nominal_height',
                'mass',
                'gravity',
                'vertical_stiffness',
                'vertical_damping',
                'maximum_vertical_force_fraction',
                'planar_damping',
                'maximum_planar_damping_force',
                'attitude_stiffness',
                'attitude_damping',
                'maximum_attitude_torque',
                'minimum_up_z',
            )
        })
        self.config.validate()
        self.captured_config = replace(
            self.config,
            planar_damping=float(
                self.get_parameter('captured_planar_damping').value),
        )
        self.captured_config.validate()
        self.captured_hold_damping_active = False
        self.v12_state = 'UNKNOWN'
        self.wrench_publisher = self.create_publisher(
            EntityWrench,
            str(self.get_parameter('wrench_topic').value),
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.v12_status_subscription = self.create_subscription(
            String,
            str(self.get_parameter('v12_status_topic').value),
            self._v12_status_callback,
            10,
        )
        self.sample_count = 0
        self.get_logger().warning(
            'Gazebo-only balance assistance active: vertical support, '
            'roll/pitch torque, and strictly dissipative planar damping; '
            'yaw torque is zero')

    def _v12_status_callback(self, message):
        try:
            state = str(json.loads(message.data).get('state', 'UNKNOWN'))
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return
        self.v12_state = state
        if state == 'CAPTURED_HOLD':
            self.captured_hold_damping_active = True
        elif state in {
                'MOVE',
                'STANDBY_HOLD',
                'FAULT_HOLD',
                'EFFORT_PID_SETTLING',
                'ARMING_EFFORT_PID',
        }:
            self.captured_hold_damping_active = False

    def _odom_callback(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        result = compute_upright_assistance(
            pose.position.z,
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
            ),
            (
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ),
            self.captured_config
            if self.captured_hold_damping_active else self.config,
        )
        wrench = EntityWrench()
        wrench.header.stamp = message.header.stamp
        wrench.header.frame_id = 'world'
        wrench.entity.name = self.entity_name
        wrench.entity.type = Entity.LINK
        wrench.wrench.force.x = float(result.force_world[0])
        wrench.wrench.force.y = float(result.force_world[1])
        wrench.wrench.force.z = float(result.force_world[2])
        wrench.wrench.torque.x = float(result.torque_world[0])
        wrench.wrench.torque.y = float(result.torque_world[1])
        self.wrench_publisher.publish(wrench)
        self.sample_count += 1
        if self.sample_count % 10 == 0:
            status = String()
            status.data = json.dumps({
                'mode': 'GAZEBO_ASSISTED_UPRIGHT',
                'active': result.active,
                'force_world': result.force_world.tolist(),
                'torque_world': result.torque_world.tolist(),
                'height': pose.position.z,
                'up_z': float(result.up_world[2]),
                'velocity_world': result.linear_velocity_world.tolist(),
                'v12_state': self.v12_state,
                'captured_hold_damping_active': (
                    self.captured_hold_damping_active),
                'planar_damping': (
                    self.captured_config.planar_damping
                    if self.captured_hold_damping_active
                    else self.config.planar_damping),
            }, separators=(',', ':'))
            self.status_publisher.publish(status)


def main(arguments=None):
    rclpy.init(args=arguments)
    node = GazeboUprightStabilizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
