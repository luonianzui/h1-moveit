"""Default-disabled ROS adapter for future Nav2 velocity commands."""

import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .locomotion_contract import EXPECTED_GATES
from .locomotion_contract import load_contract
from .velocity_gate import VelocityGate
from .velocity_gate import VelocityLimits


class Nav2VelocityGate(Node):
    """Publish only fail-closed, contract-limited policy commands."""

    def __init__(self):
        super().__init__('h1_2_nav2_velocity_gate')
        default_contract = os.path.join(
            get_package_share_directory('h1_2_navigation'),
            'config',
            'v12_locomotion.yaml',
        )
        self.declare_parameter('contract_file', default_contract)
        contract = load_contract(self.get_parameter('contract_file').value)
        deployment = contract['deployment']
        for gate_name in EXPECTED_GATES:
            self.declare_parameter(
                gate_name, bool(deployment.get(gate_name, False)))
        self.gates = {
            name: bool(self.get_parameter(name).value)
            for name in EXPECTED_GATES
        }
        self.velocity_gate = VelocityGate(VelocityLimits.from_contract(contract))
        self.publisher = self.create_publisher(
            Twist, deployment['output_topic'], 10)
        self.subscription = self.create_subscription(
            Twist, deployment['input_topic'], self._command_callback, 10)
        self.timer = self.create_timer(
            contract['control']['control_period'], self._watchdog_callback)
        self.get_logger().warning(
            'V12 velocity gate initialized fail-closed; all six gates are '
            f'required, current gates={self.gates}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _publish(self, values):
        message = Twist()
        message.linear.x = float(values[0])
        message.linear.y = float(values[1])
        message.angular.z = float(values[2])
        self.publisher.publish(message)

    def _command_callback(self, message):
        values = self.velocity_gate.apply(
            (message.linear.x, message.linear.y, message.angular.z),
            self._now(),
            self.gates,
        )
        self._publish(values)

    def _watchdog_callback(self):
        self._publish(self.velocity_gate.watchdog(self._now()))


def main(args=None):
    """Run the default-disabled Nav2 velocity gate."""
    rclpy.init(args=args)
    node = Nav2VelocityGate()
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
