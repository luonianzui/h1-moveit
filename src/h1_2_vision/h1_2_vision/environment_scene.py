#!/usr/bin/env python3

"""Keep the known Gazebo table in MoveIt's collision world."""

from functools import partial
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class EnvironmentScene(Node):
    """Apply the exact table geometry with a configurable safety margin."""

    def __init__(self):
        super().__init__('h1_2_environment_scene')
        self.declare_parameter('planning_frame', 'pelvis')
        self.declare_parameter('table_x', 0.5)
        self.declare_parameter('table_y', 0.0)
        self.declare_parameter('table_center_z', -0.2)
        self.declare_parameter('safety_margin', 0.01)
        self._client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        self._request_in_flight = False
        self._applied_once = False
        self._last_apply_monotonic = None
        self._timer = self.create_timer(0.5, self._apply_if_ready)

    def _make_table(self):
        margin = float(self.get_parameter('safety_margin').value)
        if not 0.0 <= margin <= 0.05:
            raise ValueError('safety_margin must be in [0, 0.05] m')
        frame = str(self.get_parameter('planning_frame').value)
        table_x = float(self.get_parameter('table_x').value)
        table_y = float(self.get_parameter('table_y').value)
        table_z = float(self.get_parameter('table_center_z').value)

        collision = CollisionObject()
        collision.header.frame_id = frame
        collision.id = 'environment_table_10mm_margin'
        collision.operation = CollisionObject.ADD

        surface = SolidPrimitive()
        surface.type = SolidPrimitive.BOX
        surface.dimensions = [
            0.8 + 2.0 * margin,
            1.2 + 2.0 * margin,
            0.03 + 2.0 * margin,
        ]
        surface_pose = Pose()
        surface_pose.position.x = table_x
        surface_pose.position.y = table_y
        surface_pose.position.z = table_z
        surface_pose.orientation.w = 1.0
        collision.primitives.append(surface)
        collision.primitive_poses.append(surface_pose)

        # The SDF leg poses are relative to the table model at world x=0.5.
        for local_x in (0.35, -0.35):
            for local_y in (0.55, -0.55):
                leg = SolidPrimitive()
                leg.type = SolidPrimitive.CYLINDER
                leg.dimensions = [0.8 + 2.0 * margin, 0.02 + margin]
                leg_pose = Pose()
                leg_pose.position.x = table_x + local_x
                leg_pose.position.y = table_y + local_y
                # Gazebo leg center world z=0.4 and pelvis world z=1.0.
                leg_pose.position.z = -0.6
                leg_pose.orientation.w = 1.0
                collision.primitives.append(leg)
                collision.primitive_poses.append(leg_pose)
        return collision

    def _apply_if_ready(self):
        if self._request_in_flight or not self._client.service_is_ready():
            return
        now = time.monotonic()
        if (self._last_apply_monotonic is not None and
                now - self._last_apply_monotonic < 5.0):
            return
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects.append(self._make_table())
        self._request_in_flight = True
        future = self._client.call_async(request)
        future.add_done_callback(partial(self._applied, first=not self._applied_once))

    def _applied(self, future, first):
        self._request_in_flight = False
        try:
            success = bool(future.result().success)
        except Exception as error:  # noqa: BLE001 - ROS future exceptions vary.
            self.get_logger().error(f'Failed to apply table collision scene: {error}')
            return
        if not success:
            self.get_logger().error('MoveIt rejected the table collision scene')
            return
        if first:
            self.get_logger().info(
                'Applied Gazebo table to MoveIt with a 10 mm safety margin')
        self._applied_once = True
        self._last_apply_monotonic = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentScene()
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
