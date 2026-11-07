import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, PointStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

FRAME = 'base_link'
LINK = 'tool0'


class MoveToCube(Node):
    def __init__(self, color):
        super().__init__('move_to_cube')
        self.target = None
        self.create_subscription(
            PointStamped, '/detected/' + color, self.cb, 10)
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def cb(self, msg):
        if self.target is None:      # keep the first detection only
            self.target = msg.point

    def build_goal(self, x, y, z):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'ur_manipulator'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = LINK
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]          # 1 cm tolerance
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = LINK
        oc.orientation.x = 1.0              # tool z-axis pointing straight down
        oc.orientation.w = 0.0
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 3.14  # any rotation around the vertical
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        req.goal_constraints = [c]
        goal.planning_options.plan_only = False
        return goal


def main():
    color = sys.argv[1] if len(sys.argv) > 1 else 'red'
    hover = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    rclpy.init()
    node = MoveToCube(color)

    node.get_logger().info('waiting for /detected/%s ...' % color)
    while rclpy.ok() and node.target is None:
        rclpy.spin_once(node, timeout_sec=1.0)
    t = node.target
    node.get_logger().info('target (%.3f, %.3f, %.3f), hovering %.2f m above'
                           % (t.x, t.y, t.z, hover))

    if not node.client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error('MoveIt /move_action not available')
        return
    future = node.client.send_goal_async(node.build_goal(t.x, t.y, t.z + hover))
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        node.get_logger().error('goal rejected')
        return
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    code = result_future.result().result.error_code.val
    node.get_logger().info('finished, MoveIt error code = %d (1 means success)' % code)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()