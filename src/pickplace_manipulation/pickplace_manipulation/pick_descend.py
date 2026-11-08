import sys
import time
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose, PointStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

FRAME = 'base_link'
LINK = 'tool0'


class PickDescend(Node):
    def __init__(self, color):
        super().__init__('pick_descend')
        self.target = None
        self.create_subscription(PointStamped, '/detected/' + color, self.cb, 10)
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def cb(self, msg):
        if self.target is None:
            self.target = msg.point

    def tool_pos(self):
        try:
            t = self.tf_buffer.lookup_transform(FRAME, LINK, Time()).transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    def settle(self, timeout=25.0):
        """Wait until the tool position stops changing (3 samples within 1 mm)."""
        start = time.time()
        last_sample = 0.0
        last = None
        stable = 0
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - last_sample < 0.5:
                continue
            last_sample = time.time()
            p = self.tool_pos()
            if p is None:
                continue
            if last is not None and max(abs(a - b) for a, b in zip(p, last)) < 0.001:
                stable += 1
                if stable >= 3:
                    return p
            else:
                stable = 0
            last = p
        self.get_logger().warn('arm did not fully settle within timeout')
        return self.tool_pos()

    def build_goal(self, x, y, z, tol):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'ur_manipulator'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 30.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = LINK
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tol]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w = 1.0
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = LINK
        oc.orientation.x = 1.0
        oc.orientation.w = 0.0
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        req.goal_constraints = [c]
        goal.planning_options.plan_only = False
        return goal

    def move_to(self, x, y, z, tol):
        future = self.client.send_goal_async(self.build_goal(x, y, z, tol))
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code.val
        self.get_logger().info('MoveIt error code = %d' % code)
        return code == 1


def main():
    color = sys.argv[1] if len(sys.argv) > 1 else 'red'
    hover = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    approach = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    rclpy.init()
    node = PickDescend(color)
    log = node.get_logger()

    log.info('waiting for /detected/%s ...' % color)
    while rclpy.ok() and node.target is None:
        rclpy.spin_once(node, timeout_sec=1.0)
    t = node.target
    log.info('target (%.3f, %.3f, %.3f)' % (t.x, t.y, t.z))
    if not node.client.wait_for_server(timeout_sec=20.0):
        log.error('MoveIt /move_action not available')
        return

    # 1. hover
    if not node.move_to(t.x, t.y, t.z + hover, 0.01):
        log.error('hover move failed')
        return
    p = node.settle()
    log.info('hovering at (%.3f, %.3f, %.3f)' % p)

    # 2. descend, then correct the remaining error from the real tool pose
    goal = (t.x, t.y, t.z + approach)
    cmd = goal
    for attempt in range(3):
        if not node.move_to(cmd[0], cmd[1], cmd[2], 0.003):
            log.error('descent failed')
            return
        p = node.settle()
        err = [g - a for g, a in zip(goal, p)]
        dist = math.sqrt(sum(e * e for e in err))
        log.info('attempt %d: tool (%.3f, %.3f, %.3f), error %.1f mm'
                 % (attempt + 1, p[0], p[1], p[2], dist * 1000))
        if dist < 0.005:
            log.info('within 5 mm, done')
            break
        cmd = tuple(c + e for c, e in zip(cmd, err))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()