import math
import re
import subprocess
import sys
import time

import rclpy
from pickplace_manipulation.pick_descend import PickDescend

ROBOT_ID = 10   # entity id of the "ur" model (it was 10 in every log so far)
ATTACH = '/model/ur/detachable_joint/attach'
DETACH = '/model/ur/detachable_joint/detach'


def sh(cmd, timeout=30):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr
    except Exception as e:
        return 'ERROR %s' % e


def topic_exists(name):
    return name in sh(['gz', 'topic', '-l']).split()


def publish_empty(topic):
    sh(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Empty', '-p', ' '])


def load_plugin(model):
    """First grasp of the session: loading the plugin welds the cube."""
    req = ('entity {id: %d} plugins {filename: "gz-sim-detachable-joint-system" '
           'name: "gz::sim::systems::DetachableJoint" innerxml: '
           '"<parent_link>wrist_3_link</parent_link>'
           '<child_model>%s</child_model><child_link>link</child_link>"}'
           % (ROBOT_ID, model))
    out = sh(['gz', 'service', '-s', '/world/empty/entity/system/add',
              '--reqtype', 'gz.msgs.EntityPlugin_V',
              '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
              '--req', req])
    return 'true' in out


def cube_position(model):
    out = sh(['gz', 'topic', '-e', '-n', '1', '-t',
              '/world/empty/dynamic_pose/info'])
    i = out.find('name: "%s"' % model)
    if i < 0:
        return None
    j = out.find('position {', i)
    k = out.find('}', j)
    block = out[j:k]
    vals = {}
    for axis in 'xyz':
        m = re.search(r'\b%s: ([-0-9.eE+]+)' % axis, block)
        vals[axis] = float(m.group(1)) if m else 0.0
    return (vals['x'], vals['y'], vals['z'])

def release(model, log, tries=3):
    """Send detach, then wait for the cube to drop. Returns True if it landed."""
    for n in range(tries):
        publish_empty(DETACH)
        log.info('detach message sent (%d of %d)' % (n + 1, tries))
        end = time.time() + 8.0
        while time.time() < end:
            time.sleep(1.0)
            c = cube_position(model)
            if c is not None and c[2] < 0.06:
                log.info('release OK, cube z = %.3f' % c[2])
                return True
    log.error('RELEASE FAILED: cube still in the air')
    return False

class PickPlace(PickDescend):
    def go(self, x, y, z, tol, tries=3):
        for n in range(tries):
            if self.move_to(x, y, z, tol):
                return True
            self.get_logger().warn('move failed, retry %d of %d' % (n + 1, tries))
        return False

    def go_settled(self, x, y, z, tol, tries=3):
        if not self.go(x, y, z, tol, tries):
            return None
        return self.settle()

    def descend(self, goal):
        cmd = goal
        p = None
        for attempt in range(3):
            p = self.go_settled(cmd[0], cmd[1], cmd[2], 0.003)
            if p is None:
                return None
            err = [g - a for g, a in zip(goal, p)]
            dist = math.sqrt(sum(e * e for e in err))
            self.get_logger().info('descent attempt %d: error %.1f mm'
                                   % (attempt + 1, dist * 1000))
            if dist < 0.005:
                break
            cmd = tuple(c + e for c, e in zip(cmd, err))
        return p


def main():
    color = sys.argv[1] if len(sys.argv) > 1 else 'red'
    px = float(sys.argv[2]) if len(sys.argv) > 2 else 0.50
    py = float(sys.argv[3]) if len(sys.argv) > 3 else -0.25
    hover = 0.15
    approach = 0.05
    lift = 0.30
    model = color + '_cube'

    rclpy.init()
    node = PickPlace(color)
    log = node.get_logger()
    if color != 'red':
        log.error('only the red cube is supported (gripper plugin is bound to red_cube)')
        return

    # 1. detect once and store the position
    log.info('waiting for /detected/%s (60 s max) ...' % color)
    start = time.time()
    while rclpy.ok() and node.target is None and time.time() - start < 60:
        rclpy.spin_once(node, timeout_sec=1.0)
    if node.target is None:
        log.error('no detection: is object_detector running and the arm out of view?')
        return
    t = node.target
    log.info('cube at (%.3f, %.3f, %.3f), place point (%.2f, %.2f)'
             % (t.x, t.y, t.z, px, py))
    if not node.client.wait_for_server(timeout_sec=20.0):
        log.error('MoveIt /move_action not available')
        return
    t0 = time.time()

    # 2. hover, then descend
    if node.go_settled(t.x, t.y, t.z + hover, 0.01) is None:
        log.error('hover failed')
        return
    if node.descend((t.x, t.y, t.z + approach)) is None:
        log.error('descent failed')
        return

    # 3. grasp (plugin load the first time, attach message afterwards)
    if topic_exists(ATTACH):
        publish_empty(ATTACH)
        log.info('attach message sent')
    elif load_plugin(model):
        log.info('gripper plugin loaded (cube welded)')
    else:
        log.error('could not load gripper plugin')
        return
    time.sleep(1.0)

    # 4. lift, and check the cube really came with the arm
    if node.go_settled(t.x, t.y, t.z + lift, 0.01) is None:
        log.error('lift failed')
        return
    c = cube_position(model)
    if c is None or c[2] < t.z + 0.1:
        log.error('GRASP FAILED: cube z = %s' % (c,))
        return
    log.info('grasp OK, cube z = %.3f' % c[2])

    # 5. carry to the place point and lower
    if node.go_settled(px, py, t.z + lift, 0.01) is None:
        log.error('carry failed')
        return
    if node.descend((px, py, t.z + approach + 0.01)) is None:
        log.error('lowering failed')
        return

    # 6. release and retreat
        if not release(model, log):
        log.error('stopping without retreat so the cube is not lifted again')
        return
     node.go_settled(px, py, t.z + lift, 0.01)

    # 7. report
    c = cube_position(model)
    cycle = time.time() - t0
    if c is None:
        log.warn('could not read the final cube position')
    else:
        err = math.hypot(c[0] - px, c[1] - py)
        ok = err < 0.03 and c[2] < 0.06
        log.info('final cube (%.3f, %.3f, %.3f), place error %.1f mm, %s'
                 % (c[0], c[1], c[2], err * 1000, 'SUCCESS' if ok else 'FAILED'))
    log.info('cycle time %.0f s' % cycle)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    