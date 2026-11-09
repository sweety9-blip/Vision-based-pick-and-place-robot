import math
import os
import random
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT = os.path.expanduser('~/pickplace_ws/dataset')
X_RANGE = (0.38, 0.70)
Y_RANGE = (-0.30, 0.30)
MIN_GAP = 0.12
CLASSES = ['red', 'green']


def set_pose(model, x, y, yaw):
    req = ('name: "%s" position {x: %.4f y: %.4f z: 0.035} '
           'orientation {z: %.5f w: %.5f}'
           % (model, x, y, math.sin(yaw / 2), math.cos(yaw / 2)))
    subprocess.run(['gz', 'service', '-s', '/world/empty/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000', '--req', req],
                   capture_output=True, text=True, timeout=20)


def boxes(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    masks = {
        'red': cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
               | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255)),
        'green': cv2.inRange(hsv, (40, 120, 70), (80, 255, 255)),
    }
    out = {}
    for name, m in masks.items():
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            continue
        c = max(cs, key=cv2.contourArea)
        if cv2.contourArea(c) < 150:
            continue
        out[name] = cv2.boundingRect(c)   # x, y, w, h in pixels
    return out


class Grabber(Node):
    def __init__(self):
        super().__init__('dataset_grabber')
        self.count = 0
        self.img = None
        self.create_subscription(Image, '/camera/image', self.cb, 10)

    def cb(self, msg):
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        self.img = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else a
        self.count += 1

    def fresh(self, timeout=40.0):
        """Wait for two new frames, so the picture shows the new cube poses."""
        start = self.count
        t = time.time()
        while self.count < start + 2 and time.time() - t < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.img if self.count >= start + 2 else None


def pick_two(rng):
    while True:
        r = (rng.uniform(*X_RANGE), rng.uniform(*Y_RANGE))
        g = (rng.uniform(*X_RANGE), rng.uniform(*Y_RANGE))
        if math.hypot(r[0] - g[0], r[1] - g[1]) >= MIN_GAP:
            return r, g


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    rng = random.Random(seed)
    for sub in ('images/train', 'images/val', 'labels/train', 'labels/val'):
        os.makedirs(os.path.join(OUT, sub), exist_ok=True)
    rclpy.init()
    node = Grabber()
    saved = 0
    for i in range(n):
        r, g = pick_two(rng)
        set_pose('red_cube', r[0], r[1], rng.uniform(0, math.pi / 2))
        set_pose('green_cube', g[0], g[1], rng.uniform(0, math.pi / 2))
        time.sleep(2.0)
        img = node.fresh()
        if img is None:
            print('sample %d: no image received' % i)
            continue
        found = boxes(img)
        if len(found) < 2:
            print('sample %d: skipped, found only %s' % (i, list(found)))
            continue
        split = 'val' if rng.random() < 0.2 else 'train'
        name = 's%d_%d' % (seed, i)
        h, w = img.shape[:2]
        cv2.imwrite(os.path.join(OUT, 'images', split, name + '.png'), img)
        with open(os.path.join(OUT, 'labels', split, name + '.txt'), 'w') as f:
            for cname, (x, y, bw, bh) in found.items():
                f.write('%d %.6f %.6f %.6f %.6f\n' % (
                    CLASSES.index(cname), (x + bw / 2) / w, (y + bh / 2) / h,
                    bw / w, bh / h))
        saved += 1
        print('sample %d saved (%s)' % (i, split), flush=True)
    with open(os.path.join(OUT, 'data.yaml'), 'w') as f:
        f.write('path: %s\ntrain: images/train\nval: images/val\n'
                'names:\n  0: red\n  1: green\n' % OUT)
    print('saved %d of %d' % (saved, n))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    