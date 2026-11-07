import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class D(Node):
    def __init__(self):
        super().__init__('depth_check')
        self.done = False
        self.create_subscription(Image, '/camera/depth_image', self.cb, 10)

    def cb(self, msg):
        d = np.frombuffer(msg.data, dtype=np.float32)
        fin = d[np.isfinite(d)]
        print('encoding:', msg.encoding, 'pixels:', d.size,
              'finite:', fin.size)
        if fin.size:
            print('depth min', fin.min(), 'max', fin.max(),
                  'mean', fin.mean())
        else:
            print('no finite depth values: camera sees nothing in range')
        self.done = True


rclpy.init()
n = D()
t = time.time()
while rclpy.ok() and not n.done and time.time() - t < 60:
    rclpy.spin_once(n, timeout_sec=1.0)
if not n.done:
    print('No depth image in 60 s')
n.destroy_node()
rclpy.shutdown()
