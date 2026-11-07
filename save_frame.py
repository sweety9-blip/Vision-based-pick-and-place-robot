import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Grab(Node):
    def __init__(self):
        super().__init__('frame_grabber')
        self.done = False
        self.create_subscription(Image, '/camera/image', self.cb, 10)

    def cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        print('Got image:', msg.width, 'x', msg.height, msg.encoding)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite('frame.png', img)
        print('Saved frame.png')
        self.done = True


rclpy.init()
node = Grab()
start = time.time()
while rclpy.ok() and not node.done and time.time() - start < 60:
    rclpy.spin_once(node, timeout_sec=1.0)
if not node.done:
    print('No image received in 60 s')
node.destroy_node()
rclpy.shutdown()
