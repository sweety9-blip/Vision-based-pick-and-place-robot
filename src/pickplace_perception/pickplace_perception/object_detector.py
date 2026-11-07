import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from pickplace_perception.color_detector import detect
from pickplace_perception.pixel_to_world import CAM_X, CAM_Y, CAM_Z


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.depth = None
        self.K = None
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/depth_image', self.depth_cb, 10)
        self.create_subscription(Image, '/camera/image', self.image_cb, 10)
        self.pubs = {
            'red': self.create_publisher(PointStamped, '/detected/red', 10),
            'green': self.create_publisher(PointStamped, '/detected/green', 10),
        }
        self.get_logger().info('object_detector started, waiting for camera data')

    def info_cb(self, msg):
        self.K = msg.k  # [fx 0 cx 0 fy cy 0 0 1]

    def depth_cb(self, msg):
        self.depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)

    def depth_at(self, u, v):
        """Median depth in a 5x5 window around the pixel (ignores bad values)."""
        u, v = int(round(u)), int(round(v))
        win = self.depth[max(v - 2, 0):v + 3, max(u - 2, 0):u + 3]
        win = win[np.isfinite(win)]
        return float(np.median(win)) if win.size else None

    def image_cb(self, msg):
        if self.depth is None or self.K is None:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        fx, cx, fy, cy = self.K[0], self.K[2], self.K[4], self.K[5]
        for name, (u, v, area) in detect(img).items():
            z_cam = self.depth_at(u, v)
            if z_cam is None:
                continue
            xc = (u - cx) * z_cam / fx
            yc = (v - cy) * z_cam / fy
            out = PointStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'world'
            out.point.x = CAM_X - yc
            out.point.y = CAM_Y - xc
            out.point.z = CAM_Z - z_cam
            self.pubs[name].publish(out)


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
