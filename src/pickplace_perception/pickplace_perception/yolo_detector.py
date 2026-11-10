import os
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import PointStamped

from pickplace_perception.object_detector import ObjectDetector
from pickplace_perception.pixel_to_world import CAM_X, CAM_Y, CAM_Z

MODEL = os.path.expanduser('~/pickplace_ws/models/best.onnx')
NAMES = ['red', 'green']
CONF = 0.5
SIZE = 640


def letterbox(img):
    h, w = img.shape[:2]
    s = min(SIZE / h, SIZE / w)
    nh, nw = int(round(h * s)), int(round(w * s))
    top, left = (SIZE - nh) // 2, (SIZE - nw) // 2
    out = np.full((SIZE, SIZE, 3), 114, np.uint8)
    out[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
    return out, s, left, top


def load_net():
    return ort.InferenceSession(MODEL, providers=['CPUExecutionProvider'])


def yolo_detect(net, bgr, conf=CONF):
    """Returns {name: (cx, cy, score)} in original image pixels."""
    lb, s, left, top = letterbox(bgr)
    blob = lb[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    out = net.run(None, {net.get_inputs()[0].name: np.ascontiguousarray(blob)})[0]
    out = out.reshape(out.shape[-2], out.shape[-1]).T   # (8400, 4 + classes)
    found = {}
    for c, name in enumerate(NAMES):
        sc = out[:, 4 + c]
        i = int(sc.argmax())
        if sc[i] >= conf:
            found[name] = ((out[i, 0] - left) / s, (out[i, 1] - top) / s,
                           float(sc[i]))
    return found


class YoloDetector(ObjectDetector):
    def __init__(self):
        super().__init__()
        self.net = load_net()
        self.n = 0
        self.ms = 0.0
        self.get_logger().info('YOLO model loaded: ' + MODEL)

    def image_cb(self, msg):
        if self.depth is None or self.K is None:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        t = time.time()
        found = yolo_detect(self.net, img)
        self.ms += (time.time() - t) * 1000
        self.n += 1
        if self.n % 20 == 0:
            self.get_logger().info('mean inference %.1f ms over %d frames'
                                   % (self.ms / self.n, self.n))
        fx, cx, fy, cy = self.K[0], self.K[2], self.K[4], self.K[5]
        for name, (u, v, sc) in found.items():
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


def offline(path):
    net = load_net()
    img = cv2.imread(path)
    yolo_detect(net, img)            # warm-up run
    t = time.time()
    found = yolo_detect(net, img)
    ms = (time.time() - t) * 1000
    for k, (cx, cy, sc) in found.items():
        print('%s centre x=%.1f y=%.1f score=%.2f' % (k, cx, cy, sc))
    if not found:
        print('nothing detected')
    print('inference %.1f ms' % ms)


def main():
    if len(sys.argv) > 1:
        offline(sys.argv[1])
        return
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()