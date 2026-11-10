import glob
import os
import time

import cv2
import numpy as np

from pickplace_perception.color_detector import detect
from pickplace_perception.yolo_detector import NAMES, yolo_detect, load_net


def main():
    net = load_net()
    files = sorted(glob.glob(
        os.path.expanduser('~/pickplace_ws/dataset/images/val/*.png')))
    imgs = [cv2.imread(f) for f in files]
    yolo_detect(net, imgs[0])        # warm-up
    t_c = t_y = 0.0
    both_c = both_y = 0
    diffs = []
    for img in imgs:
        t = time.time(); c = detect(img); t_c += time.time() - t
        t = time.time(); y = yolo_detect(net, img); t_y += time.time() - t
        both_c += all(n in c for n in NAMES)
        both_y += all(n in y for n in NAMES)
        for n in NAMES:
            if n in c and n in y:
                diffs.append(np.hypot(c[n][0] - y[n][0], c[n][1] - y[n][1]))
    n = len(imgs)
    print('val images: %d' % n)
    print('both cubes found: colour %d/%d, YOLO %d/%d'
          % (both_c, n, both_y, n))
    if diffs:
        print('centre difference: mean %.2f px, max %.2f px '
              '(about 1.7 mm per pixel at this camera height)'
              % (np.mean(diffs), np.max(diffs)))
    print('mean time per image: colour %.1f ms, YOLO %.1f ms'
          % (t_c / n * 1000, t_y / n * 1000))


if __name__ == '__main__':
    main()
    