import sys
import cv2
import numpy as np


def detect(bgr):
    """Find red and green objects. Returns {name: (cx, cy, area)} in pixels."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    masks = {
        'red': cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
               | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255)),
        'green': cv2.inRange(hsv, (40, 120, 70), (80, 255, 255)),
    }
    found = {}
    for name, mask in masks.items():
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 50:
            continue
        m = cv2.moments(c)
        found[name] = (m['m10'] / m['m00'], m['m01'] / m['m00'], area)
    return found


def draw(bgr, found):
    out = bgr.copy()
    for name, (cx, cy, area) in found.items():
        cv2.circle(out, (int(cx), int(cy)), 6, (255, 255, 255), 2)
        cv2.putText(out, name, (int(cx) + 10, int(cy)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out


def main():
    """ROS node entry point (added in the next step)."""
    print('ROS node not written yet. Use: python3 color_detector.py frame.png')


if __name__ == '__main__':
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        result = detect(img)
        for k, v in result.items():
            print(k, 'centre x=%.1f y=%.1f area=%d' % v)
        if not result:
            print('nothing detected')
        cv2.imwrite('detected.png', draw(img, result))
        print('Saved detected.png')
    else:
        main()
