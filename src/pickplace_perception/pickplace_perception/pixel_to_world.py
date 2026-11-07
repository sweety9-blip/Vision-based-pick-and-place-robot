import math

# Camera pose in the world (from scene.launch.py): straight down, 1.0 m up
CAM_X, CAM_Y, CAM_Z = 0.55, 0.0, 1.0
W, H = 640, 480
HFOV = 1.047  # from overhead_camera.sdf (radians)

# Intrinsics derived from the field of view (square pixels)
FX = (W / 2) / math.tan(HFOV / 2)
FY = FX
CX, CY = W / 2, H / 2


def pixel_to_world(u, v, depth):
    """Pixel + depth (m, distance along camera axis) -> world (x, y, z)."""
    # Point in the camera frame (x right, y down, z forward)
    xc = (u - CX) * depth / FX
    yc = (v - CY) * depth / FY
    # Camera looks straight down: image-right -> world -y, image-down -> world -x
    wx = CAM_X - yc
    wy = CAM_Y - xc
    wz = CAM_Z - depth
    return wx, wy, wz


if __name__ == '__main__':
    import sys
    from color_detector import detect
    import cv2
    img = cv2.imread(sys.argv[1])
    found = detect(img)
    for name, (u, v, area) in found.items():
        # Cube top is at 0.065 m, so its depth from the camera is 1.0 - 0.065
        x, y, z = pixel_to_world(u, v, 0.935)
        print('%s: pixel (%.0f, %.0f) -> world x=%.3f y=%.3f z=%.3f'
              % (name, u, v, x, y, z))
