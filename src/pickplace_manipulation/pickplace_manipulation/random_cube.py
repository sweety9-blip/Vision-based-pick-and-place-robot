import math
import random
import re
import subprocess
import sys

X_RANGE = (0.38, 0.70)
Y_RANGE = (0.00, 0.30)
GREEN = (0.55, -0.10)
MIN_GAP = 0.12      # keep this far from the green cube (m)
CUBE_Z = 0.035      # cube centre height when resting on the table


def sh(cmd, timeout=20):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.stdout + r.stderr


def pick_spot(rng):
    while True:
        x = rng.uniform(*X_RANGE)
        y = rng.uniform(*Y_RANGE)
        if math.hypot(x - GREEN[0], y - GREEN[1]) >= MIN_GAP:
            return round(x, 3), round(y, 3)


def set_cube(model, x, y):
    req = ('name: "%s" position {x: %.4f y: %.4f z: %.4f} '
           'orientation {w: 1}' % (model, x, y, CUBE_Z))
    return sh(['gz', 'service', '-s', '/world/empty/set_pose',
               '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '5000', '--req', req])


def read_cube(model):
    out = sh(['gz', 'topic', '-e', '-n', '1', '-t',
              '/world/empty/dynamic_pose/info'])
    i = out.find('name: "%s"' % model)
    if i < 0:
        return None
    j = out.find('position {', i)
    block = out[j:out.find('}', j)]
    vals = [float(re.search(r'\b%s: ([-0-9.eE+]+)' % a, block).group(1))
            if re.search(r'\b%s: ' % a, block) else 0.0 for a in 'xyz']
    return tuple(vals)


def main():
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else None
    rng = random.Random(seed)
    x, y = pick_spot(rng)
    print('moving red_cube to x=%.3f y=%.3f (seed %s)' % (x, y, seed))
    print('reply:', set_cube('red_cube', x, y).strip())
    c = read_cube('red_cube')
    if c is None:
        print('could not read the cube position back')
    else:
        print('cube now at (%.3f, %.3f, %.3f), error %.1f mm'
              % (c[0], c[1], c[2], math.hypot(c[0] - x, c[1] - y) * 1000))


if __name__ == '__main__':
    main()
    