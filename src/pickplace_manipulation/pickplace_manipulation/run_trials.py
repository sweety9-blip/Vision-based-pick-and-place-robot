import csv
import os
import random
import re
import subprocess
import sys
import time

from pickplace_manipulation.random_cube import pick_spot, set_cube

WS = os.path.expanduser('~/pickplace_ws')
CSV_PATH = os.path.join(WS, 'trials.csv')
LOG_DIR = os.path.join(WS, 'trial_logs')
DETACH = '/model/ur/detachable_joint/detach'
HEADER = ['trial', 'seed', 'cube_x', 'cube_y', 'result', 'place_err_mm',
          'final_z', 'cycle_s', 'move_retries']


def run_trial(i, seed):
    x, y = pick_spot(random.Random(seed))
    subprocess.run(['gz', 'topic', '-t', DETACH, '-m', 'gz.msgs.Empty',
                    '-p', ' '], capture_output=True)
    time.sleep(2)
    set_cube('red_cube', x, y)
    time.sleep(6)
    try:
        r = subprocess.run(
            ['ros2', 'run', 'pickplace_manipulation', 'pick_place', 'red'],
            capture_output=True, text=True, timeout=900)
        out = r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        out = 'TIMEOUT'
    with open(os.path.join(LOG_DIR, 'trial_%d.txt' % i), 'w') as f:
        f.write(out)

    m = re.search(r'final cube \(([-\d.]+), ([-\d.]+), ([-\d.]+)\), '
                  r'place error ([\d.]+) mm, (\w+)', out)
    cyc = re.search(r'cycle time (\d+) s', out)
    if m:
        result, err, fz = m.group(5), m.group(4), m.group(3)
    else:
        err = fz = ''
        if 'TIMEOUT' in out:
            result = 'TIMEOUT'
        elif 'RELEASE FAILED' in out:
            result = 'RELEASE_FAILED'
        elif 'GRASP FAILED' in out:
            result = 'GRASP_FAILED'
        elif 'no detection' in out:
            result = 'NO_DETECTION'
        else:
            result = 'ERROR'
    return [i, seed, x, y, result, err, fz,
            cyc.group(1) if cyc else '', out.count('move failed')]


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    seed0 = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    os.makedirs(LOG_DIR, exist_ok=True)
    new = not os.path.exists(CSV_PATH)
    results = []
    with open(CSV_PATH, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(HEADER)
        for i in range(1, n + 1):
            seed = seed0 + i
            print('--- trial %d of %d (seed %d)' % (i, n, seed), flush=True)
            row = run_trial(i, seed)
            w.writerow(row)
            f.flush()
            results.append(row[4])
            print('    %s  place error %s mm  cycle %s s  retries %s'
                  % (row[4], row[5], row[7], row[8]), flush=True)
            if row[4] == 'RELEASE_FAILED':
                print('stopping: cube is stuck, restart the simulation')
                break
    ok = results.count('SUCCESS')
    print('done: %d of %d trials succeeded' % (ok, len(results)))


if __name__ == '__main__':
    main()
    