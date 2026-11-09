#!/usr/bin/env python3
"""Send the Husky to named places and print what happened (JSON, one line per place).

Needs the simulation and navigation.launch.py running. Then:
  ros2 run warehouse_navigation go_to.py dock
  ros2 run warehouse_navigation go_to.py rack_a3_0 box_a_010 home
  ros2 run warehouse_navigation go_to.py --list                       (what can be asked for)
  ros2 run warehouse_navigation go_to.py --robot husky2 dock           (another robot; default husky1)
"""
import argparse
import json
import sys

from warehouse_nav.navigator import RobotNavigator
from warehouse_nav.places import Places


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('places', nargs='*')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--timeout', type=float, default=600.0)
    ap.add_argument('--robot', default='husky1', help='namespace of the robot to drive')
    args = ap.parse_args()
    if args.list or not args.places:
        print(Places().summary())
        return 0
    nav = RobotNavigator(namespace=args.robot)
    if not nav.wait_until_ready():
        print('Nav2, the map or the robot position is not available: is navigation.launch.py running?', file=sys.stderr)
        return 2
    code = 0
    try:
        for place in args.places:
            result = nav.go_to(place, timeout=args.timeout)
            print(json.dumps(result), flush=True)
            if not result['success']:
                code = 1
                break
    except KeyboardInterrupt:
        nav.cancel()
        nav.stop()
        code = 130
    nav.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
