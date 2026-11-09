"""Drive one robot to a named place, and tell what happened.

    from warehouse_nav.navigator import RobotNavigator
    nav = RobotNavigator()                      # the robot under the namespace husky1
    result = nav.go_to('dock')                   # blocks until arrived, failed or timed out
    print(result['success'], result['collisions'], result['duration_s'])

On top of Nav2 (navigate_to_pose) it adds
  * goal checking before moving (known place, room for the robot, reachable on the map),
  * getting out when the robot is pinned against something (Nav2's own recoveries refuse to move then, and the
    planner cannot start from a spot inside its keep-out zone), then trying again,
  * counting real collisions from the Gazebo contact sensors (/husky1/contacts/*/bumper_states),
  * a result dictionary with the numbers a mission planner or a test needs.
"""
import math
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .grid import GridMap, reachable_from
from .places import Places

HALF_WIDTH = 0.345            # m, base_link to the side of the Husky
HALF_LENGTH = 0.5
PASS_CLEARANCE = 0.625        # the planner's keep-out distance (half width + footprint padding)
GOAL_CLEARANCE = 0.75         # room a goal needs all round
ESCAPE_MARGIN = 0.15         # m beyond the planner's keep-out distance that unstick drives to
STALL_SECONDS = 25.0          # no 0.15 m or 0.15 rad of movement for this long means the robot is pinned
CONTACT_WHEELS = ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel')
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED'}


# The map does not change and these take long in Python (about a second, far more when the process is busy),
# so they are computed once.
_map_cache = {}
_map_lock = threading.Lock()


def map_data(msg):
    """The grid and its masks for a map message, computed once."""
    with _map_lock:
        if not _map_cache:
            gm = GridMap.from_msg(msg)
            _map_cache.update(grid=gm, passable=gm.clear_of_obstacles(PASS_CLEARANCE),
                              roomy=gm.clear_of_obstacles(GOAL_CLEARANCE),
                              free=gm.clear_of_obstacles(PASS_CLEARANCE + ESCAPE_MARGIN), reach=None)
        return _map_cache


def connected(m, start):
    """The floor reachable from start. The warehouse is one connected floor, so this is flooded once, and again
    only for a robot standing somewhere that flood did not reach."""
    with _map_lock:
        if m['reach'] is None or (start is not None and not m['reach'][start]):
            if start is None:
                return m['passable']
            m['reach'] = reachable_from(m['passable'], start)
        return m['reach']


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RobotNavigator:
    def __init__(self, places=None, namespace='husky1'):
        if not rclpy.ok():
            rclpy.init()
        self.places = places or Places()
        self.robot = namespace          # also the robot's model name in Gazebo
        # In the robot's namespace: the relative topic and action names below resolve to /husky1/...
        # The TF listener reads /tf and /tf_static, which are remapped to the robot's own tree (/husky1/tf).
        self.node = Node('navigator', namespace=namespace,
                         cli_args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'],
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.log = self.node.get_logger()

        # Every subscription and client is made before the executor starts: adding them to a running
        # executor can crash rclpy.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.nav = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.planner = ActionClient(self.node, ComputePathToPose, 'compute_path_to_pose')
        self.cmd_pub = self.node.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.scan = None
        # The scan with the other robots' outlines added (scripts/other_robots.py): the real lidar beam passes over
        # another robot's body, and the getting-out moves below must not drive into it
        self.node.create_subscription(LaserScan, 'scan_costmap', lambda m: setattr(self, 'scan', m),
                                      qos_profile_sensor_data)   # best effort, like the lidar
        self._truth = None
        self.node.create_subscription(Odometry, 'ground_truth/odom', lambda m: setattr(self, '_truth', m), 5)
        self.map = None
        self.node.create_subscription(
            OccupancyGrid, 'map', self._on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._contact_lock = threading.Lock()
        self._contact_last = -1e9
        self._contact_episodes = 0
        self._contact_with = {}
        for part in ('base',) + CONTACT_WHEELS:
            self.node.create_subscription(ContactsState, f'contacts/{part}/bumper_states',
                                          self._on_contacts, 20)
        self._cancel = threading.Event()
        self._active_goal = None

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

    # ---------------------------------------------------------------- inputs
    def _on_map(self, msg):
        self.map = msg

    def _on_contacts(self, msg):
        for s in msg.states:
            # The floor does not count (the ground plane, and the 'floor' of each workcell model), and neither does
            # one part of this robot touching another. Gazebo may name this robot first or second.
            # Another robot does count.
            me = f'{self.robot}::'
            first, second = s.collision1_name, s.collision2_name
            if first.startswith(me) and second.startswith(me):
                continue
            other = second if first.startswith(me) else first
            if other.startswith('ground_plane') or other.endswith('::floor'):
                continue
            now = time.monotonic()
            with self._contact_lock:
                if now - self._contact_last > 1.0:
                    self._contact_episodes += 1
                self._contact_last = now
                model = '::'.join(other.split('::')[::2])    # model::collision, e.g. workcell_SW::empty_pallet1
                self._contact_with[model] = self._contact_with.get(model, 0) + 1

    def collisions(self):
        """(number of separate collisions so far, {object touched: contact messages})."""
        with self._contact_lock:
            return self._contact_episodes, dict(self._contact_with)

    def pose(self, timeout=2.0):
        """(x, y, yaw) of the robot on the map, from AMCL, or None."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                t = self.tf_buffer.lookup_transform('map',
                                                    'base_link', Time()).transform
                return t.translation.x, t.translation.y, yaw_of(t.rotation)
            except Exception:
                time.sleep(0.1)
        return None

    def truth(self):
        """(x, y) of the robot according to the simulator, for tests only, or None."""
        m = self._truth
        return None if m is None else (m.pose.pose.position.x, m.pose.pose.position.y)

    def wait_until_ready(self, timeout=120.0):
        """Waits for Nav2, the map and the robot's position. Returns True when all are there."""
        t0 = time.monotonic()
        ok = self.nav.wait_for_server(timeout_sec=timeout)
        while ok and (self.map is None or self.pose(0.5) is None) and time.monotonic() - t0 < timeout:
            time.sleep(0.2)
        ready = ok and self.map is not None and self.pose(0.5) is not None
        if ready:
            self.check_goal(self.pose(0.5)[0], self.pose(0.5)[1])   # does the slow map work now, not during the first trip
        return ready

    # ---------------------------------------------------------------- goal checking
    def check_goal(self, x, y):
        """None if a goal at (x, y) is fine, else the reason it is not."""
        if self.map is None:
            return None
        m = map_data(self.map)
        gm, roomy = m['grid'], m['roomy']
        r, c = gm.cell(x, y)
        if not (0 <= r < roomy.shape[0] and 0 <= c < roomy.shape[1]):
            return 'outside the map'
        if not roomy[r, c]:
            return f'less than {GOAL_CLEARANCE} m of room around it (the robot needs {GOAL_CLEARANCE} m all round)'
        here = self.pose(0.5)
        start = None
        if here is not None:
            hr, hc = gm.cell(here[0], here[1])
            start = (hr, hc)
            if not m['passable'][hr, hc]:
                # the robot may stand closer to something than the planner allows: start from the nearest cell that is fine
                rows, cols = np.nonzero(m['passable'])
                k = np.argmin((rows - hr) ** 2 + (cols - hc) ** 2)
                start = (int(rows[k]), int(cols[k]))
        if not connected(m, start)[r, c]:
            return 'no route to it on the map (blocked off by racks or walls)'
        return None

    # ---------------------------------------------------------------- driving
    def _wait(self, future, timeout):
        t0 = time.monotonic()
        while not future.done():
            if time.monotonic() - t0 > timeout or self._cancel.is_set():
                return False
            time.sleep(0.05)
        return True

    def _goal_msg(self, x, y, yaw):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.node.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return g

    def can_plan(self, x, y, yaw):
        """True if the planner finds a route from where the robot is to (x, y)."""
        if not self.planner.wait_for_server(timeout_sec=2.0):
            return False
        g = ComputePathToPose.Goal()
        g.goal = self._goal_msg(x, y, yaw).pose
        g.use_start = False
        send = self.planner.send_goal_async(g)
        if not self._wait(send, 5.0) or not send.result().accepted:
            return False
        res = send.result().get_result_async()
        if not self._wait(res, 10.0):
            return False
        return res.result().status == GoalStatus.STATUS_SUCCEEDED and len(res.result().result.path.poses) > 1

    def _navigate_once(self, x, y, yaw, timeout, stats):
        """One navigate_to_pose goal. Returns 'SUCCEEDED', 'ABORTED', 'TIMEOUT' or 'CANCELED'."""
        def feedback(msg):
            stats['recoveries'] = max(stats['recoveries'], msg.feedback.number_of_recoveries)
            stats['remaining'] = msg.feedback.distance_remaining

        send = self.nav.send_goal_async(self._goal_msg(x, y, yaw), feedback_callback=feedback)
        if not self._wait(send, 10.0):
            return 'CANCELED' if self._cancel.is_set() else 'ABORTED'
        handle = send.result()
        if not handle.accepted:
            return 'ABORTED'
        self._active_goal = handle
        result = handle.get_result_async()
        last = self.pose(0.2)
        anchor, anchor_t = last, time.monotonic()
        while not result.done():
            if self._cancel.is_set():
                handle.cancel_goal_async()
                self._wait(result, 5.0)
                return 'CANCELED'
            if time.monotonic() - stats['t_call'] > timeout:
                handle.cancel_goal_async()
                self._wait(result, 5.0)
                return 'TIMEOUT'
            now = self.pose(0.2)
            if now is not None and last is not None:
                stats['distance'] += math.dist(now[:2], last[:2])
            last = now
            # Pinned against something: Nav2's recoveries refuse to move, and would keep trying for a minute
            if now is not None:
                if anchor is None or math.dist(now[:2], anchor[:2]) > 0.15 or abs(
                        math.atan2(math.sin(now[2] - anchor[2]), math.cos(now[2] - anchor[2]))) > 0.15:
                    anchor, anchor_t = now, time.monotonic()
                elif time.monotonic() - anchor_t > STALL_SECONDS:
                    handle.cancel_goal_async()
                    self._wait(result, 5.0)
                    return 'STALLED'
            time.sleep(0.5)
        self._active_goal = None
        return STATUS_NAMES.get(result.result().status, 'ABORTED')

    # ---------------------------------------------------------------- getting out of a tight spot
    def _clear_behind(self):
        return self._clear_along(-1.0)

    def _clear_ahead(self):
        return self._clear_along(1.0)

    def _clear_along(self, sign):
        """Free distance (m) beyond the robot's front (sign 1) or rear (-1) edge, from the lidar."""
        s = self.scan
        if s is None:
            return 1.0
        ang = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
        r = np.array(s.ranges)
        good = np.isfinite(r) & (r > s.range_min)
        px, py = 0.32 + r * np.cos(ang), r * np.sin(ang)     # in base_link; the lidar sits 0.32 m ahead of its centre
        inside = good & (np.abs(py) < HALF_WIDTH + 0.08) & (sign * px > HALF_LENGTH)
        return float(np.min(sign * px[inside]) - HALF_LENGTH) if inside.any() else 3.0

    def _drive(self, speed, distance, give_up=8.0):
        """Straight move at `speed` (m/s, negative = backwards) for up to `distance`, stopping early when blocked."""
        start = self.pose(0.5)
        t0 = time.monotonic()
        twist = Twist()
        twist.linear.x = speed
        last_move, last_pos = t0, start
        moved = 0.0
        while time.monotonic() - t0 < give_up and not self._cancel.is_set():
            clear = self._clear_ahead() if speed > 0 else self._clear_behind()
            if clear < 0.10:
                break
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
            now = self.pose(0.1)
            if now is not None and start is not None:
                moved = math.dist(now[:2], start[:2])
                if last_pos is None or math.dist(now[:2], last_pos[:2]) > 0.03:
                    last_move, last_pos = time.monotonic(), now
            if moved >= distance or time.monotonic() - last_move > 2.0:
                break
        self.cmd_pub.publish(Twist())
        time.sleep(0.3)
        return moved

    def _rotate(self, angle):
        """Turns on the spot by about `angle` rad (open loop, 0.4 rad/s)."""
        twist = Twist()
        twist.angular.z = math.copysign(0.4, angle)
        t0 = time.monotonic()
        while time.monotonic() - t0 < abs(angle) / 0.4 and not self._cancel.is_set():
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())
        time.sleep(0.3)

    def escape_to_free_space(self, max_seconds=40.0):
        """Drives the robot (slowly, by the map and AMCL) to the nearest spot that is clear of walls and racks by more
        than the planner's keep-out distance. Nav2 cannot plan or control from inside that zone, and its own recovery
        moves refuse to start when the robot already touches something. Returns True when the robot is out."""
        if self.map is None:
            return False
        m = map_data(self.map)
        gm, free = m['grid'], m['free']
        t0 = time.monotonic()
        blocked = {'fwd': 0.0, 'back': 0.0}
        ref, ref_t = None, t0
        while time.monotonic() - t0 < max_seconds and not self._cancel.is_set():
            pose = self.pose(0.3)
            if pose is None:
                time.sleep(0.1)
                continue
            x, y, yaw = pose
            r, c = gm.cell(x, y)
            if 0 <= r < free.shape[0] and 0 <= c < free.shape[1] and free[r, c]:
                self.cmd_pub.publish(Twist())
                return True
            r0, c0 = max(0, r - 60), max(0, c - 60)
            rows, cols = np.nonzero(free[r0:r + 61, c0:c + 61])
            if len(rows) == 0:
                break
            k = int(np.argmin((rows + r0 - r) ** 2 + (cols + c0 - c) ** 2))
            tx, ty = gm.xy(rows[k] + r0, cols[k] + c0)
            bearing = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw), math.cos(math.atan2(ty - y, tx - x) - yaw))
            behind = math.atan2(math.sin(bearing - math.pi), math.cos(bearing - math.pi))   # error of the rear, when reversing
            now = time.monotonic()
            twist = Twist()
            direction = None
            # Only towards free space in the scan too: the map does not know where the other robots are
            if abs(bearing) < 1.0 and now > blocked['fwd'] and self._clear_ahead() > 0.15:
                direction, twist.linear.x, twist.angular.z = 'fwd', 0.15, max(-0.3, min(0.3, bearing))
            elif abs(bearing) > 2.14 and now > blocked['back'] and self._clear_behind() > 0.15:
                direction, twist.linear.x, twist.angular.z = 'back', -0.15, max(-0.3, min(0.3, behind))
            else:     # turn on the spot until the front or the rear points at it
                twist.angular.z = math.copysign(0.35, bearing if abs(bearing) < math.pi / 2 else behind)
            # blocked: it was told to move but did not
            if direction:
                if ref is None or direction != ref[0] or math.dist((x, y), ref[1:]) > 0.04:
                    ref, ref_t = (direction, x, y), now
                elif now - ref_t > 2.0:
                    blocked[direction] = now + 6.0
                    ref = None
            else:
                ref = None
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())
        return False

    def unstick(self, goal):
        """Moves the robot out of whatever it is pinned against until the planner can plan again.
        Tries short straight moves back and forth first, then turns a little and tries again.
        Returns True if a route exists afterwards."""
        self.log.warning('robot is pinned or too close to a wall: driving to free space')
        if self.escape_to_free_space() and self.can_plan(*goal):
            return True
        if self.can_plan(*goal):
            return True
        for turn in (0.0, 0.6, -1.2, 1.8):
            if turn:
                self._rotate(turn)
                if self.can_plan(*goal):
                    return True
            for speed, distance in ((-0.15, 0.6), (0.15, 0.6), (-0.15, 1.2), (0.15, 1.2)):
                self._drive(speed, distance)
                if self.can_plan(*goal):
                    return True
        return False

    # ---------------------------------------------------------------- the public call
    def go_to(self, place, timeout=600.0, max_attempts=3):
        """Drive to a named place (see Places). Returns the result dictionary of go_to_pose."""
        found = self.places.find(place)
        if found is None:
            return self._result(False, place, None, 'UNKNOWN_PLACE', f'no place called {place!r}', {})
        name, p = found
        return self.go_to_pose(p['x'], p['y'], p['yaw'], timeout=timeout, max_attempts=max_attempts, place=name)

    def go_to_pose(self, x, y, yaw=0.0, timeout=600.0, max_attempts=3, place=None, check=True):
        self._cancel.clear()
        stats = {'t_call': time.monotonic(), 'recoveries': 0, 'distance': 0.0, 'remaining': None,
                 'attempts': 0, 'unsticks': 0, 'sim_t0': self.node.get_clock().now(),
                 'collisions0': self.collisions()[0], 'touched0': self.collisions()[1]}
        goal = (x, y, yaw)
        if check:
            reason = self.check_goal(x, y)
            if reason:
                return self._result(False, place, goal, 'INVALID_GOAL', f'goal ({x:.2f}, {y:.2f}) rejected: {reason}', stats)
        if not self.nav.wait_for_server(timeout_sec=60.0):     # start_fleet.py may be restarting this robot's Nav2
            return self._result(False, place, goal, 'NAV2_DOWN', 'Nav2 is not available', stats)
        status = 'ABORTED'
        for attempt in range(1, max_attempts + 1):
            stats['attempts'] = attempt
            if attempt > 1:
                if not self.unstick(goal):
                    status = 'STUCK'
                    break
                stats['unsticks'] += 1
            status = self._navigate_once(x, y, yaw, timeout, stats)
            if status in ('SUCCEEDED', 'CANCELED', 'TIMEOUT'):
                break
        here = self.pose(0.5)
        if status == 'SUCCEEDED' and here is not None and math.dist(here[:2], (x, y)) > 0.6:
            status = 'ABORTED'      # reported success but the position estimate says otherwise
        messages = {'SUCCEEDED': 'arrived', 'ABORTED': 'Nav2 gave up after all attempts',
                    'TIMEOUT': 'did not arrive in time', 'CANCELED': 'cancelled',
                    'STUCK': 'robot is stuck and could not back out', 'STALLED': 'robot made no progress'}
        return self._result(status == 'SUCCEEDED', place, goal, status, messages.get(status, status), stats)

    def _result(self, success, place, goal, status, message, stats):
        here = self.pose(0.3)
        episodes, with_ = self.collisions()
        sim_dt = (self.node.get_clock().now() - stats['sim_t0']).nanoseconds / 1e9 if 'sim_t0' in stats else 0.0
        return {
            'success': bool(success), 'status': status, 'message': message,
            'place': place, 'goal': None if goal is None else [round(v, 2) for v in goal],
            'final_error_m': None if (here is None or goal is None) else round(math.dist(here[:2], goal[:2]), 2),
            'true_error_m': None if (self.truth() is None or goal is None) else round(math.dist(self.truth(), goal[:2]), 2),
            'duration_s': round(sim_dt, 1), 'distance_m': round(stats.get('distance', 0.0), 1),
            'attempts': stats.get('attempts', 0), 'recoveries': stats.get('recoveries', 0),
            'unsticks': stats.get('unsticks', 0),
            'collisions': episodes - stats.get('collisions0', episodes), 'touched': sorted(k for k, v in with_.items() if v > stats.get('touched0', with_).get(k, 0)),
        }

    def cancel(self):
        self._cancel.set()

    def stop(self):
        self.cmd_pub.publish(Twist())

    def shutdown(self):
        self.executor.shutdown()
        self.node.destroy_node()
