"""Drive known test moves using /odom as feedback, then compare position estimates.

Run inside the container while the odometry node is running:
    python3 /workspace/odometry/move_test.py straight 1.0       # drive 1 m forward
    python3 /workspace/odometry/move_test.py turn 90            # turn 90 deg left (-90 = right)
    python3 /workspace/odometry/move_test.py square 1.0         # 1 m square, counter-clockwise
    python3 /workspace/odometry/move_test.py square 1.0 --cw    # 1 m square, clockwise
    python3 /workspace/odometry/move_test.py goto 1.0 0.5       # drive to world point (1.0, 0.5)

Commands go to /pico/cmd_vel: the real Pico, or sim_pico_bridge.py in Gazebo.
At the end it prints what YOUR odometry measured and, when available, what
diff_drive_controller measured and where Gazebo says the robot really is.
Press Ctrl+C at any time to stop the robot.
"""

import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from tf2_msgs.msg import TFMessage

from transforms import distance_and_bearing, wrap_angle

MAX_SPEED = 0.15          # m/s
MIN_SPEED = 0.05          # m/s, slow enough to stop accurately, fast enough to move
MAX_TURN = 0.6            # rad/s
MIN_TURN = 0.3            # rad/s
DIST_TOL = 0.005          # m
ANGLE_TOL = math.radians(1.0)
SIM_ROBOT_NAME = "course_robot"


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mover(Node):
    def __init__(self, cmd_topic="/pico/cmd_vel", node_name="move_test"):
        super().__init__(node_name)
        self.lock = threading.Lock()
        self.poses = {"odom": None, "diff_drive_controller": None, "ground_truth": None}
        self.cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, "/odom", lambda m: self._set("odom", m.pose.pose), 10)
        self.create_subscription(Odometry, "/diff_drive_controller/odom",
                                 lambda m: self._set("diff_drive_controller", m.pose.pose), 10)
        self.create_subscription(TFMessage, "/sim/ground_truth", self._on_truth, 10)

    def _set(self, key, pose):
        with self.lock:
            self.poses[key] = (pose.position.x, pose.position.y, yaw(pose.orientation))

    def _on_truth(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == SIM_ROBOT_NAME:
                with self.lock:
                    self.poses["ground_truth"] = (t.transform.translation.x,
                                                  t.transform.translation.y,
                                                  yaw(t.transform.rotation))

    def pose(self, key="odom"):
        with self.lock:
            return self.poses[key]

    def send(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.cmd.publish(m)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            time.sleep(0.05)

    def wait_for_odom(self, timeout=10.0):
        end = time.time() + timeout
        while self.pose() is None and time.time() < end:
            time.sleep(0.05)
        if self.pose() is None:
            raise RuntimeError("No /odom received. Is the odometry node running?")
        # In simulation, also wait for the reference topics (they exist only there).
        # Give ROS discovery 2 s before deciding there is no simulation.
        refs = {"diff_drive_controller": "/diff_drive_controller/odom",
                "ground_truth": "/sim/ground_truth"}
        start = time.time()
        while time.time() - start < 6.0:
            waiting = [k for k, topic in refs.items()
                       if self.count_publishers(topic) > 0 and self.pose(k) is None]
            if not waiting and time.time() - start > 2.0:
                break
            time.sleep(0.1)

    # ----- motion primitives (feedback from /odom) -----
    def straight(self, dist):
        x0, y0, th0 = self.pose()
        end = time.time() + abs(dist) / MIN_SPEED * 1.5 + 5
        sign = 1 if dist >= 0 else -1
        while time.time() < end:
            x, y, th = self.pose()
            along = (x - x0) * math.cos(th0) + (y - y0) * math.sin(th0)
            remaining = abs(dist) - sign * along
            if remaining <= DIST_TOL:
                break
            v = sign * min(MAX_SPEED, max(MIN_SPEED, 1.5 * remaining))
            w = max(-0.5, min(0.5, 2.0 * wrap_angle(th0 - th)))   # hold the start heading
            self.send(v, w)
            time.sleep(0.05)
        self.stop()

    def turn(self, angle):
        target = wrap_angle(self.pose()[2] + angle)
        end = time.time() + abs(angle) / MIN_TURN * 1.5 + 5
        while time.time() < end:
            err = wrap_angle(target - self.pose()[2])
            if abs(err) <= ANGLE_TOL:
                break
            self.send(0.0, math.copysign(min(MAX_TURN, max(MIN_TURN, 1.5 * abs(err))), err))
            time.sleep(0.05)
        self.stop()


def local_change(start, end):
    """Motion from start to end, expressed in the start pose's frame: forward, left, turn."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return dx * c + dy * s, -dx * s + dy * c, math.degrees(wrap_angle(end[2] - start[2]))


def main():
    parser = argparse.ArgumentParser(description="Drive known test moves using /odom feedback")
    sub = parser.add_subparsers(dest="move", required=True)
    sub.add_parser("straight").add_argument("meters", type=float)
    sub.add_parser("turn").add_argument("degrees", type=float)
    sq = sub.add_parser("square")
    sq.add_argument("side", type=float)
    sq.add_argument("--cw", action="store_true", help="clockwise (right turns)")
    gt = sub.add_parser("goto")
    gt.add_argument("x", type=float)
    gt.add_argument("y", type=float)
    parser.add_argument("--cmd-topic", default="/pico/cmd_vel")
    args = parser.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Mover(args.cmd_topic)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    try:
        node.wait_for_odom()
        starts = {k: node.pose(k) for k in node.poses}
        if args.move == "straight":
            node.straight(args.meters)
            expected = (args.meters, 0.0, 0.0)
        elif args.move == "turn":
            node.turn(math.radians(max(-180.0, min(180.0, args.degrees))))
            expected = (0.0, 0.0, args.degrees)
        elif args.move == "square":
            turn = -90.0 if args.cw else 90.0
            for _ in range(4):
                node.straight(args.side)
                time.sleep(0.3)
                node.turn(math.radians(turn))
                time.sleep(0.3)
            expected = (0.0, 0.0, 0.0)
        else:
            dist, bearing = distance_and_bearing(node.pose(), args.x, args.y)
            print(f"Goal ({args.x}, {args.y}) is {dist:.3f} m away, bearing "
                  f"{math.degrees(bearing):+.1f} deg (+ = left). Turning, then driving.")
            node.turn(bearing)
            time.sleep(0.3)
            dist, _ = distance_and_bearing(node.pose(), args.x, args.y)
            node.straight(dist)
            expected = None
        time.sleep(1.0)   # let the robot settle

        print(f"\nCommanded move: {args.move}")
        print(f"{'source':24s} {'forward (m)':>12s} {'left (m)':>10s} {'turn (deg)':>11s}"
              f"   end pose (x, y, deg)")
        if expected:
            print(f"{'commanded':24s} {expected[0]:12.3f} {expected[1]:10.3f} {expected[2]:11.1f}")
        for key in ("odom", "diff_drive_controller", "ground_truth"):
            s, e = starts[key], node.pose(key)
            if s is None or e is None:
                continue
            f, l, t = local_change(s, e)
            print(f"{key:24s} {f:12.3f} {l:10.3f} {t:11.1f}   "
                  f"({e[0]:.3f}, {e[1]:.3f}, {math.degrees(e[2]):.1f})")
        if starts["ground_truth"] is None:
            print("(Real robot: measure the true position with a tape measure and compare.)")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.stop()
        print("Robot stopped.")
        ex.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
