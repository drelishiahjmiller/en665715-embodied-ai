"""Use coordinate transformations with the robot's live pose from /odom.

Run inside the container while the odometry node is running:
    python3 /workspace/odometry/goal_bearing.py 1.0 0.5     # where is world point (1.0, 0.5) from the robot?
    python3 /workspace/odometry/goal_bearing.py --ahead 0.4  # world position of a point 0.4 m ahead
"""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry

from transforms import distance_and_bearing, robot_to_world, world_to_robot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float, nargs="?", help="world x of a goal (m)")
    parser.add_argument("y", type=float, nargs="?", help="world y of a goal (m)")
    parser.add_argument("--ahead", type=float, help="distance of a point straight ahead (m)")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("goal_bearing")
    pose = {}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        pose["p"] = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    end = time.time() + 5
    while "p" not in pose and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    if "p" not in pose:
        print("No /odom received. Is the odometry node running?")
        return

    p = pose["p"]
    print(f"Robot pose (world):  x = {p[0]:.3f} m,  y = {p[1]:.3f} m,  theta = {math.degrees(p[2]):.1f} deg")
    if args.x is not None and args.y is not None:
        xr, yr = world_to_robot(p, args.x, args.y)
        dist, bearing = distance_and_bearing(p, args.x, args.y)
        print(f"World point ({args.x}, {args.y}) in the ROBOT frame:  "
              f"{xr:.3f} m ahead, {yr:.3f} m to the left")
        print(f"  distance {dist:.3f} m,  bearing {math.degrees(bearing):+.1f} deg "
              f"({'turn left' if bearing > 0 else 'turn right'})")
    if args.ahead is not None:
        wx, wy = robot_to_world(p, args.ahead, 0.0)
        print(f"Point {args.ahead} m straight ahead, in the WORLD frame: ({wx:.3f}, {wy:.3f})")


if __name__ == "__main__":
    main()
