"""Record the robot's path from every available source, then save a CSV and a plot.

Run inside the container in its own terminal BEFORE a test move, and press
Ctrl+C after the move finishes (or use --seconds):
    python3 /workspace/odometry/odom_compare.py
    python3 /workspace/odometry/odom_compare.py --seconds 60 --name square_ccw

Sources: /odom (your node), /diff_drive_controller/odom (simulation reference),
/sim/ground_truth (simulation only). Files go to /workspace/odometry/results/.
"""

import argparse
import csv
import math
import os
import time

import warnings

import matplotlib

warnings.filterwarnings("ignore", message="Unable to import Axes3D")
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
STYLE = {"odom": ("your /odom", "#e65100", "-"),
         "diff_drive_controller": ("diff_drive_controller", "#1565c0", "--"),
         "ground_truth": ("Gazebo true position", "#2e7d32", ":")}


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=0, help="stop after this many seconds")
    parser.add_argument("--name", default=time.strftime("path_%Y%m%d_%H%M%S"))
    args = parser.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("odom_compare")
    latest = {k: None for k in STYLE}
    rows = []

    def setter(key):
        def cb(msg):
            p = msg.pose.pose
            latest[key] = (p.position.x, p.position.y, yaw(p.orientation))
        return cb

    def on_truth(msg):
        for t in msg.transforms:
            if t.child_frame_id == "course_robot":
                latest["ground_truth"] = (t.transform.translation.x, t.transform.translation.y,
                                          yaw(t.transform.rotation))

    node.create_subscription(Odometry, "/odom", setter("odom"), 10)
    node.create_subscription(Odometry, "/diff_drive_controller/odom",
                             setter("diff_drive_controller"), 10)
    node.create_subscription(TFMessage, "/sim/ground_truth", on_truth, 10)

    print("Recording... press Ctrl+C when the move is finished.")
    start = time.time()
    next_sample = start
    try:
        while not args.seconds or time.time() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.time() >= next_sample:
                next_sample += 0.1
                for key, p in latest.items():
                    if p:
                        rows.append((round(time.time() - start, 2), key, *p))
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

    if not rows:
        print("Nothing recorded. Is the odometry node running?")
        return
    os.makedirs(RESULTS, exist_ok=True)
    csv_path = os.path.join(RESULTS, f"{args.name}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "source", "x_m", "y_m", "theta_rad"])
        w.writerows(rows)

    fig, ax = plt.subplots(figsize=(8, 7))
    print(f"\n{'source':24s} {'end x (m)':>10s} {'end y (m)':>10s} {'end theta (deg)':>16s}")
    for key, (name, color, ls) in STYLE.items():
        pts = [r for r in rows if r[1] == key]
        if not pts:
            continue
        ax.plot([r[2] for r in pts], [r[3] for r in pts], color=color, ls=ls, lw=2.5, label=name)
        ax.plot(pts[-1][2], pts[-1][3], "o", color=color)
        print(f"{name:24s} {pts[-1][2]:10.3f} {pts[-1][3]:10.3f} {math.degrees(pts[-1][4]):16.1f}")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Robot path: {args.name}")
    ax.legend()
    png_path = os.path.join(RESULTS, f"{args.name}.png")
    fig.savefig(png_path, dpi=130, bbox_inches="tight")
    print(f"\nSaved {csv_path}\nSaved {png_path}")


if __name__ == "__main__":
    main()
