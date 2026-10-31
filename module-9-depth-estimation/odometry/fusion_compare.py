"""Record the robot's path from the wheels alone, the fused EKF, and (in Gazebo) the
true position, then save a CSV and a plot and print how far each estimate ended up
from the truth.

Run inside the container in its own terminal BEFORE a test move, and press
Ctrl+C after the move finishes (or use --seconds):
    python3 /workspace/odometry/fusion_compare.py
    python3 /workspace/odometry/fusion_compare.py --seconds 90 --name square_ccw

Sources: /wheel/odom (wheels only), /odometry/filtered (EKF: wheels + IMU),
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
STYLE = {"wheel": ("wheels only (/wheel/odom)", "#e65100", "--"),
         "fused": ("fused EKF (/odometry/filtered)", "#1565c0", "-"),
         "ground_truth": ("Gazebo true position", "#2e7d32", ":")}


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=0, help="stop after this many seconds")
    parser.add_argument("--name", default=time.strftime("fusion_%Y%m%d_%H%M%S"))
    args = parser.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("fusion_compare")
    latest = {k: None for k in STYLE}
    first = {k: None for k in STYLE}
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

    node.create_subscription(Odometry, "/wheel/odom", setter("wheel"), 10)
    node.create_subscription(Odometry, "/odometry/filtered", setter("fused"), 10)
    node.create_subscription(TFMessage, "/sim/ground_truth", on_truth, 10)

    if args.seconds:
        print(f"Recording for {args.seconds:g} s (stops by itself; Ctrl+C stops early)...")
    else:
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
                        if first[key] is None:
                            first[key] = p
                        rows.append((round(time.time() - start, 2), key, *p))
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

    if not rows:
        print("Nothing recorded. Is the sensor fusion launch running?")
        return

    # Gazebo's world frame and the odom frame start in different places. Express
    # every path relative to its own first sample so all three start at (0, 0, 0).
    def relative(key, p):
        x0, y0, t0 = first[key]
        dx, dy = p[0] - x0, p[1] - y0
        c, s = math.cos(t0), math.sin(t0)
        th = math.atan2(math.sin(p[2] - t0), math.cos(p[2] - t0))
        return dx * c + dy * s, -dx * s + dy * c, th

    os.makedirs(RESULTS, exist_ok=True)
    csv_path = os.path.join(RESULTS, f"{args.name}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "source", "x_m", "y_m", "theta_rad"])
        for r in rows:
            w.writerow([r[0], r[1], *[round(v, 4) for v in relative(r[1], r[2:])]])

    fig, ax = plt.subplots(figsize=(8, 7))
    ends = {}
    print(f"\n{'source':32s} {'end x (m)':>10s} {'end y (m)':>10s} {'end heading (deg)':>18s}")
    for key, (name, color, ls) in STYLE.items():
        pts = [relative(key, r[2:]) for r in rows if r[1] == key]
        if not pts:
            continue
        ends[key] = pts[-1]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, ls=ls, lw=2.5, label=name)
        ax.plot(pts[-1][0], pts[-1][1], "o", color=color)
        print(f"{name:32s} {pts[-1][0]:10.3f} {pts[-1][1]:10.3f} {math.degrees(pts[-1][2]):18.1f}")

    truth = ends.get("ground_truth")
    if truth:
        print("\nEnd error compared with the Gazebo true position:")
        for key in ("wheel", "fused"):
            if key in ends:
                e = ends[key]
                pos = math.hypot(e[0] - truth[0], e[1] - truth[1])
                head = math.degrees(math.atan2(math.sin(e[2] - truth[2]), math.cos(e[2] - truth[2])))
                print(f"  {STYLE[key][0]:32s} position {pos * 100:6.1f} cm   heading {head:+6.1f} deg")
    else:
        print("\n(Real robot: measure the true end position with a tape measure.)")

    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (m), forward from the start")
    ax.set_ylabel("y (m), left of the start")
    ax.set_title(f"Robot path: {args.name}")
    ax.legend()
    png_path = os.path.join(RESULTS, f"{args.name}.png")
    fig.savefig(png_path, dpi=130, bbox_inches="tight")
    print(f"\nSaved {csv_path}\nSaved {png_path}")


if __name__ == "__main__":
    main()
