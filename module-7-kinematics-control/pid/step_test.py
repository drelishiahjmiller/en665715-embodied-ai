"""Step-response test for the wheel speed PID: command speed steps, record, plot.

Run inside the container with the PID firmware running (wheels LIFTED first):
    python3 /workspace/pid/step_test.py                                # 0 -> 0.15 -> 0.25 -> 0 m/s
    python3 /workspace/pid/step_test.py --kp 2 --ki 10 --kd 0 --no-ramp --name kp2_ki10
    python3 /workspace/pid/step_test.py --speeds 0.10 0.20 --hold 3

It records /pico/pid/telemetry and saves a CSV and a PNG plot to
/workspace/pid/results/, then prints rise time, overshoot, settling time,
and steady-state error for each wheel and each step.
"""

import argparse
import csv
import os
import threading
import time
import warnings

import matplotlib

warnings.filterwarnings("ignore", message="Unable to import Axes3D")
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402

from set_gains import send_gains  # noqa: E402

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
COLUMNS = ["time_s", "left_setpoint", "left_measured", "left_effort",
           "right_setpoint", "right_measured", "right_effort", "status"]
SETTLE_BAND = 0.05     # +/- 5 % of the step size counts as "settled"


def median5(values):
    """5-sample running median: removes single-sample spikes before computing metrics."""
    out = []
    for i in range(len(values)):
        window = sorted(values[max(0, i - 2):i + 3])
        out.append(window[len(window) // 2])
    return out


def step_metrics(t, measured, start, end, t0, t1):
    """Rise time, overshoot, settling time, steady-state error for one step t0..t1."""
    idx = [i for i, ti in enumerate(t) if t0 <= ti < t1]
    if len(idx) < 10 or abs(end - start) < 1e-6:
        return None
    tt = [t[i] - t0 for i in idx]
    y = median5([measured[i] for i in idx])
    step = end - start
    frac = [(v - start) / step for v in y]            # 0 at the start, 1 at the target
    t10 = next((ti for ti, f in zip(tt, frac) if f >= 0.1), None)
    t90 = next((ti for ti, f in zip(tt, frac) if f >= 0.9), None)
    rise = (t90 - t10) if t10 is not None and t90 is not None else None
    overshoot = max(0.0, (max(frac) - 1.0) * 100.0)
    settle = None
    for k in range(len(frac)):
        if all(abs(f - 1.0) <= SETTLE_BAND for f in frac[k:]):
            settle = tt[k]
            break
    tail = [v for ti, v in zip(tt, y) if ti >= tt[-1] - 1.0]  # last 1 s of the step
    sse = (end - sum(tail) / len(tail)) / abs(end) * 100.0 if end != 0 else None
    return rise, overshoot, settle, sse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.15, 0.25],
                        help="speed steps in m/s (the test ends with a step back to 0)")
    parser.add_argument("--hold", type=float, default=4.0, help="seconds at each speed")
    parser.add_argument("--kp", type=float)
    parser.add_argument("--ki", type=float)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--no-ff", action="store_true")
    parser.add_argument("--no-ramp", action="store_true")
    parser.add_argument("--name", default=time.strftime("step_%Y%m%d_%H%M%S"))
    args = parser.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("step_test")
    rows = []
    lock = threading.Lock()

    def on_telemetry(msg):
        if len(msg.data) >= len(COLUMNS):
            with lock:
                rows.append([time.time()] + list(msg.data[:len(COLUMNS)]))

    node.create_subscription(Float32MultiArray, "/pico/pid/telemetry", on_telemetry,
                             qos_profile_sensor_data)
    cmd_pub = node.create_publisher(Twist, "/pico/cmd_vel", 10)

    if args.kp is not None and args.ki is not None:
        if not send_gains(node, args.kp, args.ki, args.kd, not args.no_ff, not args.no_ramp):
            print("Nobody is listening on /pico/pid/gains. Is the Pico running the PID firmware?")
            return
        print(f"Gains: Kp={args.kp} Ki={args.ki} Kd={args.kd} "
              f"feed-forward={'off' if args.no_ff else 'on'} ramp={'off' if args.no_ramp else 'on'}")
        time.sleep(0.3)

    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()

    profile = [(0.0, 1.0)] + [(v, args.hold) for v in args.speeds] + [(0.0, 2.5)]
    steps = []   # (target_before, target_after, wall_start, wall_end)
    try:
        time.sleep(1.0)
        if not rows:
            print("No /pico/pid/telemetry received. Is the Pico running the PID firmware?")
            return
        print("Running: " + " -> ".join(f"{v:.2f}" for v, _ in profile) + " m/s")
        prev = 0.0
        for speed, hold in profile:
            t_start = time.time()
            steps.append((prev, speed, t_start, t_start + hold))
            msg = Twist()
            msg.linear.x = speed
            while time.time() - t_start < hold:
                cmd_pub.publish(msg)      # keep commanding: the Pico stops after 0.5 s of silence
                time.sleep(0.05)
            prev = speed
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        for _ in range(5):
            cmd_pub.publish(Twist())
            time.sleep(0.05)
        ex.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()

    with lock:
        data = list(rows)
    if len(data) < 20:
        print("Too little telemetry recorded.")
        return
    t0 = data[0][0]
    t = [r[0] - t0 for r in data]
    col = {name: [r[i + 1] for r in data] for i, name in enumerate(COLUMNS)}

    os.makedirs(RESULTS, exist_ok=True)
    csv_path = os.path.join(RESULTS, f"{args.name}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["elapsed_s"] + COLUMNS)
        for ti, r in zip(t, data):
            w.writerow([round(ti, 3)] + [round(v, 4) for v in r[1:]])

    print(f"\n{'step (m/s)':>14s} {'wheel':>6s} {'rise (s)':>9s} {'overshoot':>10s} "
          f"{'settle (s)':>11s} {'steady err':>11s}")
    fmt = lambda v, unit: "-" if v is None else f"{v:.2f}{unit}"   # noqa: E731
    for before, after, ws, we in steps[1:]:
        for wheel in ("left", "right"):
            m = step_metrics(t, col[f"{wheel}_measured"], before, after, ws - t0, we - t0)
            if m:
                print(f"{before:5.2f} -> {after:4.2f} {wheel:>6s} {fmt(m[0], ''):>9s} "
                      f"{fmt(m[1], ' %'):>10s} {fmt(m[2], ''):>11s} {fmt(m[3], ' %'):>11s}")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                   gridspec_kw={"height_ratios": [3, 1.3]})
    ax1.step([s[2] - t0 for s in steps] + [steps[-1][3] - t0],
             [s[1] for s in steps] + [steps[-1][1]], where="post", color="black", ls=":",
             lw=1.5, label="command")
    for wheel, color in (("left", "#1565c0"), ("right", "#e65100")):
        ax1.plot(t, col[f"{wheel}_setpoint"], color=color, ls="--", lw=1.2,
                 label=f"{wheel} setpoint (after ramp)")
        ax1.plot(t, col[f"{wheel}_measured"], color=color, lw=2.2, label=f"{wheel} measured")
        ax2.plot(t, col[f"{wheel}_effort"], color=color, lw=1.5, label=f"{wheel} effort")
    ax1.set_ylabel("wheel speed (m/s)")
    gains = (f"Kp={args.kp} Ki={args.ki} Kd={args.kd}" if args.kp is not None
             else "current gains")
    ax1.set_title(f"Wheel speed step response: {args.name}  ({gains})")
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9, ncol=2)
    ax2.set_ylabel("PID effort (-1..1)")
    ax2.set_xlabel("time (s)")
    ax2.grid(alpha=0.3)
    png_path = os.path.join(RESULTS, f"{args.name}.png")
    fig.savefig(png_path, dpi=130, bbox_inches="tight")
    print(f"\nSaved {csv_path}\nSaved {png_path}")


if __name__ == "__main__":
    main()
