"""ROSA tools for the PID-controlled robot: drive at a speed and check the tracking.

Each drive command goes to /pico/cmd_vel. The Pico's PID turns it into wheel
speeds, and this tool records /pico/pid/telemetry to report how well the
measured wheel speeds tracked their targets (with a saved plot).
"""

import csv
import math
import os
import threading
import time
import warnings
from datetime import datetime

import matplotlib

warnings.filterwarnings("ignore", message="Unable to import Axes3D")
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from langchain_core.tools import tool  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
MAX_SPEED = 0.25          # m/s
MAX_TURN = 1.5            # rad/s
MAX_SECONDS = 10.0
REQUIRE_APPROVAL = True   # ask the human before every move
WHEEL_SEPARATION = 0.127


class PidRobot(Node):
    def __init__(self):
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        super().__init__("rosa_pid_robot")
        self.lock = threading.Lock()
        self.latest = None
        self.latest_time = 0.0
        self.recording = None
        self.cmd = self.create_publisher(Twist, "/pico/cmd_vel", 10)
        self.create_subscription(Float32MultiArray, "/pico/pid/telemetry", self.on_tel,
                                 qos_profile_sensor_data)
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.thread = threading.Thread(target=self.executor_.spin, daemon=True)
        self.thread.start()

    def on_tel(self, msg):
        if len(msg.data) < 8:
            return
        with self.lock:
            self.latest = list(msg.data[:8])
            self.latest_time = time.time()
            if self.recording is not None:
                self.recording.append([time.time()] + self.latest)

    def connected(self, timeout=3.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.latest and time.time() - self.latest_time < 1.0:
                    return True
            time.sleep(0.05)
        return False

    def send(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.cmd.publish(m)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            time.sleep(0.05)

    def drive(self, v, w, seconds):
        with self.lock:
            self.recording = []
        end = time.time() + seconds
        try:
            while time.time() < end:
                self.send(v, w)
                time.sleep(0.05)
        finally:
            self.stop()
            time.sleep(1.0)
            with self.lock:
                data, self.recording = self.recording, None
        return data

    def close(self):
        self.stop()
        self.executor_.shutdown()
        self.thread.join(timeout=2.0)
        self.destroy_node()
        rclpy.try_shutdown()


_robot = None


def get_robot():
    global _robot
    if _robot is None:
        _robot = PidRobot()
    return _robot


def close_robot():
    global _robot
    if _robot is not None:
        _robot.close()
        _robot = None


def _number(text):
    try:
        value = float(str(text).strip().split()[0])
    except (ValueError, IndexError):
        return None
    return value if math.isfinite(value) else None


def _approved(description):
    if not REQUIRE_APPROVAL:
        return True
    return input(f"\n[APPROVAL NEEDED] {description}. Allow? [y/N] ").strip().lower() in ("y", "yes")


def _save_plot(data, v, w, seconds):
    os.makedirs(RESULTS, exist_ok=True)
    name = datetime.now().strftime("agent_%Y%m%d_%H%M%S")
    t0 = data[0][0]
    t = [r[0] - t0 for r in data]
    with open(os.path.join(RESULTS, name + ".csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["elapsed_s", "pico_time_s", "left_setpoint", "left_measured", "left_effort",
                     "right_setpoint", "right_measured", "right_effort", "status"])
        for ti, r in zip(t, data):
            wr.writerow([round(ti, 3)] + [round(x, 4) for x in r[1:]])
    fig, ax = plt.subplots(figsize=(10, 4.5))
    for i_set, i_meas, color, wheel in ((2, 3, "#1565c0", "left"), (5, 6, "#e65100", "right")):
        ax.plot(t, [r[i_set] for r in data], color=color, ls="--", label=f"{wheel} target")
        ax.plot(t, [r[i_meas] for r in data], color=color, lw=2.2, label=f"{wheel} measured")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("wheel speed (m/s)")
    ax.set_title(f"Agent command: v = {v} m/s, w = {w} rad/s for {seconds} s")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    path = os.path.join(RESULTS, name + ".png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


# ===== ROSA tools =====
# return_direct=True shows each result exactly as written; numbers are strings
# because small models sometimes drop decimals from float arguments.
def _run(v, w, sec, description):
    """Check limits, ask for approval, drive, and report the tracking."""
    if v is None or w is None or sec is None:
        return f"Invalid command: {description}. The robot did not move."
    if abs(v) > MAX_SPEED or abs(w) > MAX_TURN or not 0 < sec <= MAX_SECONDS:
        return (f"Refused: limits are speed <= {MAX_SPEED} m/s, turn rate <= {MAX_TURN} rad/s, "
                f"0 < seconds <= {MAX_SECONDS}. The robot did not move.")
    robot = get_robot()
    if not robot.connected():
        return "No /pico/pid/telemetry. Is the Pico running the PID firmware? The robot did not move."
    vl, vr = v - w * WHEEL_SEPARATION / 2, v + w * WHEEL_SEPARATION / 2
    if not _approved(f"{description}: v = {v} m/s, w = {w} rad/s for {sec} s "
                     f"(wheel targets: left {vl:.3f}, right {vr:.3f} m/s)"):
        return "The human declined. The robot did not move."
    data = robot.drive(v, w, sec)
    if len(data) < 10:
        return "The robot was commanded, but too little telemetry arrived to check tracking."
    t0 = data[0][0]
    steady = [r for r in data if sec / 2 <= r[0] - t0 <= sec]   # second half of the run
    if not steady:
        return "Drove, but no steady-state telemetry was recorded."
    avg = lambda i: sum(r[i] for r in steady) / len(steady)   # noqa: E731
    lines = [f"{description}: v = {v} m/s, w = {w} rad/s for {sec} s. "
             "Steady-state tracking (second half):"]
    for wheel, i_set, i_meas in (("left", 2, 3), ("right", 5, 6)):
        target, measured = avg(i_set), avg(i_meas)
        err = (measured - target) / target * 100 if abs(target) > 1e-3 else 0.0
        lines.append(f"  {wheel}: target {target:.3f} m/s, measured {measured:.3f} m/s ({err:+.1f} %)")
    lines.append(f"Plot saved: {_save_plot(data, v, w, sec)}")
    return "\n".join(lines)


def _word(text, allowed):
    word = str(text).strip().lower()
    return word if word in allowed else None


# Direction comes from a WORD and size from a positive NUMBER: small models often
# get signs wrong, and they copy short argument lists more reliably.
@tool(return_direct=True)
def drive_straight(direction: str, speed_mps: str, seconds: str) -> str:
    """Drive straight. direction: 'forward' or 'backward'. speed_mps: the speed exactly as
    the user wrote it, for example '0.15'. seconds: for example '3'."""
    d, speed, sec = _word(direction, ("forward", "backward")), _number(speed_mps), _number(seconds)
    v = None if d is None or speed is None else (abs(speed) if d == "forward" else -abs(speed))
    return _run(v, 0.0, sec, f"Drive {direction} {speed_mps} m/s for {seconds} s")


@tool(return_direct=True)
def drive_arc(turn: str, speed_mps: str, turn_rate_radps: str, seconds: str) -> str:
    """Drive forward while turning. turn: 'left' or 'right'. speed_mps: for example '0.2'.
    turn_rate_radps: a positive number, for example '0.5'. seconds: for example '4'."""
    t, speed, rate, sec = (_word(turn, ("left", "right")), _number(speed_mps),
                           _number(turn_rate_radps), _number(seconds))
    w = None if t is None or rate is None else (abs(rate) if t == "left" else -abs(rate))
    v = None if speed is None else abs(speed)
    return _run(v, w, sec, f"Arc {turn} at {speed_mps} m/s, {turn_rate_radps} rad/s")


@tool(return_direct=True)
def spin(turn: str, turn_rate_radps: str, seconds: str) -> str:
    """Turn in place without moving forward. turn: 'left' or 'right'.
    turn_rate_radps: a positive number, for example '1'. seconds: for example '2'."""
    t, rate, sec = _word(turn, ("left", "right")), _number(turn_rate_radps), _number(seconds)
    w = None if t is None or rate is None else (abs(rate) if t == "left" else -abs(rate))
    return _run(0.0, w, sec, f"Spin {turn} at {turn_rate_radps} rad/s")


@tool(return_direct=True)
def wheel_speeds() -> str:
    """Report the current target and measured speed of each wheel."""
    robot = get_robot()
    if not robot.connected():
        return "No /pico/pid/telemetry. Is the Pico running the PID firmware?"
    with robot.lock:
        d = robot.latest
    return (f"Left: target {d[1]:.3f}, measured {d[2]:.3f} m/s. "
            f"Right: target {d[4]:.3f}, measured {d[5]:.3f} m/s.")


@tool(return_direct=True)
def stop_robot() -> str:
    """Stop the robot immediately."""
    get_robot().stop()
    return "Stop command sent. The robot is stopped."
