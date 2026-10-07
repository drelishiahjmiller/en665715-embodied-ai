"""Send the same timed move to the Gazebo robot or the real robot.

A move is: drive at `linear` m/s and turn at `angular` rad/s for `seconds`.
Gazebo moves are timed in simulation time, so a slow simulation still covers
the full distance. Real robot moves are timed with the computer's clock.

Gazebo results use the robot's true position (/sim/ground_truth, from
sim_with_ground_truth.launch.py) and also report what wheel odometry (/odom)
claimed. They differ when the simulated robot is blocked and its wheels slip.

    from robot_motion import RobotMotion
    robots = RobotMotion()
    print(robots.move("sim", linear=0.15, angular=0.0, seconds=3.0).summary())
    print(robots.move("real", linear=0.15, angular=0.0, seconds=3.0).summary())
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, Int32
from tf2_msgs.msg import TFMessage

SIM_MODEL_NAME = "gazebo_car"

# Same limits as the Gazebo robot and the Pico firmware
MAX_LINEAR = 0.3     # m/s
MAX_ANGULAR = 1.5    # rad/s
MAX_SECONDS = 10.0
PUBLISH_HZ = 20      # the Pico stops if commands pause for 0.5 s
# If turning the real robot left by hand makes /pico/imu/heading go down,
# your IMU is mounted upside down: change this to -1.0.
REAL_HEADING_SIGN = 1.0

ROBOTS = ("sim", "real")


def _wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _yaw_degrees(q) -> float:
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


@dataclass
class MoveResult:
    robot: str
    linear: float
    angular: float
    seconds: float
    completed: bool
    message: str
    start_heading_deg: Optional[float] = None
    end_heading_deg: Optional[float] = None
    distance_m: Optional[float] = None       # Gazebo only; measure the real robot yourself
    odom_distance_m: Optional[float] = None  # what Gazebo's wheel odometry claimed
    used_ground_truth: bool = False
    end_x: Optional[float] = None
    end_y: Optional[float] = None

    @property
    def heading_change_deg(self) -> Optional[float]:
        if self.start_heading_deg is None or self.end_heading_deg is None:
            return None
        return _wrap_degrees(self.end_heading_deg - self.start_heading_deg)

    @property
    def expected_distance_m(self) -> float:
        return abs(self.linear) * self.seconds

    @property
    def expected_heading_change_deg(self) -> float:
        return math.degrees(self.angular * self.seconds)

    @property
    def possibly_blocked(self) -> bool:
        """True if the Gazebo robot moved much less than commanded, or its
        wheels kept turning (odometry) while it stayed in place (true position)."""
        if not self.completed or self.distance_m is None or self.expected_distance_m < 0.05:
            return False
        short = self.distance_m < 0.7 * self.expected_distance_m
        slipped = (self.used_ground_truth and self.odom_distance_m is not None
                   and self.odom_distance_m - self.distance_m > 0.1)
        return short or slipped

    def summary(self) -> str:
        name = "Gazebo robot" if self.robot == "sim" else "Real robot"
        command = (f"{self.linear:.2f} m/s, {self.angular:.2f} rad/s "
                   f"for {self.seconds:.1f} s")
        if not self.completed:
            return f"{name}: {command}. {self.message}"
        parts = [f"{name}: {command}."]
        if self.distance_m is not None:
            source = "true position" if self.used_ground_truth else "odometry only"
            odom = (f", wheel odometry said {self.odom_distance_m:.2f} m"
                    if self.used_ground_truth and self.odom_distance_m is not None else "")
            parts.append(f"Moved {self.distance_m:.2f} m "
                         f"(expected {self.expected_distance_m:.2f} m{odom}), "
                         f"ended at ({self.end_x:.2f}, {self.end_y:.2f}) [{source}].")
        else:
            parts.append(f"Expected distance {self.expected_distance_m:.2f} m "
                         "(measure the real distance with a tape measure).")
        if self.heading_change_deg is not None:
            parts.append(f"Turned {self.heading_change_deg:.0f} deg "
                         f"(expected {self.expected_heading_change_deg:.0f} deg).")
        else:
            parts.append("Heading not available.")
        if self.possibly_blocked:
            parts.append("The robot moved much less than commanded: it may have hit something.")
        return " ".join(parts)


class RobotMotion:
    def __init__(self) -> None:
        if not rclpy.ok():
            # Keep Python's Ctrl+C handling so a move can always send its stop
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._node = Node("sim_to_real_motion")
        self._lock = threading.Lock()
        self._move_lock = threading.Lock()
        self._publishers = {
            "sim": self._node.create_publisher(Twist, "/cmd_vel", 10),
            "real": self._node.create_publisher(Twist, "/pico/cmd_vel", 10),
        }
        self._sim_pose = None        # (x, y, heading_deg) from /odom
        self._sim_pose_time = 0.0
        self._sim_truth = None       # (x, y, heading_deg) from Gazebo
        self._sim_truth_time = 0.0
        self._sim_clock = None       # seconds of simulation time
        self._real_heading = None
        self._real_heading_time = 0.0
        self._real_status = None
        self._real_status_time = 0.0

        self._node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._node.create_subscription(Clock, "/clock", self._on_clock, 10)
        self._node.create_subscription(TFMessage, "/sim/ground_truth", self._on_truth, 10)
        self._node.create_subscription(Float32, "/pico/imu/heading", self._on_heading,
                                       qos_profile_sensor_data)
        self._node.create_subscription(Int32, "/pico/drive/status", self._on_status,
                                       qos_profile_sensor_data)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ----- incoming data -----
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        with self._lock:
            self._sim_pose = (p.x, p.y, _yaw_degrees(msg.pose.pose.orientation))
            self._sim_pose_time = time.monotonic()

    def _on_truth(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.child_frame_id == SIM_MODEL_NAME:
                t = transform.transform.translation
                with self._lock:
                    self._sim_truth = (t.x, t.y, _yaw_degrees(transform.transform.rotation))
                    self._sim_truth_time = time.monotonic()
                return

    def _on_clock(self, msg: Clock) -> None:
        with self._lock:
            self._sim_clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_heading(self, msg: Float32) -> None:
        with self._lock:
            self._real_heading = REAL_HEADING_SIGN * msg.data
            self._real_heading_time = time.monotonic()

    def _on_status(self, msg: Int32) -> None:
        with self._lock:
            self._real_status = msg.data
            self._real_status_time = time.monotonic()

    # ----- robot state -----
    def sim_pose(self, max_age: float = 1.0):
        """(x, y, heading_deg) of the Gazebo robot from wheel odometry (/odom), or None."""
        with self._lock:
            fresh = time.monotonic() - self._sim_pose_time < max_age
            return self._sim_pose if fresh else None

    def sim_true_pose(self, max_age: float = 1.0):
        """(x, y, heading_deg) of the Gazebo robot from Gazebo itself, or None."""
        with self._lock:
            fresh = time.monotonic() - self._sim_truth_time < max_age
            return self._sim_truth if fresh else None

    def real_heading(self, max_age: float = 1.0) -> Optional[float]:
        """Real robot heading in degrees (left turn positive), or None."""
        with self._lock:
            fresh = time.monotonic() - self._real_heading_time < max_age
            return self._real_heading if fresh else None

    def real_drive_status(self, max_age: float = 3.0) -> Optional[int]:
        """0 = stopped, 1 = driving, 2 = stopped because commands stopped arriving."""
        with self._lock:
            fresh = time.monotonic() - self._real_status_time < max_age
            return self._real_status if fresh else None

    def sim_ready(self) -> bool:
        return self.sim_pose() is not None and self._sim_clock is not None

    def real_ready(self) -> bool:
        listening = self._node.count_subscribers("/pico/cmd_vel") > 0
        return listening and self.real_drive_status() is not None

    def wait_until_ready(self, robot: str, timeout: float = 5.0) -> bool:
        check = self.sim_ready if robot == "sim" else self.real_ready
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if check():
                return True
            time.sleep(0.1)
        return check()

    def wait_for_ground_truth(self, timeout: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.sim_true_pose() is not None:
                return True
            time.sleep(0.1)
        return self.sim_true_pose() is not None

    # ----- motion -----
    def _publish(self, robot: str, linear: float, angular: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._publishers[robot].publish(cmd)

    def stop(self, robot: Optional[str] = None) -> None:
        """Stop one robot, or both if robot is None."""
        for name in ([robot] if robot else ROBOTS):
            for _ in range(3):
                self._publish(name, 0.0, 0.0)
                time.sleep(0.03)

    def _heading(self, robot: str) -> Optional[float]:
        if robot == "sim":
            pose = self.sim_true_pose() or self.sim_pose()
            return pose[2] if pose else None
        return self.real_heading()

    def move(self, robot: str, linear: float, angular: float, seconds: float) -> MoveResult:
        """Drive one robot at (linear, angular) for `seconds`, then stop."""
        linear, angular, seconds = float(linear), float(angular), float(seconds)
        result = MoveResult(robot, linear, angular, seconds, False, "")
        if robot not in ROBOTS:
            result.message = f"Unknown robot '{robot}'. Use 'sim' or 'real'."
            return result
        if abs(linear) > MAX_LINEAR or abs(angular) > MAX_ANGULAR or not 0 < seconds <= MAX_SECONDS:
            result.message = (f"Refused: limits are |linear| <= {MAX_LINEAR} m/s, "
                              f"|angular| <= {MAX_ANGULAR} rad/s, 0 < seconds <= {MAX_SECONDS}.")
            return result
        if not self.wait_until_ready(robot, timeout=3.0):
            result.message = ("Not connected: start the simulation." if robot == "sim" else
                              "Not connected: check the Pico and the micro-ROS Agent.")
            return result

        if robot == "sim":
            self.wait_for_ground_truth(timeout=2.0)
        with self._move_lock:
            start_odom = self.sim_pose() if robot == "sim" else None
            start_truth = self.sim_true_pose() if robot == "sim" else None
            result.start_heading_deg = self._heading(robot)
            period = 1.0 / PUBLISH_HZ
            wall_deadline = time.monotonic() + seconds * 5 + 5
            try:
                if robot == "sim":
                    with self._lock:
                        sim_start = self._sim_clock
                    while True:
                        with self._lock:
                            elapsed = self._sim_clock - sim_start
                        if elapsed >= seconds:
                            break
                        if time.monotonic() > wall_deadline:
                            result.message = "Stopped: the simulation is not running (paused?)."
                            return result
                        self._publish(robot, linear, angular)
                        time.sleep(period)
                else:
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        self._publish(robot, linear, angular)
                        time.sleep(period)
            finally:
                self.stop(robot)
            time.sleep(0.7)  # let the robot coast to a stop before measuring

        result.end_heading_deg = self._heading(robot)
        if robot == "sim":
            end_odom, end_truth = self.sim_pose(), self.sim_true_pose()
            if start_odom and end_odom:
                result.odom_distance_m = math.hypot(end_odom[0] - start_odom[0],
                                                    end_odom[1] - start_odom[1])
            start, end = ((start_truth, end_truth) if start_truth and end_truth
                          else (start_odom, end_odom))
            if start and end:
                result.used_ground_truth = start is start_truth
                result.distance_m = math.hypot(end[0] - start[0], end[1] - start[1])
                result.end_x, result.end_y = end[0], end[1]
        result.completed = True
        result.message = "Done."
        return result

    def close(self) -> None:
        try:
            self.stop()
        finally:
            self._executor.shutdown()
            self._spin_thread.join(timeout=2.0)
            self._node.destroy_node()
            rclpy.try_shutdown()
