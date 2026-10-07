"""ROSA tools for the simulated robot car in Gazebo.

The tools read /odom, /imu, and /camera/image_raw and publish /cmd_vel.
Every move is checked against the obstacles in the world file, limited in
speed, stopped automatically, and approved by a human before it starts.
"""

import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from langchain_core.tools import tool
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, Imu

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_FILE = os.path.join(PROJECT_DIR, "worlds", "obstacle_course.sdf")
SNAPSHOT_DIR = os.path.join(PROJECT_DIR, "snapshots")

# ===== Safety limits =====
LINEAR_SPEED = 0.15        # m/s while driving
ANGULAR_SPEED = 0.6        # rad/s while turning
MAX_MOVE_DISTANCE = 3.5    # m, longest single move
ROBOT_RADIUS = 0.15        # m, circle that contains the whole robot
SAFETY_MARGIN = 0.05       # m, extra space kept around obstacles
POSITION_TOLERANCE = 0.04  # m
HEADING_TOLERANCE = math.radians(2.0)
REQUIRE_APPROVAL = True    # ask the human before every move


# ===== World map (read from the .sdf file) =====
@dataclass
class Box:
    name: str
    x: float
    y: float
    size_x: float
    size_y: float

    def contains(self, px: float, py: float, grow: float = 0.0) -> bool:
        return (abs(px - self.x) <= self.size_x / 2 + grow
                and abs(py - self.y) <= self.size_y / 2 + grow)


def _numbers(text: str) -> list:
    return [float(value) for value in text.split()]


def load_world(path: str = WORLD_FILE):
    """Return (obstacles, places). Obstacles are every box with a collision shape."""
    obstacles, places = [], {"home": (0.0, 0.0)}
    for model in ET.parse(path).getroot().iter("model"):
        name = model.get("name")
        pose = _numbers(model.findtext("pose", "0 0 0 0 0 0"))
        collision_box = model.find("link/collision/geometry/box/size")
        visual_box = model.find("link/visual/geometry/box/size")
        if collision_box is not None:
            size = _numbers(collision_box.text)
            obstacles.append(Box(name, pose[0], pose[1], size[0], size[1]))
        elif visual_box is not None:
            # Flat markers without a collision shape (such as goal_pad) are places
            places[name.replace("_pad", "")] = (pose[0], pose[1])
    return obstacles, places


OBSTACLES, PLACES = load_world()
CLEARANCE = ROBOT_RADIUS + SAFETY_MARGIN


def describe_world() -> str:
    lines = ["Coordinates are in meters. +x is forward from home, +y is to the left."]
    for box in OBSTACLES:
        kind = "wall" if box.name.startswith("wall") else "obstacle"
        lines.append(
            f"- {kind} {box.name}: center ({box.x:.2f}, {box.y:.2f}), "
            f"size {box.size_x:.2f} x {box.size_y:.2f}"
        )
    for name, (x, y) in PLACES.items():
        lines.append(f"- place '{name}': ({x:.2f}, {y:.2f})")
    return "\n".join(lines)


def _first_blocker(x0: float, y0: float, x1: float, y1: float):
    """Return the first obstacle the robot would hit moving from (x0, y0) to (x1, y1)."""
    length = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, int(length / 0.02))
    for i in range(steps + 1):
        px = x0 + (x1 - x0) * i / steps
        py = y0 + (y1 - y0) * i / steps
        for box in OBSTACLES:
            # If the robot already starts close to a box, let it move away from it
            if box.contains(x0, y0, CLEARANCE) and not box.contains(x1, y1, CLEARANCE):
                continue
            if box.contains(px, py, CLEARANCE):
                return box
    return None


def _detour_candidates(x0: float, y0: float, x1: float, y1: float, box: Box):
    """Points beside the blocking box, sorted from shortest to longest total path."""
    length = math.hypot(x1 - x0, y1 - y0) or 1.0
    nx, ny = -(y1 - y0) / length, (x1 - x0) / length
    base = math.hypot(box.size_x, box.size_y) / 2 + CLEARANCE
    options = []
    for side in (1, -1):
        for extra in (0.1, 0.2, 0.3, 0.4, 0.5):
            wx = round(box.x + side * nx * (base + extra), 2)
            wy = round(box.y + side * ny * (base + extra), 2)
            if any(b.contains(wx, wy, CLEARANCE) for b in OBSTACLES):
                continue
            if not _first_blocker(x0, y0, wx, wy):
                total = math.hypot(wx - x0, wy - y0) + math.hypot(x1 - wx, y1 - wy)
                options.append((total, wx, wy))
    return [(wx, wy) for _, wx, wy in sorted(options)]


def plan_route(x0: float, y0: float, x1: float, y1: float, depth: int = 0):
    """Return a list of waypoints ending at (x1, y1) that avoids every obstacle, or None."""
    blocker = _first_blocker(x0, y0, x1, y1)
    if blocker is None:
        return [(x1, y1)]
    if depth >= 3:
        return None
    for wx, wy in _detour_candidates(x0, y0, x1, y1, blocker):
        rest = plan_route(wx, wy, x1, y1, depth + 1)
        if rest:
            return [(wx, wy)] + rest
    return None


# ===== ROS 2 connection =====
def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboCarBridge:
    def __init__(self) -> None:
        if not rclpy.ok():
            # Keep Python's normal Ctrl+C handling. rclpy's default handler shuts ROS
            # down first, which would stop the tool from sending the final stop command.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._node = Node("rosa_gazebo_car")
        self._cmd_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
        self._node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._node.create_subscription(Imu, "/imu", self._on_imu, qos_profile_sensor_data)
        self._node.create_subscription(Image, "/camera/image_raw", self._on_image,
                                       qos_profile_sensor_data)
        self._lock = threading.Lock()
        self._move_lock = threading.Lock()
        self.pose = None          # (x, y, yaw)
        self.pose_time = 0.0
        self.imu = None
        self.imu_time = 0.0
        self.image = None
        self.image_time = 0.0
        self.image_count = 0

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def close(self) -> None:
        self.stop()
        self._executor.shutdown()
        self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        rclpy.try_shutdown()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        with self._lock:
            self.pose = (p.position.x, p.position.y, _yaw_from_quaternion(p.orientation))
            self.pose_time = time.monotonic()

    def _on_imu(self, msg: Imu) -> None:
        with self._lock:
            self.imu = msg
            self.imu_time = time.monotonic()

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            self.image = msg
            self.image_time = time.monotonic()
            self.image_count += 1

    def wait_for_pose(self, timeout: float = 5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self.pose and time.monotonic() - self.pose_time < 1.0:
                    return self.pose
            time.sleep(0.05)
        return None

    def send(self, linear: float, angular: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._cmd_pub.publish(cmd)

    def stop(self) -> None:
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.05)

    def turn_to(self, target_yaw: float) -> str:
        pose = self.wait_for_pose()
        if pose is None:
            return "no odometry"
        timeout = time.monotonic() + abs(_wrap(target_yaw - pose[2])) / ANGULAR_SPEED * 3 + 10
        try:
            while True:
                pose = self.wait_for_pose(1.0)
                if pose is None:
                    return "lost odometry"
                error = _wrap(target_yaw - pose[2])
                if abs(error) < HEADING_TOLERANCE:
                    return "ok"
                if time.monotonic() > timeout:
                    return "turn timed out"
                speed = max(0.15, min(ANGULAR_SPEED, 2.0 * abs(error)))
                self.send(0.0, math.copysign(speed, error))
                time.sleep(0.05)
        finally:
            self.stop()

    def drive_to(self, x: float, y: float) -> str:
        pose = self.wait_for_pose()
        if pose is None:
            return "no odometry"
        timeout = time.monotonic() + math.hypot(x - pose[0], y - pose[1]) / LINEAR_SPEED * 3 + 10
        try:
            while True:
                pose = self.wait_for_pose(1.0)
                if pose is None:
                    return "lost odometry"
                dx, dy = x - pose[0], y - pose[1]
                distance = math.hypot(dx, dy)
                if distance < POSITION_TOLERANCE:
                    return "ok"
                if time.monotonic() > timeout:
                    return "drive timed out"
                heading_error = _wrap(math.atan2(dy, dx) - pose[2])
                if abs(heading_error) > math.radians(60):
                    return "overshot the target"
                speed = max(0.05, min(LINEAR_SPEED, 1.0 * distance))
                turn = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, 2.0 * heading_error))
                self.send(speed, turn)
                time.sleep(0.05)
        finally:
            self.stop()


_bridge = None
_bridge_lock = threading.Lock()


def get_bridge() -> GazeboCarBridge:
    global _bridge
    with _bridge_lock:
        if _bridge is None:
            _bridge = GazeboCarBridge()
        return _bridge


def close_bridge() -> None:
    global _bridge
    with _bridge_lock:
        if _bridge is not None:
            _bridge.close()
            _bridge = None


def _approved(description: str) -> bool:
    if not REQUIRE_APPROVAL:
        return True
    answer = input(f"\n[APPROVAL NEEDED] {description}. Allow? [y/N] ").strip().lower()
    return answer in ("y", "yes")


def _pose_text(pose) -> str:
    return f"({pose[0]:.2f}, {pose[1]:.2f}) heading {math.degrees(pose[2]):.0f} deg"


def _number(text):
    """Convert a tool argument such as '0.75' to a float, or return None."""
    try:
        value = float(str(text).strip().rstrip("m").strip())
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _points_text(points) -> str:
    return " -> ".join(f"({x:.2f}, {y:.2f})" for x, y in points)


def _move_to(bridge: GazeboCarBridge, x: float, y: float, description: str,
             avoid_obstacles: bool = True) -> str:
    pose = bridge.wait_for_pose()
    if pose is None:
        return "No odometry on /odom. Is the simulation running? The robot did not move."
    for box in OBSTACLES:
        if box.contains(x, y, CLEARANCE):
            return (f"Refused: ({x:.2f}, {y:.2f}) is too close to {box.name}. "
                    "The robot did not move.")

    if avoid_obstacles:
        route = plan_route(pose[0], pose[1], x, y)
        if route is None:
            return "No safe route was found to that point. The robot did not move."
    else:
        blocker = _first_blocker(pose[0], pose[1], x, y)
        if blocker:
            return f"Refused: {blocker.name} is in the way. The robot did not move."
        route = [(x, y)]

    length, last = 0.0, (pose[0], pose[1])
    for point in route:
        length += math.hypot(point[0] - last[0], point[1] - last[1])
        last = point
    if length > MAX_MOVE_DISTANCE:
        return (f"Refused: the route is {length:.2f} m; the limit is {MAX_MOVE_DISTANCE} m. "
                "The robot did not move.")

    plan = _points_text([(pose[0], pose[1])] + route)
    if not _approved(f"{description}. Route {plan} ({length:.2f} m)"):
        return "The human declined the move. The robot did not move."

    result = "ok"
    with bridge._move_lock:
        for wx, wy in route:
            now = bridge.wait_for_pose()
            if now is None:
                result = "lost odometry"
                break
            if math.hypot(wx - now[0], wy - now[1]) >= POSITION_TOLERANCE:
                result = bridge.turn_to(math.atan2(wy - now[1], wx - now[0]))
                if result == "ok":
                    result = bridge.drive_to(wx, wy)
            if result != "ok":
                break
    final = bridge.wait_for_pose()
    where = _pose_text(final) if final else "unknown"
    if result != "ok":
        return f"Move stopped early ({result}). The robot is stopped at {where}."
    detours = len(route) - 1
    via = f" via {detours} detour waypoint(s) {_points_text(route[:-1])}" if detours else ""
    return f"Arrived{via}. The robot is stopped at {where}."


# ===== ROSA tools =====
# return_direct=True shows each tool result to the user exactly as written and ends
# ROSA's turn, so a small model cannot misreport where the robot is or what it did.
# Number arguments are strings: small models such as llama3.2 sometimes drop the
# decimals from float arguments (0.75 becomes 0) but copy strings exactly.
@tool(return_direct=True)
def get_robot_status() -> str:
    """Report the robot's position and heading (from /odom), IMU readings, and camera status."""
    bridge = get_bridge()
    pose = bridge.wait_for_pose()
    now = time.monotonic()
    with bridge._lock:
        imu, imu_age = bridge.imu, now - bridge.imu_time
        image, image_age, count = bridge.image, now - bridge.image_time, bridge.image_count
    lines = [f"Position: {_pose_text(pose)}" if pose else "Position: no /odom data"]
    if imu and imu_age < 1.0:
        lines.append(
            f"IMU: turn rate {math.degrees(imu.angular_velocity.z):.1f} deg/s, "
            f"forward acceleration {imu.linear_acceleration.x:.2f} m/s^2, "
            f"gravity {imu.linear_acceleration.z:.2f} m/s^2"
        )
    else:
        lines.append("IMU: no recent /imu data")
    if image and image_age < 2.0:
        lines.append(f"Camera: {image.width}x{image.height} images, {count} received")
    else:
        lines.append("Camera: no recent /camera/image_raw data")
    return "\n".join(lines)


@tool(return_direct=True)
def go_to(x: str, y: str) -> str:
    """Drive to the point (x, y), planning a route around obstacles, then stop.
    x and y are decimal numbers in meters, for example '2.5' and '-0.3'."""
    tx, ty = _number(x), _number(y)
    if tx is None or ty is None:
        return f"Invalid point ({x}, {y}). The robot did not move."
    return _move_to(get_bridge(), tx, ty, f"Drive to ({tx:.2f}, {ty:.2f})")


@tool(return_direct=True)
def go_to_place(place: str) -> str:
    """Drive to a named place on the map: 'goal' (the green pad) or 'home' (where the
    robot began). Plans a route around obstacles."""
    name = place.strip().lower().replace("_pad", "")
    if name not in PLACES:
        return f"Unknown place '{place}'. Known places: {', '.join(PLACES)}."
    x, y = PLACES[name]
    return _move_to(get_bridge(), x, y, f"Drive to the {name} at ({x:.2f}, {y:.2f})")


@tool(return_direct=True)
def drive_forward(meters: str) -> str:
    """Drive forward in a straight line. meters is a decimal number, for example '0.75'."""
    bridge = get_bridge()
    pose = bridge.wait_for_pose()
    if pose is None:
        return "No odometry on /odom. The robot did not move."
    distance = _number(meters)
    if distance is None or distance <= 0:
        return f"Invalid distance '{meters}'. The robot did not move."
    x = pose[0] + distance * math.cos(pose[2])
    y = pose[1] + distance * math.sin(pose[2])
    return _move_to(bridge, x, y, f"Drive forward {distance:.2f} m", avoid_obstacles=False)


def _turn(degrees: float) -> str:
    bridge = get_bridge()
    pose = bridge.wait_for_pose()
    if pose is None:
        return "No odometry on /odom. The robot did not move."
    direction = "left" if degrees > 0 else "right"
    if not _approved(f"Turn {direction} {abs(degrees):.0f} deg in place at {_pose_text(pose)}"):
        return "The human declined the turn. The robot did not move."
    with bridge._move_lock:
        result = bridge.turn_to(pose[2] + math.radians(degrees))
    final = bridge.wait_for_pose()
    where = _pose_text(final) if final else "unknown"
    return f"Turn {'finished' if result == 'ok' else 'stopped: ' + result}. Robot at {where}."


@tool(return_direct=True)
def turn_left(degrees: str) -> str:
    """Turn left (counterclockwise) in place. degrees is a number up to 180, for example '90'."""
    angle = _number(degrees)
    if angle is None:
        return f"Invalid angle '{degrees}'. The robot did not move."
    return _turn(min(abs(angle), 180.0))


@tool(return_direct=True)
def turn_right(degrees: str) -> str:
    """Turn right (clockwise) in place. degrees is a number up to 180, for example '45'."""
    angle = _number(degrees)
    if angle is None:
        return f"Invalid angle '{degrees}'. The robot did not move."
    return _turn(-min(abs(angle), 180.0))


@tool(return_direct=True)
def stop_robot() -> str:
    """Stop the robot immediately."""
    get_bridge().stop()
    return "Stop command sent. The robot is stopped."


@tool(return_direct=True)
def save_camera_image() -> str:
    """Save the latest camera image as a PNG and report which obstacle colors are in view."""
    from PIL import Image as PILImage

    bridge = get_bridge()
    deadline = time.monotonic() + 3.0
    while bridge.image is None and time.monotonic() < deadline:
        time.sleep(0.1)
    with bridge._lock:
        msg = bridge.image
    if msg is None:
        return "No camera image received on /camera/image_raw."
    image = PILImage.frombuffer("RGB", (msg.width, msg.height), bytes(msg.data),
                                "raw", "RGB", msg.step, 1)
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)
    path = os.path.join(SNAPSHOT_DIR, datetime.now().strftime("camera_%Y%m%d_%H%M%S.png"))
    image.save(path)

    # Classify each pixel by hue; gray floor and walls have low saturation and are skipped
    counts = {"red": 0, "orange": 0, "green": 0, "blue": 0}
    pixels = list(image.convert("HSV").getdata())
    for hue, saturation, value in pixels:
        if saturation < 100 or value < 40:
            continue
        degrees = hue * 360 / 255
        if degrees < 15 or degrees >= 340:
            counts["red"] += 1
        elif degrees < 50:
            counts["orange"] += 1
        elif 80 <= degrees < 170:
            counts["green"] += 1
        elif 190 <= degrees < 260:
            counts["blue"] += 1
    seen = [f"{color} {100 * n / len(pixels):.0f}%" for color, n in counts.items()
            if n / len(pixels) > 0.005]
    view = ", ".join(seen) if seen else "no colored obstacles"
    return f"Saved {path}. Colors in view: {view}."
