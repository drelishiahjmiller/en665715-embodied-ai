"""Wheel odometry node: encoder counts in, /odom and TF out.

Subscribes:  /pico/wheel_ticks   std_msgs/Int32MultiArray  [left, right] total counts
                                 (both count UP when the robot drives forward)
             /odom_reset         std_msgs/Empty             set the pose back to (0, 0, 0)
Publishes:   /odom               nav_msgs/Odometry          pose and velocity
             /tf                 odom -> base_link

Run inside the container:
    python3 /workspace/odometry/odometry_node.py
    python3 /workspace/odometry/odometry_node.py --ros-args -p wheel_separation:=0.1279
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty, Int32MultiArray
from tf2_ros import TransformBroadcaster

# A jump this large between two messages (about 0.9 m in one update) means the
# Pico restarted and its counters went back to 0, not that the robot moved.
MAX_COUNTS_PER_UPDATE = 20000


class DiffDriveOdometry:
    """Dead-reckoning pose (x, y, theta) from wheel encoder count changes."""

    def __init__(self, wheel_diameter, wheel_separation, counts_per_rev):
        self.m_per_count = math.pi * wheel_diameter / counts_per_rev
        self.L = wheel_separation
        self.reset()

    def reset(self):
        self.x = self.y = self.theta = 0.0
        self.v = self.w = 0.0

    def update(self, d_left, d_right, dt):
        # Step 1: counts -> meters traveled by each wheel
        s_left = d_left * self.m_per_count
        s_right = d_right * self.m_per_count
        # Step 2: forward kinematics (distances instead of speeds)
        ds = (s_left + s_right) / 2.0
        dtheta = (s_right - s_left) / self.L
        # Step 3: move along the heading HALFWAY through the step (midpoint method)
        theta_mid = self.theta + dtheta / 2.0
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        # Step 4: new heading, wrapped into (-pi, pi]
        self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))
        # Step 5: velocities for /odom
        if dt > 0:
            self.v, self.w = ds / dt, dtheta / dt
        return self.x, self.y, self.theta


class OdometryNode(Node):
    def __init__(self):
        super().__init__("wheel_odometry")
        self.declare_parameter("wheel_diameter", 0.0635)    # m (2.5 in)
        self.declare_parameter("wheel_separation", 0.127)   # m (5 in, center to center)
        self.declare_parameter("counts_per_rev", 4320)      # x4 counts per wheel turn
        p = lambda name: self.get_parameter(name).value     # noqa: E731
        self.odom = DiffDriveOdometry(p("wheel_diameter"), p("wheel_separation"),
                                      p("counts_per_rev"))
        self.last_counts = None
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Int32MultiArray, "/pico/wheel_ticks", self.on_ticks,
                                 qos_profile_sensor_data)
        self.create_subscription(Empty, "/odom_reset", self.on_reset, 10)
        self.get_logger().info(
            f"Wheel odometry ready: D = {p('wheel_diameter')} m, L = {p('wheel_separation')} m, "
            f"CPR = {p('counts_per_rev')} ({self.odom.m_per_count * 1000:.4f} mm per count)")

    def on_reset(self, _msg):
        self.odom.reset()
        self.get_logger().info("Pose reset to (0, 0, 0)")

    def on_ticks(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return
        now = self.get_clock().now()
        left, right = msg.data[0], msg.data[1]
        if self.last_counts is None:
            self.last_counts, self.last_time = (left, right), now
            return
        d_left = left - self.last_counts[0]
        d_right = right - self.last_counts[1]
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_counts, self.last_time = (left, right), now
        if abs(d_left) > MAX_COUNTS_PER_UPDATE or abs(d_right) > MAX_COUNTS_PER_UPDATE:
            self.get_logger().warn("Encoder counts jumped (Pico restarted?); skipping this update")
            return
        x, y, theta = self.odom.update(d_left, d_right, dt)
        self.publish(now, x, y, theta)

    def publish(self, now, x, y, theta):
        qz, qw = math.sin(theta / 2.0), math.cos(theta / 2.0)
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = self.odom.v
        msg.twist.twist.angular.z = self.odom.w
        self.odom_pub.publish(msg)

        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_pub.sendTransform(tf)


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
