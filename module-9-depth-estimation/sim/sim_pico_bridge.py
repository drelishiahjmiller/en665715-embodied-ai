"""Simulation stand-in for the Pico: makes Gazebo look like the real robot.

Real robot (Pico firmware)              Simulation (this node)
--------------------------              ----------------------------------------
publishes /pico/wheel_ticks       <-    wheel joint angles from /joint_states,
  [left, right] encoder counts          converted to counts: angle / 2pi * CPR
publishes /pico/imu/data          <-    Gazebo's IMU (/sim/imu), turned into what the
  BNO055 in IMU mode                    BNO055 reports: heading from the integrated
                                        gyroscope, gravity removed, same covariance
subscribes /pico/imu/std          ->    same live uncertainty tuning as the firmware
subscribes /pico/cmd_vel          ->    forwarded to /cmd_vel (diff_drive_controller)

So the fusion launch, the EKF, and your test tools work unchanged in Gazebo and on
the robot.

Parameter left_wheel_error (default 0.0): makes the left encoder read that fraction
too far, like a left tire that is slightly larger. 0.03 = 3 %.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Int32MultiArray

COUNTS_PER_REV = 4320   # x4 quadrature counts per wheel turn (12 PPR x 90 x 4)
GRAVITY = 9.80665       # m/s^2

# Same starting uncertainty as the firmware (pico_fusion.c)
DEFAULT_ORIENTATION_STD_DEG = 2.0
DEFAULT_GYRO_STD_RAD_S = 0.02
DEFAULT_ACCEL_STD_M_S2 = 0.30


def diagonal(variance):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class SimPicoBridge(Node):
    def __init__(self):
        super().__init__("sim_pico_bridge")
        self.declare_parameter("left_wheel_error", 0.0)
        self.left_scale = 1.0 + float(self.get_parameter("left_wheel_error").value)
        self.std = [DEFAULT_ORIENTATION_STD_DEG, DEFAULT_GYRO_STD_RAD_S,
                    DEFAULT_ACCEL_STD_M_S2]
        self.yaw = 0.0
        self.last_imu_time = None

        self.ticks_pub = self.create_publisher(Int32MultiArray, "/pico/wheel_ticks",
                                               qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/pico/imu/data", qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Imu, "/sim/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "/pico/imu/std", self.on_std, 10)
        self.create_subscription(Twist, "/pico/cmd_vel", self.cmd_pub.publish,
                                 qos_profile_sensor_data)
        note = (f", left wheel error {self.left_scale - 1.0:+.1%}"
                if self.left_scale != 1.0 else "")
        self.get_logger().info("Simulated Pico ready: /joint_states -> /pico/wheel_ticks, "
                               "/sim/imu -> /pico/imu/data, /pico/cmd_vel -> /cmd_vel" + note)

    def on_joints(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if "left_wheel_joint" not in pos or "right_wheel_joint" not in pos:
            return
        ticks = Int32MultiArray()
        ticks.data = [
            round(pos["left_wheel_joint"] * self.left_scale / (2 * math.pi) * COUNTS_PER_REV),
            round(pos["right_wheel_joint"] / (2 * math.pi) * COUNTS_PER_REV)]
        self.ticks_pub.publish(ticks)

    def on_std(self, msg: Float32MultiArray):
        for i, value in enumerate(msg.data[:3]):
            if value > 0.0:
                self.std[i] = float(value)
        self.get_logger().info(f"IMU uncertainty: orientation {self.std[0]:.2f} deg, "
                               f"gyro {self.std[1]:.4f} rad/s, accel {self.std[2]:.3f} m/s^2")

    def on_imu(self, msg: Imu):
        # The BNO055 in IMU mode builds its heading by adding up gyroscope
        # readings, starting from 0 at power-on. Do the same with Gazebo's gyro,
        # so the noise and bias make the simulated heading drift realistically.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is not None:
            dt = t - self.last_imu_time
            if 0.0 < dt < 0.5:
                self.yaw += msg.angular_velocity.z * dt
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.last_imu_time = t

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "imu_link"
        out.orientation.z = math.sin(self.yaw / 2.0)
        out.orientation.w = math.cos(self.yaw / 2.0)
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration.z -= GRAVITY     # the BNO055 reports gravity-free acceleration
        out.orientation_covariance = diagonal(math.radians(self.std[0]) ** 2)
        out.angular_velocity_covariance = diagonal(self.std[1] ** 2)
        out.linear_acceleration_covariance = diagonal(self.std[2] ** 2)
        self.imu_pub.publish(out)


def main():
    rclpy.init()
    node = SimPicoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
