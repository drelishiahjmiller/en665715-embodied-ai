"""Simulation stand-in for the Pico: makes Gazebo look like the real robot.

Real robot (Pico firmware)              Simulation (this node)
--------------------------              ----------------------------------------
publishes /pico/wheel_ticks       <-    wheel joint angles from /joint_states,
  [left, right] encoder counts          converted to counts: angle / 2pi * CPR
subscribes /pico/cmd_vel          ->    forwarded to /cmd_vel (diff_drive_controller)

So your odometry node and test tools work unchanged in Gazebo and on the robot.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

COUNTS_PER_REV = 4320   # x4 quadrature counts per wheel turn (12 PPR x 90 x 4)


class SimPicoBridge(Node):
    def __init__(self):
        super().__init__("sim_pico_bridge")
        self.ticks_pub = self.create_publisher(Int32MultiArray, "/pico/wheel_ticks",
                                               qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Twist, "/pico/cmd_vel", self.cmd_pub.publish,
                                 qos_profile_sensor_data)
        self.get_logger().info("Simulated Pico ready: /joint_states -> /pico/wheel_ticks, "
                               "/pico/cmd_vel -> /cmd_vel")

    def on_joints(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if "left_wheel_joint" not in pos or "right_wheel_joint" not in pos:
            return
        ticks = Int32MultiArray()
        ticks.data = [round(pos["left_wheel_joint"] / (2 * math.pi) * COUNTS_PER_REV),
                      round(pos["right_wheel_joint"] / (2 * math.pi) * COUNTS_PER_REV)]
        self.ticks_pub.publish(ticks)


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
