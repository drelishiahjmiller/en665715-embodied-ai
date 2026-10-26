"""Module 8 sensor fusion: wheel odometry + BNO055 IMU -> robot_localization EKF.

Starts the wheel odometry node (/wheel/odom), the IMU's position on the robot
(real robot only; in Gazebo the robot model provides it), and the EKF, which
publishes /odometry/filtered and the odom -> base_link transform.

Run inside the container:
    Real robot:  ros2 launch /workspace/launch/sensor_fusion.launch.py
    Gazebo:      ros2 launch /workspace/launch/sensor_fusion.launch.py sim:=true
    Calibrated:  ... wheel_diameter:=0.0629 wheel_separation:=0.1291
    Wheel trust: ... wheel_speed_std:=0.03 wheel_turn_rate_std:=0.10
    Upside-down IMU (chip facing the floor): ... imu_roll:=3.14159
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ODOMETRY_NODE = "/workspace/odometry/odometry_node.py"
EKF_CONFIG = "/workspace/config/ekf.yaml"


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false",
                              description="true in Gazebo (uses simulation time)"),
        DeclareLaunchArgument("wheel_diameter", default_value="0.0635"),
        DeclareLaunchArgument("wheel_separation", default_value="0.127"),
        # Uncertainty (1 standard deviation) of the wheel speed and turn rate
        DeclareLaunchArgument("wheel_speed_std", default_value="0.03"),
        DeclareLaunchArgument("wheel_turn_rate_std", default_value="0.10"),
        # Where the BNO055 sits on the robot, measured from base_link (the
        # midpoint between the wheels): meters and radians.
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.04"),
        DeclareLaunchArgument("imu_roll", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),

        # Wheel encoders -> /wheel/odom (no TF: the EKF owns odom -> base_link)
        ExecuteProcess(cmd=[
            "python3", ODOMETRY_NODE, "--ros-args",
            "-p", ["use_sim_time:=", sim],
            "-p", ["wheel_diameter:=", arg("wheel_diameter")],
            "-p", ["wheel_separation:=", arg("wheel_separation")],
            "-p", ["linear_speed_std:=", arg("wheel_speed_std")],
            "-p", ["turn_rate_std:=", arg("wheel_turn_rate_std")],
        ], output="screen"),

        # base_link -> imu_link: tells the EKF how the IMU is mounted.
        # In Gazebo the robot model (URDF) already publishes this transform.
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="imu_mount", condition=UnlessCondition(sim),
             arguments=["--x", arg("imu_x"), "--y", arg("imu_y"), "--z", arg("imu_z"),
                        "--roll", arg("imu_roll"), "--pitch", "0", "--yaw", arg("imu_yaw"),
                        "--frame-id", "base_link", "--child-frame-id", "imu_link"],
             output="screen"),

        # The EKF: /wheel/odom + /pico/imu/data -> /odometry/filtered + TF
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[EKF_CONFIG, {"use_sim_time": sim}], output="screen"),
    ])
