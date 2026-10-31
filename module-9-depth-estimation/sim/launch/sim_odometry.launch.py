"""Module 8 simulation: the course robot, with its IMU, in the odometry test arena.

Starts Gazebo, the robot (driven by diff_drive_controller), the bridge for the
clock, true position, and IMU, and the simulated Pico (sim_pico_bridge.py).

Run inside the container:
    ros2 launch /workspace/sim/launch/sim_odometry.launch.py
    ros2 launch /workspace/sim/launch/sim_odometry.launch.py gui:=false
    ros2 launch /workspace/sim/launch/sim_odometry.launch.py left_wheel_error:=0.03
        (the left encoder reads 3% too far, like a slightly larger left tire)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(SIM_DIR, "worlds", "odometry_test.sdf")
ROBOT = os.path.join(SIM_DIR, "urdf", "course_robot.urdf.xacro")
BRIDGE_NODE = os.path.join(SIM_DIR, "sim_pico_bridge.py")
WORLD_POSES = "/world/odometry_test/dynamic_pose/info"


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    left_wheel_error = LaunchConfiguration("left_wheel_error")
    description = ParameterValue(Command(["xacro ", ROBOT]), value_type=str)
    camera = ("pose: {position: {x: 0.5, y: -1.3, z: 1.6}, "
              "orientation: {x: -0.2236, y: 0.2236, z: 0.6708, w: 0.6708}}")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("left_wheel_error", default_value="0.0"),
        ExecuteProcess(cmd=["ign", "gazebo", "-s", "-r", "-v", "2", WORLD], output="screen"),
        ExecuteProcess(cmd=["nice", "-n", "10", "ign", "gazebo", "-g", "-v", "1"],
                       output="screen", condition=IfCondition(gui)),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description, "use_sim_time": True}],
             output="screen"),
        # Simulation clock, the robot's TRUE position (ground truth), and the raw IMU
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="sim_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                        f"{WORLD_POSES}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                        "/sim/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
             remappings=[(WORLD_POSES, "/sim/ground_truth")], output="screen"),
        TimerAction(period=5.0, actions=[
            Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-world", "odometry_test", "-topic", "robot_description",
                            "-name", "course_robot", "-z", "0.04"])]),
        TimerAction(period=10.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster"], output="screen"),
            Node(package="controller_manager", executable="spawner",
                 arguments=["diff_drive_controller"], output="screen"),
            ExecuteProcess(cmd=["python3", BRIDGE_NODE, "--ros-args",
                                "-p", ["left_wheel_error:=", left_wheel_error]],
                           output="screen")]),
        TimerAction(period=12.0, actions=[
            ExecuteProcess(condition=IfCondition(gui), cmd=["bash", "-c",
                "for i in $(seq 1 30); do ign service -s /gui/move_to/pose "
                "--reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean "
                f"--timeout 2000 --req '{camera}' > /dev/null 2>&1 && break; sleep 2; done"])]),
    ])
