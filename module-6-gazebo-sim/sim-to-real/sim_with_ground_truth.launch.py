"""Start the Module 6 simulation plus a bridge for the robot's true position.

Gazebo knows exactly where the robot is. This adds that "ground truth" to ROS 2
as /sim/ground_truth, so a rehearsal can tell when the simulated robot hit
something even though its wheel odometry (/odom) kept counting.

Run inside the container:
    ros2 launch /workspace/sim-to-real/sim_with_ground_truth.launch.py
    ros2 launch /workspace/sim-to-real/sim_with_ground_truth.launch.py gui:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_LAUNCH = os.path.join(os.path.dirname(HERE), "launch", "sim.launch.py")
WORLD_POSES = "/world/obstacle_course/dynamic_pose/info"


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Show the Gazebo window in the browser desktop"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(SIM_LAUNCH),
            launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ground_truth_bridge",
            arguments=[f"{WORLD_POSES}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
            remappings=[(WORLD_POSES, "/sim/ground_truth")],
            output="screen",
        ),
    ])
