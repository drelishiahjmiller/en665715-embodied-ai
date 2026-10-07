"""Start Gazebo with the obstacle course, spawn the robot car, and bridge topics to ROS 2.

Run inside the container:
    ros2 launch /workspace/launch/sim.launch.py
    ros2 launch /workspace/launch/sim.launch.py gui:=false   # no Gazebo window (faster)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_FILE = os.path.join(PROJECT_DIR, "worlds", "obstacle_course.sdf")
ROBOT_FILE = os.path.join(PROJECT_DIR, "urdf", "gazebo_car.urdf.xacro")
BRIDGE_FILE = os.path.join(PROJECT_DIR, "config", "bridge.yaml")


def generate_launch_description() -> LaunchDescription:
    gui = LaunchConfiguration("gui")
    robot_description = ParameterValue(Command(["xacro ", ROBOT_FILE]), value_type=str)

    # Gazebo simulator (server). -r starts the simulation running.
    gazebo_server = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", "-v", "2", WORLD_FILE],
        output="screen",
    )

    # Gazebo window, shown in the browser desktop. nice gives the server priority.
    gazebo_gui = ExecuteProcess(
        cmd=["nice", "-n", "10", "ign", "gazebo", "-g", "-v", "1"],
        output="screen",
        condition=IfCondition(gui),
    )

    # Publishes the robot model on /robot_description and the fixed sensor frames on /tf
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Adds the robot to the running world at (0, 0), facing +x
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "obstacle_course",
            "-topic", "robot_description",
            "-name", "gazebo_car",
            "-x", "0", "-y", "0", "-z", "0.04",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": BRIDGE_FILE}],
        output="screen",
    )

    # Point the Gazebo window's camera at the arena once the window is open
    camera_request = (
        "pose: {position: {x: 1.5, y: -2.4, z: 1.8}, "
        "orientation: {x: -0.2236, y: 0.2236, z: 0.6708, w: 0.6708}}"
    )
    set_gui_camera = ExecuteProcess(
        cmd=["bash", "-c",
             "for i in $(seq 1 30); do "
             "ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera "
             f"--reptype ignition.msgs.Boolean --timeout 2000 --req '{camera_request}' "
             "> /dev/null 2>&1 && break; sleep 2; done"],
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="Show the Gazebo window in the browser desktop"),
        gazebo_server,
        gazebo_gui,
        robot_state_publisher,
        bridge,
        # Give the server a few seconds to load the world before spawning the robot
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=8.0, actions=[set_gui_camera]),
    ])
