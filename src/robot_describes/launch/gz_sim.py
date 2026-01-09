# -*- coding: utf-8 -*-
import os
import subprocess
import shlex
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge

def is_in_wsl():
    return "WSLENV" in os.environ

def generate_launch_description():
    share_dir = FindPackageShare("robot_describes")
    default_model_path = PathJoinSubstitution([share_dir, "urdf", "xbot", "xbot.urdf.xacro"])
    bridge_config_path = PathJoinSubstitution([share_dir, "config", "bridge.yaml"])
    default_world_path = PathJoinSubstitution([share_dir, "world", "xbot.sdf"])
    rviz_config_path =PathJoinSubstitution([share_dir, "config", "display_robot.rviz"])

    model_path = LaunchConfiguration("model", default=default_model_path)
    
    urdf = Command(["xacro ", model_path])
    
    gz_partition = SetEnvironmentVariable("GZ_PARTITION", "xbot")
    os.putenv("GZ_PARTITION", "xbot")

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "xbot",
            "-topic", "robot_description"
        ]
    )

    if is_in_wsl():
        # wsl 环境下运行 gazebo
        _ = subprocess.Popen(["powershell.exe", "run_gazebo.ps1", "d:/Applications/Manual/gazebo/worlds/xbot.sdf"])
        gz_launch = LogInfo(msg="wsl")
    else:
        gz_launch_path = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), 'launch', 'gz_sim.launch.py'])
        gz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [default_world_path],
                'on_exit_shutdown': 'True'
            }.items()
        )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(urdf, value_type=str)}],
        output="screen"
    )

    
    bridge = RosGzBridge(bridge_name="xbot_bridge", config_file=bridge_config_path)
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model_path, description="模型路径"),
        gz_partition,
        robot_state_publisher,
        gz_launch,
        spawn_entity,
        bridge,
        rviz2,
    ])


