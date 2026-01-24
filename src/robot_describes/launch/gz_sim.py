# -*- coding: utf-8 -*-
from launch.substitutions.text_substitution import TextSubstitution
import os
import subprocess
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_service import OnShutdown
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge

def is_in_wsl():
    # Looking forward to using ros2_control on Windows.
    return False
    # return "WSLENV" in os.environ

gazebo_pid = "";

def run_gazebo():
    global gazebo_pid
    result = subprocess.run(["powershell.exe", "-Command", "Start-Process", "-FilePath", "powershell.exe", "-ArgumentList", "run_gazebo.ps1","-PassThru"], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError("start gazebo failed")
    gazebo_pid = result.stdout.splitlines()[3].split()[5].strip()
    print("gazebo pid: " + gazebo_pid)

def stop_gazebo():
    global gazebo_pid
    _ = subprocess.run(["taskkill.exe", "/PID",  gazebo_pid, "/F", "/T"])

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
        run_gazebo()
        gz_launch = LogInfo(msg="wsl")
    else:
        gz_launch_path = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), 'launch', 'gz_sim.launch.py'])
        gz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [TextSubstitution(text='-r '), default_world_path],
                'on_exit_shutdown': 'True'
            }.items()
        )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(urdf, value_type=str)}],
        output="screen"
    )

    robot_controllers_config = PathJoinSubstitution(
        [
            FindPackageShare('robot_describes'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xbot_joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "xbot_diff_drive_controller",
            "--param-file",
            robot_controllers_config
        ]
    )

    bridge = RosGzBridge(bridge_name="xbot_bridge", config_file=bridge_config_path)

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path]
    )

    def on_shutdown_handler(event, context):
        if gazebo_pid:
            stop_gazebo()

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model_path, description="模型路径"),
        gz_partition,
        robot_state_publisher,
        gz_launch,
        spawn_entity,
        bridge,
        rviz2,
        RegisterEventHandler(OnShutdown(on_shutdown=on_shutdown_handler)),
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[diff_drive_controller_spawner])),
    ])
