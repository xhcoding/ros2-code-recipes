# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FileContent, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    share_dir = FindPackageShare("robot_describes")
    default_model_path = PathJoinSubstitution([share_dir, "urdf", "first_robot.urdf"])
    default_config_path =PathJoinSubstitution([share_dir, "config", "display_robot.rviz"])
    
    model_path = LaunchConfiguration("model", default=default_model_path)
    config_path = LaunchConfiguration("config", default=default_config_path)
    
    urdf = FileContent(model_path)
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(urdf, value_type=str)}],
        output="screen"
    )
    
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", config_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model_path, description="模型路径"),
        DeclareLaunchArgument("config", default_value=default_config_path, description="配置路径"),
        robot_state_publisher,
        joint_state_publisher,
        rviz2])


