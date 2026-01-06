# -*- coding: utf-8 -*-
from launch import LaunchDescription, substitutions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """函数名字不能修改
    """
    
    int_param_arg = DeclareLaunchArgument("int_param", default_value="777777")
    
    int_param_node = Node(
        package="cpp_param_declare_demo",
        executable="cpp_param_declare_demo",
        output="screen",
        # arguments=["--ros-args", "-p", "int_param:=1234578"]
        # ros_arguments=["-p", "int_param:=222333"],
        parameters=[{"int_param": substitutions.LaunchConfiguration("int_param")}]
    )
    
    # 定义一个节点动作，表示启动这个节点
    topic_publish_node = Node(
        package="cpp_topic_publish_demo", # 包名
        executable="cpp_topic_publish_demo", # 可执行文件名
        output="screen" # 日志输出位置
    )
    
    topic_subscribe_node = Node(
        package="cpp_topic_subscribe_demo",
        executable="cpp_topic_subscribe_demo",
        output="screen"
    )
    
    # 返回启动描述，是一个 launch_ros.actions 类型的数组
    return LaunchDescription([
        int_param_arg,
        int_param_node,
        topic_publish_node,
        topic_subscribe_node])

