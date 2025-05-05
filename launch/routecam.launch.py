#! /usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    
    camera_host_arg = DeclareLaunchArgument('hostname', default_value='192.168.0.9')
    
    node = Node(
            package='routecam_ros2',
            executable='routecam',
            name='routecam',
            parameters=[
                {'hostname': LaunchConfiguration("hostname")}
            ],
            respawn=True,
        )
    
    
    return LaunchDescription([
        camera_host_arg,
        node,
    ])