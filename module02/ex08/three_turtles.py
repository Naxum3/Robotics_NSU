import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim1 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='raph',
    	arguments=['--ros-args', '--log-level', 'warn']
    )
    
    turtlesim2 = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='mickey',
    	arguments=['--ros-args', '--log-level', 'warn']
    )
    turtlesim3 =  Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='leo',
    	arguments=['--ros-args', '--log-level', 'warn']
    )

    forward_turtlesim_commands_to_second_turtlesim_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic1',
        remappings=[
            ('/input/pose', '/turtle1/pose'),
            ('/output/cmd_vel', '/turtle2/cmd_vel'),
        ]
    )
    forward_turtlesim_commands_to_third_turtlesim_node = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic2',
        remappings=[
            ('/input/pose', '/turtle2/pose'),
            ('/output/cmd_vel', '/turtle3/cmd_vel'),
        ]
    )


    return LaunchDescription([
        turtlesim1,
        turtlesim2,
        turtlesim3,
        forward_turtlesim_commands_to_second_turtlesim_node,
        forward_turtlesim_commands_to_third_turtlesim_node,
])
