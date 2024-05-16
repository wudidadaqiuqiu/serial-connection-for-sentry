import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from launch.actions import TimerAction


def generate_launch_description():

    serial_connection_node = Node(
        package='serial_connection',
        executable='connector',
        output='screen',
        parameters=[{'referee_pub': False}]
    )

    ld = LaunchDescription()

    ld.add_action(serial_connection_node)

    return ld
