from ast import arguments
from http.server import executable
import os
from matplotlib import container
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    list = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.263', '0.250', '0', '0', '0', '0', '1', 'base_link', 'static_tf_wheel_place_0'],
            respawn = True
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.263', '0.250', '0', '0', '0', '0', '1', 'base_link', 'static_tf_wheel_place_1'],
            respawn = True
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.323', '-0.170', '0', '0', '0', '0', '1', 'base_link', 'static_tf_wheel_place_2'],
            respawn = True
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.323', '-0.170', '0', '0', '0', '0', '1', 'base_link', 'static_tf_wheel_place_3'],
            respawn = True
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.28', '0.0', '0.05', '0', '0', '1', '0', 'base_link', 'static_tf_self_position_estimate_lrf'],
            respawn = True
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '-0.15', '0.8', '0', '0', '0.707106781187', '0.707106781187', 'base_link', 'static_tf_bh_lrf'],
            respawn = True
        ),
    ]

    return LaunchDescription(list)