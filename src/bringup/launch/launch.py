from ast import arguments
from http.server import executable
import os
from matplotlib import container
import yaml
import launch
import datetime
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
    pkg_dir = get_package_share_directory("bringup")
    list = [
        Node(
            package='umap_client',
            executable='umap_client',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "umap_client_param.yaml")
            ]
        ),
        Node(
            package='scout_base',
            executable='scout_base_node',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "scout_base_param.yaml")
            ]
        ),
        # Node(
        #     package='image_tools',
        #     executable='cam2image',
        #     output='screen',
        #     arguments=['--ros-args', '--log-level', "WARN"],
        #     parameters=[{"device_id": 3}]
        # ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{"video_device": "/dev/video4"}]
        ),
        Node(
            package='urg_node',
            executable='urg_node_driver',
            arguments=['--ros-args', '-p', 'ip_address:=192.168.0.10']
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_dir, 'launch', 'common.launch.py')]
            )
        )
    ]

    return LaunchDescription(list)