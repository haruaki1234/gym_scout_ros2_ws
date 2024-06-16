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
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', os.path.join(get_package_share_directory("bringup"), "config", "vgm_config.rviz")],
        #     parameters=[{'use_sim_time': True}]
        # ),

        Node(
            package='slam_bridge',
            executable='slam_bridge',
            output='screen',
            parameters=[
                {'start_pos': [3.9 , 19.95 , 0.00], 'use_sim_time': True}
            ],
        ),
        Node(
            package='umap_client',
            executable='umap_client',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "umap_client_param.yaml"),
                {'use_sim_time': True}
            ]
        ),
        Node(
            package='localization',
            executable='localization',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "localization_param.yaml"),
                {'use_sim_time': True}
            ]
        ),

        Node(
            name='emcl2',
            package='emcl2',
            executable='emcl2_node',
            output='log',
            parameters=[
                os.path.join(pkg_dir, "config", "emcl_param.yaml"),
                {'use_sim_time': True}
            ],
            ros_arguments=['--log-level', 'warn']
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "map_server_param.yaml")
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.255', '-0.1425', '0', '0', '0', '0', '1', 'slam_base_link', 'slam_laser'],
            respawn = True
        ),

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '../rosbag/2024.06.14_gym/trial07/log', '--clock'],
            output='screen'
        ),
    ]

    return LaunchDescription(list)