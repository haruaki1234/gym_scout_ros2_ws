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
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory("bringup"), "config", "slam_rviz_config.rviz")]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.0', '0.0', '0', '0', '0', '0', '1', 'map', 'initial_pos'],
            respawn = True
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.2', '-0.1', '0', '0', '0', '0', '1', 'slam_base_link', 'slam_laser'],
            respawn = True
        ),
        Node(
            package='slam_bridge',
            executable='slam_bridge',
            output='screen',
            parameters=[
                {'start_pos': [2.6 , 42.0 , -1.642]}
            ]
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "slam_toolbox_param.yaml")
            ]
        ),
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     output='screen',
        #     parameters=[
        #         os.path.join(pkg_dir, "config", "map_server_param.yaml")
        #     ]
        # ),
        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[
        #         os.path.join(pkg_dir, "config", "amcl_param.yaml")
        #     ]
        # ),
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[
        #         {'autostart': True},
        #         {'node_names': ['map_server', 'amcl']}
        #     ]
        # ),

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '../rosbag/panorama_d4f_scout_continuous_test10/log'],
            output='screen'
        ),
    ]

    return LaunchDescription(list)