from ast import arguments
from http.server import executable
import os
import shutil
import time
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
    if os.path.isdir("rosbag/log") == True:
        shutil.rmtree("rosbag/log")
    

    pkg_dir = get_package_share_directory("bringup")

    list = [
        Node(
            package='global_path_planner',
            executable='global_path_planner',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "global_path_planner_param.yaml")
            ]
        ),
        
        Node(
            package='route_following',
            executable='route_following',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "route_following_param.yaml")
            ]
        ),

        Node(
            package='localization',
            executable='localization',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "localization_param.yaml")
            ]
        ),

        Node(
            package='controller',
            executable='controller',
            output='screen'
        ),

        Node(
            package='velocity_control',
            executable='velocity_control',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, "config", "velocity_control_param.yaml")
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[os.path.join(get_package_share_directory("scout_description"), "urdf", "scout_mini.urdf")]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_dir, "config", "rviz_config.rviz")]
        ),

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-s', 'mcap', '-o', 'rosbag/log'],
            output='screen'
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(pkg_dir, 'launch', 'static_tf.launch.py')]
        #     )
        # ),
    ]

    return LaunchDescription(list)
