#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim_car = get_package_share_directory('sim_car')

    bringup_dir = get_package_share_directory('sim_car')
    launch_dir = os.path.join(bringup_dir, 'launch')
    world_config = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_sim_car, 'worlds', 'simple.world'), ''],
          description='SDF world file')

    # # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_car, 'launch', 'spawn_car.launch.py'),
        )
    )

    joy = Node(
            package='joy', executable='joy_node', output='screen')
    teleop_twist = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            output='screen',
            parameters=[os.path.join(pkg_sim_car, "config", "teleop_twist_joy.yaml")])

    ld = LaunchDescription()
    ld.add_action(world_config)
    ld.add_action(gazebo)
    ld.add_action(car)
    ld.add_action(joy)
    ld.add_action(teleop_twist)
    return ld
