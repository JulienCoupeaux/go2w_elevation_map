#!/usr/bin/env python3
"""
bringup_mppi.launch.py
======================
Lance la chaine de navigation MPPI 2.5D du go2w (ROS 2 Humble) :

  - traversability_mapper : /pointcloud -> costmap 2.5D + nuage obstacles + grid_map
  - twist_to_stamped      : /cmd_vel_nav (Twist) -> /cmd_vel (TwistStamped)
  - Nav2 : controller_server (MPPI), planner_server, behavior_server,
           bt_navigator, lifecycle_manager (autostart)
  - RViz (optionnel)

Prerequis : la sim tourne deja  ->  ros2 run go2w_sim mujoco_node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('go2w_navigation')
    bringup_dir = get_package_share_directory('go2w_bringup')

    params_file = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'go2w_mppi.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ── Perception 2.5D ──────────────────────────────────────────
        Node(
            package='go2w_perception',
            executable='traversability_mapper',
            name='traversability_mapper',
            output='screen',
        ),

        # ── Relais commande ─────────────────────────────────────────
        Node(
            package='go2w_perception',
            executable='twist_to_stamped',
            name='twist_to_stamped',
            output='screen',
            parameters=[{
                'in_topic': '/cmd_vel_nav',
                'out_topic': '/cmd_vel',
                'frame_id': 'base_link',
            }],
        ),

        # ── Nav2 ─────────────────────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),

        # ── RViz ─────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
