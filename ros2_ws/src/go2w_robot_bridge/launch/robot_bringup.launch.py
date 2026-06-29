#!/usr/bin/env python3
"""
robot_bringup.launch.py
=======================
Chaine de navigation MPPI 2.5D sur le VRAI go2w (ROS 2 Humble).

Difference avec la sim (bringup_mppi.launch.py) :
  - PAS de mujoco_node ni twist_to_stamped.
  - sportmode_to_odom : /sportmodestate -> /odom + TF odom->base_link
    (remplace la TF ground-truth de la sim).
  - cmd_vel_to_sport  : /cmd_vel_nav -> Unitree Move (/api/sport/request).
  - TF statique map->odom (identite) : on navigue en frame odom, sans carte.
  - TF statique base_link->lidar : extrinseque du LiDAR 3D (A CALIBRER).

PREREQUIS cote robot, AVANT ce launch :
  1. unitree_ros2 source + CYCLONEDDS_URI sur la bonne interface (cf README).
  2. Le driver du LiDAR 3D publie /pointcloud (sinon remap ci-dessous).
  3. Robot en BalanceStand (debout, pret a recevoir Move). Voir doc REAL_ROBOT.
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
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # Topic du nuage LiDAR. Le Go2 publie nativement /utlidar/cloud
        # (sensor_msgs/PointCloud2, frame 'utlidar_lidar') — pas besoin du driver
        # Hesai separe. Override possible si on veut /utlidar/cloud_base, etc.
        DeclareLaunchArgument('pointcloud_topic',
                              default_value='/utlidar/cloud'),

        # ── Localisation : etat Unitree -> odom + TF ─────────────────
        Node(
            package='go2w_robot_bridge',
            executable='sportmode_to_odom',
            name='sportmode_to_odom',
            output='screen',
            parameters=[{
                'state_topic': '/sportmodestate',
                'odom_topic': '/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
            }],
        ),

        # ── TF statique map->odom (identite) : navigation sans carte ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),

        # ── TF statique base_link->utlidar_lidar (extrinseque LiDAR) ──
        # Extrinseque XT-16 dans le repere IMU du Go2 : (0.171, 0, 0.0908),
        # rotation identite. Notre base_link EST le repere IMU (odom->base_link
        # construit depuis position+quaternion IMU du SportModeState), donc la
        # valeur s'applique directement. args = x y z yaw pitch roll parent child.
        # NB : si le robot publie deja un /tf parent pour 'utlidar_lidar',
        # RETIRER ce node (un frame ne peut avoir qu'un seul parent).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_lidar',
            arguments=['0.171', '0', '0.0908', '0', '0', '0',
                       'base_link', 'utlidar_lidar'],
            output='screen',
        ),

        # ── Perception 2.5D ──────────────────────────────────────────
        # Le mapper lit la frame du nuage (msg.header.frame_id = utlidar_lidar) :
        # tf2 compose odom<-base_link<-utlidar_lidar. /pointcloud est remappe
        # vers le topic LiDAR reel du robot.
        Node(
            package='go2w_perception',
            executable='traversability_mapper',
            name='traversability_mapper',
            output='screen',
            remappings=[('/pointcloud', pointcloud_topic)],
        ),

        # ── Commande : cmd_vel Nav2 -> Unitree Move ──────────────────
        Node(
            package='go2w_robot_bridge',
            executable='cmd_vel_to_sport',
            name='cmd_vel_to_sport',
            output='screen',
            parameters=[{
                'in_topic': '/cmd_vel_nav',
                'request_topic': '/api/sport/request',
                'vx_max': 0.6,
                'vy_max': 0.4,
                'wz_max': 1.0,
                'cmd_timeout': 0.5,
            }],
        ),

        # ── Nav2 (INCHANGE par rapport a la sim) ─────────────────────
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
                'use_sim_time': False,
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
