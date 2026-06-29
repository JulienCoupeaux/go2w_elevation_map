from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'queue_size': 10,
                'Mem/IncrementalMemory': 'true',
                'Grid/3D': 'true',
                'Grid/GroundIsObstacle': 'false',
                'Grid/NormalsSegmentation': 'true',
                'Grid/MaxGroundAngle': '35',
                'Grid/MaxObstacleHeight': '2.0',
                'Grid/MinClusterSize': '5',
                'Grid/CellSize': '0.05',
            }],
            remappings=[
                ('scan_cloud', '/pointcloud'),
                ('odom', '/odom'),
            ]
        ),
    ])
