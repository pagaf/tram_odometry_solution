from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('tram_reserve_odometry')
    params = os.path.join(share, 'config', 'odom_params.yaml')
    return LaunchDescription([
        Node(
            package='tram_reserve_odometry',
            executable='reserve_odometry',
            name='tram_reserve_odometry',
            output='screen',
            parameters=[params],
        )
    ])
