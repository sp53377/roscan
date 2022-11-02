from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roscan_pkg',
            executable='roscan_node',
        ),
        Node(
            package='gps_service_pkg',
            executable='gps_service_node',
        )
    ])