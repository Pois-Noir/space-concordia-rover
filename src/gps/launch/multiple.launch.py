from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gps', executable='added_gps_node', name='gps_publisher_1'),
        Node(package='gps', executable='added_gps_node', name='gps_publisher_2')
    ])