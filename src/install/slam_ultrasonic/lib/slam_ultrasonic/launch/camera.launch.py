import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform from base_footprint to base_link
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{'image_size': [640, 480]},
            {'camera_frame_id': 'camera_link_optical'}
            ] 
        ),
    ])