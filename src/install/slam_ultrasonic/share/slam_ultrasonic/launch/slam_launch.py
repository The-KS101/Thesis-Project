import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform from base_footprint to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),
        # Static transform from base_link to ultrasonic_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        #     output='screen'
        # ),
        # Ultrasonic node
        Node(
            package='slam_ultrasonic',
            executable='ultrasonic_node.py',
            name='ultrasonic_node',
            output='screen'
        ),
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[
        #         {'use_sim_time': False},
        #         {'odom_frame': 'odom'},
        #         {'map_frame': 'map'},
        #         {'base_frame': 'base_link'},
        #         {'scan_topic': 'scan'},
        #         {'mode': 'mapping'},
        #     ],
        # ),
    ])
