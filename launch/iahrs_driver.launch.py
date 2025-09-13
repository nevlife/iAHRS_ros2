#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    iahrs_driver_node = Node(
        package='iahrs_ros2',
        executable='iahrs_driver',
        name='iahrs_driver',
        output="screen",
        parameters=[
            {'port': '/dev/iAHRS'},
            {'parent_frame_id': 'base_link'},
            {'publish_tf': True},
            {'tf_pos_x': 0.0},
            {'tf_pos_y': 0.0},
            {'tf_pos_z': 0.2},
        ]
    )
    
    return LaunchDescription([
        iahrs_driver_node
    ])