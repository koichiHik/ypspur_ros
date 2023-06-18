

import os
import sys
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="ypspur_ros",
            executable="joint_position_to_joint_trajectory",
            name="joint_position_to_joint_trajectory_node",
            output="screen",
            parameters=[
                {"accel": "0.3"},
                {"skip_same": True}
            ]
        )
    ])
