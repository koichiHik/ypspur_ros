

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
            executable="ypspur_ros_node",
            name="ypspur_ros_node",
            output="screen",
            parameters=[]
        )
    ])
