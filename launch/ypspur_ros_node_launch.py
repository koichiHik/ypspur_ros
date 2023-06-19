

import os
import sys
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  param_set_dirname = 'param_set_1'

  frame_ids_yaml = os.path.join(
    get_package_share_directory('configs'),
      param_set_dirname,
      'common',
      'frame_ids.yaml'
    )

  ypspur_ros_yaml = os.path.join(
    get_package_share_directory('configs'),
      param_set_dirname,
      'common',
      'ypspur_ros.yaml'
    )
  
  ypspur_param_file = \
    os.path.join(get_package_share_directory('configs'), \
      param_set_dirname, 'common', 'yp_spur.param')

  return LaunchDescription([
    Node(
      package="ypspur_ros",
      executable="ypspur_ros_node",
      name="ypspur_ros",
      output="screen",
      parameters=[frame_ids_yaml, ypspur_ros_yaml, \
        {'param_file':ypspur_param_file}]
    )
  ])
