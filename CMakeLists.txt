cmake_minimum_required(VERSION 3.8)
project(ypspur_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#########################
#
# find dependencies
#
#########################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DigitalOutput.msg"
  "msg/DigitalInput.msg"
  "msg/ControlMode.msg"
  "msg/JointPositionControl.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

find_package(glog REQUIRED)
find_package(ypspur REQUIRED)

#########################
#
# solve directory
#
#########################
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${ypspur_INCLUDE_DIRS}
)

#########################
# joint_tf_publisher
#########################
add_executable(joint_tf_publisher src/joint_tf_publisher.cpp)
target_link_libraries(joint_tf_publisher 
  glog::glog
)
ament_target_dependencies(joint_tf_publisher
  rclcpp
  builtin_interfaces
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

#########################
# ypspur_ros_node
#########################
add_executable(ypspur_ros_node src/ypspur_ros.cpp)
target_link_libraries(ypspur_ros_node 
  ypspur
  glog::glog
)
ament_target_dependencies(ypspur_ros_node
  rclcpp
  builtin_interfaces
  std_msgs
  sensor_msgs
  trajectory_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(ypspur_ros_node "${cpp_typesupport_target}")

#########################
# joint_position_to_joint_trajectory
#########################
add_executable(joint_position_to_joint_trajectory src/joint_position_to_joint_trajectory.cpp)
target_link_libraries(joint_position_to_joint_trajectory 
  ypspur
  glog::glog
)
ament_target_dependencies(joint_position_to_joint_trajectory
  rclcpp
  builtin_interfaces
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  trajectory_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(joint_position_to_joint_trajectory "${cpp_typesupport_target}")

#############
## Install ##
#############

install(TARGETS
  ypspur_ros_node
  joint_tf_publisher
  joint_position_to_joint_trajectory
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#   add_subdirectory(test)
#   find_package(roslint REQUIRED)
#   roslint_cpp()
#   roslint_add_test()
# endif()

ament_package()
