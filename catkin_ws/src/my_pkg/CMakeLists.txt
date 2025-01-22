cmake_minimum_required(VERSION 3.0.2)
project(my_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_commander
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
  add_executable(ur5_ik src/ur5_ik.cpp)

target_link_libraries(ur5_ik
  ${catkin_LIBRARIES}
)