cmake_minimum_required(VERSION 3.0.2)
project(msn_example)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  byh_uav
  livox_ros_driver2
  nlink_parser
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(subscribe_all subscribe_all.cpp)
target_link_libraries(subscribe_all ${catkin_LIBRARIES})