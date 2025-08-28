cmake_minimum_required(VERSION 2.8.3)
project(ultra_sound)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_runtime
  pnt_msgs
  byh_uav
)

add_message_files(
  FILES
  UltraSound.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

# 头文件路径
include_directories( 
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_generation message_runtime
)
