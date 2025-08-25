cmake_minimum_required(VERSION 3.0.2)
project(pnt_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(FILES VIO.msg LIO.msg UltraSound.msg UWB.msg Filter.msg GNSS.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
