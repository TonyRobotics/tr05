cmake_minimum_required(VERSION 2.8.3)
project(patrol_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  patrol_node.msg
  patrol_data.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  std_msgs 
  geometry_msgs
  message_runtime
)
