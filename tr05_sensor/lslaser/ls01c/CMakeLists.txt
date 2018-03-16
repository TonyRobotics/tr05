cmake_minimum_required(VERSION 2.8.3)
project(ls01c)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}/src
)

add_executable(ls01c src/main.cpp src/uart_driver.cpp)
target_link_libraries(ls01c ${catkin_LIBRARIES})

