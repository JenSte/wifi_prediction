cmake_minimum_required(VERSION 2.8)
project(wifi_prediction)

find_package(catkin
  REQUIRED COMPONENTS
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime  
)
