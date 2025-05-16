cmake_minimum_required(VERSION 2.8.3)
project(tcpip_topic_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tcpip_topic_publisher_node src/tcpip_topic_publisher_node.cpp)
target_link_libraries(tcpip_topic_publisher_node ${catkin_LIBRARIES})
