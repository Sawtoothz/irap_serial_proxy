cmake_minimum_required(VERSION 3.0.2)
project(irap_serial_proxy)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system date_time)
find_package(Threads REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irap_serial_proxy
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

set(iRAP_SERIAL_PROXY_SOURCES
  src/AsyncSerial.cpp
  src/irap_serial_proxy.cpp
  src/irap_serial_proxy_node.cpp  
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${iRAP_SERIAL_PROXY_SOURCES})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)