cmake_minimum_required(VERSION 3.0.2)
project(gnss_pose_to_socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gnss_pose_to_socket src/gnss_pose_to_socket.cpp)

target_link_libraries(gnss_pose_to_socket
  ${catkin_LIBRARIES}
)
