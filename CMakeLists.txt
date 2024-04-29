cmake_minimum_required(VERSION 3.0.2)
project(tko_docking)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  tf2
  tf2_ros 
  std_msgs
)

roslint_cpp()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lift_tf_pub
  CATKIN_DEPENDS roscpp rospy std_msgs

  # DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tko_center_tf_publisher src/center_tf_publisher.cpp)
target_link_libraries(tko_center_tf_publisher
 ${catkin_LIBRARIES}
)
