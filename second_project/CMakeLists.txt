cmake_minimum_required(VERSION 3.0.2)
project(second_project)

## Trova le dipendenze richieste
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  nav_msgs
  laser_geometry
  pcl_ros
  actionlib
  move_base_msgs
)


## Dichiarazione del pacchetto catkin
catkin_package()

## Includi le directory di intestazione
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Build talker and listener
add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(laser_scan_merger src/laser_scan_merger.cpp)
target_link_libraries(laser_scan_merger ${catkin_LIBRARIES})

add_executable(goal_publisher src/goal_publisher.cpp)
target_link_libraries(goal_publisher ${catkin_LIBRARIES})

