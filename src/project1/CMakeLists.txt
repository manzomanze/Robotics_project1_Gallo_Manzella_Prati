cmake_minimum_required(VERSION 3.0.2)
project(project1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2 
  tf2_ros
  message_generation
  dynamic_reconfigure
)

## Add custom message and its dependencies
add_message_files(
  FILES
  Num.msg
)

add_service_files(
  FILES
  ResetPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
) 

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(gt_pose src/gt_pose.cpp)
add_dependencies(gt_pose ${catkin_EXPORTED_TARGETS})
target_link_libraries(gt_pose ${catkin_LIBRARIES})

add_executable(broadcaster_tf2 src/broadcaster_tf2.cpp)
add_dependencies(broadcaster_tf2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(broadcaster_tf2 ${catkin_LIBRARIES})
