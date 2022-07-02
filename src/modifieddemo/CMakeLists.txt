cmake_minimum_required(VERSION 3.0.2)
project(modifieddemo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  message_generation
  sensor_msgs
  image_transport
  cv_bridge
)

# add_message_files(
#   FILES
# )

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime cv_bridge)



include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf_publish
  src/odom_tf_publish.cpp
)
add_dependencies(odom_tf_publish ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf_publish
  ${catkin_LIBRARIES}
)



