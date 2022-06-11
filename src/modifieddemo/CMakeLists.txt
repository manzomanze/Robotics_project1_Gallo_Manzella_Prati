cmake_minimum_required(VERSION 3.0.2)
project(modifieddemo)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(

)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf_modified
  src/odom_tf_modified.cpp
)
add_dependencies(odom_tf_modified ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf_modified
  ${catkin_LIBRARIES}
)

