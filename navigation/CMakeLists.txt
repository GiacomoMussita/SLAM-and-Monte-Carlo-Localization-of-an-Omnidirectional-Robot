cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  map_msgs
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(

)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf src/odom_tf.cpp)
add_dependencies(odom_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf ${catkin_LIBRARIES})
