cmake_minimum_required(VERSION 3.0.2)
project(project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  rosbag
  rosgraph_msgs
  nav_msgs
  message_generation
  dynamic_reconfigure
  tf2
  tf2_ros
)

add_service_files(
  FILES
  reset.srv
)

add_message_files(
  FILES
  CustomRpm.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/mode.cfg
  cfg/param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinematics_class odometry_class forwardkin_class tfbroad_class
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(kinematics_class src/Kinematics.cpp)
add_dependencies(kinematics_class ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematics_class ${catkin_LIBRARIES})

add_executable(kinematics src/main_kinematics.cpp)
target_link_libraries(kinematics ${catkin_LIBRARIES} kinematics_class)

add_library(odometry_class src/Odometry.cpp)
add_dependencies(odometry_class ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry_class ${catkin_LIBRARIES})

add_executable(odometry src/main_odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry ${catkin_LIBRARIES} odometry_class)

add_library(forwardkin_class src/Forwardkin.cpp)
add_dependencies(forwardkin_class ${catkin_EXPORTED_TARGETS})
target_link_libraries(forwardkin_class ${catkin_LIBRARIES})

add_executable(controlspeed src/main_controlspeed.cpp)
add_dependencies(controlspeed ${catkin_EXPORTED_TARGETS})
target_link_libraries(controlspeed ${catkin_LIBRARIES} forwardkin_class)

add_library(tfbroad_class src/TfBroad.cpp)
add_dependencies(tfbroad_class ${catkin_EXPORTED_TARGETS})
target_link_libraries(tfbroad_class ${catkin_LIBRARIES})

add_executable(broadcaster src/main_broadcaster.cpp)
add_dependencies(broadcaster ${catkin_EXPORTED_TARGETS})
target_link_libraries(broadcaster ${catkin_LIBRARIES} tfbroad_class)
