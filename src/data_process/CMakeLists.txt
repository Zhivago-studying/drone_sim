cmake_minimum_required(VERSION 3.0.2)
project(data_process)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  sensors
  tf
  mavros_msgs
  nav_msgs
  gazebo_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages
################################################
add_message_files(
  FILES
  UwbProcessed.msg
  CameraAngle.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

################################################
## catkin specific configuration
################################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs sensors tf mavros_msgs nav_msgs gazebo_msgs message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## ---------- uwb_zero_score ----------
add_executable(${PROJECT_NAME}_uwb_zero_score_node
  src/uwb_zero_score.cpp
)

set_target_properties(${PROJECT_NAME}_uwb_zero_score_node PROPERTIES
  OUTPUT_NAME uwb_zero_score
  PREFIX ""
)

add_dependencies(${PROJECT_NAME}_uwb_zero_score_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_uwb_zero_score_node
  ${catkin_LIBRARIES}
)

## ---------- camera_relative_angle_cal ----------
add_executable(${PROJECT_NAME}_camera_angle_node
  src/camera_relative_angle_cal.cpp
)

set_target_properties(${PROJECT_NAME}_camera_angle_node PROPERTIES
  OUTPUT_NAME camera_relative_angle_cal
  PREFIX ""
)

add_dependencies(${PROJECT_NAME}_camera_angle_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_camera_angle_node
  ${catkin_LIBRARIES}
)

## ---------- ins_eskf ----------
add_executable(${PROJECT_NAME}_ins_eskf_node
  src/ins_eskf.cpp
)

set_target_properties(${PROJECT_NAME}_ins_eskf_node PROPERTIES
  OUTPUT_NAME ins_eskf
  PREFIX ""
)

add_dependencies(${PROJECT_NAME}_ins_eskf_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_ins_eskf_node
  ${catkin_LIBRARIES}
)

## ---------- ins_eskf_test ----------
add_executable(${PROJECT_NAME}_ins_eskf_test_node
  src/ins_eskf_test.cpp
)

set_target_properties(${PROJECT_NAME}_ins_eskf_test_node PROPERTIES
  OUTPUT_NAME ins_eskf_test
  PREFIX ""
)

add_dependencies(${PROJECT_NAME}_ins_eskf_test_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_ins_eskf_test_node
  ${catkin_LIBRARIES}
)
