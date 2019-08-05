cmake_minimum_required(VERSION 2.8.3)
project(gripbot_sim_kinematics)

find_package(catkin
  REQUIRED COMPONENTS
  message_generation
  geometry_msgs  
  std_msgs
  roscpp
  sensor_msgs
  tf
  tf_conversions
  kdl_parser
  gazebo_msgs
  gripbot_core_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
  gripbot_core_msgs
  gazebo_msgs
  sensor_msgs
  kdl_parser
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/kinematics.cpp
  src/gripbot_grpr_knmt_ctrl.cpp
  src/gripbot_arm_knmt_ctrl.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(gripbot_kinematics src/gripbot_ctrl.cpp)
target_link_libraries(gripbot_kinematics ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(gripbot_kinematics ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS gripbot_kinematics
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
