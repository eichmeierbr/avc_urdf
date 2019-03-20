################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(avc_motion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)


################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES avc_motion
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(avc_vehicle src/avc_vehicle.cpp)
add_executable(bicycle_dynamics src/bicycle.cpp)
add_executable(simple_map_tf src/simple_map_tf.cpp)
add_executable(list_goal_generator src/list_goal_generator.cpp)

add_dependencies(avc_vehicle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bicycle_dynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simple_map_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(list_goal_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(avc_vehicle ${catkin_LIBRARIES})
target_link_libraries(bicycle_dynamics ${catkin_LIBRARIES})
target_link_libraries(simple_map_tf ${catkin_LIBRARIES})
target_link_libraries(list_goal_generator ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS avc_vehicle bicycle_dynamics simple_map_tf
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
