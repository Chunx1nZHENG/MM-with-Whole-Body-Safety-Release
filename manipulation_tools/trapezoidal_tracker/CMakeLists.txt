cmake_minimum_required(VERSION 3.0.8)
project(trapezoidal_tracker)

add_compile_options(-std=c++17 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
  manipulation_msgs
  polling_action_server
  tf2_ros
  eigen_conversions
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
)

find_package(catkin REQUIRED
  ${PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/JointActionHandler.cpp
  src/TaskSpaceActionHandler.cpp
  src/TrapezoidalVelocityProfile.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(task_space_action_server_node
  src/task_space_action_server_node.cpp
)

add_dependencies(task_space_action_server_node ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(task_space_action_server_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME} task_space_action_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
