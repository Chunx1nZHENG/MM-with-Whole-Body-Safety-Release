cmake_minimum_required(VERSION 2.8.3)
project(odometry_controller)

set(CATKIN_DEPS
controller_interface
diff_drive_controller
hardware_interface
nav_msgs
geometry_msgs
pluginlib
realtime_tools
roscpp
tf
urdf
swerve_msgs
tf2_ros
eigen_conversions
)


find_package(catkin REQUIRED COMPONENTS
${CATKIN_DEPS}
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_DEPS}
  DEPENDS 
)

include_directories(
  include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/odometry_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES odometry_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})