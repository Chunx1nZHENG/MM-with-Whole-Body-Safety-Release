cmake_minimum_required(VERSION 3.0.2)

project(swerve_hw_controller)
add_compile_options(-std=c++14)

if(WIN32)
  add_definitions(-D_OS_WINDOWS)
elseif(UNIX AND NOT APPLE)
  add_definitions(-D_OS_UNIX)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  hardware_interface
  message_generation
  class_loader
  pluginlib
  controller_manager
  urdf
  dh_gripper_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs
    hardware_interface
    controller_manager
    pluginlib
    urdf
    dh_gripper_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  Utils/common
  Utils/interface
  Utils/multiThread
  Utils/time
  Utils/kortex_api/include
  Utils/kortex_api/include/client
  Utils/kortex_api/include/client_stubs
  Utils/kortex_api/include/common
  Utils/kortex_api/include/google
  Utils/kortex_api/include/messages
)

link_directories(
  ${PROJECT_NAME}
  ${catkin_LIB_DIRS}
  Utils/kortex_api/lib/release
)
add_library(${PROJECT_NAME}
  src/hardware_interface.cpp
  src/swerve_hw.cpp
  src/swerve_hw_loop.cpp
  src/kortex_interface.cpp
  src/dh_gripper.cpp
  Utils/interface/uart.cpp
  Utils/multiThread/Loop.cpp
  Utils/time/AbsoluteTimer.cpp
)
find_library(IPOPT_LIBRARY NAMES ipopt HINTS /usr/local/lib)
target_link_libraries(${PROJECT_NAME}

  ${catkin_LIBRARIES}
  KortexApiCpp_without_protobuf
  KortexApiCpp
)

add_executable(${PROJECT_NAME}_node src/swerve_hw_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)