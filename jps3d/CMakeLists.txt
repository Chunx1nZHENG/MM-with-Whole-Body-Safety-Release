cmake_minimum_required(VERSION 2.8)
project(jps3d)

# Set C++ standard to C++14 as required by PCL (remove the C++11 flag)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()

set(BUILD_SHARED_LIBS ON)

# Find required libraries
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

find_package(PCL REQUIRED COMPONENTS common io kdtree)

# Ensure YAML-CPP is found
IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add definitions for PCL
add_definitions(${PCL_DEFINITIONS})

# Add libraries for the project
add_library(jps_lib src/jps_planner/graph_search.cpp
  src/jps_planner/jps_planner.cpp)

add_library(dmp_lib src/distance_map_planner/graph_search.cpp
  src/distance_map_planner/distance_map_planner.cpp)

# Add executables and link libraries
add_executable(create_map test/create_map.cpp)
target_link_libraries(create_map jps_lib ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_planner_2d test/test_planner_2d.cpp)
target_link_libraries(test_planner_2d jps_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_3d test/test_planner_3d.cpp)
target_link_libraries(test_planner_3d jps_lib ${YAMLCPP_LIBRARIES} ${PCL_LIBRARIES})
add_test(test_planner_3d test_planner_3d ${CMAKE_SOURCE_DIR}/data/simple3d.yaml)

add_executable(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
target_link_libraries(test_distance_map_planner_2d jps_lib dmp_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_test(test_distance_map_planner_2d test_distance_map_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

# Installation steps
install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS jps_lib dmp_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/jps_basis include/jps_planner include/jps_collision
  DESTINATION include)
