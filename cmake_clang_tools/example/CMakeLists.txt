cmake_minimum_required (VERSION 2.8)
project(clang_tools_example)

#########################
###   CLANG TOOLING   ###
#########################

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#########################
###       BUILD       ###
#########################

# Build a simple library
include_directories(include)
add_definitions(-std=c++11)
add_library(${PROJECT_NAME} src/ClangToolsExample.cpp)

# Build a simple executable
add_executable(${PROJECT_NAME}_exec src/main.cpp)
target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME})

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_exec)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_exec
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif(cmake_clang_tools_FOUND)
