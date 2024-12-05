cmake_minimum_required(VERSION 2.8.3)
project(polling_action_server)

add_compile_options(-std=c++14 -Wall -Werror)

set(PACKAGE_DEPENDENCIES
  actionlib
  actionlib_tutorials

  #For testing
  genmsg
  actionlib_msgs
)

find_package(catkin REQUIRED
  ${PACKAGE_DEPENDENCIES}
)

add_action_files(DIRECTORY test/action FILES FibonacciPollingActionTest.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/utils.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  # Test Action Server
  add_executable(fibonacci_node
    test/FibonacciNode.cpp
    test/FibonacciComputer.cpp
  )
  target_link_libraries(fibonacci_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  add_dependencies(fibonacci_node
    ${PROJECT_NAME}_gencpp
  )

  # rostest
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test
    test/ros/FibonacciTest.test
    test/test_main.cpp
    test/FibonacciServerClientTest.cpp
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
  add_dependencies(${PROJECT_NAME}-test
    ${PROJECT_NAME}_gencpp
    # Force fibonacci_node to be built before the tests
    fibonacci_node
  )

endif()
