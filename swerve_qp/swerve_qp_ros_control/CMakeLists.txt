cmake_minimum_required(VERSION 3.0.2)
project(swerve_qp_ros_control)

set(CATKIN_PACKAGE_DEPENDENCIES
    roscpp
    controller_interface
    hardware_interface
    swerve_msgs
    ddynamic_reconfigure
    tf
    tf2_ros
    tf2_geometry_msgs
    urdf
    kdl_parser
    robot_state_publisher
    visualization_msgs
    geometry_msgs
    nav_msgs
    ocs2_ros_interfaces
    ocs2_core
    ocs2_ddp
    ocs2_mpc
    ocs2_msgs
    ocs2_robotic_tools
    ocs2_pinocchio_interface
    trapezoidal_tracker
    manipulation_msgs
    swerve_msgs
    ocs2_thirdparty
    decomp_ros_utils)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)

# find_library(fmt REQUIRED)

find_library(
  IPOPT_LIBRARY
  NAMES ipopt
  HINTS /usr/local/lib)
if(NOT IPOPT_LIBRARY)
  message(FATAL_ERROR "Ipopt library not found")
endif()

set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
# find_package(altro REQUIRED) find_library( fmt NAMES libfmt.so.9 REQUIRED
# HINTS ${CMAKE_PREFIX_PATH}/lib)

find_package(PkgConfig REQUIRED)
find_library(
  ModernRoboticsCpp
  NAMES
  HINTS /usr/local/lib)

find_library(COPT_LIB PATHS /opt/copt71/lib)

find_package(jps3d REQUIRED)

# if(NOT COPT_LIB) message(FATAL_ERROR "LibOptimization not found") endif()

pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
  INCLUDE_DIRS
  include
  ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
  Boost
  pinocchio)
set(FLAGS
    ${OCS2_CXX_FLAGS} ${pinocchio_CFLAGS_OTHER} -Wno-ignored-attributes
    -Wno-invalid-partial-specialization -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
    -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  /usr/local/include/coin
  /usr/local/include
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /opt/copt71/include/coptcpp_inc
  /opt/copt71/include
  ${JPS3D_INCLUDE_DIRS})

link_directories(${pinocchio_LIBRARY_DIRS})
link_directories(/opt/copt71/lib)
add_library(
  ${PROJECT_NAME}
  src/constraint/EndEffectorConstraint.cpp
  src/constraint/JointVelocityLimits.cpp
  src/constraint/JointLimitsConstraint.cpp
  src/constraint/LegsInputConstraints.cpp
  src/constraint/WheelRollingConstraintCppAd.cpp
  src/constraint/LegsConstraintCppAd.cpp
  src/dynamics/SwerveDynamics.cpp
  src/reference_manager/SwerveReferenceManager.cpp
  src/cost/NominalSteerStateCppAd.cpp
  src/cost/NominalArmStateCppAd.cpp
  src/swerve_qp_ros_control.cpp
  src/swerve_ipopt.cpp
  src/SwervePinocchioMapping.cpp
  src/SwerveInterface.cpp
  src/SwerveTarget.cpp
  # src/Altro_control/Altro_control.cpp
  src/Altro_control/Altro_model_base_x.cpp
  src/Altro_control/Altro_model_base_y.cpp
  src/Altro_control/Altro_model_base_yaw.cpp
  src/Altro_control/Altro_model_joint1.cpp
  src/Altro_control/Altro_model_joint2.cpp
  src/Altro_control/Altro_model_joint3.cpp
  src/Altro_control/Altro_model_joint4.cpp
  src/Altro_control/Altro_model_joint5.cpp
  src/Altro_control/Altro_model_joint6.cpp
  src/Altro_control/unicycle.cpp
  src/Altro_control/Altro_problem.cpp
  src/Altro_control/quadratic_cost.cpp
  src/Altro_control/quadratic_cost_u.cpp
  src/Altro_control/quadratic_cost_f.cpp
  src/Altro_control/basic_constraints.cpp
  src/Altro_control/RelaxBarrierPenalty.cpp
  src/Altro_control/rotation_tool.cpp
  src/space_decomp.cpp
  src/SDPsolver/SDPsolver.cpp)

add_executable(MAPGEN src/map_generator/map_generator.cpp)
target_link_libraries(MAPGEN Eigen3::Eigen ${PCL_LIBRARIES})

# add_executable(SDPTEST src/sdptest.cpp) target_link_libraries(SDPTEST
# altro::altro fmt::fmt Eigen3::Eigen ${PCL_LIBRARIES}
# /opt/copt71/lib/libcopt_cpp.so)

# message(WARNING "AltroCpp_LIBRARIES: ${fmt}")
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES}
         ${Boost_LIBRARIES}
         ${IPOPT_LIBRARY}
         /usr/local/lib/libModernRoboticsCpp.so
         /opt/copt71/lib/libcopt_cpp.so
         /usr/local/lib/libaugmented_lagrangian.so
         /usr/local/lib/libilqr.so
         /usr/local/lib/libproblem.so
         /usr/local/lib/libconstraints.so
         /usr/local/lib/libthreadpool.so
         /usr/local/lib/libcommon.so
         /usr/local/lib/libutils.so
  # fmt::fmt
  PRIVATE Eigen3::Eigen ${PCL_LIBRARIES} ${JPS3D_LIBRARIES})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES swerve_qp_ros_control_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
