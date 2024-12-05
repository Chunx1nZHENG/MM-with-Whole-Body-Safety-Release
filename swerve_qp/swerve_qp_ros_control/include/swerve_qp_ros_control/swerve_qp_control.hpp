#pragma once

#include <controller_interface/controller.h>
#include "SwerveTarget.hpp"

#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include "nav_msgs/Odometry.h"
#include "SwerveModelInfo.hpp"

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <nav_msgs/Path.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "reference_manager/SwerveReferenceManager.hpp"

#include <tf/transform_listener.h>
#include <memory>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"
#include "SwerveInterface.hpp"
#include <ros/ros.h>
# include "swerve_ipopt.hpp"
#include "std_msgs/Bool.h"
#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/common/solver_options.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "Altro_control/Altro_problem.hpp"
#include "altro/problem/problem.hpp"
#include "space_decomp.hpp"
using namespace ocs2;
using namespace swerve;


namespace swerve_qp_ros_control {

class Swerve_qp_RosControl : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
 public:
  Swerve_qp_RosControl(){};

  /**
   * \brief Initialize controller
   * \param hw            Velocity joint interface for the wheels
   * \param root_nh       Node handle at root namespace
   * \param controller_nh Node handle inside the controller namespace
   */
  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  /**
   * \brief Updates controller and sets the new velocity commands
   * \param time   Current time
   * \param period Time since the last called to update
   */
  void update(const ros::Time& time, const ros::Duration& period);
  void QPcomputeThread();
  void AlilqrThread();
  
  struct model_size {
    Eigen::Vector3d x_arm_link;
    Eigen::Vector3d bicep_link;
    Eigen::Vector3d forearm_link;
    Eigen::Vector3d base_link;
    Eigen::Vector3d shoulder_link;
    Eigen::Vector3d spherical_wrist_1_link;
    Eigen::Vector3d spherical_wrist_2_link;
    Eigen::Vector3d bracelet_link;
    ros::Publisher marker_pub;
    ros::NodeHandle nh_;
    decomp_util decomp_util_;
    pcl::PointCloud<pcl::PointXYZ>  cloud_;
    // std::vector<LinearConstraint3D> LinearConstraints_;
    void publishmarker(PinocchioInterface& pinocchioInterface);
    model_size()
        : x_arm_link(Eigen::Vector3d::Zero()),
          bicep_link(Eigen::Vector3d::Zero()),
          forearm_link(Eigen::Vector3d::Zero()), 
          base_link(Eigen::Vector3d::Zero()),
          spherical_wrist_2_link(Eigen::Vector3d::Zero()){}
  } model_size_;
  // Define NStates and NControls
  static constexpr int NStates = 6; // Replace with the actual number of states
  static constexpr int NControls = 1; // Replace with the actual number of controls
  // build a unique pointer to store the MobileManipulatorProblem
  std::unique_ptr<altro::problems::MobileManipulatorProblem> MobileManipulatorProblem_ptr_;
  // build a unique pointer to alsolver
  std::unique_ptr<altro::augmented_lagrangian::AugmentedLagrangianiLQR<NStates, NControls>> solver_al_ptr_;
  std::shared_ptr<altro::Trajectory<NStates, NControls>> traj_ptr_;
  std::unique_ptr<PinocchioStateInputMapping<ocs2::scalar_t>> mappingPtr_;
  std::unique_ptr<altro::problem::Problem> probPtr_;
  //build a subscriber to get the goal position
  ros::Subscriber sub_goal_;
  vector_t uoutput_ = vector_t::Zero(9);
  //build a vector to store the goal position and gesture 6D
  vec_Vec3f jps_path_3d_;
  vec_Vec2f jps_path_;
  nav_msgs::Path ee_path_;
  bool goal_reached_ = false;
  ros::Publisher ee_path_pub_;
  // altro::problem::Problem prob;

  
 protected:
  std::unique_ptr<SwerveInterface> swerveInterface_;
  std::shared_ptr<MRT_ROS_Interface> mpcInterface_;
  ros::Timer mpcUpdateTimer_;

 private:
  void mapInputs(vector_t optimalInput);
  void odomCallback(const nav_msgs::OdometryPtr& msg);
  void cmdVelCallback(const geometry_msgs::TwistPtr& msg);
  void keyboardCallback(const std_msgs::Bool::ConstPtr& msg );
  void missionPathCallback(const nav_msgs::PathPtr& msg);
  void adjustPathTimeStamps(nav_msgs::Path& path);

  SystemObservation observation_;
  bool firstRun_;
  bool realtimeLoop_ = false;
  unsigned int loopCounter_ = 0;
  int frequencyRatio_;
  bool policyUpdated = false;
  bool init_ = false;
  scalar_t timeStep_ = 0;
  SystemObservation currObservation_;
  ros::Subscriber odometrySub_;
  vector_t initTarget_;

  std::shared_ptr<SwerveReferenceManager> referenceManagerPtr_;

  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
  double brakesPGain_ = 100.0;

  std::mutex updatePolicyMutex_;
  std::mutex updateOdomMutex_;
  std::mutex currObservationMutex_;
  
  std::thread QP_thread_;
  std::thread ALILQR_thread_;
  /// Hardware handles
  std::vector<hardware_interface::JointHandle> steerJoints_;
  std::vector<hardware_interface::JointHandle> wheelJoints_;
  std::vector<hardware_interface::JointHandle> armJoints_;
  std::vector<hardware_interface::JointHandle> brakeJoints_;
  std::vector<hardware_interface::JointHandle> gripperJoints_;

  geometry_msgs::Pose currentObservedPose_;
  struct RobotInputs {
    Eigen::Vector4d wheelsInput;
    Eigen::Vector4d steersInput;
    Eigen::VectorXd armInputs;
    Eigen::Vector4d brakesInput;
    scalar_t gripperInput;
    ros::Time stamp;
    
    RobotInputs()
        : wheelsInput(Eigen::Vector4d::Zero()),
          steersInput(Eigen::Vector4d::Zero()),
          armInputs(Eigen::VectorXd::Zero(6)),
          brakesInput(Eigen::Vector4d::Zero()),
          stamp(0.0) {}
  };
  realtime_tools::RealtimeBuffer<RobotInputs> robotInputs_;

  ModelSettings modelSettings_;

  Eigen::Vector4d currBrakePos_;
  Eigen::Vector4d desiredBrakePos_;
  std::unique_ptr<tf::TransformListener> listener_;
  tf::StampedTransform ee_transform_;

  std::unique_ptr<SwerveTarget> swerveTarget_;
  SwerveModelInfo swerveModelInfo_;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  
  std::shared_ptr<RosReferenceManager> rosReferenceManagerPtr_;
  
  ros::Publisher optimalBasePathPublisher_;
  ros::Timer rosDebugInfoTimer_;

  std::unique_ptr<swerve_qp_ipopt::FG_eval> qp_solverPtr_;

  vector_t keyCmdVel_;
  bool keyCmdMod_ = false;
  ros::Subscriber subCmdVel_;
  ros::Subscriber subKeyboard_;
  bool initTargetSet_ = false;
  
};



}  // namespace swerve_mpc_ros_control
