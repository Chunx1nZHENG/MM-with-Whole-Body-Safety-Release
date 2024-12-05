#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "pinocchio/multibody/fcl.hpp"

#include "swerve_qp_ros_control/swerve_qp_control.hpp"
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <pluginlib/class_list_macros.h>
#include "SwerveDynamics.hpp"
#include "SwervePinocchioMapping.hpp"
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h"
# include "swerve_ipopt.hpp"
#include <thread>
#include <visualization_msgs/Marker.h>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>
#include <iostream>

#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/common/solver_options.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "Altro_control/Altro_problem.hpp"
#include "SwervePinocchioMapping.hpp"
#include "modernrobotics/modern_robotics.h"
#include "Altro_control/rotation_tool.hpp"
#include "space_decomp.hpp"
#include <chrono>
using namespace ocs2;
using namespace swerve;
using namespace altro;
using namespace problems;

namespace swerve_qp_ros_control {
using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;
bool Swerve_qp_RosControl::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // Controller initialization
  typedef hardware_interface::VelocityJointInterface VelIface;

  const std::string complete_ns = controller_nh.getNamespace();

  std::size_t id = complete_ns.find_last_of("/");
  std::string name_ = complete_ns.substr(id + 1);


  firstRun_ = true;

  // Get node parameters
  std::string taskFile, libFolder, urdfString;
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/libFolder", libFolder);
  controller_nh.getParam("/swerve_mpc/robot_description", urdfString);
  std::string model_size;
  controller_nh.getParam("/model_size", model_size);
  // Robot interface
  swerveInterface_.reset(new SwerveInterface(taskFile, libFolder, urdfString));
 
  swerveModelInfo_ = swerveInterface_->getSwerveModelInfo();

  rosReferenceManagerPtr_.reset(
      new ocs2::RosReferenceManager("swerve", swerveInterface_->getReferenceManagerPtr()));
  rosReferenceManagerPtr_->subscribe(root_nh);

  
  vector_t upperBound(swerveModelInfo_.inputDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound", upperBound);
  
  vector_t joint_upperLimit(swerveModelInfo_.jointLimitsDim);
  loadData::loadEigenMatrix(taskFile, "jointLimits.upperLimit", joint_upperLimit);
  pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(swerveInterface_->getPinocchioInterface());
  qp_solverPtr_.reset(new swerve_qp_ipopt::FG_eval(swerveModelInfo_.armDim, *pinocchioInterfacePtr_, 
                                                  swerveModelInfo_, upperBound, joint_upperLimit, controller_nh));
                                             
 


  loadData::loadEigenMatrix(model_size, "model_size_information.bicep_link", model_size_.bicep_link);
  loadData::loadEigenMatrix(model_size, "model_size_information.forearm_link", model_size_.forearm_link);
  loadData::loadEigenMatrix(model_size, "model_size_information.x_arm_link", model_size_.x_arm_link);
  loadData::loadEigenMatrix(model_size, "model_size_information.base_link", model_size_.base_link);
  loadData::loadEigenMatrix(model_size, "model_size_information.shoulder_link", model_size_.shoulder_link);
  loadData::loadEigenMatrix(model_size, "model_size_information.spherical_wrist_2_link", model_size_.spherical_wrist_2_link);
  loadData::loadEigenMatrix(model_size, "model_size_information.spherical_wrist_1_link", model_size_.spherical_wrist_1_link);
  loadData::loadEigenMatrix(model_size, "model_size_information.bracelet_link", model_size_.bracelet_link);
 

  model_size_.marker_pub = root_nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  model_size_.decomp_util_ = decomp_util(root_nh);
  ee_path_pub_ = root_nh.advertise<nav_msgs::Path>("ee_path", 1, true);
  std::string  pcd_file;
  controller_nh.getParam("/pcd_file", pcd_file);
  model_size_.cloud_ = model_size_.decomp_util_.readPCDFile(pcd_file);
  
  // initial state
  
  observation_.state = swerveInterface_->getInitialState();
  observation_.input.setZero(swerveModelInfo_.inputDim);
  observation_.time = ros::Time().now().toSec();

  currObservation_ = observation_;

  // Initialize SwerveTarget
  swerveTarget_.reset(new SwerveTarget(root_nh, swerveModelInfo_, "odom"));
  if (swerveModelInfo_.armPresent) {
    swerveTarget_->initInterectiveMarker();
  } else {
    swerveTarget_->initSetGoalPose();
  }

  std::string odomTopic = controller_nh.param<std::string>("odom_topic", "odometry_controller/odom");
  ROS_INFO_STREAM("MPC odom topic: " << odomTopic);
  odometrySub_ = root_nh.subscribe(odomTopic, 10, &Swerve_qp_RosControl::odomCallback, this);

  subCmdVel_ = root_nh.subscribe("/cmd_vel", 1, &Swerve_qp_RosControl::cmdVelCallback, this);
  subKeyboard_ = root_nh.subscribe("/keyboard", 1, &Swerve_qp_RosControl::keyboardCallback, this);
  keyCmdVel_ = vector_t::Zero(3);

  // Initialize tf listener
  listener_.reset(new tf::TransformListener);

  // Load joints names
  std::vector<std::string> steerJointNames = controller_nh.param("steer_joint_names", std::vector<std::string>());
  std::vector<std::string> wheelJointNames = controller_nh.param("wheel_joint_names", std::vector<std::string>());
  // std::vector<std::string> brakeJointNames = controller_nh.param("brake_joint_names", std::vector<std::string>());
  std::vector<std::string> armJointNames = controller_nh.param("arm_joint_names", std::vector<std::string>());
  std::vector<std::string> gripperJointNames = controller_nh.param("gripper_joint_names", std::vector<std::string>());
  
  for (const auto& jointName : steerJointNames) {
    steerJoints_.push_back(hw->getHandle(jointName));
    ROS_INFO_STREAM_NAMED(name_, "Adding steering velocity joint: " << jointName);
  }

  for (const auto& jointName : wheelJointNames) {
    wheelJoints_.push_back(hw->getHandle(jointName));
    ROS_INFO_STREAM_NAMED(name_, "Adding wheel velocity joint: " << jointName);
  }

  if (swerveModelInfo_.armPresent) {
    for (const auto& jointName : armJointNames) {
      armJoints_.push_back(hw->getHandle(jointName));
      ROS_INFO_STREAM_NAMED(name_, "Adding arm velocity joint: " << jointName);
    }
    for (const auto& jointName : gripperJointNames) {
      gripperJoints_.push_back(hw->getHandle(jointName));
      ROS_INFO_STREAM_NAMED(name_, "Adding gripper velocity joint: " << jointName);
    }
  }
  
  ROS_INFO_STREAM_NAMED(name_, "Finished controller initialization");

  mappingPtr_.reset(new SwervePinocchioMapping(swerveModelInfo_));

  ROS_INFO_STREAM_NAMED(name_, "Finished controller initialization4");
  return true;
}

void Swerve_qp_RosControl::update(const ros::Time& time, const ros::Duration& period) {
    static std::vector<double>  arm_position(6, 0);
    static bool elapsed_time_flag = false;
    currObservation_.time = ros::Time().now().toSec();
    {
      std::lock_guard<std::mutex> lockGuard(updateOdomMutex_);

      // Write current odometry
      currObservation_.state(x_state_ind) = currentObservedPose_.position.x;
      currObservation_.state(y_state_ind) = currentObservedPose_.position.y;
      currObservation_.state(z_state_ind) = currentObservedPose_.position.z;

      currObservation_.state(x_quat_state_ind) = currentObservedPose_.orientation.x;
      currObservation_.state(y_quat_state_ind) = currentObservedPose_.orientation.y;
      currObservation_.state(z_quat_state_ind) = currentObservedPose_.orientation.z;
      currObservation_.state(w_quat_state_ind) = currentObservedPose_.orientation.w;
    }

    // Set observations
    for (size_t i = 0; i < steerJoints_.size(); ++i) {
      currObservation_.state(modelSettings_.steersStateIndex[i]) = steerJoints_[i].getPosition();
    }
    for (size_t i = 0; i < wheelJoints_.size(); ++i) {
      currObservation_.state(modelSettings_.wheelsStateIndex[i]) = wheelJoints_[i].getPosition();
    }
    if (swerveModelInfo_.armPresent) {
      for (size_t i = 0; i < armJoints_.size(); ++i) {
        currObservation_.state(modelSettings_.armJointsStateIndex[i]) = armJoints_[i].getPosition();
      }
    }
    
    if (firstRun_) {
      try {
        listener_->lookupTransform("odom", swerveModelInfo_.eeFrame, ros::Time(0), ee_transform_);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
      firstRun_ = false;
      // initial command
      vector_t initTarget(7);
      initTarget.head(3) << ee_transform_.getOrigin().x(), ee_transform_.getOrigin().y(), ee_transform_.getOrigin().z();
      initTarget.tail(4).head(4) << ee_transform_.getRotation().x(), ee_transform_.getRotation().y(), ee_transform_.getRotation().z(),
          ee_transform_.getRotation().w();

      const vector_t zeroInput = vector_t::Zero(swerveModelInfo_.inputDim);
      TargetTrajectories initTargetTrajectories({observation_.time}, {initTarget}, {zeroInput});
      loopCounter_ = 0;
      rosReferenceManagerPtr_->setTargetTrajectories(std::move(initTargetTrajectories));
      // QP_thread_ = std::thread(&Swerve_qp_RosControl::QPcomputeThread, this);
      ALILQR_thread_ = std::thread(&Swerve_qp_RosControl::AlilqrThread, this);

    }
    
    RobotInputs currInputs = *(robotInputs_.readFromRT());

    auto currtime = ros::Time::now();
    auto elapsed = currtime - currInputs.stamp;
    if (elapsed.toSec() > 0.3) {
      currInputs.wheelsInput.setZero();
      currInputs.steersInput.setZero();
      currInputs.armInputs.setZero();
      currInputs.gripperInput = 0;
      if (!elapsed_time_flag)
      {
        arm_position = std::vector<double>(currObservation_.state.tail(6).data(), 
                                   currObservation_.state.tail(6).data() + 6);

        elapsed_time_flag = true;
        std::cout << "\033[31mNo input received for 0.3s\033[0m" << std::endl;
      }
      for (int i = 0; i < 6; i++)
      {
        currInputs.armInputs(i) = 50 * (arm_position[i] - currObservation_.state(sh_rot_state_ind + i));
      }
     

    }
    else
    {
      elapsed_time_flag = false;
    }
    if (!initTargetSet_)
    {
        currInputs.wheelsInput.setZero();
        currInputs.steersInput.setZero();
        currInputs.armInputs.setZero();
        arm_position = std::vector<double>{0.0, -0.5, -1, 0.0, -1.1, 0.0};
        for (int i = 0; i < 6; i++)
        {
          currInputs.armInputs(i) = 50 * (arm_position[i] - currObservation_.state(sh_rot_state_ind + i));
        }
    } 
    for (size_t i = 0; i < steerJoints_.size(); ++i) {
      steerJoints_[i].setCommand(currInputs.steersInput(i));
    }
    for (size_t i = 0; i < wheelJoints_.size(); ++i) {
      wheelJoints_[i].setCommand(currInputs.wheelsInput(i));
    }
    if (swerveModelInfo_.armPresent) {
      for (size_t i = 0; i < armJoints_.size(); ++i) {
        armJoints_[i].setCommand(currInputs.armInputs(i));
      }
      for (size_t i = 0; i < gripperJoints_.size(); ++i) {
        gripperJoints_[i].setCommand(currInputs.gripperInput);
      }
    }
    // std::cout<<"arm_position: "<<currObservation_.state.segment<6>(sh_rot_state_ind).transpose()<<std::endl;
  
}

void Swerve_qp_RosControl::AlilqrThread()
{
  static int freeregion_generate_flag = 3;
  while (true)
  {
    //////////trajectory generation//////////////////
    rosReferenceManagerPtr_->preSolverRun(0,0,currObservation_.state);
    const auto& targetTrajectories = rosReferenceManagerPtr_->getTargetTrajectories();
    vector_t disiredState;
    vector_t end_disiredState;
    if (targetTrajectories.empty()) {
      std::cerr << "[Swerve_qp_RosControl] TargetTrajectories is empty!"<< std::endl;
      disiredState = vector_t::Zero(7);
      disiredState[3] = 1;

    } else {
      disiredState= targetTrajectories.getDesiredState(currObservation_.time);
      end_disiredState = targetTrajectories.getDesiredState(10000);
    }
    Eigen::Vector3d disiredPosition = disiredState.segment<3>(0);
    quaternion_t disiredOrientation(disiredState.segment<4>(3));
    //convert the quaternion to euler angle

    Eigen::Vector3d euler = rotationMatrixToRPY(disiredOrientation.toRotationMatrix());

    auto quat = rpyToQuaternion(euler[0], euler[1], euler[2]);

    // auto eeq = ee_transform_.getRotation();
    // Eigen::Quaterniond eigen_quat(eeq.w(), eeq.x(), eeq.y(), eeq.z());

 
    // auto ee_euler = rotationMatrixToRPY(eigen_quat.toRotationMatrix());

    // for (int i=0; i<3; i++)
    // {
    //   if (euler[i] - ee_euler[i] > M_PI)
    //   {
    //     euler[i] -= 2*M_PI;
    //   }
    //   else if (euler[i] - ee_euler[i] < -M_PI)
    //   {
    //     euler[i] += 2*M_PI;
    //   } 
    // }
    // // Eigen::Vector3d euler ;
    // //print the desired position and orientation
    // std::cout << "disiredPosition: " << disiredPosition.transpose() << std::endl;
    // std::cout << "euler: " << 180 / M_PI * euler.transpose() << std::endl;  
    // std::cout << "euler test" << 180 / M_PI * rotationMatrixToRPYtest(disiredOrientation.toRotationMatrix()).transpose() << std::endl;
    // std::cout << "euler eigen" << 180 / M_PI * disiredOrientation.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << std::endl;
    /////////////////////////////////////////////////////////////////////


    auto start = std::chrono::high_resolution_clock::now();
    //update the kinematics
    auto currState_ = currObservation_.state;
    // desired_position_ = desired_position;
    // desired_orientation_ = desired_orientation;
    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData();
    const auto q = mappingPtr_->getPinocchioJointPosition(currState_);
    pinocchio::forwardKinematics(model, data, q);
    //calculate the gesture of each joint wrt the world frame
    pinocchio::updateFramePlacements(model, data);




    int indBaseLink = model.getFrameId("base_link");
    int indXarmLink = model.getFrameId("x_arm_link");
    int indShoulder = model.getFrameId("shoulder_link");
    int indBicepLink = model.getFrameId("bicep_link");
    int indForearmLink = model.getFrameId("forearm_link");
    int indWrist_1Link = model.getFrameId("spherical_wrist_1_link");
    int indWrist_2Link = model.getFrameId("spherical_wrist_2_link");
    int indEndEffector = model.getFrameId("end_effector");

    pinocchio::SE3Tpl<scalar_t, 0> worldToBaseLinkTransf = data.oMf[indBaseLink];
    pinocchio::SE3Tpl<scalar_t, 0> worldToXarmLinkTransf = data.oMf[indXarmLink];
    pinocchio::SE3Tpl<scalar_t, 0> worldToShoulderLinkTransf = data.oMf[indShoulder];
    pinocchio::SE3Tpl<scalar_t, 0> worldToBicepLinkTransf = data.oMf[indBicepLink];
    pinocchio::SE3Tpl<scalar_t, 0> worldToForearmLinkTransf = data.oMf[indForearmLink];
    pinocchio::SE3Tpl<scalar_t, 0> worldToWrist_1LinkTransf = data.oMf[indWrist_1Link];
    pinocchio::SE3Tpl<scalar_t, 0> worldToWrist_2LinkTransf = data.oMf[indWrist_2Link];
    pinocchio::SE3Tpl<scalar_t, 0> worldToEndEffectorTransf = data.oMf[indEndEffector];
    
    //get the 3*3 from the 4*4 matrix
    auto BaseM = mr::TransToRp(worldToBaseLinkTransf.toHomogeneousMatrix_impl());
    auto XarmM = mr::TransToRp(worldToXarmLinkTransf.toHomogeneousMatrix_impl());
    auto ShoulderM = mr::TransToRp(worldToShoulderLinkTransf.toHomogeneousMatrix_impl());
    auto BicepM = mr::TransToRp(worldToBicepLinkTransf.toHomogeneousMatrix_impl());
    auto ForearmM = mr::TransToRp(worldToForearmLinkTransf.toHomogeneousMatrix_impl());
    auto Wrist_1M = mr::TransToRp(worldToWrist_1LinkTransf.toHomogeneousMatrix_impl());
    auto Wrist_2M = mr::TransToRp(worldToWrist_2LinkTransf.toHomogeneousMatrix_impl());
    auto EndEffectorM = mr::TransToRp(worldToEndEffectorTransf.toHomogeneousMatrix_impl());
    
    Eigen::Matrix4d xarmToShoulderLinkTransf = worldToXarmLinkTransf.inverse().toHomogeneousMatrix()* worldToShoulderLinkTransf.toHomogeneousMatrix();
    Eigen::Matrix4d shoulderLinkToBicepLinkTransf = worldToShoulderLinkTransf.inverse().toHomogeneousMatrix()* worldToBicepLinkTransf.toHomogeneousMatrix();
    Eigen::Matrix4d bicepToForearmLinkTransf = worldToBicepLinkTransf.inverse().toHomogeneousMatrix()* worldToForearmLinkTransf.toHomogeneousMatrix();
    Eigen::Matrix4d forearmToWrist_1LinkTransf = worldToForearmLinkTransf.inverse().toHomogeneousMatrix()* data.oMf[indWrist_1Link].toHomogeneousMatrix();
    Eigen::Matrix4d wrist_1LinkToWrist_2LinkTransf = worldToWrist_1LinkTransf.toHomogeneousMatrix().inverse()* worldToWrist_2LinkTransf.toHomogeneousMatrix();
    Eigen::Matrix4d wrist_2LinkToEndEffectorTransf = worldToWrist_2LinkTransf.inverse().toHomogeneousMatrix()* worldToEndEffectorTransf.toHomogeneousMatrix();
    auto BaseM_R = rotationMatrixToRPY(worldToBaseLinkTransf.rotation()).transpose();auto BaseM_T = worldToBaseLinkTransf.translation().transpose();
    Eigen::Matrix<double,1,6> BaseM_TR;
    BaseM_TR << BaseM_T, BaseM_R;

    /////////////build the problem////////////////////////////////////////////////////
    model_size_.publishmarker(*pinocchioInterfacePtr_);
    Vec2f start_pt( BaseM_TR(0,0) , BaseM_TR(0,1));
    auto ee_pos =  EndEffectorM[1].transpose();
    Vec3f start_pt_3d(ee_pos(0) , ee_pos(1), ee_pos(2));
    Vec3f goal_3d( end_disiredState[0],  end_disiredState[1],  end_disiredState[2]);
    Vec2f goal(end_disiredState[0], end_disiredState[1]);
    if(swerveTarget_->getstate()==1)
    {
      
      // Vec2f goal(disiredPosition[0], disiredPosition[1]);
      initTargetSet_ = true;

      jps_path_ = model_size_.decomp_util_.jps_planner(start_pt,goal);
     
      bool solved;
      
      jps_path_3d_ = model_size_.decomp_util_.jps_planner_3d(start_pt_3d, goal_3d,&solved, 1);
      std::cout << "jps_path finish " << std::endl;
      goal_reached_ = false;

      swerveTarget_->setstate(0);
    }
    
    Vec2f base_goal = model_size_.decomp_util_.get_goal_from_path(jps_path_, start_pt, 0.3, 0.5);
    Vec3f ee_goal = model_size_.decomp_util_.get_goal_from_path(jps_path_3d_, start_pt_3d, 0.15, 0.15);
  
    MobileManipulatorProblem_ptr_ = std::make_unique<MobileManipulatorProblem>();
    // set xf
    ee_goal(0) = worldToXarmLinkTransf.translation()[0];
    ee_goal(1) = worldToXarmLinkTransf.translation()[1];
    ee_goal(2) = 1.5;
    geometry_msgs::PointStamped ee_goal_msg;
    ee_goal_msg.header.frame_id = "odom";
    ee_goal_msg.header.stamp = ros::Time::now();
    ee_goal_msg.point.x = ee_goal(0);
    ee_goal_msg.point.y = ee_goal(1);
    ee_goal_msg.point.z = ee_goal(2);
    model_size_.decomp_util_.ee_goal_pub.publish(ee_goal_msg);
    MobileManipulatorProblem_ptr_->xf << ee_goal(0), ee_goal(1), ee_goal(2), euler;
    // MobileManipulatorProblem_ptr_->xf << BaseM_TR(0,0), BaseM_TR(0,1), 1.2, euler;
    MobileManipulatorProblem_ptr_->x0 << BaseM_TR(0,0) , BaseM_TR(0,1) , BaseM_TR(0,2) , 0 , 0 , BaseM_TR(0,5);
    
    
    MobileManipulatorProblem_ptr_->base_target << base_goal(0), base_goal(1),0.2,0,0,0;
    //print the desired position and current position
    std::cout << "disiredPosition: " << disiredPosition.transpose() << std::endl;
   
    MobileManipulatorProblem_ptr_->state = currState_.tail(6); 

    auto freeregionCenters = model_size_.decomp_util_.getFreeRegionCenters();
    auto LinearConstraints_ = model_size_.decomp_util_.getLinearConstraints();
    MobileManipulatorProblem_ptr_-> setFreeRegion(LinearConstraints_, freeregionCenters);
    if ((start_pt_3d - goal_3d).norm() < 1.5)
    {
       probPtr_.reset(new altro::problem::Problem(MobileManipulatorProblem_ptr_->MakeProblem(true, true))); 
       std::cout << "mode: track ee" << std::endl;
    }
    else
    {
      MobileManipulatorProblem_ptr_->xf << ee_goal(0),ee_goal(1), ee_goal(2), euler;
      probPtr_.reset(new altro::problem::Problem(MobileManipulatorProblem_ptr_->MakeProblem(true, false))); 
      std::cout << "mode: track base" << std::endl;
    }
    // probPtr_.reset(new altro::problem::Problem(MobileManipulatorProblem_ptr_->MakeProblem(true, false))); 
    // if ((start_pt - goal).norm() < 0.05 &&  !goal_reached_)
    if ((start_pt_3d - goal_3d).norm() < 0.05 &&  !goal_reached_)
    {
        goal_reached_ = true;
        ee_path_.header.stamp = ros::Time::now();
        ee_path_.header.frame_id = "odom";
        ee_path_pub_.publish(ee_path_);
        
        double path_length = 0; 
        for (int i = 0; i < ee_path_.poses.size(); i++)
        {
          if (i > 0)
          {
            path_length += sqrt(pow(ee_path_.poses[i].pose.position.x - ee_path_.poses[i-1].pose.position.x, 2) + pow(ee_path_.poses[i].pose.position.y - ee_path_.poses[i-1].pose.position.y, 2) + pow(ee_path_.poses[i].pose.position.z - ee_path_.poses[i-1].pose.position.z, 2));
          }
        }
        ee_path_.poses.clear();
        std::cout << "=====================================\n";
        std::cout << "path length: " << path_length << std::endl;
    }
    if (!goal_reached_)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = ee_pos(0);
        pose_stamped.pose.position.y = ee_pos(1);
        pose_stamped.pose.position.z = ee_pos(2);
        ee_path_.poses.push_back(pose_stamped);

    }
    std::cout << "problem finish " << std::endl;
    // prob = MobileManipulatorProblem_ptr_->MakeProblem();
    traj_ptr_ = std::make_shared<altro::Trajectory<NStates, NControls>>(MobileManipulatorProblem_ptr_->InitialTrajectory());
    solver_al_ptr_ = std::make_unique<altro::augmented_lagrangian::AugmentedLagrangianiLQR<6, 1>>(*probPtr_);
    // ROS_INFO_STREAM_NAMED(name_, "Finished controller initialization4");

    // solver_al_ptr_->SetTrajectory(std::make_shared<altro::Trajectory<6, 1>>(straj_ptr));
    solver_al_ptr_->SetPenalty(1);
    solver_al_ptr_->GetOptions().verbose = altro::LogLevel::kDebug;
    solver_al_ptr_->GetOptions().max_iterations_total = 100;
    solver_al_ptr_->GetOptions().max_iterations_outer = 4;
    solver_al_ptr_->GetOptions().max_iterations_inner = 15;
    solver_al_ptr_->GetOptions().line_search_max_iterations = 6;
    solver_al_ptr_->GetOptions().line_search_lower_bound = 1e-4;
    solver_al_ptr_->GetOptions().line_search_decrease_factor = 3;
    solver_al_ptr_->GetOptions().constraint_tolerance= 1e-3;
    solver_al_ptr_->GetOptions().cost_tolerance= 8e-4;
    // solver_al_ptr_->GetOptions().constraint_tolerance= 1e-3;
    
    solver_al_ptr_->GetOptions().nthreads = 1  ;
    solver_al_ptr_->GetOptions().initial_penalty = 1;
    solver_al_ptr_->GetOptions().profiler_enable = true;
    solver_al_ptr_->GetOptions().profiler_output_to_file = true;
    // solver_al_ptr_->GetOptions().log_directory = "~/PF/ALQP";


    //////////////////////////////////////////////////////////////////////////////////////


   
    traj_ptr_->State(0) << BaseM_TR(0,0) , BaseM_TR(0,1) , BaseM_TR(0,2) , 0 , 0 , BaseM_TR(0,5);
    traj_ptr_->State(1) << BaseM_TR(0,0) , BaseM_TR(0,1) , BaseM_TR(0,2) , 0 , 0 , BaseM_TR(0,5);
    traj_ptr_->State(2) << BaseM_TR(0,0) , BaseM_TR(0,1) , BaseM_TR(0,2) , 0 , 0 , BaseM_TR(0,5);
    traj_ptr_->State(3) << BaseM_TR(0,0) , BaseM_TR(0,1) , BaseM_TR(0,2) , 0 , 0 , BaseM_TR(0,5);
  

    auto ShoulderM_R = rotationMatrixToRPY(worldToShoulderLinkTransf.rotation()).transpose();auto ShoulderM_T = worldToShoulderLinkTransf.translation().transpose();
    Eigen::Matrix<double,1,6> ShoulderM_TR;ShoulderM_TR << ShoulderM_T, ShoulderM_R; traj_ptr_->State(4) = ShoulderM_TR;

    auto BicepM_R = rotationMatrixToRPY(Eigen::Matrix<double,3,3>(BicepM[0])).transpose();auto BicepM_T = BicepM[1].transpose();
    Eigen::Matrix<double,1,6> BicepM_TR;BicepM_TR << BicepM_T, BicepM_R; traj_ptr_->State(5) = BicepM_TR;

    auto ForearmM_R = rotationMatrixToRPY(Eigen::Matrix<double,3,3>(ForearmM[0])).transpose();auto ForearmM_T = ForearmM[1].transpose();
    Eigen::Matrix<double,1,6> ForearmM_TR;ForearmM_TR << ForearmM_T, ForearmM_R; traj_ptr_->State(6) = ForearmM_TR;

    auto Wrist_1M_R = rotationMatrixToRPY(Eigen::Matrix<double,3,3>(Wrist_1M[0])).transpose();auto Wrist_1M_T = Wrist_1M[1].transpose();
    Eigen::Matrix<double,1,6> Wrist_1M_TR;Wrist_1M_TR << Wrist_1M_T, Wrist_1M_R; traj_ptr_->State(7) = Wrist_1M_TR;

    auto Wrist_2M_R = rotationMatrixToRPY(Eigen::Matrix<double,3,3>(Wrist_2M[0])).transpose();auto Wrist_2M_T = Wrist_2M[1].transpose();
    Eigen::Matrix<double,1,6> Wrist_2M_TR;Wrist_2M_TR << Wrist_2M_T, Wrist_2M_R; traj_ptr_->State(8) = Wrist_2M_TR;

    auto EndEffectorM_R = rotationMatrixToRPY(Eigen::Matrix<double,3,3>(EndEffectorM[0])).transpose();auto EndEffectorM_T = EndEffectorM[1].transpose();
    Eigen::Matrix<double,1,6> EndEffectorM_TR;EndEffectorM_TR << EndEffectorM_T, EndEffectorM_R; traj_ptr_->State(9) = EndEffectorM_TR;

    std::cout << "currentPosition: " << EndEffectorM_TR.transpose() << std::endl;
    probPtr_->SetInitialState(BaseM_TR.transpose());
    solver_al_ptr_->GetiLQRSolver().GetKnotPointFunction(3).GetModelPtr()->SetInitTransform(xarmToShoulderLinkTransf);
    solver_al_ptr_->GetiLQRSolver().GetKnotPointFunction(4).GetModelPtr()->SetInitTransform(shoulderLinkToBicepLinkTransf);
    solver_al_ptr_->GetiLQRSolver().GetKnotPointFunction(5).GetModelPtr()->SetInitTransform(bicepToForearmLinkTransf);
    solver_al_ptr_->GetiLQRSolver().GetKnotPointFunction(6).GetModelPtr()->SetInitTransform(forearmToWrist_1LinkTransf);
    solver_al_ptr_->GetiLQRSolver().GetKnotPointFunction(7).GetModelPtr()->SetInitTransform(wrist_1LinkToWrist_2LinkTransf);
    solver_al_ptr_->GetiLQRSolver().GetKnotPointFunction(8).GetModelPtr()->SetInitTransform(wrist_2LinkToEndEffectorTransf);
    // solver_al_ptr_->GetiLQRSolver().GetKnotPointFunction(9).GetModelPtr()->SetInitTransform(xarmToShoulderLinkTransf);
    

 


      
      //freeregion generation

// Vec2f start_pt( BaseM_TR(0,0) , BaseM_TR(0,1));
//       Vec2f goal(disiredPosition[0], disiredPosition[1]);
//       Vec2f base_goal;
//       auto jps_path = model_size_.decomp_util_.jps_planner(start_pt,goal);
//       base_goal = model_size_.decomp_util_.get_goal_from_path(jps_path, start_pt, 0.3, 0.3);
      
      
      // int numk = solver_al_ptr_->NumSegments();
      // std::cout << "numk: " << numk << std::endl;
      // for (int i =0; i<= numk; ++i)
      // {
      //   auto cost = std::dynamic_pointer_cast<altro::costs::sumCost>(solver_al_ptr_->GetALCost(i)->GetCostFunction());
      //   for (int j = 0; j < cost->GetCosts().size(); ++j)
      //   {
      //     auto sdp_cost = cost->GetCosts()[j];
      //     if (sdp_cost->GetLabel() == "SDP cost")
      //     {
      //       auto sdp_cost_ = std::dynamic_pointer_cast<altro::costs::SDPRelaxedBarrierPenalty>(sdp_cost);
      //       std::cout << "SDP cost" <<i<< std::endl;
      //       if (i<=3)
      //       {
      //         sdp_cost_->F_ = LinearConstraints_[0].A_;
      //         sdp_cost_->g_ = LinearConstraints_[0].b_;
      //         sdp_cost_->c_ = freeregionCenters[0];
      //       }
      //       else
      //       {
      //         sdp_cost_->F_ = LinearConstraints_[i-2].A_;
      //         sdp_cost_->g_ = LinearConstraints_[i-2].b_;
      //         sdp_cost_->c_ = freeregionCenters[i-2];
      //       }
      //     }
      //   }
      // }
      // for (int i = 0; i <= numk; ++i)
      // {
      //   int ineq_cnt = solver_al_ptr_->GetALCost(i)->GetInequalityConstraints().size();
      //   for (int j = 0; j < ineq_cnt; ++j)
      //   {
      //     auto sdp_ineqconstraint = solver_al_ptr_->GetALCost(i)->GetInequalityConstraints()[j]->GetConstraint();
      //     if (sdp_ineqconstraint->GetLabel()== "SDP Constraint")//SDP_BF SDP Constraint
      //     {
      //       auto sdp_constraint = std::dynamic_pointer_cast<altro::examples::SDPConstraint>(solver_al_ptr_->GetALCost(i)->GetInequalityConstraints()[j]->GetConstraint());
      //       if (i<=3)
      //       {
      //         sdp_constraint->F_ = LinearConstraints_[0].A_;
      //         sdp_constraint->g_ = LinearConstraints_[0].b_;
      //         sdp_constraint->c_ = freeregionCenters[0];
      //       }
      //       else
      //       {
      //         sdp_constraint->F_ = LinearConstraints_[i-2].A_;
      //         sdp_constraint->g_ = LinearConstraints_[i-2].b_;
      //         sdp_constraint->c_ = freeregionCenters[i-2];
      //       }
      //     }
      //   }
      // }

    for (int k = 0; k < 9; ++k) {
     traj_ptr_->Control(k)[0] =  uoutput_[k];
      // std::cout<<"traj_ptr_: "<<traj_ptr_->State(k).transpose()<<std::endl;
    }
    solver_al_ptr_->SetTrajectory(traj_ptr_);
// std::cout<<"Before solve problem"<<std::endl;
    solver_al_ptr_->Solve();
    
    if (solver_al_ptr_->GetStatus() == altro::SolverStatus::kSolved)
    {
      std::cout << "kSolved!!!!!!!!!!!!!!!!!!!" << std::endl;
      uoutput_ = vector_t::Zero(9);
      for (int k = 0; k < 9; ++k) {
          uoutput_[k] = traj_ptr_->Control(k)[0];
      }
      std::cout<<"uoutput: "<<uoutput_.transpose()<<std::endl;
    }
    else if (solver_al_ptr_->GetStatus() == altro::SolverStatus::kMaxPenalty)
    {
      std::cout << "kMaxPenalty!!!!!!!!!!!!!!!!!!!" << std::endl;
      uoutput_ = vector_t::Zero(9);
      for (int k = 0; k < 9; ++k) {
          uoutput_[k] = traj_ptr_->Control(k)[0];
      }
      std::cout<<"uoutput: "<<uoutput_.transpose()<<std::endl;
    }

    else if (solver_al_ptr_->GetStatus() == altro::SolverStatus::kMaxInnerIterations)
    {
      std::cout << "kMaxInnerIterations!!!!!!!!!!!!!!!!!!!" << std::endl;
      uoutput_ = vector_t::Zero(9);
      for (int k = 0; k < 9; ++k) {
          uoutput_[k] = traj_ptr_->Control(k)[0];
      }
      std::cout<<"uoutput: "<<uoutput_.transpose()<<std::endl;
    }
    else if (solver_al_ptr_->GetStatus() == altro::SolverStatus::kMaxOuterIterations)
    {
      std::cout << "kMaxOuterIterations!!!!!!!!!!!!!!!!!!!" << std::endl;
      uoutput_ = vector_t::Zero(9);
      for (int k = 0; k < 9; ++k) {
          uoutput_[k] = traj_ptr_->Control(k)[0];
      }
      std::cout<<"uoutput: "<<uoutput_.transpose()<<std::endl;
    }
    else if (solver_al_ptr_->GetStatus() == altro::SolverStatus::kMaxIterations)
    {
      std::cout << "kMaxIterations!!!!!!!!!!!!!!!!!!!" << std::endl;
      uoutput_ = vector_t::Zero(9);
      for (int k = 0; k < 9; ++k) {
          uoutput_[k] = traj_ptr_->Control(k)[0];
      }
      std::cout<<"uoutput: "<<uoutput_.transpose()<<std::endl;
    }
    else
    {
      std::cout << "Solver did not converge: " << std::endl;
      uoutput_ = vector_t::Zero(9);
      for (int k = 0; k < 9; ++k) {
          uoutput_[k] = traj_ptr_->Control(k)[0];
      }

    }
    std::cout<<"joiint angle:  "<<MobileManipulatorProblem_ptr_->state<<std::endl;


    // if ((solver_al_ptr_->GetStatus() != altro::SolverStatus::kSolved) || \
    //     (solver_al_ptr_->GetStatus() != altro::SolverStatus::kMaxPenalty)|| \
    //     (solver_al_ptr_->GetStatus() != altro::SolverStatus::kMaxOuterIterations)||\
    //     (solver_al_ptr_->GetStatus() != altro::SolverStatus::kMaxIterations))
    // {
    //   std::cout << "Solver did not converge: " << std::endl;
    //   uoutput_ = vector_t::Zero(9);
    // }
    // else
    // {
    //   uoutput_ = vector_t::Zero(9);
    //   for (int k = 0; k < 9; ++k) {
    //       uoutput_[k] = traj_ptr_->Control(k)[0];
    //   }
    //   std::cout<<"uoutput: "<<uoutput_.transpose()<<std::endl;
    // }

    vector_t wheelVels = vector_t::Zero(8);
    vector_t baseVel = vector_t::Zero(3);
    RobotInputs tempInputs;
    baseVel<< uoutput_[2], uoutput_[0], uoutput_[1];
  // std::cout<<"baseVel: "<<baseVel.transpose()<<std::endl;
    qp_solverPtr_->Set_currentState(currState_);
    wheelVels = qp_solverPtr_->baseKinemics(baseVel);
    std::cout<<"wheelVels: "<<wheelVels.transpose()<<std::endl;
    tempInputs.steersInput = wheelVels.tail(4);
    tempInputs.wheelsInput = wheelVels.head(4);
    tempInputs.armInputs = uoutput_.segment<6>(3);
    tempInputs.gripperInput = 1;
    tempInputs.stamp = ros::Time::now();
    ////////////////////////////////////////////////////////
    ///////////////////Control Input/////////////////////
    robotInputs_.writeFromNonRT(tempInputs);
    /////////////////////////////////////////////////////
    /////////////////////////////////////////////////////



    auto optstate = solver_al_ptr_->GetiLQRSolver().GetTrajectory();
    auto finish = std::chrono::high_resolution_clock::now();
    //print time in ms
    std::chrono::duration<double> elapsed = finish - start;
    //change the cnt into ms
    std::cout << "Time: " << elapsed.count() << " s" << std::endl;
  }
}
void Swerve_qp_RosControl::QPcomputeThread()
{
  while (true)
  {
    rosReferenceManagerPtr_->preSolverRun(0,0,currObservation_.state);
    const auto& targetTrajectories = rosReferenceManagerPtr_->getTargetTrajectories();
    vector_t disiredState;
    if (targetTrajectories.empty()) {
      std::cerr << "[Swerve_qp_RosControl] TargetTrajectories is empty!"<< std::endl;
      disiredState = vector_t::Zero(7);
      disiredState[3] = 1;
    } else {
      disiredState= targetTrajectories.getDesiredState(currObservation_.time);
    }
    auto disiredPosition = disiredState.segment<3>(0);
    quaternion_t disiredOrientation(disiredState.segment<4>(3));
 
    vector_t dq = vector_t::Zero(9);
    dq = qp_solverPtr_->get_solution(currObservation_.state, disiredPosition, disiredOrientation);
    vector_t wheelVels = vector_t::Zero(8);
    vector_t baseVel = vector_t::Zero(3);
    RobotInputs tempInputs;
    std::cout<<"keyCmdMod: "<<keyCmdMod_<<std::endl;
    if(!keyCmdMod_){
      baseVel<< dq[0], dq[1], dq[2];
      wheelVels = qp_solverPtr_->baseKinemics(baseVel);
      tempInputs.steersInput = wheelVels.tail(4);
      tempInputs.wheelsInput = wheelVels.head(4);
      tempInputs.armInputs = dq.segment<6>(3);
      tempInputs.gripperInput = 1;
      std::cout<<"gripper open: "<<std::endl;
    }
    else{
      baseVel<< keyCmdVel_[0], keyCmdVel_[1], keyCmdVel_[2];
      wheelVels = qp_solverPtr_->baseKinemics(baseVel);
      tempInputs.steersInput = wheelVels.tail(4);
      tempInputs.wheelsInput = wheelVels.head(4);
      tempInputs.armInputs = vector_t::Zero(6);
      tempInputs.gripperInput = -1;
      std::cout<<"gripper close: "<<std::endl;
    }
    robotInputs_.writeFromNonRT(tempInputs);

    model_size_.publishmarker(*pinocchioInterfacePtr_);
    
  }
}

void Swerve_qp_RosControl::odomCallback(const nav_msgs::OdometryPtr& msg) {
  std::lock_guard<std::mutex> lockGuard(updateOdomMutex_);
  currentObservedPose_.orientation = msg->pose.pose.orientation;
  currentObservedPose_.position.x = msg->pose.pose.position.x;
  currentObservedPose_.position.y = msg->pose.pose.position.y;
}

void Swerve_qp_RosControl::cmdVelCallback(const geometry_msgs::TwistPtr& msg) {
  keyCmdVel_ << msg->angular.z, msg->linear.x, msg->linear.y ;
}

void Swerve_qp_RosControl::keyboardCallback(const std_msgs::Bool::ConstPtr& msg ) {
  keyCmdMod_ = msg->data;
  vector_t initTarget(7);
  try {
    listener_->lookupTransform("odom", swerveModelInfo_.eeFrame, ros::Time(0), ee_transform_);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  initTarget.head(3) << ee_transform_.getOrigin().x(), ee_transform_.getOrigin().y(), ee_transform_.getOrigin().z();
  initTarget.tail(4).head(4) << ee_transform_.getRotation().x(), ee_transform_.getRotation().y(), ee_transform_.getRotation().z(),
      ee_transform_.getRotation().w();

  const vector_t zeroInput = vector_t::Zero(swerveModelInfo_.inputDim);
  TargetTrajectories initTargetTrajectories({observation_.time}, {initTarget}, {zeroInput});
  rosReferenceManagerPtr_->setTargetTrajectories(std::move(initTargetTrajectories));
}

void Swerve_qp_RosControl::model_size::publishmarker(PinocchioInterface& pinocchioInterface) {
  const pinocchio::Model& model = pinocchioInterface.getModel();
  pinocchio::Data data = pinocchio::Data(pinocchioInterface.getData());

  std::vector<std::string> link_names = {
       "base_link", "x_arm_link", "shoulder_link", "bicep_link", "forearm_link", 
       "spherical_wrist_1_link", "spherical_wrist_2_link","bracelet_link", "end_effector_link"
  };

  std::vector<int> link_indices;
  std::vector<pinocchio::SE3Tpl<scalar_t, 0>> link_transforms;

  for (const auto& name : link_names) {
    link_indices.push_back(model.getFrameId(name));
    link_transforms.push_back(data.oMf[model.getFrameId(name)]);
  }

  // TEST
  const pinocchio::Model& model_test = pinocchioInterface.getModel();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_test.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_test.nv);
  pinocchio::Data data_test = pinocchio::Data(model_test);
  pinocchio::forwardKinematics(model_test, data_test, q);
  pinocchio::updateFramePlacements(model_test, data_test);

  std::vector<pinocchio::SE3Tpl<scalar_t, 0>> link_transforms_test;
  for (const auto& name : link_names) {
    link_transforms_test.push_back(data_test.oMf[model_test.getFrameId(name)]);
  }

  // Calculate transformation matrices
  std::vector<Eigen::Matrix4d> transformation_matrices;
  for (size_t i = 1; i < link_names.size(); ++i) {
    Eigen::Matrix4d transform = link_transforms_test[i-1].inverse().toHomogeneousMatrix() * link_transforms_test[i].toHomogeneousMatrix();
    transformation_matrices.push_back(transform);
  }

  // Publish markers
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time();
  marker.ns = "model_size";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  auto publish_marker = [&marker, this](const vector_t& transformed_point, const Eigen::Quaternion<double>& quat, const vector_t& link) {
    marker.pose.position.x = transformed_point[0];
    marker.pose.position.y = transformed_point[1];
    marker.pose.position.z = transformed_point[2];
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = link[0];
    marker.scale.y = link[1];
    marker.scale.z = link[2];
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_pub.publish(marker);
  };

  vector_t link_dimensions[] = { base_link, x_arm_link,  shoulder_link,bicep_link, forearm_link,spherical_wrist_1_link, spherical_wrist_2_link,bracelet_link };
  std::vector<int> dimension_indices = { 2, 2, -2, -1, 1, -2, 1, -2 };  // Specify the dimension to use for each link
  std::vector<int> link_inds = { 0, 1, 2, 3, 4, 5, 6, 7 };
  Eigen::MatrixXd A(6, 3);
  A << 1, 0, 0,
        -1, 0, 0,
        0, 1, 0,
        0, -1, 0,
        0, 0, 1,
        0, 0, -1;
  Eigen::VectorXd b(6);
  // LinearConstraints_.clear();
  for (size_t i = 0; i < 8; ++i) {
    Eigen::Vector3d point;
    int ind = link_inds[i];
    if (fabs(dimension_indices[i])==0) {
      point = Eigen::Vector3d(link_dimensions[i][0] / 2, 0, 0);
    } else if (fabs(dimension_indices[i])==1) {
      point = Eigen::Vector3d(0, link_dimensions[i][1] / 2, 0);
    } else {
      point = Eigen::Vector3d(0, 0, link_dimensions[i][2] / 2);
    }
    if (dimension_indices[i] < 0) {
      point = -point;
    }
    // b << point[0]+link_dimensions[i][0]/2, -point[0]+link_dimensions[i][0]/2, point[1]+link_dimensions[i][1]/2, -point[1]+link_dimensions[i][1]/2, point[2]+link_dimensions[i][2]/2, -point[2]+link_dimensions[i][2]/2;
    // LinearConstraints_.push_back(LinearConstraint3D(A,b));
    Eigen::Vector3d transformed_point = link_transforms[ind ].act(point);
    Eigen::Quaternion<double> quat(link_transforms[ind].rotation());
    publish_marker(transformed_point, quat, link_dimensions[i]);
    marker.id++;
  }

  // Path calculation
  vec_Vec3f path;
  Vec3f translation = link_transforms[1].translation();
  Vec3f base_translation = link_transforms[0].translation();

  base_translation[2] = translation[2];
  base_translation[0] = 2 * base_translation[0] - translation[0];
  base_translation[1] = 2 * base_translation[1] - translation[1];

  path.push_back(base_translation);
  for (size_t i = 1; i < link_transforms.size()-2; ++i) {
    translation = link_transforms[i].translation();
    path.push_back(translation);
  }
  translation = link_transforms[link_transforms.size()-1].translation();
  path.push_back(translation);
  ros::WallTime start_time = ros::WallTime::now();
  decomp_util_.decomp_and_publish(path, cloud_,40);
  ros::WallTime end_time = ros::WallTime::now();
  ROS_INFO_STREAM("Decomposition time: " << (end_time - start_time).toSec() << " s");
  // auto LinearConstraint = decomp_util_.getLinearConstraint(0);
}
PLUGINLIB_EXPORT_CLASS(swerve_qp_ros_control::Swerve_qp_RosControl, controller_interface::ControllerBase)

}  // namespace swerve_qp_ros_control
