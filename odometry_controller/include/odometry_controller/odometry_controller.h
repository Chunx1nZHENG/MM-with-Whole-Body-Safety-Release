/*
 * class OdometryController
 *
 * This class subscribes the twist message and gives for swerve driving
 * the corresponding inputs to the controller manager.
 *
 */

#pragma once
#include <Eigen/Eigen>

#include <nav_msgs/Odometry.h>  //To publish and subscribe the robot odometry
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <diff_drive_controller/speed_limiter.h>
#include <hardware_interface/joint_state_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>
#include <memory>

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <urdf_parser/urdf_parser.h>
#include <cmath>

namespace odometry_controller {

/**
 * This class makes some assumptions on the model of the robot:
 *  - the rotation axes of wheels are collinear
 *  - the wheels are identical in radius
 * Additional assumptions to not duplicate information readily available in the URDF:
 *  - the wheels have the same parent frame
 *  - a wheel collision geometry is a cylinder in the urdf
 *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
 */
class OdometryController : public controller_interface::Controller<hardware_interface::JointStateInterface> {
 public:
  OdometryController();

  /**
   * \brief Initialize controller
   * \param hw            Velocity joint interface for the wheels
   * \param root_nh       Node handle at root namespace
   * \param controller_nh Node handle inside the controller namespace
   */
  bool init(hardware_interface::JointStateInterface* joint_state_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  /**
   * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
   * \param time   Current time
   * \param period Time since the last called to update
   */
  void update(const ros::Time& time, const ros::Duration& period);

 private:
  bool updateUsingVelocities(const Eigen::VectorXd& wheelSpeeds, const ros::Time& time);
  geometry_msgs::Twist getTwist(const Eigen::VectorXd& wheelSpeeds);

  std::string name_;

  ros::Duration publish_period_;
  ros::Time last_state_publish_time_;

  std::vector<hardware_interface::JointStateHandle> wheelJointHandles_;
  std::vector<hardware_interface::JointStateHandle> brakeJointHandles_;
  std::vector<std::string> wheelLinkNames_;
  std::vector<std::string> brakeLinkNames_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_twist_world_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
  ros::Subscriber lidar_odom_sub_;
  /// Wheel radius (assuming it's the same for all wheels):
  double wheel_radius_;

  /// Frame to use for the robot base:
  std::string base_frame_id_;

  /// Frame to use for odometry and odom tf:
  std::string odom_frame_id_;

  /// Whether to publish odometry to tf or not:
  bool enable_odom_tf_;
  bool lidar_global_odom_;

  /**
   * \brief Sets the odometry publishing fields
   * \param root_nh Root node handle
   * \param controller_nh Node handle inside the controller namespace
   */
  void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  /**
   * \brief Integrates the velocities (linear on x and y and angular)
   * \param linear_x  Linear  velocity along x of the robot frame  [m] (linear  displacement, i.e. m/s * dt) computed by encoders
   * \param linear_y  Linear  velocity along y of the robot frame   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
   * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
   */
  void integrateXY(double linear_x, double linear_y, double angular);

  void integrateVelocities(double linear_x, double linear_y, double angular, double dt);

  /// Current timestamp:
  ros::Time timestamp_;

  /// Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  /// Current velocity:
  double linear_x_;  //   [m/s]
  double linear_y_;  //   [m/s]
  double angular_;   // [rad/s]

  ros::Timer steeringPositionUpdateTimer_;
  void steeringPositionCallback(const ros::TimerEvent& event);

  std::mutex steeringPositionMutex_;
  struct WheelState {
    Eigen::Vector2d steeringPosition;
    Eigen::Vector2d wheelAxis;
    Eigen::Vector2d brakeToSteeringPositionBaseFrame;
  };
  std::vector<WheelState> wheelStates_;

  Eigen::Vector3d localPos_ ;
  geometry_msgs::Quaternion orientation_;
  
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace odometry_controller