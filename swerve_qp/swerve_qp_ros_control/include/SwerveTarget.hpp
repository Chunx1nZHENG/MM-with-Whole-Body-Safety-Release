#pragma once
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <geometry_msgs/PoseStamped.h>
#include <manipulation_msgs/ReachPoseAction.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "SwerveModelInfo.hpp"
#include "swerve_msgs/BrakesService.h"
namespace ocs2 {
namespace swerve {

class SwerveTarget {
 public:
  SwerveTarget(ros::NodeHandle nh, SwerveModelInfo swerveModelInfo, std::string controlFrame = "odom");
  ~SwerveTarget() = default;

  void initDummyVisualizationTarget();
  void initSetGoalPose();
  bool getBrakeState(int index);
  void setBrakeState(int index, bool value);
  Eigen::Vector4d getDesiredBrakesPosition();
  void initInterectiveMarker();
  void setstate(int state) { state_ = state; };
  int getstate() { return state_; };


 private:
  void goalCallback(const geometry_msgs::PoseStampedPtr& msg);
  void missionPathCallback(const nav_msgs::PathPtr& msg);
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);
  void initBrakesService();
  bool brakesServiceCallback(swerve_msgs::BrakesService::Request& req, swerve_msgs::BrakesService::Response& res);

  visualization_msgs::InteractiveMarker createInteractiveMarker() const;
  void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  std::shared_ptr<actionlib::SimpleActionClient<manipulation_msgs::ReachPoseAction>> actionClient_;
  ros::Subscriber goal_sub;
  ros::NodeHandle nh_;

  // Brakes
  ros::Publisher brakesStatePub_;
  ros::ServiceServer brakesService_;
  vector_t oldbrakesAngles_;
  vector_t brakesAngles_;
  bool brakesState_[4];

  // Path variables
  ros::Subscriber missionPathSub_;
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;

  interactive_markers::MenuHandler menuHandler_;
  interactive_markers::InteractiveMarkerServer server_;

  // Dummy visualization
  bool dummyVisualization_;
  ros::Subscriber observationSubscriber_;
  std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;

  SwerveModelInfo swerveModelInfo_;

  std::string controlFrame_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  int state_;
};

}  // namespace swerve
}  // namespace ocs2
