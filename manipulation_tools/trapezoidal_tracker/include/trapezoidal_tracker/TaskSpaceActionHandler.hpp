/*
 * TaskSpaceActionHandler.hpp
 *
 *  Created on: 3 June 2021
 *      Author: Johannes Pankert
 */

#pragma once

#include <optional>

#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

#include <manipulation_msgs/ReachPoseAction.h>
#include <polling_action_server/PollingActionServer.hpp>

#include <Eigen/Core>

#include <trapezoidal_tracker/TrapezoidalVelocityProfile.hpp>

namespace trapezoidal_tracker {

struct TaskSpaceActionHandlerConfig {
  double maxLinearVelocity = 0.5;
  double maxLinearAcceleration = 1.0;
  double maxAngularVelocity = 1.0;
  double maxAngularAcceleration = 2.0;
  double linearGoalTolerance = 0.008;
  double angularGoalTolerance = 1.0 / 180 * M_PI;
  double goalWaitTime = 0.0;
  double currentPosePublishingRate = 200.0;
  std::string controlFrame = "arm_mount";
  std::string endEffectorFrame = "ENDEFFECTOR";
};

class TaskSpaceActionHandler {
 public:
  TaskSpaceActionHandler(ros::NodeHandle& nodeHandle, std::string actionNamespace,
                         TaskSpaceActionHandlerConfig config = TaskSpaceActionHandlerConfig());
  virtual ~TaskSpaceActionHandler() = default;

 private:
  using TaskSpaceActionServer = actionlib::SimpleActionServer<manipulation_msgs::ReachPoseAction>;
  using Goal = TaskSpaceActionServer::Goal;
  using Result = TaskSpaceActionServer::Result;
  using Feedback = TaskSpaceActionServer::Feedback;

  void loadConfig(ros::NodeHandle& nodeHandle);
  void goalCB();
  void preemptCB();
  void feedbackCB(const ros::TimerEvent& event);
  bool getEndEffectorPose(Eigen::Matrix4d& currentEndEffectorPose);
  void cancel(const std::string& reason);
  Result createResult();

  struct ProfileInformation {
    double startTime;
    double endTime;
    Eigen::Matrix4d goalPose_;
  };

  std::unique_ptr<ProfileInformation> profileInformation_;
  std::unique_ptr<TrapezoidalVelocityProfile> linearTrapezoidalProfile_;
  std::unique_ptr<TrapezoidalVelocityProfile> angularTrapezoidalProfile_;
  nav_msgs::Path currentPath_;
  TaskSpaceActionHandlerConfig config_;

  TaskSpaceActionServer actionServer_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  ros::Publisher pathPublisher_;
  ros::Publisher goalPosePublisher_;
  ros::Publisher currentPosePublisher_;
  ros::Timer feedbackTimer_;
};

}  // namespace trapezoidal_tracker
