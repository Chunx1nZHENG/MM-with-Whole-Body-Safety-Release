/*
 * TaskSpaceActionHandler.cpp
 *
 *  Created on: 2 Apr 2020
 *      Author: Perry Franklin
 */

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trapezoidal_tracker/TaskSpaceActionHandler.hpp>

namespace trapezoidal_tracker {

TaskSpaceActionHandler::TaskSpaceActionHandler(ros::NodeHandle& nodeHandle, std::string actionNamespace,
                                               TaskSpaceActionHandlerConfig config)
    : config_(std::move(config)), actionServer_(nodeHandle, std::move(actionNamespace), false), buffer_(), listener_(buffer_) {
  loadConfig(nodeHandle);

  actionServer_.registerGoalCallback(boost::bind(&TaskSpaceActionHandler::goalCB, this));
  actionServer_.registerPreemptCallback(boost::bind(&TaskSpaceActionHandler::preemptCB, this));

  feedbackTimer_ = nodeHandle.createTimer(ros::Duration(1.0 / config_.currentPosePublishingRate),
                                          boost::bind(&TaskSpaceActionHandler::feedbackCB, this, _1));

  pathPublisher_ = nodeHandle.advertise<nav_msgs::Path>("/mission_control/planned_path", 1, false);

  goalPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseStamped>("goal_pose", 1, false);
  currentPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseStamped>("current_pose", 1, false);

  actionServer_.start();
}

void TaskSpaceActionHandler::loadConfig(ros::NodeHandle& nodeHandle) {
  config_.maxLinearVelocity = nodeHandle.param<double>("max_linear_velocity", config_.maxLinearVelocity);
  config_.maxLinearAcceleration = nodeHandle.param<double>("max_linear_acceleration", config_.maxLinearAcceleration);
  config_.maxAngularVelocity = nodeHandle.param<double>("max_angular_velocity", config_.maxAngularVelocity);
  config_.maxAngularAcceleration = nodeHandle.param<double>("max_angular_acceleration", config_.maxAngularAcceleration);
  config_.linearGoalTolerance = nodeHandle.param<double>("linear_goal_tolerance", config_.linearGoalTolerance);
  config_.angularGoalTolerance = nodeHandle.param<double>("angular_goal_tolerance", config_.angularGoalTolerance);
  config_.controlFrame = nodeHandle.param<std::string>("control_frame", config_.controlFrame);
  config_.endEffectorFrame = nodeHandle.param<std::string>("end_effector_frame", config_.endEffectorFrame);
  config_.goalWaitTime = nodeHandle.param<double>("goal_wait_time", config_.goalWaitTime);
  config_.currentPosePublishingRate = nodeHandle.param<double>("current_pose_publishing_rate", config_.currentPosePublishingRate);
}

bool TaskSpaceActionHandler::getEndEffectorPose(Eigen::Matrix4d& currentEndEffectorPose) {
  try {
    auto eeTransform = buffer_.lookupTransform(config_.controlFrame, config_.endEffectorFrame, ros::Time(0));
    Eigen::Affine3d eePoseAffine;
    tf::transformMsgToEigen(eeTransform.transform, eePoseAffine);
    currentEndEffectorPose = eePoseAffine.matrix();

  } catch (const tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    cancel("Requesting the end effector pose failed.");
    return false;
  }
  return true;
}

void TaskSpaceActionHandler::goalCB() {
  if (auto goal = actionServer_.acceptNewGoal()) {
    auto goalPoseStamped = goal->poseStamped;
    ROS_INFO_STREAM("Goal is " << goalPoseStamped);
    auto goalFrameId = goalPoseStamped.header.frame_id;
    if (goalFrameId != config_.controlFrame) {
      try {
        goalPoseStamped.header.stamp = ros::Time(0);  // set timestamp to 0 to get latest transformation available
        goalPoseStamped = buffer_.transform(goalPoseStamped, config_.controlFrame);
        goalPosePublisher_.publish(goalPoseStamped);
      } catch (const tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        cancel("Transforming the target pose into the control frame failed.");
        return;
      }
    }
    Eigen::Matrix4d currentEndEffectorPose;
    if (!getEndEffectorPose(currentEndEffectorPose)) {
      return;
    }

    profileInformation_ = std::make_unique<ProfileInformation>();
    double endTime = 0;
    auto now = ros::Time::now();

    Eigen::Matrix4d goalHom;
    Eigen::Affine3d aff;
    tf::poseMsgToEigen(goalPoseStamped.pose, aff);
    goalHom = aff.matrix();
    profileInformation_->goalPose_ = goalHom;

    double linearDistance = (goalHom.topRightCorner<3, 1>() - currentEndEffectorPose.topRightCorner<3, 1>()).norm();
    if (linearDistance > 1e-8) {
      linearTrapezoidalProfile_ = std::make_unique<TrapezoidalVelocityProfile>(config_.maxLinearVelocity / linearDistance,
                                                                               config_.maxLinearAcceleration / linearDistance);
    } else {
      linearTrapezoidalProfile_ = nullptr;
    }
    Eigen::Matrix3d currentOrientation = currentEndEffectorPose.topLeftCorner<3, 3>();
    Eigen::Matrix3d goalOrientation = goalHom.topLeftCorner<3, 3>();
    auto disparityAngle = std::acos(((currentOrientation * goalOrientation.transpose()).trace() - 1) / 2);
    if (disparityAngle > 1e-8) {
      angularTrapezoidalProfile_ = std::make_unique<TrapezoidalVelocityProfile>(config_.maxAngularVelocity / disparityAngle,
                                                                                config_.maxAngularAcceleration / disparityAngle);
    } else {
      angularTrapezoidalProfile_ = nullptr;
    }

    if (linearTrapezoidalProfile_ == nullptr && angularTrapezoidalProfile_ == nullptr) {
      cancel("Goal is equal to current pose");
      return;
    } else if (linearTrapezoidalProfile_ == nullptr) {
      endTime = angularTrapezoidalProfile_->getEndTimePoint();
    } else if (angularTrapezoidalProfile_ == nullptr) {
      endTime = linearTrapezoidalProfile_->getEndTimePoint();
    } else if (linearTrapezoidalProfile_->getEndTimePoint() > angularTrapezoidalProfile_->getEndTimePoint()) {
      endTime = linearTrapezoidalProfile_->getEndTimePoint();
      double newAcceleration = disparityAngle * std::pow(2.0 / endTime, 2);
      double newVelocity = newAcceleration * endTime / 2;
      angularTrapezoidalProfile_ =
          std::make_unique<TrapezoidalVelocityProfile>(newVelocity / disparityAngle, newAcceleration / disparityAngle);
    } else {
      endTime = angularTrapezoidalProfile_->getEndTimePoint();
      double newAcceleration = linearDistance * std::pow(2.0 / endTime, 2);
      double newVelocity = newAcceleration * endTime / 2;
      linearTrapezoidalProfile_ =
          std::make_unique<TrapezoidalVelocityProfile>(newVelocity / linearDistance, newAcceleration / linearDistance);
    }

    nav_msgs::Path path;
    path.header.frame_id = config_.controlFrame;
    path.header.stamp = now;
    int numPoses = 50;
    for (int i = 0; i < numPoses; ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = config_.controlFrame;
      double t = (double)i / numPoses * endTime;
      pose.header.stamp = ros::Time(now.toSec() + t);
      pose.header.seq = i;

      if (linearTrapezoidalProfile_ != nullptr) {
        double s, ds, dds;
        linearTrapezoidalProfile_->getState(t, s, ds, dds);
        Eigen::Vector3d position = (1 - s) * currentEndEffectorPose.topRightCorner<3, 1>() + s * goalHom.topRightCorner<3, 1>();
        tf::pointEigenToMsg(position, pose.pose.position);
      } else {
        Eigen::Vector3d position = goalHom.topRightCorner<3, 1>();
        tf::pointEigenToMsg(position, pose.pose.position);
      }

      if (angularTrapezoidalProfile_ != nullptr) {
        double s, ds, dds;
        angularTrapezoidalProfile_->getState(t, s, ds, dds);
        Eigen::Quaterniond currentQuat(currentEndEffectorPose.topLeftCorner<3, 3>());
        Eigen::Quaterniond goalQuat(goalHom.topLeftCorner<3, 3>());
        Eigen::Quaterniond orientation = currentQuat.slerp(s, goalQuat);
        tf::quaternionEigenToMsg(orientation, pose.pose.orientation);
      } else {
        Eigen::Quaterniond goalQuat(goalHom.topLeftCorner<3, 3>());
        tf::quaternionEigenToMsg(goalQuat, pose.pose.orientation);
      }
      path.poses.push_back(std::move(pose));
    }
    profileInformation_->startTime = now.toSec();
    profileInformation_->endTime = profileInformation_->startTime + endTime + config_.goalWaitTime;
    currentPath_ = path;

    pathPublisher_.publish(path);
  }
}

void TaskSpaceActionHandler::preemptCB() {
  Eigen::Matrix4d pose;
  if (getEndEffectorPose(pose)) {
    nav_msgs::Path path;
    path.header.frame_id = config_.controlFrame;
    path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header = path.header;
    Eigen::Affine3d poseAff;
    poseAff.matrix() = pose;
    tf::poseEigenToMsg(poseAff, poseStamped.pose);
    path.poses.push_back(poseStamped);
    currentPath_ = path;
    currentPosePublisher_.publish(poseStamped);
    pathPublisher_.publish(path);
  }
  actionServer_.setPreempted();
}

void TaskSpaceActionHandler::cancel(const std::string& reason) {
  ROS_WARN_STREAM("Goal got canceled. Reason: " << reason);
  if (actionServer_.isActive()) {
    actionServer_.setAborted(createResult(), reason);
  }
  profileInformation_.reset();
}

void TaskSpaceActionHandler::feedbackCB(const ros::TimerEvent& event) {
  auto now = ros::Time::now().toSec();

  if (actionServer_.isActive()) {
    {
      geometry_msgs::PoseStamped currentPose;
      currentPose.header.stamp = ros::Time::now();

      double t = currentPose.header.stamp.toSec() - currentPath_.header.stamp.toSec();
      Eigen::Affine3d firstPose;
      tf::poseMsgToEigen(currentPath_.poses.front().pose, firstPose);
      Eigen::Affine3d lastPose;
      tf::poseMsgToEigen(currentPath_.poses.back().pose, lastPose);

      if (linearTrapezoidalProfile_ != nullptr) {
        double s, ds, dds;
        linearTrapezoidalProfile_->getState(t, s, ds, dds);
        Eigen::Vector3d position = (1 - s) * firstPose.translation() + s * lastPose.translation();
        tf::pointEigenToMsg(position, currentPose.pose.position);
      } else {
        Eigen::Vector3d position = lastPose.translation();
        tf::pointEigenToMsg(position, currentPose.pose.position);
      }

      if (angularTrapezoidalProfile_ != nullptr) {
        double s, ds, dds;
        angularTrapezoidalProfile_->getState(t, s, ds, dds);
        Eigen::Quaterniond firstRotQuat(firstPose.rotation());
        Eigen::Quaterniond latRotQuat(lastPose.rotation());
        Eigen::Quaterniond orientation = firstRotQuat.slerp(s, latRotQuat);
        tf::quaternionEigenToMsg(orientation, currentPose.pose.orientation);
      } else {
        Eigen::Quaterniond goalQuat(lastPose.rotation());
        tf::quaternionEigenToMsg(goalQuat, currentPose.pose.orientation);
      }
      currentPosePublisher_.publish(currentPose);
    }

    // Check for termination
    if (now >= profileInformation_->endTime) {
      actionServer_.setSucceeded(createResult());
      profileInformation_.reset();
      return;
    }

    Feedback feedback;
    feedback.timeRemaining = profileInformation_->endTime - now;
    feedback.percentComplete = (now - profileInformation_->startTime) / (profileInformation_->endTime - profileInformation_->startTime);
    actionServer_.publishFeedback(feedback);
  }
}

TaskSpaceActionHandler::Result TaskSpaceActionHandler::createResult() {
  Result result;
  result.goalReached = false;

  Eigen::Matrix4d currentPose;
  if (!getEndEffectorPose(currentPose)) {
    return result;
  }

  result.goalReached = true;
  auto linearDistance = (currentPose.topRightCorner<3, 1>() - profileInformation_->goalPose_.topRightCorner<3, 1>()).norm();

  if (linearDistance > config_.linearGoalTolerance) {
    ROS_ERROR_STREAM("Goal not reached. Linear distance is " << linearDistance << " but the tolerance is only "
                                                             << config_.linearGoalTolerance << ".");
    result.goalReached = false;
  }

  Eigen::Matrix3d currentOrientation = currentPose.topLeftCorner<3, 3>();
  Eigen::Matrix3d goalOrientation = profileInformation_->goalPose_.topLeftCorner<3, 3>();
  auto disparityAngle = std::acos(((currentOrientation * goalOrientation.transpose()).trace() - 1) / 2);

  if (disparityAngle > config_.angularGoalTolerance) {
    ROS_ERROR_STREAM("Goal not reached. Disparity angle is " << disparityAngle << " but the tolerance is only "
                                                             << config_.angularGoalTolerance << ".");
    result.goalReached = false;
  }

  if (result.goalReached) {
    ROS_INFO_STREAM("Goal reached. Linear distance: " << linearDistance << ". Disparity angle: " << disparityAngle);
  }

  return result;
}

}  // namespace trapezoidal_tracker
