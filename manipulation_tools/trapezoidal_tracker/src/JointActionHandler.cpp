/*
 * JointActionHandler.cpp
 *
 *  Created on: 2 Apr 2020
 *      Author: Perry Franklin
 */

#include <trapezoidal_tracker/JointActionHandler.hpp>

namespace trapezoidal_tracker {

JointActionHandler::JointActionHandler(const ros::NodeHandle& nodeHandle, std::string actionNamespace)
    : jointActionServer_(nodeHandle, std::move(actionNamespace)), initialized_(false) {
  jointActionServer_.initialize();
  setServerStopped();
}

void JointActionHandler::initialize(const Eigen::VectorXd& currentPosition, const Eigen::VectorXd& maxVelocity,
                                    const Eigen::VectorXd& maxAcceleration) {
  if (currentPosition.size() == maxVelocity.size() && currentPosition.size() == maxAcceleration.size()) {
    currentGoal_ = currentPosition;
    maxVelocity_ = maxVelocity;
    maxAcceleration_ = maxAcceleration;
    jointActionServer_.initialize();
    initialized_ = true;
  } else {
    ROS_ERROR_STREAM("The sizes of the provided elements don't make sense: "
                     << std::endl
                     << "size of currentPosition = " << currentPosition.size() << std::endl
                     << "size of maxVelocity = " << maxVelocity.size() << std::endl
                     << "size of maxAcceleration = " << maxAcceleration.size() << std::endl
                     << "Not initializing JointActionHandler.");
  }
}

void JointActionHandler::setServerStopped() {
  if (profileInformation_) {
    cancel("The JointActionHandler is being stopped");
  }
  jointActionServer_.pause();
  initialized_ = false;
}

void JointActionHandler::cancel(const std::string& reason) {
  profileInformation_.reset();
  if (jointActionServer_.getActive()) {
    jointActionServer_.setAborted(reason, createResult());
  }
}

void JointActionHandler::getDesiredJointState(const ros::Time& time, Eigen::VectorXd& positionOut, Eigen::VectorXd& velocityOut,
                                              Eigen::VectorXd& accelerationOut) {
  if (!initialized_) {
    ROS_ERROR_STREAM("The JointActionHandler was not initialized; do so first!");
    return;
  }

  if (profileInformation_) {
    // Check for termination
    const double elapsedTime = (time - profileInformation_->startTime).toSec();
    if (elapsedTime >= trapezoidalProfile_->getEndTimePoint()) {
      currentGoal_ = profileInformation_->finalPosition_;
      jointActionServer_.setSucceeded(createResult());
      profileInformation_.reset();
    }

    if (jointActionServer_.getPreemptRequested()) {
      profileInformation_.reset();
      jointActionServer_.setPreempted("Preempt requested", createResult());
    }
  } else {
    // check for new goal
    Goal goal;
    if (jointActionServer_.getGoal(goal)) {
      if (goal.position.size() != (size_t)maxVelocity_.size()) {
        std::stringstream message;
        message << "Got goal of size " << goal.position.size() << " but I'm expecting size " << maxVelocity_.size();
        ROS_ERROR_STREAM(message.str());
        jointActionServer_.setAborted(message.str(), createResult());
      } else {
        ROS_INFO_STREAM("Goal is " << goal);
        Eigen::VectorXd finalPosition(goal.position.size());
        for (size_t i = 0; i < goal.position.size(); ++i) {
          finalPosition[i] = goal.position[i];
        }

        profileInformation_ = std::make_unique<ProfileInformation>(currentGoal_, finalPosition, time);

        double minSVelocity = std::numeric_limits<double>::max();
        double minSAcceleration = std::numeric_limits<double>::max();
        for (int i = 0; i < maxVelocity_.size(); i++) {
          if (profileInformation_->difference_[i] != 0.0) {
            minSVelocity = std::min(minSVelocity, std::abs(maxVelocity_(i) / profileInformation_->difference_[i]));
            minSAcceleration = std::min(minSAcceleration, std::abs(maxAcceleration_(i) / profileInformation_->difference_[i]));
          }
        }
        trapezoidalProfile_ = std::make_unique<TrapezoidalVelocityProfile>(minSVelocity, minSAcceleration);
      }
    }
  }

  if (!profileInformation_) {
    positionOut = currentGoal_;
    velocityOut.setZero(positionOut.size());
    accelerationOut.setZero(positionOut.size());
  } else {
    if (!trapezoidalProfile_) {
      ROS_ERROR_STREAM("Something went wrong...");
      positionOut = currentGoal_;
      velocityOut.setZero();
      accelerationOut.setZero();
    }
    double s, ds, dds;
    const ros::Duration elapsedTime = time - profileInformation_->startTime;
    trapezoidalProfile_->getState(elapsedTime.toSec(), s, ds, dds);
    currentGoal_ = s * profileInformation_->difference_ + profileInformation_->startingPosition_;
    positionOut = currentGoal_;
    velocityOut = ds * profileInformation_->difference_;
    accelerationOut = dds * profileInformation_->difference_;

    Feedback feedback;
    feedback.percentComplete = s;
    feedback.timeRemaining = (time - elapsedTime).toSec();
    jointActionServer_.setFeedback(feedback);
  }
}

void JointActionHandler::getDesiredJointState(const ros::Time& time, const Eigen::VectorXd& positionCurr, Eigen::VectorXd& positionOut,
                                              Eigen::VectorXd& velocityOut, Eigen::VectorXd& accelerationOut) {
  if (!profileInformation_) {
    currentGoal_ = positionCurr;
  }
  getDesiredJointState(time, positionOut, velocityOut, accelerationOut);
}

void JointActionHandler::updateCurrentGoal(const Eigen::VectorXd& currentPosition) {
  currentGoal_ = currentPosition;
}

JointActionHandler::Result JointActionHandler::createResult() {
  Result result;
  for (int i = 0; i < currentGoal_.size(); ++i) {
    result.position.emplace_back(currentGoal_[i]);
  }
  return result;
}
}  // namespace trapezoidal_tracker
