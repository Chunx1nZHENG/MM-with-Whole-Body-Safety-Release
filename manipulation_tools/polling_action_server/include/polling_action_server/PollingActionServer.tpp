/*
 * PollingActionServer.tpp
 *
 *  Created on: Oct 31, 2019
 *      Author: Perry Franklin
 */
#include <ros/console.h>
#include <polling_action_server/PollingActionServer.hpp>

namespace polling_action_server {

template <class ActionSpec>
PollingActionServer<ActionSpec>::PollingActionServer(const ros::NodeHandle& nh, std::string actionNamespace,
                                                     ros::Duration feedbackUpdatePeriod)
    : nh_(nh),
      actionNamespace_(std::move(actionNamespace)),
      hasNewGoal_(false),
      feedbackUpdatePeriod_(std::move(feedbackUpdatePeriod)),
      serverState_(ServerState::SHUTDOWN) {}

template <class ActionSpec>
bool PollingActionServer<ActionSpec>::initialize() {
  switch (serverState_) {
    case ServerState::SHUTDOWN: {
      serverState_ = ServerState::IDLE;
      actionServer_ = std::make_unique<SimpleActionServerType>(
          nh_, actionNamespace_, boost::bind(&PollingActionServer<ActionSpec>::communicationThreadFunction, this, _1), false);
      actionServer_->start();
      break;
    }
    case ServerState::PAUSED: {
      serverState_ = ServerState::IDLE;
      break;
    }
    default: {
      ROS_WARN_STREAM("PollingActionServer with namespace '" << actionNamespace_
                                                             << "' had initialize() called, but is neither shutdown nor paused.");
    }
  }

  return true;
}

template <class ActionSpec>
bool PollingActionServer<ActionSpec>::pause() {
  bool successfullySetPaused = false;

  if (serverState_ > ServerState::PAUSED) {
    serverState_ = ServerState::PAUSED;
    successfullySetPaused = true;

  } else if (serverState_ < ServerState::PAUSED) {
    ROS_WARN_STREAM("Pause was requested, but the server is already in a higher priority state (" << serverState_ << ")");
    successfullySetPaused = false;
  }
  return successfullySetPaused;
}

template <class ActionSpec>
bool PollingActionServer<ActionSpec>::shutdown() {
  bool successfullySetShutdown = false;

  if (serverState_ == ServerState::IDLE) {
    actionServer_->shutdown();
    serverState_ = ServerState::SHUTDOWN;
    successfullySetShutdown = true;

  } else if (serverState_ > ServerState::SHUTDOWN) {
    serverState_ = ServerState::SHUTDOWN;
    successfullySetShutdown = true;

  } else if (serverState_ < ServerState::SHUTDOWN) {
    ROS_WARN_STREAM("Shutdown was requested, but the server is already in a higher priority state (" << serverState_ << ")");
    successfullySetShutdown = false;
  }
  return successfullySetShutdown;
}

template <class ActionSpec>
bool PollingActionServer<ActionSpec>::hasNewGoal() {
  std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
  return hasNewGoal_;
}

template <class ActionSpec>
bool PollingActionServer<ActionSpec>::getGoal(Goal& goal) {
  std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);

  bool isThisANewGoal = hasNewGoal_;
  hasNewGoal_ = false;

  goal = currentGoal_;

  return isThisANewGoal;
}

template <class ActionSpec>
bool PollingActionServer<ActionSpec>::getGoalSilently(Goal& goal) const {
  std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
  goal = currentGoal_;

  return hasNewGoal_;
}

template <class ActionSpec>
bool PollingActionServer<ActionSpec>::getPreemptRequested() const {
  std::lock_guard<std::mutex> lockServer(actionServerMutex_);

  // The SimpleActionServer will stay preempted until a new goal is received. Hence, we will only return true if a preempt is requested AND
  // we are active
  return (serverState_ == ServerState::ACTIVE && actionServer_->isPreemptRequested());
}

template <class ActionSpec>
void PollingActionServer<ActionSpec>::setFeedback(const Feedback& feedback) {
  std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
  std::lock_guard<std::mutex> lockServer(actionServerMutex_);

  if (serverState_ != ServerState::ACTIVE) {
    ROS_WARN_STREAM_THROTTLE(5.0, "You are setting feedback in action server "
                                      << actionNamespace_ << " but the ServerState is currently not active (WARNING throttled to 5 hz");
  }

  currentFeedback_ = feedback;
}

template <class ActionSpec>
void PollingActionServer<ActionSpec>::setSucceeded(const Result& result) {
  if (serverState_ == ServerState::IDLE) {
    ROS_WARN_STREAM("Tried to set Succeeded, but the server is not active (there is no goal to succeed)");

  } else if (serverState_ > ServerState::SUCCEEDED) {
    std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
    currentResult_ = result;
    terminationMessage_ = "";
    serverState_ = ServerState::SUCCEEDED;

  } else if (serverState_ < ServerState::SUCCEEDED) {
    ROS_WARN_STREAM("Tried to set Succeeded, but the server is already in a higher priority state (" << serverState_ << ")");
  }
}

template <class ActionSpec>
void PollingActionServer<ActionSpec>::setPreempted(const std::string& reason, const Result& result) {
  if (serverState_ == ServerState::IDLE) {
    ROS_WARN_STREAM("Tried to set Preempted, but server is not active (there is no goal to preempt)");

  } else if (serverState_ > ServerState::PREEMPTED) {
    std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
    currentResult_ = result;
    terminationMessage_ = reason;
    serverState_ = ServerState::PREEMPTED;

  } else if (serverState_ < ServerState::PREEMPTED) {
    ROS_WARN_STREAM("Tried to set Preempted, but the server is already in a higher priority state (" << serverState_ << ")");
  }
}

template <class ActionSpec>
void PollingActionServer<ActionSpec>::setAborted(const std::string& reason, const Result& result) {
  if (serverState_ == ServerState::IDLE) {
    ROS_WARN_STREAM("Tried to set Aborted, but server is not active (there is no goal to abort)");

  } else if (serverState_ > ServerState::ABORTED) {
    std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
    currentResult_ = result;
    terminationMessage_ = reason;
    serverState_ = ServerState::ABORTED;

  } else if (serverState_ < ServerState::ABORTED) {
    ROS_WARN_STREAM("Tried to set Aborted, but the server is already in a higher priority state (" << serverState_ << ")");
  }
}

template <class ActionSpec>
bool PollingActionServer<ActionSpec>::getActive() const {
  return serverState_ == ServerState::ACTIVE;
}

template <class ActionSpec>
void PollingActionServer<ActionSpec>::communicationThreadFunction(const GoalConstPtr& newGoal) {
  // Consists of three steps:
  // Preprocessing -> check if we are going to reject this message or not (with SimpleActionServer we can only abort though)
  //   If the state is SHUTDOWN, this probably means we somehow got a message just before the actionserver finished cleaning up.
  //   We reject the message
  // Communication Loop -> continually publishes feedback and checks for termination
  //   This should only publish feedback or check for termination, nothing else!
  // Postprocessing -> Checks how the loop terminated and cleans up appropriately
  //   In most cases, just publishes the appropriate result and switches to IDLE
  //   For paused, publishes the appropriate result and switches to PAUSED
  //   For shutdown, breaks

  // Preproccesing step: If we are Paused or Shutdown, we need to reject this goal
  if (serverState_ == ServerState::PAUSED) {
    std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
    actionServer_->setAborted(Result(), "The server is currently paused (we are rejecting this goal)");
    return;
  }
  if (serverState_ == ServerState::SHUTDOWN) {
    actionServer_->setAborted(Result(), "The server is currently shutting down (we are rejecting this goal)");
    return;
  }

  // Prepare the goal and status
  {
    std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
    currentGoal_ = *newGoal;
    hasNewGoal_ = true;
    serverState_ = ServerState::ACTIVE;
  }

  // Communication Loop: Continually provide feedback until we are terminated

  ros::Rate feedbackRate(feedbackUpdatePeriod_);

  bool terminate = false;
  while (!terminate) {
    // Check termination conditions
    if (!ros::ok()) {
      ROS_ERROR_STREAM("ROS has died, apparently (action server " << actionNamespace_ << ")");
      serverState_ = ServerState::SHUTDOWN;
      break;
    }

    switch (serverState_) {
      case ServerState::SHUTDOWN:
      case ServerState::PAUSED:
      case ServerState::ABORTED:
      case ServerState::SUCCEEDED:
      case ServerState::PREEMPTED:
        terminate = true;
        break;
      case ServerState::IDLE:
        ROS_ERROR_STREAM("Action server " << actionNamespace_ << " is running, but has state IDLE; this is a bug");
        break;
      default:
        break;
    }

    // Publish Feedback
    {
      std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
      std::lock_guard<std::mutex> lockServer(actionServerMutex_);
      actionServer_->publishFeedback(currentFeedback_);
    }

    feedbackRate.sleep();
  }

  // Postproccessing: decide what to do with our result
  {
    std::lock_guard<std::mutex> lockMessages(actionMessagesMutex_);
    std::lock_guard<std::mutex> lockServer(actionServerMutex_);
    switch (serverState_) {
      case ServerState::SHUTDOWN:
        break;
      case ServerState::PAUSED:
        actionServer_->setAborted(Result(), "The action server was paused");
        serverState_ = ServerState::PAUSED;
        break;
      case ServerState::ABORTED:
        actionServer_->setAborted(currentResult_, terminationMessage_);
        serverState_ = ServerState::IDLE;
        break;
      case ServerState::SUCCEEDED:
        actionServer_->setSucceeded(currentResult_, "");
        serverState_ = ServerState::IDLE;
        break;
      case ServerState::PREEMPTED:
        actionServer_->setPreempted(currentResult_, terminationMessage_);
        serverState_ = ServerState::IDLE;
        break;
      default:
        ROS_ERROR_STREAM("Action server " << actionNamespace_ << " has finished a goal, but has state" << serverState_
                                          << "; this is a bug");
        actionServer_->setAborted(currentResult_, "The any action server appears to have a bug in it");
        serverState_ = ServerState::IDLE;
        break;
    }
  }
}

}  // namespace polling_action_server
