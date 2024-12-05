/*
 * FibonacciComputer.cpp
 *
 *  Created on: 1 Nov 2019
 *      Author: Perry Franklin
 */

#include "FibonacciComputer.hpp"
#include "test_typedefs.hpp"

namespace polling_action_server_test {

FibonacciComputer::FibonacciComputer(const std::string& actionNamespace)
    : nh_(),
      actionServer_(nh_, actionNamespace, serverFeedbackPeriod),
      haveGoal_(false),
      lastValue_(0),
      currentValue_(0),
      currentIteration_(0),
      targetIteration_(0),
      failIteration_(0) {
  actionServer_.initialize();
}

void FibonacciComputer::advance() {
  if (haveGoal_) {
    FibonacciServer::Feedback feedback;
    feedback.currentIterate = currentIteration_;
    feedback.currentNumber = currentValue_;
    actionServer_.setFeedback(feedback);

    // Check for preempt
    if (actionServer_.getPreemptRequested()) {
      ROS_INFO_STREAM("Preempt was requested");
      FibonacciServer::Result result;
      result.finalNumber = currentValue_;
      haveGoal_ = false;
      FibonacciServer::Feedback feedback;
      feedback.currentIterate = 0;
      feedback.currentNumber = 0;
      actionServer_.setFeedback(feedback);
      actionServer_.setPreempted("preempted", result);

      // Check if we are supposed to fail
    } else if (failIteration_ == currentIteration_) {
      ROS_INFO_STREAM("Failure iteration reached");
      haveGoal_ = false;
      FibonacciServer::Result result;
      result.finalNumber = currentValue_;
      FibonacciServer::Feedback feedback;
      feedback.currentIterate = 0;
      feedback.currentNumber = 0;
      actionServer_.setFeedback(feedback);
      actionServer_.setAborted("failAt reached", result);

      // Else iterate to the next value
    } else {
      ++currentIteration_;
      const int nextValue = lastValue_ + currentValue_;
      lastValue_ = currentValue_;
      currentValue_ = nextValue;

      // Check if we've finished
      if (targetIteration_ == currentIteration_) {
        ROS_INFO_STREAM("Reached target Iteration " << targetIteration_);
        FibonacciServer::Result result;
        result.finalNumber = currentValue_;
        haveGoal_ = false;
        actionServer_.setSucceeded(result);
      }
    }
  } else {
    // Check for new goal
    FibonacciServer::Goal goal;
    if (actionServer_.getGoal(goal)) {
      currentIteration_ = 1;
      lastValue_ = 0;
      currentValue_ = 1;
      targetIteration_ = goal.countTo;
      failIteration_ = goal.failAt;
      haveGoal_ = true;
    }
  }
}

bool FibonacciComputer::haveGoal() {
  return haveGoal_;
}

}  // namespace polling_action_server_test
