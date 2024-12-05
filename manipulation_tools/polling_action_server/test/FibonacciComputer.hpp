/*
 * FibonacciComputer.hpp
 *
 *  Created on: 1 Nov 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <ros/ros.h>

#include "polling_action_server/PollingActionServer.hpp"
#include "test_typedefs.hpp"

namespace polling_action_server_test {

/**! Simulated polling algorithm for computing a fibonacci sequence
 *   This is designed to emulate our controllers, ie the "advance" function is called
 *   in a loop and the action server is handled in a polling fashion
 *
 *   The action server can accept a target iteration, where upon it will begin computing
 *   the fibonacci sequence up to that iteration.
 *   If a fail iteration is specified, then
 */
class FibonacciComputer {
 public:
  FibonacciComputer(const std::string& actionNamespace);
  ~FibonacciComputer() = default;

  void advance();

  bool haveGoal();

 private:
  ros::NodeHandle nh_;

  FibonacciServer actionServer_;

  bool haveGoal_;
  int lastValue_;
  int currentValue_;
  int currentIteration_;
  int targetIteration_;
  int failIteration_;
};

}  // namespace polling_action_server_test
