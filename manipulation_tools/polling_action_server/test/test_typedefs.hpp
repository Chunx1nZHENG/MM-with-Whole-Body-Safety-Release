/*
 * test_typedefs.hpp
 *
 *  Created on: 1 Nov 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <polling_action_server/FibonacciPollingActionTestAction.h>
#include "polling_action_server/PollingActionServer.hpp"

namespace polling_action_server_test {

using FibonacciClient = actionlib::SimpleActionClient<polling_action_server::FibonacciPollingActionTestAction>;

using FibonacciServer = polling_action_server::PollingActionServer<polling_action_server::FibonacciPollingActionTestAction>;

const ros::Duration serverFeedbackPeriod(0.02);
constexpr double serverAdvanceRate(5);

}  // namespace polling_action_server_test
