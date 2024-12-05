/*
 * FibonacciNode.cpp
 *
 *  Created on: 30 Mar 2020
 *      Author: Perry Franklin
 */

#include <ros/ros.h>

#include "FibonacciComputer.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "fibonacci_node");

  polling_action_server_test::FibonacciComputer fibonacciComputer("fibonacci_test");

  ros::Rate r(polling_action_server_test::serverAdvanceRate);
  while (ros::ok()) {
    ros::spinOnce();
    fibonacciComputer.advance();
    r.sleep();
  }

  return 0;
}
