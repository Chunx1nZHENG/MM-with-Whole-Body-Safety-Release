/*
 * test_main.cpp
 *
 *  Created on: 27 Mar 2020
 *      Author: Perry Franklin
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

/// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "polling_action_server_test");
  ros::AsyncSpinner async(1);
  testing::InitGoogleTest(&argc, argv);
  async.start();
  return RUN_ALL_TESTS();
}
