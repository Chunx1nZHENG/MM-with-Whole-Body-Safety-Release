/*
 * FibonacciServerClientTest.cpp
 *
 *  Created on: 27 Mar 2020
 *      Author: Perry Franklin
 */

#include <gtest/gtest.h>

#include "FibonacciComputer.hpp"
#include "test_typedefs.hpp"

namespace polling_action_server_test {

// Correct Fibonacci Sequence, using F0 = 0, F1 = 1
const std::array<int, 10> fibonacciSequence = {0, 1, 1, 2, 3, 5, 8, 13, 21, 34};

class FibonacciClientHandler {
 public:
  FibonacciClientHandler() : client_("fibonacci_test"), currentIterate_(0) {
    while (!client_.isServerConnected() && ros::ok()) {
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }

    if (!client_.isServerConnected()) {
      throw std::runtime_error("Fibonacci server did not connect to namespace fibonacci_test");
    }

    {
      std::lock_guard<std::mutex> lock(feedbackArrayMutex_);
      feedbackArray_.fill(0);
    }
  }
  ~FibonacciClientHandler() = default;

  void fibonacciClientFeedback(const FibonacciServer::FeedbackConstPtr& feedback) {
    if (client_.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
      std::lock_guard<std::mutex> lock(feedbackArrayMutex_);
      feedbackArray_[feedback->currentIterate] = feedback->currentNumber;
      currentIterate_ = feedback->currentIterate;
    }
  }

  void sendGoal(const FibonacciServer::Goal& goal) {
    client_.sendGoal(goal, FibonacciClient::SimpleDoneCallback(), FibonacciClient::SimpleActiveCallback(),
                     boost::bind(&FibonacciClientHandler::fibonacciClientFeedback, this, _1));
  }

  void waitForResult() { client_.waitForResult(); }

  FibonacciServer::ResultConstPtr getResult() { return client_.getResult(); }

  void cancelGoal() { client_.cancelGoal(); }

  void checkFeedbackArrayUpTo(size_t upTo) {
    for (size_t i = 0; i < upTo; ++i) {
      std::lock_guard<std::mutex> lock(feedbackArrayMutex_);
      EXPECT_EQ(feedbackArray_[i], fibonacciSequence[i]) << "Sequence number " << i << " had the incorrect value";
    }
  }

  size_t getCurrentIterate() {
    std::lock_guard<std::mutex> lock(feedbackArrayMutex_);
    return currentIterate_;
  }

  actionlib::SimpleClientGoalState getState() { return client_.getState(); }

 protected:
  FibonacciClient client_;

 private:
  std::mutex feedbackArrayMutex_;
  size_t currentIterate_;
  std::array<int, 10> feedbackArray_;
};

TEST(FibonacciTests, Success) {
  FibonacciClientHandler client;

  const size_t resultAt = 9;

  FibonacciServer::Goal goal;
  goal.countTo = resultAt;
  goal.failAt = -1;

  client.sendGoal(goal);

  client.waitForResult();

  EXPECT_EQ(client.getResult()->finalNumber, fibonacciSequence[9]) << "The result was not correct";

  client.checkFeedbackArrayUpTo(resultAt - 1);

  EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
}

TEST(FibonacciTests, Fail) {
  FibonacciClientHandler client;

  const size_t failAt = 5;

  FibonacciServer::Goal goal;
  goal.countTo = 9;
  goal.failAt = failAt;

  client.sendGoal(goal);

  client.waitForResult();

  EXPECT_EQ(client.getResult()->finalNumber, fibonacciSequence[failAt]) << "The result was not correct";

  client.checkFeedbackArrayUpTo(failAt - 1);

  EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::ABORTED);
}

TEST(FibonacciTests, Preempt) {
  FibonacciClientHandler client;

  const size_t preemptIterate = 5;

  FibonacciServer::Goal goal;
  goal.countTo = 9;
  goal.failAt = -1;

  client.sendGoal(goal);

  // Wait until we are at iterate 5 to preempt
  while (client.getCurrentIterate() != preemptIterate) {
    ros::Duration(0.01).sleep();
  }

  client.cancelGoal();

  client.waitForResult();

  // Ends up advancing one more time after preempt is called
  EXPECT_EQ(client.getResult()->finalNumber, fibonacciSequence[preemptIterate + 1]) << "The result was not correct";

  client.checkFeedbackArrayUpTo(preemptIterate - 1);

  EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::PREEMPTED);
}

TEST(FibonacciTests, SecondClient) {
  FibonacciClientHandler client;
  FibonacciClientHandler client2;

  const size_t preemptIterate = 5;
  const size_t resultAt = 9;

  FibonacciServer::Goal goal;
  goal.countTo = resultAt;
  goal.failAt = -1;

  client.sendGoal(goal);

  // Wait until we are at iterate 5 to preempt
  while (client.getCurrentIterate() != preemptIterate) {
    ros::Duration(0.01).sleep();
  }

  client2.sendGoal(goal);

  client.waitForResult();
  client2.waitForResult();

  // Ends up advancing one more time after preempt is called
  EXPECT_EQ(client.getResult()->finalNumber, fibonacciSequence[preemptIterate + 1]) << "The result was not correct";
  EXPECT_EQ(client2.getResult()->finalNumber, fibonacciSequence[resultAt]) << "The result was not correct";

  client.checkFeedbackArrayUpTo(preemptIterate - 1);
  client2.checkFeedbackArrayUpTo(resultAt - 1);

  EXPECT_EQ(client.getState(), actionlib::SimpleClientGoalState::PREEMPTED);
  EXPECT_EQ(client2.getState(), actionlib::SimpleClientGoalState::SUCCEEDED);
}

}  // namespace polling_action_server_test
