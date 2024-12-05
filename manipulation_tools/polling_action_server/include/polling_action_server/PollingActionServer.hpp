/*
 * PollingActionServer.hpp
 *
 *  Created on: Oct 31, 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <mutex>

#include <actionlib/server/simple_action_server.h>

#include <polling_action_server/utils.hpp>

namespace polling_action_server {

/**\brief A wrapper class for creating and handling a simple action server
 * \note The primary benefits are that the multithreading and state logic of the action server is abstracted away
 *       Normally the "Simple"ActionServer requires users to define a function that executes in a multithreaded fashion,
 *       and may or may not contain the actual execution logic. In this implementation, the user simply checks this
 *       class for updates and updates the action server as to the current state of execution
 */
template <class ActionSpec>
class PollingActionServer {
 public:
  // Server states, in order of priority (lower numbers can override higher numbers)
  // SHUTDOWN : the server does not even exist (or will not exist after the communicationThread loops once)
  // PAUSED : the server is paused (exists but is not accepting new requests, will abort any currently running goals)
  // ABORTED : the server is active (communicationThreadFunction is running) but needs to abort
  // SUCCEEDED : the server is active (communicationThreadFunction is running) but has already succeeded
  // PREEMPTED : the server is active (communicationThreadFunction is running) but needs to be preempted
  // ACTIVE : the server is active (communicationThreadFunction is running)
  // IDLE : the server is waiting for a goal (communicationThreadFunction is not running, but the actionServer is active)

 public:
  ACTION_DEFINITION(ActionSpec)

  /**\brief Constructor
   * \param[in] nh                       : NodeHandle to ros
   * \param[in] actionNameSpace          : Action namespace
   * \param[in] feedbackUpdateFrequency  : frequency that feedback is updated (and that the result can be returned)
   */
  PollingActionServer(const ros::NodeHandle& nh, std::string actionNamespace, ros::Duration feedbackUpdatePeriod = ros::Duration(0.1));
  ~PollingActionServer() = default;

  /**\brief initializes all connections and handles creating the action server
   * \return success
   */
  bool initialize();

  /**\brief "pauses" the action server; aborts any current current actions
   * \note the action server is still visible to the ros system, however new goals are immediately rejected.
   * \note to unpause, call "initialize()"
   * \return success
   */
  bool pause();

  /**\brief shuts the action server down; aborts any current current actions
   * \return success
   */
  bool shutdown();

  /**\brief Gets whether a new goal has been received.
   * \return True if this is a "new" message, False if getGoal has already been called with this goal.
   */
  bool hasNewGoal();

  /**\brief Gets the current goal.
   * \param[out] goal : The current action goal
   * \return True if this is a "new" message, False if getGoal has already been called with this goal.
   */
  bool getGoal(Goal& goal);

  /**\brief Gets the current goal, but does not count as already getting the goal.
   * \param[out] goal : The current action goal
   * \return True if this is a "new" message, False if getGoal has already been called with this goal.
   */
  bool getGoalSilently(Goal& goal) const;

  /**\brief Gets whether the goal has a preempt requested (the action is still considered running until you call setPreempt!)
   * \return If the current goal has been preempted
   */
  bool getPreemptRequested() const;

  /**\brief Sets the current feedback (this is not published if there is no active goal)
   * \param[in] feedback: feedback for the action server to publish
   */
  void setFeedback(const Feedback& feedback);

  /**\brief Sets the current goal as Succeeded
   * \param result : the final result
   */
  void setSucceeded(const Result& result);

  /**\brief Sets the current goal as preempted (ie the action was stopped cleanly)
   * \param[in] reason: a message for why the action server was preempted
   * \param[in] result: the final result of the action
   */
  void setPreempted(const std::string& reason, const Result& result);

  /**\brief Sets the current goal as aborted (ie the action failed)
   * \param[in] reason: a message for why the action server was aborted
   * \param[in] result: the final result of the action
   */
  void setAborted(const std::string& reason, const Result& result);

  /**\brief Gets whether the server is active or not
   * \return Whether the server is active or not
   */
  bool getActive() const;

 private:
  ros::NodeHandle nh_;

  using SimpleActionServerType = actionlib::SimpleActionServer<ActionSpec>;

  // Handles the feedback and result publishing
  void communicationThreadFunction(const GoalConstPtr&);

  const std::string actionNamespace_;

  std::atomic<bool> hasNewGoal_;
  ros::Duration feedbackUpdatePeriod_;

  std::atomic<ServerState> serverState_;

  mutable std::mutex actionServerMutex_;
  std::unique_ptr<SimpleActionServerType> actionServer_;

  mutable std::mutex actionMessagesMutex_;

  Goal currentGoal_;
  Feedback currentFeedback_;
  Result currentResult_;
  std::string terminationMessage_;
};

}  // namespace polling_action_server

#include <polling_action_server/PollingActionServer.tpp>
