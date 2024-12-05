/*
 * JointActionHandler.hpp
 *
 *  Created on: 2 Apr 2020
 *      Author: Perry Franklin
 */

#pragma once


#include <manipulation_msgs/JointPositionAction.h>
#include <polling_action_server/PollingActionServer.hpp>

#include <Eigen/Core>

#include <trapezoidal_tracker/TrapezoidalVelocityProfile.hpp>

namespace trapezoidal_tracker {


/* Handles joint action requests and tracks trapezoidal velocity profiles in joint space.
 * Creates an action server that takes in a joint position goal, then moves the current joint goal
 * to that position while following velocity and acceleration constraints of each joint.
 * Strictly speaking, this just tracks a goal in a euclidian space while maintaining constraints
 * on the derivatives.
 */
class JointActionHandler {
public:

  /**\brief Constructor
   * \param[in] nodeHandle          : Nodehandle to create action server with
   * \param[in] actionNamespace     : Namespace of the action server (eg topics are .../actionNamespace/goal, etc)
   */
  JointActionHandler(const ros::NodeHandle& nodeHandle, std::string actionNamespace);
  virtual ~JointActionHandler() = default;

  /**\brief Initializes the action handler and makes the action server active
   * \param[in] nodeHandle          : Nodehandle to create action server with
   * \param[in] actionNamespace     : Namespace of the action server (eg topics are .../actionNamespace/goal, etc)
   */
  void initialize(const Eigen::VectorXd& currentPosition,
                  const Eigen::VectorXd& maxVelocity,
                  const Eigen::VectorXd& maxAcceleration);

  /**\brief Stops the server (puts the action server into paused, where topics are available but it
   *        not accepting new goals)
   */
  void setServerStopped();

  /** \brief Cancels any current goal of the action server.
   *  \param[in] reason     : the reason the goal was cancelled.
   */
  void cancel(const std::string& reason);

  /**\brief Ensures to start from the curr arm position
   * \param[in] time                : Current time
   * \param[in] positionCurr        : Current position of the joints
   * \param[out] positionOut        : position of the joints
   * \param[out] velocityOut        : velocity of the joints
   * \param[out] accelerationOut    : acceleratio nof the joints
   */
  void getDesiredJointState(const ros::Time& time,
                            const Eigen::VectorXd& positionCurr,
                            Eigen::VectorXd& positionOut,
                            Eigen::VectorXd& velocityOut,
                            Eigen::VectorXd& accelerationOut);

  /**\brief Gets the position, velocity, and acceleration of s at a given timePoint
   * \param[in] time                : Current time
   * \param[out] positionOut        : position of the joints
   * \param[out] velocityOut        : velocity of the joints
   * \param[out] accelerationOut    : acceleratio nof the joints
   */
  void getDesiredJointState(const ros::Time& time,
                            Eigen::VectorXd& positionOut,
                            Eigen::VectorXd& velocityOut,
                            Eigen::VectorXd& accelerationOut);

  /**\brief Updates the current goal of the controller
   * \note Use this when initializing to prevent jumps, or when controller moves the desired position
   *       Any new action requests will start from this position
   */
  void updateCurrentGoal(const Eigen::VectorXd& currentPosition);

private:

  using JointActionServer = polling_action_server::PollingActionServer<manipulation_msgs::JointPositionAction>;
  using Goal = JointActionServer::Goal;
  using Result = JointActionServer::Result;
  using Feedback = JointActionServer::Feedback;

  Result createResult();

  JointActionServer jointActionServer_;


  // Information needed for tracking the Trapezoidal Profile
  struct ProfileInformation{
    ProfileInformation(Eigen::VectorXd startingPosition,
                       Eigen::VectorXd finalPosition,
                       ros::Time startTime):
                         startingPosition_(startingPosition),
                         finalPosition_(finalPosition),
                         difference_(finalPosition-startingPosition),
                         startTime(startTime){}

    const Eigen::VectorXd startingPosition_;
    const Eigen::VectorXd finalPosition_;
    const Eigen::VectorXd difference_;
    const ros::Time startTime;
  };
  std::unique_ptr<TrapezoidalVelocityProfile> trapezoidalProfile_;
  std::unique_ptr<ProfileInformation> profileInformation_;

   Eigen::VectorXd currentGoal_;
   bool initialized_;

   Eigen::VectorXd maxVelocity_;
   Eigen::VectorXd maxAcceleration_;
};

} // namespace trapezoidal_tracker
