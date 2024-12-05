#include "SwerveTarget.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ocs2 {
namespace swerve {

SwerveTarget::SwerveTarget(ros::NodeHandle nh, SwerveModelInfo swerveModelInfo, std::string controlFrame)
    : nh_(nh),
      dummyVisualization_(false),
      swerveModelInfo_(swerveModelInfo),
      server_("simple_marker"),
      controlFrame_(controlFrame),
      buffer_(),
      listener_(buffer_) {
  actionClient_.reset(new actionlib::SimpleActionClient<manipulation_msgs::ReachPoseAction>(nh_, "task_space_action_server/task_space"));

  ROS_INFO("Waiting for action server to start.");
  actionClient_->waitForServer();
  ROS_INFO("Action server started.");

  // Subscribe to path publisher
  missionPathSub_ = nh_.subscribe("/mission_control/planned_path", 10, &SwerveTarget::missionPathCallback, this);

  // Trajectory publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nh_, "swerve"));
  state_ = 0;
  // Init brakes
  // initBrakesService();
}

void SwerveTarget::initDummyVisualizationTarget() {
  observationSubscriber_ = nh_.subscribe<ocs2_msgs::mpc_observation>("swerve_mpc_observation", 1, &SwerveTarget::observationCallback, this);
  dummyVisualization_ = true;
}

void SwerveTarget::initSetGoalPose() {
  goal_sub = nh_.subscribe("/move_base_simple/goal", 10, &SwerveTarget::goalCallback, this);
}

void SwerveTarget::initInterectiveMarker() {
  menuHandler_.insert("Send Command", boost::bind(&SwerveTarget::interactiveMarkerCallback, this, _1));

  auto interactiveMarker = createInteractiveMarker();

  server_.insert(interactiveMarker);
  menuHandler_.apply(server_, interactiveMarker.name);

  server_.applyChanges();
}

void SwerveTarget::interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg) {
  manipulation_msgs::ReachPoseGoal goal_pose;
  goal_pose.poseStamped.pose = msg->pose;
  goal_pose.poseStamped.header = msg->header;

  actionClient_->sendGoal(goal_pose);
  state_ = 1;
}

visualization_msgs::InteractiveMarker SwerveTarget::createInteractiveMarker() const {
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = controlFrame_;
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Goal";
  interactiveMarker.scale = 0.5;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.z = 1.0;
  interactiveMarker.pose.orientation.w =0;
  interactiveMarker.pose.orientation.x = 0.7071;
  interactiveMarker.pose.orientation.z = 0.7071;
  // create a grey box marker
  const auto boxMarker = []() {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

void SwerveTarget::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(latestObservationMutex_);
  latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
}

// bool SwerveTarget::getBrakeState(int index) {
//   return brakesState_[index];
// }

// void SwerveTarget::setBrakeState(int index, bool value) {
//   brakesState_[index] = value;
// }

// Eigen::Vector4d SwerveTarget::getDesiredBrakesPosition() {
//   return brakesAngles_;
// }

// void SwerveTarget::initBrakesService() {
//   brakesAngles_ = Eigen::Vector4d::Zero();
//   oldbrakesAngles_ = Eigen::Vector4d::Zero();
//   brakesStatePub_ = nh_.advertise<ocs2_msgs::mode_schedule>("swerve_mode_schedule", 1, false);
//   brakesService_ = nh_.advertiseService("brakes_service", &SwerveTarget::brakesServiceCallback, this);
// }

// bool SwerveTarget::brakesServiceCallback(swerve_msgs::BrakesService::Request& req, swerve_msgs::BrakesService::Response& res) {
//   brakesAngles_ << req.lb_angle, req.lf_angle, req.rb_angle, req.rf_angle;
//   brakesState_[0] = req.lb_brake_active;
//   brakesState_[1] = req.lf_brake_active;
//   brakesState_[2] = req.rb_brake_active;
//   brakesState_[3] = req.rf_brake_active;

//   int brakesStatePub = (brakesState_[0] << 3) + (brakesState_[1] << 2) + (brakesState_[2] << 1) + brakesState_[3];
//   ocs2_msgs::mode_schedule modeSchedule;
//   modeSchedule.eventTimes.push_back(ros::Time::now().toSec());
//   modeSchedule.modeSequence.push_back(brakesStatePub);
//   brakesStatePub_.publish(modeSchedule);

//   res.result = true;
//   return true;
// }

void SwerveTarget::missionPathCallback(const nav_msgs::PathPtr& msg) {
  int numPosesPath = 0;
  TargetTrajectories desiredTrajectory;

  nav_msgs::Path tempPath = *msg;

  if (dummyVisualization_) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    auto timeSec = latestObservation_.time;

    if (tempPath.poses.size() > 0) {
      ros::Time lastPoseOriginalTimestamp(tempPath.poses[0].header.stamp);
      tempPath.poses[0].header.stamp = ros::Time(timeSec);
      for (int i = 1; i < tempPath.poses.size(); i++) {
        double deltaOriginal = ros::Time(tempPath.poses[i].header.stamp).toSec() - lastPoseOriginalTimestamp.toSec();
        timeSec += deltaOriginal;
        lastPoseOriginalTimestamp = ros::Time(tempPath.poses[i].header.stamp);
        tempPath.poses[i].header.stamp = ros::Time(timeSec);
      }
    }
  }

  if (tempPath.header.frame_id != controlFrame_) {
    auto currentFrame = tempPath.header.frame_id;
    try {
      auto transform = buffer_.lookupTransform(controlFrame_, currentFrame, tempPath.header.stamp, ros::Duration(0.5));
      tempPath.header.frame_id = controlFrame_;
      for (auto& ps : tempPath.poses) {
        ps.header.frame_id = controlFrame_;
        tf2::doTransform(ps.pose, ps.pose, transform);
      }
    } catch (const tf2::TransformException& e) {
      ROS_ERROR_STREAM("An error has occured: " << e.what());
      return;
    }
  }

  numPosesPath = tempPath.poses.size();
  desiredTrajectory.stateTrajectory.reserve(numPosesPath);
  desiredTrajectory.inputTrajectory.reserve(numPosesPath);
  desiredTrajectory.timeTrajectory.reserve(numPosesPath);

  double startTime = tempPath.poses.front().header.stamp.toSec();
  double endTime = tempPath.poses.back().header.stamp.toSec();

  for (int i = 0; i < numPosesPath; ++i) {
    const Eigen::Vector3d position(tempPath.poses[i].pose.position.x, tempPath.poses[i].pose.position.y, tempPath.poses[i].pose.position.z);
    const Eigen::Quaterniond orientation(tempPath.poses[i].pose.orientation.w, tempPath.poses[i].pose.orientation.x,
                                         tempPath.poses[i].pose.orientation.y, tempPath.poses[i].pose.orientation.z);

    double currentTime = tempPath.poses[i].header.stamp.toSec();
    double alpha = (currentTime - startTime) / (endTime - startTime);
    // Eigen::VectorXd currentBreakAngle = brakesAngles_ * alpha + oldbrakesAngles_ * (1 - alpha);
    vector_t target = (vector_t(7) << position, orientation.coeffs()).finished();
    desiredTrajectory.timeTrajectory.push_back(currentTime);
    desiredTrajectory.stateTrajectory.push_back(target);
    desiredTrajectory.inputTrajectory.push_back(vector_t::Zero(swerveModelInfo_.inputDim));
  }

  // Publish trajectory
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(desiredTrajectory);
  // oldbrakesAngles_ = brakesAngles_;
}

void SwerveTarget::goalCallback(const geometry_msgs::PoseStampedPtr& msg) {
  manipulation_msgs::ReachPoseGoal goal_pose;
  goal_pose.poseStamped = *msg;

  actionClient_->sendGoal(goal_pose);
}

}  // namespace swerve
}  // namespace ocs2
