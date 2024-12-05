#include <eigen_conversions/eigen_msg.h>
#include <odometry_controller/odometry_controller.h>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <chrono>
#include <exception>

namespace odometry_controller {

OdometryController::OdometryController()
    : wheel_radius_(0.0),
      base_frame_id_("base_link"),
      odom_frame_id_("odom"),
      enable_odom_tf_(false),
      timestamp_(0.0),
      x_(0.0),
      y_(0.0),
      heading_(0.0),
      linear_x_(0.0),
      linear_y_(0.0),
      angular_(0.0),
      buffer_(),
      listener_(buffer_),
      wheelStates_(),
      wheelLinkNames_() {
        orientation_.x = 0.0;
        orientation_.y = 0.0;
        orientation_.z = 0.0;
        orientation_.w = 1.0;
        localPos_.setZero();
      }

bool OdometryController::init(hardware_interface::JointStateInterface* joint_state_interface, ros::NodeHandle& root_nh,
                              ros::NodeHandle& controller_nh) {
  const std::string complete_ns = controller_nh.getNamespace();

  std::size_t id = complete_ns.find_last_of("/");
  name_ = complete_ns.substr(id + 1);

  controller_nh.getParam("wheel_links", wheelLinkNames_);
  std::vector<std::string> wheelJointNames;
  controller_nh.getParam("wheel_joints", wheelJointNames);
  for (const auto& wheelJointName : wheelJointNames) {
    wheelJointHandles_.push_back(joint_state_interface->getHandle(wheelJointName));
  }

  controller_nh.getParam("brake_links", brakeLinkNames_);
  std::vector<std::string> brakeJointNames;
  controller_nh.getParam("brake_joints", brakeJointNames);
  // for (const auto& brakeJointName : brakeJointNames) {
  //   brakeJointHandles_.push_back(joint_state_interface->getHandle(brakeJointName));
  // }

  if (wheelLinkNames_.size() != wheelJointHandles_.size() || wheelJointHandles_.size() != brakeLinkNames_.size()
      // || brakeLinkNames_.size() != brakeJointHandles_.size()
      ) {
    ROS_ERROR_STREAM("wheelLinkNames_.size=" << wheelLinkNames_.size() << "; wheelJointHandles_.size=" << wheelJointHandles_.size()
                                            //  << "; brakeLinkNames_.size=" << brakeLinkNames_.size()
                                            //  << "; brakeJointHandles_.size=" << brakeJointHandles_.size() 
                                             << " must be equal.");
    return false;
  }

  // Odometry related:
  double publish_rate;
  controller_nh.param("publish_rate", publish_rate, 50.0);
  ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at " << publish_rate << "Hz.");
  publish_period_ = ros::Duration(1.0 / publish_rate);

  controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

  controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
  ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

  controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
  ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

  // If a parameter is not available, we need to look up the value in the URDF
  controller_nh.getParam("wheel_radius", wheel_radius_);

  setOdomPubFields(root_nh, controller_nh);
  
  // Reset timestamp:
  timestamp_ = ros::Time::now();
  x_ = 0;
  y_ = 0;
  heading_ = 0;

  steeringPositionUpdateTimer_ = controller_nh.createTimer(ros::Duration(0.05), [this](const auto& e) { steeringPositionCallback(e); });
  controller_nh.param("lidar_global_odom", lidar_global_odom_, false);
  ROS_INFO_STREAM_NAMED(name_, "lidar global odom is " << (enable_odom_tf_ ? "enabled" : "disabled"));
  
  if(lidar_global_odom_){
     
    lidar_odom_sub_ = controller_nh.subscribe<nav_msgs::Odometry>("/odom", 1, 
      [this](const boost::shared_ptr<const nav_msgs::Odometry>& msg) {
        auto lidarPos = msg->pose.pose.position; 
        lidarPos.x = -lidarPos.x;
        lidarPos.y = -lidarPos.y;
        lidarPos.z = 0;
  
        auto lidarOrient = msg->pose.pose.orientation;
    
        auto lidarTransform = buffer_.lookupTransform("lidar_frame" ,base_frame_id_ , ros::Time(0), ros::Duration(0));
        Eigen::Quaterniond quat;
        Eigen::Vector3d pos;
        tf::quaternionMsgToEigen(lidarOrient, quat);
        tf::pointMsgToEigen(lidarPos, pos);
        Eigen::Matrix3d mat = quat.toRotationMatrix();

        Eigen::Affine3d baseTolidar;
        tf::transformMsgToEigen(lidarTransform.transform, baseTolidar);
        Eigen::Affine3d lidarTolocal;
        lidarTolocal.setIdentity();
        lidarTolocal.translation() = pos;
        lidarTolocal.rotate(mat);

  
        auto baseTlocal = lidarTolocal*baseTolidar;
        localPos_ = baseTlocal.translation();
        Eigen::Quaterniond localQuat(baseTlocal.rotation());
        tf::quaternionEigenToMsg(localQuat, orientation_);
        
      });
  }

  ROS_INFO_STREAM_NAMED(name_, "Finished controller initialization");
  return true;
}

void OdometryController::update(const ros::Time& time, const ros::Duration& period) {
  Eigen::VectorXd wheelSpeeds(wheelJointHandles_.size());
  for (size_t i = 0; i < wheelJointHandles_.size(); ++i) {
    const auto& handle = wheelJointHandles_[i];
    wheelSpeeds[i] = handle.getVelocity();
  }

  // Estimate linear and angular velocity using joint information
  updateUsingVelocities(wheelSpeeds, time);

  // Publish odometry message
  if (last_state_publish_time_ + publish_period_ < time) {
    last_state_publish_time_ += publish_period_;
    // Compute and store orientation info
    const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(heading_));

    // Populate odom message and publish
   
    if (odom_pub_->trylock()) {
      odom_pub_->msg_.header.stamp = time;
      if(lidar_global_odom_){
        odom_pub_->msg_.pose.pose.position.x = localPos_.x();
        odom_pub_->msg_.pose.pose.position.y = localPos_.y();
        odom_pub_->msg_.pose.pose.orientation = orientation_;
      }
      else{
        odom_pub_->msg_.pose.pose.position.x = x_;
        odom_pub_->msg_.pose.pose.position.y = y_;
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x = linear_x_;
        odom_pub_->msg_.twist.twist.linear.y = linear_y_;
        odom_pub_->msg_.twist.twist.angular.z = angular_;

      }

      odom_pub_->unlockAndPublish();
    }
    

    if (odom_twist_world_pub_->trylock()) {
      odom_twist_world_pub_->msg_ = odom_pub_->msg_;
      odom_twist_world_pub_->msg_.twist.twist.linear.x =
          odom_pub_->msg_.twist.twist.linear.x * cos(heading_) - odom_pub_->msg_.twist.twist.linear.y * sin(heading_);
      odom_twist_world_pub_->msg_.twist.twist.linear.y =
          odom_pub_->msg_.twist.twist.linear.x * sin(heading_) + odom_pub_->msg_.twist.twist.linear.y * cos(heading_);
      odom_twist_world_pub_->unlockAndPublish();
    }
    // Publish tf /odom frame
    if (tf_odom_pub_->trylock()) 
    {
      geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
      odom_frame.header.stamp = time;
      if(lidar_global_odom_){
        odom_frame.transform.translation.x = localPos_.x();
        odom_frame.transform.translation.y = localPos_.y();
        odom_frame.transform.rotation = orientation_;
      }
      else{
        odom_frame.transform.translation.x = x_;
        odom_frame.transform.translation.y = y_;
        odom_frame.transform.rotation = orientation;
      }
      tf_odom_pub_->unlockAndPublish();
    }
  }
}

void OdometryController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i) ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i) ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = odom_frame_id_;
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;
  odom_pub_->msg_.pose.covariance = {
      static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0., 0., 0.,
      static_cast<double>(pose_cov_list[2]), 0., 0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0., 0., 0., 0., 0.,
      static_cast<double>(pose_cov_list[4]), 0., 0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5])};

  odom_pub_->msg_.twist.twist.linear.z = 0;
  odom_pub_->msg_.twist.twist.angular.x = 0;
  odom_pub_->msg_.twist.twist.angular.y = 0;
  odom_pub_->msg_.twist.covariance = {
      static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0., 0., 0.,
      static_cast<double>(twist_cov_list[2]), 0., 0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0., 0., 0., 0., 0.,
      static_cast<double>(twist_cov_list[4]), 0., 0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])};
  odom_twist_world_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom_twist_world", 100));
  odom_twist_world_pub_->msg_ = odom_pub_->msg_;
  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
}

bool OdometryController::updateUsingVelocities(const Eigen::VectorXd& wheelSpeeds, const ros::Time& time) {
  geometry_msgs::Twist speed;
  speed = getTwist(wheelSpeeds);

  linear_x_ = speed.linear.x;
  linear_y_ = speed.linear.y;
  angular_ = speed.angular.z;

  // Integrate speed to get position
  const double dt = (time - timestamp_).toSec();
  integrateVelocities(linear_x_, linear_y_, angular_, dt);
  timestamp_ = time;

  return true;
}

geometry_msgs::Twist OdometryController::getTwist(const Eigen::VectorXd& wheelSpeeds) {
  geometry_msgs::Twist res;
  std::lock_guard<std::mutex> lock(steeringPositionMutex_);
  if (wheelStates_.size() != 0) {
    Eigen::VectorXd wheelVelocities = wheelSpeeds * wheel_radius_;
    Eigen::VectorXd steerFrameVelocity(8);

    Eigen::Matrix<double, 8, 3> A;
    A << 1.0, 0.0, -wheelStates_[0].steeringPosition[1], 1.0, 0.0, -wheelStates_[1].steeringPosition[1], 1.0, 0.0,
        -wheelStates_[2].steeringPosition[1], 1.0, 0.0, -wheelStates_[3].steeringPosition[1], 0.0, 1.0, wheelStates_[0].steeringPosition[0],
        0.0, 1.0, wheelStates_[1].steeringPosition[0], 0.0, 1.0, wheelStates_[2].steeringPosition[0], 0.0, 1.0,
        wheelStates_[3].steeringPosition[0];

    for (int i = 0; i < wheelVelocities.size(); ++i) {
      steerFrameVelocity[i] = wheelVelocities[i] * wheelStates_[i].wheelAxis[1];
      steerFrameVelocity[i + 4] = -wheelVelocities[i] * wheelStates_[i].wheelAxis[0];
    }

    // correct for brake velocities
    // Eigen::VectorXd brakeJointCorrections(brakeJointHandles_.size() * 2);
    // for (int i = 0; i < wheelVelocities.size(); ++i) {
    //   brakeJointCorrections[i] = -wheelStates_[i].brakeToSteeringPositionBaseFrame[1] * brakeJointHandles_[i].getVelocity() *
    //                              -1;  // -1 is needed since the z axis of the brake joint frame is perpendicular to the base frame
    //   brakeJointCorrections[i + 4] = wheelStates_[i].brakeToSteeringPositionBaseFrame[0] * brakeJointHandles_[i].getVelocity() * -1;
    // }

    // steerFrameVelocity = steerFrameVelocity - brakeJointCorrections;

    // Evaluate result
    Eigen::Vector3d x = Eigen::Vector3d::Zero();
    x = (A.transpose() * A).inverse() * A.transpose() * steerFrameVelocity;

    // Eigen::Vector3d resultingCorrections = (A.transpose() * A).inverse() * A.transpose() * brakeJointCorrections;
    // std::cout << "brake joint corrections: " << brakeJointCorrections.transpose() << std::endl
    //           << "resulting correction: " << resultingCorrections.transpose() << std::endl
    //           << "odom: " << x.transpose() << std::endl
    //           << std::endl;

    res.linear.x = x(0);
    res.linear.y = x(1);
    res.angular.z = x(2);
  }
  return res;
}

void OdometryController::integrateVelocities(double linear_x, double linear_y, double angular, double dt) {
  // Using explicit Euler
  // Velocities are wrt robot frame, so we need to transform.
  // NextPosition = ThisPosition + ThisVelocity * dt;
  double delta_x = (linear_x * cos(heading_) - linear_y * sin(heading_)) * dt;
  double delta_y = (linear_x * sin(heading_) + linear_y * cos(heading_)) * dt;

  x_ += delta_x;
  y_ += delta_y;
  heading_ += angular * dt;
}

void OdometryController::steeringPositionCallback(const ros::TimerEvent& event) {
  std::lock_guard<std::mutex> lock(steeringPositionMutex_);

  try {
    wheelStates_.clear();
    for (size_t i = 0; i < wheelLinkNames_.size(); ++i) {
      const auto& steeringLink = wheelLinkNames_[i];
      // const auto& brakeLink = brakeLinkNames_[i];
      auto wheelLinkTransform = buffer_.lookupTransform(base_frame_id_, steeringLink, ros::Time(0), ros::Duration(0));
      WheelState wheelState;
      wheelState.steeringPosition[0] = wheelLinkTransform.transform.translation.x;
      wheelState.steeringPosition[1] = wheelLinkTransform.transform.translation.y;

      Eigen::Quaterniond quat;
      tf::quaternionMsgToEigen(wheelLinkTransform.transform.rotation, quat);
      Eigen::Matrix3d mat = quat.toRotationMatrix();
      wheelState.wheelAxis = mat.block<2, 1>(0, 2).normalized();

      // auto brakeLinkTransform = buffer_.lookupTransform(base_frame_id_, brakeLink, ros::Time(0), ros::Duration(0));
      // wheelState.brakeToSteeringPositionBaseFrame[0] = wheelState.steeringPosition[0] - brakeLinkTransform.transform.translation.x;
      // wheelState.brakeToSteeringPositionBaseFrame[1] = wheelState.steeringPosition[1] - brakeLinkTransform.transform.translation.y;

      wheelStates_.push_back(wheelState);
    }
  } catch (std::exception& e) {
    ROS_ERROR(e.what());
    wheelStates_.clear();
  }
}

PLUGINLIB_EXPORT_CLASS(odometry_controller::OdometryController, controller_interface::ControllerBase)

}  // namespace odometry_controller