#include "dh_gripper.hpp"
#include "dh_gripper_msgs/GripperCtrl.h"
#include "dh_gripper_msgs/GripperState.h"
#include "ros/ros.h"

void DhGripper::gripperStateCallback(const dh_gripper_msgs::GripperState::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(gripperStatemutex_);
    gripperState_.grip_state = msg->grip_state;
    gripperState_.is_initialized = msg->is_initialized;
    gripperState_.position = msg->position;
    gripperState_.target_position = msg->target_position;
    gripperState_.target_force = msg->target_force;
}

void DhGripper::init(ros::NodeHandle &nh)
{
    gripperStateSub_ = nh.subscribe("/gripper/states", 1, &DhGripper::gripperStateCallback, this);
    gripperCtrlPub_ = nh.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl", 1);
    release();
}

DhGripper::DhGripper(ros::NodeHandle &nh)
    : nh_(nh)
{
    gripperStateSub_ = nh_.subscribe("/gripper/states", 1, &DhGripper::gripperStateCallback, this);
    gripperCtrlPub_ = nh_.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl", 1);
}

void DhGripper::initialize(void)
{
    dh_gripper_msgs::GripperCtrl msg;
    msg.speed = 100;
    msg.force = 50;
    msg.position = 950;
    msg.initialize = true;
    gripperCtrlPub_.publish(msg);
}

void DhGripper::grip(void)
{
    dh_gripper_msgs::GripperCtrl msg;
    msg.speed = 100;
    msg.force = 50;
    msg.position = 50;
    msg.initialize = false;
    gripperCtrlPub_.publish(msg);
}

void DhGripper::release(void)
{
    dh_gripper_msgs::GripperCtrl msg;
    msg.speed = 100;
    msg.force = 50;
    msg.position = 950;
    msg.initialize = false;
    gripperCtrlPub_.publish(msg);
}

DhGripper::gripperState DhGripper::getGripperState(void)
{
    std::lock_guard<std::mutex> lock(gripperStatemutex_);
    return gripperState_;
}

DhGripper::DhGripper() {
}

DhGripper::~DhGripper()
{
    std::cout<<"gripper release"<<"\n";
    release();
}


