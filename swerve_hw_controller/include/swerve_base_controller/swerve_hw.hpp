#pragma once

#include <iostream>
#include <urdf/model.h>
#include <string>
#include <vector>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "swerve_base_controller/hardware_interface.hpp"
#include "RobotMessage.h"

#include <dh_gripper.hpp>


#define NUM_JOINTS 14
namespace swerve
{
    class SwerveHW : public hardware_interface::RobotHW
    {
    public:
        SwerveHW() = default;

        bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

        RobotCmd cmd_;
        RobotState state_;
        RobotState test_;
        // std::shared_ptr<HardwareInterfaceArm> HWArm;
        HardwareInterface *HW_;
        DhGripper gripper_;
        int num_joints = NUM_JOINTS;

        void read(const ros::Time &time, const ros::Duration &period);
        void write(const ros::Time &time, const ros::Duration &period);

    protected:

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
    
        std::vector<string> joint_name;

        std::shared_ptr<urdf::Model> urdfModel_;  // NOLINT(misc-non-private-member-variables-in-classes)

        double joint_position_state[NUM_JOINTS];
        double joint_velocity_state[NUM_JOINTS];
        double joint_effort_state[NUM_JOINTS];
        double joint_velocity_cmd[NUM_JOINTS];
        double joint_position_cmd[NUM_JOINTS];
        double joint_effort_cmd[NUM_JOINTS];
    private:

        bool setupJoints();
        bool loadUrdf(ros::NodeHandle& rootNh);
    };
}
