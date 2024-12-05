#pragma once
#include <ros/ros.h>
#include <mutex>

#include "dh_gripper_msgs/GripperCtrl.h"
#include "dh_gripper_msgs/GripperState.h"

class DhGripper
{
    public:
        DhGripper(ros::NodeHandle &nh);
        DhGripper();
        ~DhGripper();
        void grip (void);
        void release (void);
        void initialize(void);
        struct gripperState
        {
            bool    is_initialized;
            int     grip_state;
            double  position;
            double  target_position;
            double  target_force;
            
            gripperState() : is_initialized(false), grip_state(0), position(0), target_position(0), target_force(0) {}
        };
        void init(ros::NodeHandle& nh) ;

        gripperState getGripperState(void);

    private:
        void gripperStateCallback(const dh_gripper_msgs::GripperState::ConstPtr &msg);
        ros::NodeHandle nh_;
        ros::Subscriber gripperStateSub_;
        ros::Publisher gripperCtrlPub_;
        gripperState gripperState_;
        std::mutex gripperStatemutex_;
        
};