#ifndef JOINT_MESSAGE_H
#define JOINT_MESSAGE_H

#include "modelDefine.h"
#include <vector>

#include <Eigen/Dense>

struct JointState{
    float v = 0;
    float p = 0;
    float tau = 0;
};

struct JointControl
{
    float p_des = 0;
    float v_des = 0;
    float kp = 0;
    float kd = 0;
    float t_ff = 0;
};



struct ModeControl
{
    JointControl TmotorLeftCtrl, TmotorRightCtrl, ZLWheelhubLeftCtrl, ZLWheelhubRightCtrl;
};

struct ModeState
{
    JointState TmotorLeftState, TmotorRightState, ZLWheelhubLeftState, ZLWheelhubRightState;
};


struct JointMsg
{
    unsigned char data[30];
};

struct ModeMsg
{
    JointMsg TmotorLeftMsg, TmotorRightMsg, ZLWheelhubMsg;
};

// struct RobotState{
//     std::vector<double> q;
//     std::vector<double> qd;
//     std::vector<double> tau;
// };

// struct RobotCmd{
//     std::vector<double> q;
//     std::vector<double> qd;
//     std::vector<double> tau;
//     std::vector<double> kp;
//     std::vector<double> kd;
// };

struct RobotState{
    Eigen::Matrix<double,ROBOT_ACTUATOR_NUM,1> q;
    Eigen::Matrix<double,ROBOT_ACTUATOR_NUM,1> qd;
    Eigen::Matrix<double,ROBOT_ACTUATOR_NUM,1> tau;

    Eigen::Vector4d quaternion;     //wxyz
    Eigen::Vector3d acc;     
    Eigen::Vector3d gyro;

    Eigen::Vector3d realPosition;
    Eigen::Vector3d realVelocity;
    Eigen::Vector3d realAngularVelocity;

};

struct RobotCmd{

    Eigen::Matrix<double,ROBOT_ACTUATOR_NUM,1> q;
    Eigen::Matrix<double,ROBOT_ACTUATOR_NUM,1> qd;
    Eigen::Matrix<double,ROBOT_ACTUATOR_NUM,1> tau;
    Eigen::Matrix<double,ROBOT_ACTUATOR_NUM,1> kp;
    Eigen::Matrix<double,ROBOT_ACTUATOR_NUM,1> kd;

    void setZero(){
        q.setZero();
        qd.setZero();
        tau.setZero();
        kp.setZero();
        kd.setZero();
    }

};


#endif // JOINT_MESSAGE_H