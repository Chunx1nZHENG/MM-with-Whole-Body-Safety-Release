#include <swerve_base_controller/swerve_hw.hpp>

namespace swerve
{
    bool SwerveHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {

        if (!loadUrdf(root_nh))
        {
            ROS_ERROR("Error occurred while setting up urdf");
            return false;
        }

        // state_.q.reserve(num_joints);
        // state_.qd.reserve(num_joints);
        // state_.tau.reserve(num_joints);

        // cmd_.q.reserve(num_joints);
        // cmd_.qd.reserve(num_joints);
        // cmd_.tau.reserve(num_joints);
        // cmd_.kp.reserve(num_joints);
        // cmd_.kd.reserve(num_joints);

        for(int i=0; i<num_joints; i++){
            cmd_.q[i] = 0.0;
            cmd_.qd[i] = 0.0;
        }

        double kp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        double kd[8] = {5, 5, 5, 5, 5, 5, 5, 5};
        for (int i = 0; i < num_joints; i++)
        {
            cmd_.kp[i] = kp[i];
            cmd_.kd[i] = kd[i];
        }

        // Register interfaces
        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);

        setupJoints();

        // motor enabler
        HW_ = new HardwareInterface();
        gripper_.init(robot_hw_nh);
        return true;
    }

    bool SwerveHW::setupJoints()
    {
        for (const auto &joint : urdfModel_->joints_)
        {
            int index;

            if (joint.first.find("left_back_steer_joint") != std::string::npos)
            {
                index = 2; 
            } else if (joint.first.find("right_back_steer_joint") != std::string::npos)
            {
                index = 4;
            } else if (joint.first.find("left_front_steer_joint") != std::string::npos)
            {
                index = 0;
            } else if (joint.first.find("right_front_steer_joint") != std::string::npos)
            {
                index = 6;
            } else if (joint.first.find("left_front_wheel_joint") != std::string::npos)
            {
                index = 1;
            } else if (joint.first.find("right_front_wheel_joint") != std::string::npos)
            {
                index = 7;
            } else if (joint.first.find("right_back_wheel_joint") != std::string::npos)
            {
                index = 3;
            } else if (joint.first.find("left_back_wheel_joint") != std::string::npos)
            {
                index = 5;
            }else if (joint.first.find("SH_JOINT_1") != std::string::npos)
            {
                index = 8;
            }
            else if (joint.first.find("SH_JOINT_2") != std::string::npos)
            {
                index = 9;
            }
            else if (joint.first.find("SH_JOINT_3") != std::string::npos)
            {
                index = 10;
            }
            else if (joint.first.find("SH_JOINT_4") != std::string::npos)
            {
                index = 11;
            }
            else if (joint.first.find("SH_JOINT_5") != std::string::npos)
            {
                index = 12;
            }
            else if (joint.first.find("SH_JOINT_6") != std::string::npos)
            {
                index = 13;
            }
            else if (joint.first.find("left_outer_knuckle") != std::string::npos)
            {
                index = 14;
            }
            else
            {
                continue;
            }

            hardware_interface::JointStateHandle state_handle(joint.first, &state_.q[index], &state_.qd[index],
                                                              &state_.tau[index]);
            joint_state_interface_.registerHandle(state_handle);

            hardware_interface::JointHandle command_handle(state_handle, &cmd_.qd[index]);
            velocity_joint_interface_.registerHandle(command_handle);
        }

        return true;
    }

    void SwerveHW::read(const ros::Time &time, const ros::Duration &period)
    {
        // use '.' to visit the data, q is position, qd is velocity and tau is effort
        HW_->recv(&state_);
        state_.q[14] = 1-(gripper_.getGripperState().position/1000.0);
    }

    void SwerveHW::write(const ros::Time &time, const ros::Duration &period)
    {   
        // 速度控制模式
        // 转向电机的位置比例、微分系数
        // cmd_.qd[3] = 0.0;
        // cmd_.qd[5] = 0.0;
        // cmd_.qd[1] = 0.0;
        // cmd_.qd[7] = -0.5;
        // cmd_.qd[0] = 0.;
        // cmd_.qd[2] = 0.;
        // cmd_.qd[4] = 0.;
        // cmd_.qd[6] = 0.;
        // std::cout << "[cmd] " << cmd_.qd[0] << " " <<  cmd_.qd[1] << " " << cmd_.qd[2] << " " << cmd_.qd[3] 
        //           << " " << cmd_.qd[4] << " " << cmd_.qd[5] << " " << cmd_.qd[6] << " " << cmd_.qd[7] << std::endl;
     
        
        HW_->send(&cmd_);
        if (cmd_.qd[14]>=0)
        {
            gripper_.grip();
        } else
        {
            gripper_.release();
        }
    }

    bool SwerveHW::loadUrdf(ros::NodeHandle &rootNh)
    {
        std::string urdfString; 
        if (urdfModel_ == nullptr)
        {
            urdfModel_ = std::make_shared<urdf::Model>();
        }
        // get the urdf param on param server
        rootNh.getParam("robot_description", urdfString);
        return !urdfString.empty() && urdfModel_->initString(urdfString);
    }

}

PLUGINLIB_EXPORT_CLASS(swerve::SwerveHW, hardware_interface::RobotHW)