#include <iostream>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <swerve_base_controller/hardware_interface.hpp>
#include <swerve_base_controller/swerve_hw.hpp>
#include <swerve_base_controller/swerve_hw_loop.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    ros::AsyncSpinner spinner(3);
    spinner.start();

    try {
        std::shared_ptr<swerve::SwerveHW> swerveHw = std::make_shared<swerve::SwerveHW>();
        bool init_success = swerveHw->init(nh, privateNh);
        std::cout << "init success: " << init_success << std::endl;
        swerve::SwerveHWLoop controlLoop(nh, swerveHw);
        std::cout << "init success: " << init_success << std::endl;
        ros::waitForShutdown();
    } catch (const ros::Exception& e) {
         ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
        return 1;
    }

    return 0;
    

}
