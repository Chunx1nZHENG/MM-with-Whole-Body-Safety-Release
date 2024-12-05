#pragma once

#include "swerve_base_controller/swerve_hw.hpp"

#include <chrono>
#include <thread>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

namespace swerve
{

    class SwerveHWLoop
    {
        using Clock = std::chrono::high_resolution_clock;
        using Duration = std::chrono::duration<double>;

    public:
        SwerveHWLoop(ros::NodeHandle &nh, std::shared_ptr<SwerveHW> hardware_interface);
        ~SwerveHWLoop();

        void update();

    private:
        ros::NodeHandle nh_;

        // Timing
        double cycleTimeErrorThreshold_{}, loopHz_{};
        std::thread loopThread_;
        std::atomic_bool loopRunning_{};
        ros::Duration elapsedTime_;
        Clock::time_point lastTime_;

        /** ROS Controller Manager and Runner

            This class advertises a ROS interface for loading, unloading, starting, and
            stopping ros_control-based controllers. It also serializes execution of all
            running controllers in \ref update.
        **/
        std::shared_ptr<controller_manager::ControllerManager> controllerManager_;

        std::shared_ptr<SwerveHW> hardware_interface_;
    };
}
