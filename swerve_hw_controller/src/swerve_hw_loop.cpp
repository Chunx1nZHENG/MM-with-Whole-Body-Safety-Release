#include "swerve_base_controller/swerve_hw_loop.hpp"

namespace swerve
{

    SwerveHWLoop::SwerveHWLoop(ros::NodeHandle &nh, std::shared_ptr<SwerveHW> hardware_interface)
        : nh_(nh), hardware_interface_(std::move(hardware_interface)), loopRunning_(true)
    {
        // Create the controller manager
        std::cout<<"loop debug "<<std::endl;
        controllerManager_ = std::make_shared<controller_manager::ControllerManager>(hardware_interface_.get(), nh_);
        std::cout<<"loop debug 1"<<std::endl;
        // Load ros params
        int error = 0;
        int threadPriority = 0;
        ros::NodeHandle nhP("~");
        error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
        error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
        error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
        if (error > 0)
        {
            std::string error_message =
                "could not retrieve one of the required parameters: loop_hz or cycle_time_error_threshold or thread_priority";
            ROS_ERROR_STREAM(error_message);
            throw std::runtime_error(error_message);
        }
        // Get current time for use with first update
        lastTime_ = Clock::now();
        std::cout<<"loop debug 2"<<std::endl;
        // Setup loop thread
        loopThread_ = std::thread([&]() { while (loopRunning_) {
            update();
            }
        });

        sched_param sched{.sched_priority = threadPriority};
        // if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0)
        // {
        //     ROS_WARN(
        //         "Failed to set threads priority (one possible reason could be that the user and the group permissions "
        //         "are not set properly.).\n");
        // }
        if (int rc = pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched); rc != 0)
        {
            ROS_WARN("Failed to set thread priority: %s", strerror(rc));
        }
    }

    void SwerveHWLoop::update()
    {
        const auto currentTime = Clock::now();
        const Duration desiredDuration(1.0 / loopHz_);

        Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
        elapsedTime_ = ros::Duration(time_span.count());
        lastTime_ = currentTime;

        // Check cycle time for excess delay
        const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
        if (cycle_time_error > cycleTimeErrorThreshold_)
        {
            ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                                       << "cycle time: " << elapsedTime_ << "s, "
                                                                       << "threshold: " << cycleTimeErrorThreshold_ << "s");
        }

        // Input
        // get the hardware's state
        hardware_interface_->read(ros::Time::now(), elapsedTime_);
    
        // Control
        // let the controller compute the new command (via the controller manager)
        controllerManager_->update(ros::Time::now(), elapsedTime_);

        // Output
        // send the new command to hardware
        hardware_interface_->write(ros::Time::now(), elapsedTime_);

        // Sleep
        const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
        std::this_thread::sleep_until(sleepTill);
    }

    SwerveHWLoop::~SwerveHWLoop()
    {
        loopRunning_ = false;
        if (loopThread_.joinable())
        {
            loopThread_.join();
        }
    }
}