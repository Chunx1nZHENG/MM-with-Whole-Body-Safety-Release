#include <ros/ros.h>
#include <trapezoidal_tracker/TaskSpaceActionHandler.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_space_action_server_node");
    ros::NodeHandle nh("~");
    trapezoidal_tracker::TaskSpaceActionHandler actionServer(nh, "task_space");
    ros::spin();
    return 0;
}