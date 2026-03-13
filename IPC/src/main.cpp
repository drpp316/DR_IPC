#include <ros/ros.h>
#include "planner.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ipc2_node");
    ros::NodeHandle nh;

    PlannerClass planner(nh);
    
    // ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
