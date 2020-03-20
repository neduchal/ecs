#include <ecs_detection/ecs_detection.hpp>

#include <ros/ros.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "environment_detection_basic_node");
    ros::NodeHandle nodeHandle("~");
    bool success;
    ecs_detection::EcsDetection ecsDetection(nodeHandle, success);
    if (!success) exit(1);

    ros::Time current_time = ros::Time::now();
    ros::Rate rate(20.0);
    while(nodeHandle.ok())
    {
        if ((ros::Time::now() - current_time).toSec() > 1)
        {
            ecsDetection.publishGridMap();
            current_time = ros::Time::now();
        }
        ecsDetection.getPose();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}