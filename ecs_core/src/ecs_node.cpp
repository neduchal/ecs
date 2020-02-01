#include <ecs_core/ecs.hpp>

#include <ros/ros.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "environment_classification_system_node");
    ros::NodeHandle nodeHandle("~");
    ecs::ECS env_class(nodeHandle);
    /*
    ros::Time current_time = ros::Time::now();
    ros::Rate rate(20.0);
    while(nodeHandle.ok())
    {
        if ((ros::Time::now() - current_time).toSec() > 1)
        {
            envDetection.publishGridMap();
            current_time = ros::Time::now();
        }
        envDetection.getPose();
        ros::spinOnce();
        rate.sleep();
    }
    *
    */

    ros::spin();
    return 0;
}

