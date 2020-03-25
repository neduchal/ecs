#include <ecs/map.hpp>

#include <ros/ros.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ecs_map_node");
    ros::NodeHandle nodeHandle("~");
    ecs::Map map(nodeHandle);

    ros::Time current_time = ros::Time::now();
    ros::Rate rate(20.0);
    while(nodeHandle.ok())
    {
        if ((ros::Time::now() - current_time).toSec() > 1)
        {
            map.publishGridMap();
            current_time = ros::Time::now();
        }
        map.getPose();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}