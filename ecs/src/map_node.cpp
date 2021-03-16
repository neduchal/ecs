#include <ecs/map.hpp>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ecs_map_node");
    ros::NodeHandle nodeHandle("~");
    ecs::Map map(nodeHandle);
    double grid_map_publish_rate = 0;
    double rate_value = 0;

    nodeHandle.param("grid_map_publish_rate", grid_map_publish_rate, ecs::GRID_MAP_PUBLISH_OFF);
    nodeHandle.param("node_rate", rate_value, 20.0);

    ros::Time current_time = ros::Time::now();
    ros::Rate rate(rate_value);

    while (nodeHandle.ok())
    {
        if (grid_map_publish_rate != ecs::GRID_MAP_PUBLISH_OFF)
        {
            if ((ros::Time::now() - current_time).toSec() > (1 / grid_map_publish_rate))
            {
                map.publishGridMap();
                current_time = ros::Time::now();
            }
            map.getPose();
            ros::spinOnce();
            rate.sleep();
        }
    }
    return 0;
}