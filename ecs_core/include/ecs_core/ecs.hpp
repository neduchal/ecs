/**
 *  Environment Detection System (EDS)
 * 
 *  Class EnvDetection
 * 
 *  Main Class of the EDS
 * 
 *  @author Petr Neduchal
 *  @mail neduchal@kky.zcu.cz
 *  
 */
#pragma once

// CPP Headers
#include <string>
#include <cmath>
// ROS headers
#include <ros/ros.h>


namespace ecs {

class ECS
{
    public:

    ECS(ros::NodeHandle& nodeHandle);

    virtual ~ECS();

    void read_parameters();
    void run();
    
    private:
    
    ros::NodeHandle& nh_;

    std::string base_frame_;
    std::string map_frame_;


};

}