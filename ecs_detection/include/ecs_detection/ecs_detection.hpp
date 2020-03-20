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
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <ecs_detection/EnvValue.h>
#include <ecs_detection/GetLayers.h>
#include <ecs_detection/GetLayer.h>

using namespace grid_map; // Namespace of GRID MAP

namespace ecs_detection {

    const ros::V_string BASE_LAYER       = ros::V_string({"base"});
    const std::string   BASE_LAYER_STR   =                  "base";    
    const std::string   MAP_LAYER        =                   "map";    

class EcsDetection  
{
    public:

    EcsDetection(ros::NodeHandle& nodeHandle, bool& success);

    virtual ~EcsDetection();

    bool readParameters();

    void envValueCallback(const ecs_detection::EnvValue& msg);
    void inputMapCallback(const nav_msgs::OccupancyGrid& msg);
    void getLayers(ecs_detection::GetLayers::Request &req, ecs_detection::GetLayers::Response &res);
    void getLayer(ecs_detection::GetLayer::Request &req, ecs_detection::GetLayer::Response &res);
    void setLayerCallback(const grid_map_msgs::GridMap& msg);
    void publishGridMap();
    void getPose();
    
    private:
    
    ros::NodeHandle& nodeHandle_;

    std::string envValueTopic_;
    std::string inputMapTopic_;
    std::string setLayerTopic_;
    std::string inputMapMetaDataTopic_;
    std::string gridMapTopic_;

    std::string baseFrame_;
    std::string mapFrame_;

    ros::Subscriber envValueSubscriber_;
    ros::Subscriber inputMapSubscriber_;
    ros::Subscriber setLayerSubscriber_;

    ros::Publisher gridMapPublisher_;

    tf2_ros::TransformListener *transformListener_;
    tf2_ros::Buffer transformBuffer_;

    geometry_msgs::TransformStamped pose_;

    GridMap map_;
    GridMapRosConverter converter_;


    bool setup_done_;

};

}