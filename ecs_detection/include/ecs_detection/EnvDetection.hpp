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
#include <env_detection_msgs/EnvValue.h>
#include <env_detection_msgs/GetLayers.h>
#include <env_detection_msgs/GetLayer.h>

using namespace grid_map; // Namespace of GRID MAP

namespace env_detection {

class EnvDetection  
{
    public:

    EnvDetection(ros::NodeHandle& nodeHandle, bool& success);

    virtual ~EnvDetection();

    bool readParameters();

    void envValueCallback(const env_detection_msgs::EnvValue& msg);
    void inputMapCallback(const nav_msgs::OccupancyGrid& msg);
    void getLayers(env_detection_msgs::GetLayers::Request &req, env_detection_msgs::GetLayers::Response &res);
    void getLayer(env_detection_msgs::GetLayer::Request &req, env_detection_msgs::GetLayer::Response &res);
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