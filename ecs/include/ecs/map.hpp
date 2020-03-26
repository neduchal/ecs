#ifndef ECS_MAP_H
#define ECS_MAP_H

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
#include <ecs/EnvValue.h>
#include <ecs/GetLayers.h>
#include <ecs/GetLayer.h>

using namespace grid_map;

namespace ecs
{
W
const double GRID_MAP_PUBLISH_OFF = 0.0;
const std::string BASE_LAYER = "base";
const std::string DEFAULT_MAP_TOPIC = "map";
const std::string DEFAULT_ENV_VALUE_TOPIC = "/ecs/env_value";
const std::string DEFAULT_SET_LAYER_TOPIC = "/ecs/set_layer";
const std::string DEFAULT_GRID_MAP_TOPIC = "/ecs/grid_map";
const std::string DEFAULT_BASE_FRAME = "base_link";
const std::string DEFAULT_MAP_FRAME = "map";
W
class Map
{
public:
    Map(ros::NodeHandle &node_handle);
    virtual ~Map();
    void readParameters();
    void envValueCallback(const ecs::EnvValue &msg);
    void inputMapCallback(const nav_msgs::OccupancyGrid &msg);
    void getLayers(ecs::GetLayers::Request &req, ecs::GetLayers::Response &res);
    void getLayer(ecs::GetLayer::Request &req, ecs::GetLayer::Response &res);
    void setLayerCallback(const grid_map_msgs::GridMap &msg);
    void publishGridMap();
    void getPose();

private:
    ros::NodeHandle &nh_;
    std::string env_value_topic_;
    std::string input_map_topic_;
    std::string set_layer_topic_;
    std::string input_map_metadata_topic_;
    std::string grid_map_topic_;
    std::string base_frame_;
    std::string map_frame_;
    ros::Subscriber env_value_subscriber_;
    ros::Subscriber input_map_subscriber_;
    ros::Subscriber set_layer_subscriber_;
    ros::Publisher grid_map_publisher_;
    tf2_ros::TransformListener *transform_listener_;
    tf2_ros::Buffer transformBuffer_;
    geometry_msgs::TransformStamped pose_;
    GridMap map_;
    GridMapRosConverter converter_;
    bool setup_done_;
};
} // namespace ecs

#endif