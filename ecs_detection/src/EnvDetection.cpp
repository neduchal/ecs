#include <ecs_detection/EnvDetection.hpp>

/* 
 *  Environment detection system
 *
 *  Class EnvDetection is an entry point of the system.
 *
 *
 */
using namespace grid_map;

namespace env_detection {

EnvDetection::EnvDetection(ros::NodeHandle& nodeHandle, bool& success) 
    : nodeHandle_(nodeHandle)    
{
    if (!readParameters()) 
    {
        success = false;
        return;
    }

    envValueSubscriber_ = nodeHandle_.subscribe(envValueTopic_, 100, &EnvDetection::envValueCallback, this);
    inputMapSubscriber_ = nodeHandle_.subscribe(inputMapTopic_, 1, &EnvDetection::inputMapCallback, this);
    setLayerSubscriber_ = nodeHandle_.subscribe(setLayerTopic_, 10, &EnvDetection::setLayerCallback, this); 
    gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(gridMapTopic_, 1, true);

    map_ = GridMap({"base"});
    map_.setBasicLayers({"base"});
    map_.setFrameId("map");

    transformListener_ = new tf2_ros::TransformListener(transformBuffer_);

    converter_ = GridMapRosConverter();

    setup_done_ = false;
    success = true;
}

EnvDetection::~EnvDetection()
{
}

bool EnvDetection::readParameters()
{
    if (!nodeHandle_.getParam("input_map_topic", inputMapTopic_))
    {
        ROS_ERROR("Could not read parameter 'inputMapTopic'.");
        return false;
    }

    if (!nodeHandle_.getParam("env_value_topic", envValueTopic_))
    {
        ROS_ERROR("Could not read parameter 'env_value_topic'.");
        return false;
    }  

    if (!nodeHandle_.getParam("set_layer_topic", setLayerTopic_))
    {
        ROS_ERROR("Could not read parameter 'set_layer_topic'");
        return false;
    }
    nodeHandle_.param<std::string>("base_frame", baseFrame_, "base_link");
    nodeHandle_.param<std::string>("map_frame", mapFrame_, "map");
    nodeHandle_.param<std::string>("grid_map_topic", gridMapTopic_, "/grid_map");

    return true;
}

void EnvDetection::envValueCallback(const env_detection_msgs::EnvValue& msg)
{
    if (!setup_done_) return;
    if (!map_.exists(msg.layer)) {
        map_.add(msg.layer);
    }    
    Position position(pose_.transform.translation.x, pose_.transform.translation.y);
    Index indexPosition;
    map_.getIndex(position, indexPosition);
    if (isnan(map_.at(msg.layer, indexPosition))) {
    map_.at(msg.layer, indexPosition) = msg.value;
    map_.at(msg.layer, indexPosition-2) = msg.value;
    map_.at(msg.layer, indexPosition-1) = msg.value;
    map_.at(msg.layer, indexPosition+1) = msg.value;
    map_.at(msg.layer, indexPosition+2) = msg.value;
    map_.at(msg.layer, indexPosition+3) = msg.value;
    map_.at(msg.layer, indexPosition-3) = msg.value;
    } else{
        map_.at(msg.layer, indexPosition) += msg.value;
        map_.at(msg.layer, indexPosition-2) += msg.value;
        map_.at(msg.layer, indexPosition-1) += msg.value;
        map_.at(msg.layer, indexPosition+1) += msg.value;
        map_.at(msg.layer, indexPosition+2) += msg.value;
        map_.at(msg.layer, indexPosition+3) += msg.value;
        map_.at(msg.layer, indexPosition-3) += msg.value;
        map_.at(msg.layer, indexPosition) /= 2;
        map_.at(msg.layer, indexPosition-2) /= 2;
        map_.at(msg.layer, indexPosition-1) /= 2;
        map_.at(msg.layer, indexPosition+1) /= 2;
        map_.at(msg.layer, indexPosition+2) /= 2;
        map_.at(msg.layer, indexPosition+3) /= 2;
        map_.at(msg.layer, indexPosition-3) /= 2;               
    }

}

void EnvDetection::getPose()
{
    try 
    {
        pose_ = transformBuffer_.lookupTransform("map", "base_link", ros::Time(0));
    } catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }  
}

void EnvDetection::inputMapCallback(const nav_msgs::OccupancyGrid& msg)
{
    if (!converter_.fromOccupancyGrid(msg, "base", map_)) 
    {
        ROS_ERROR("Could not transform occupancy grid into the GridMap");
    } else 
    {
        ROS_INFO("MAP -- size (%f x %f m), %i x %i cells -- center (%f, %f) -- frame %s", map_.getLength().x(), map_.getLength().y(),
            map_.getSize()(0), map_.getSize()(1), map_.getPosition().x(), map_.getPosition().y(), map_.getFrameId().c_str());
        if (!setup_done_) setup_done_ = true;
        try 
        {
            pose_ = transformBuffer_.lookupTransform("map", "base_link", ros::Time(0));
        } catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }    
}

void EnvDetection::publishGridMap()
{
    ros::Time time = ros::Time::now();
    map_.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap msg;
    converter_.toMessage(map_, msg);
    gridMapPublisher_.publish(msg);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", msg.info.header.stamp.toSec());
}

void EnvDetection::setLayerCallback(const grid_map_msgs::GridMap& msg)
{
    GridMap grid_map_layer = GridMap();
    converter_.fromMessage(msg, grid_map_layer);
    std::vector<std::string> layers = grid_map_layer.getLayers();
    for(int i; i < layers.size(); i++)
    {
        if (layers[i].compare("base") > 0)
        {
            map_.add(layers[i], grid_map_layer.get(layers[i]));
        }
    }
}


void EnvDetection::getLayers(env_detection_msgs::GetLayers::Request &req, env_detection_msgs::GetLayers::Response &res)
{
    std::vector<std::string> layers = map_.getLayers();
    for(std::size_t i=0; i < layers.size(); ++i)
    {
        res.layers.push_back(layers[i]);
    }
}

void EnvDetection::getLayer(env_detection_msgs::GetLayer::Request &req, env_detection_msgs::GetLayer::Response &res)
{
    GridMap map_msg = GridMap();
    grid_map_msgs::GridMap msg;

    if (map_.exists(req.layer))
    {
        std::vector<std::string> layers;
        layers.push_back("base");
        layers.push_back(req.layer);
        res.valid = true;
        map_msg.addDataFrom(map_, true, true, false, layers);
        converter_.toMessage(map_msg, msg);
        res.map = msg;
    } else
    {
        res.valid = false;
    }
}

}