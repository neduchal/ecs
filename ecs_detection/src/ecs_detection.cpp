#include <ecs_detection/ecs_detection.hpp>

/* 
 *  ECS detection system
 *
 *  Class EcsDetection is an entry point of the system.
 *
 *
 */
using namespace grid_map;

namespace ecs_detection
{
    EcsDetection::EcsDetection(ros::NodeHandle &node_handle)
        : nh_(node_handle)
    {
        readParameters();
        env_value_subscriber_ = nh_.subscribe(env_value_topic_,
                                                      100, 
                                                      &EcsDetection::envValueCallback, 
                                                      this
                                                      );
        input_map_subscriber_ = nh_.subscribe(input_map_topic_, 
                                                      1, 
                                                      &EcsDetection::inputMapCallback, 
                                                      this
                                                      );
        set_layer_subscriber_ = nh_.subscribe(set_layer_topic_, 
                                                      10, 
                                                      &EcsDetection::setLayerCallback, 
                                                      this
                                                      );
        grid_map_publisher_ = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_, 1, true);
        //map_ = GridMap({"base"});
        //map_.setBasicLayers({"base"});
        map_ = GridMap(ros::V_string({"BASE_LAYER"}));
        map_.setBasicLayers(ros::V_string({"BASE_LAYER"}));
        map_.setFrameId(map_frame_);
        transform_listener_ = new tf2_ros::TransformListener(transformBuffer_);
        converter_ = GridMapRosConverter();
        setup_done_ = false;
    }

    EcsDetection::~EcsDetection()
    {
    }

    void EcsDetection::readParameters()
    {
        nh_.param<std::string>("input_map_topic", input_map_topic_,       DEFAULT_MAP_TOPIC);
        nh_.param<std::string>("env_value_topic", env_value_topic_, DEFAULT_ENV_VALUE_TOPIC);
        nh_.param<std::string>("set_layer_topic", set_layer_topic_, DEFAULT_SET_LAYER_TOPIC);
        nh_.param<std::string>("base_frame",      base_frame_,           DEFAULT_BASE_FRAME);
        nh_.param<std::string>("map_frame",       map_frame_,             DEFAULT_MAP_FRAME);
        nh_.param<std::string>("grid_map_topic",  grid_map_topic_,   DEFAULT_GRID_MAP_TOPIC);
    }

    void EcsDetection::envValueCallback(const ecs_detection::EnvValue &msg)
    {
        if (!setup_done_)
            return;
        if (!map_.exists(msg.layer))
        {
            map_.add(msg.layer);
        }
        Position position(pose_.transform.translation.x, pose_.transform.translation.y);
        Index indexPosition;
        map_.getIndex(position, indexPosition);
        if (isnan(map_.at(msg.layer, indexPosition)))
        {
            map_.at(msg.layer, indexPosition) = msg.value; 
        } 
        else 
        {
            map_.at(msg.layer, indexPosition) += msg.value;
            map_.at(msg.layer, indexPosition) /= 2.0;
        }
    }

    void EcsDetection::getPose()
    {
        try
        {
            pose_ = transformBuffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    void EcsDetection::inputMapCallback(const nav_msgs::OccupancyGrid &msg)
    {
        if (!converter_.fromOccupancyGrid(msg, BASE_LAYER, map_))
        {
            ROS_ERROR("Could not transform occupancy grid into the GridMap");
        }
        else
        {
            /*ROS_INFO("MAP -- size (%f x %f m), %i x %i cells -- center (%f, %f) -- frame %s", map_.getLength().x(), map_.getLength().y(),
                    map_.getSize()(0), map_.getSize()(1), map_.getPosition().x(), map_.getPosition().y(), map_.getFrameId().c_str());*/
            if (!setup_done_)
                setup_done_ = true;
            EcsDetection::getPose();
        }
    }

    void EcsDetection::publishGridMap()
    {
        ros::Time time = ros::Time::now();
        map_.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap msg;
        converter_.toMessage(map_, msg);
        grid_map_publisher_.publish(msg);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", msg.info.header.stamp.toSec());
    }

    void EcsDetection::setLayerCallback(const grid_map_msgs::GridMap &msg)
    {
        GridMap grid_map_layer = GridMap();
        converter_.fromMessage(msg, grid_map_layer);
        std::vector<std::string> layers = grid_map_layer.getLayers();
        for (int i; i < layers.size(); i++)
        {
            if (layers[i].compare(BASE_LAYER) > 0)
            {
                map_.add(layers[i], grid_map_layer.get(layers[i]));
            }
        }
    }

    void EcsDetection::getLayers(ecs_detection::GetLayers::Request &req, ecs_detection::GetLayers::Response &res)
    {
        std::vector<std::string> layers = map_.getLayers();
        for (std::size_t i = 0; i < layers.size(); ++i)
        {
            res.layers.push_back(layers[i]);
        }
    }

    void EcsDetection::getLayer(ecs_detection::GetLayer::Request &req, ecs_detection::GetLayer::Response &res)
    {
        GridMap map_msg = GridMap();
        grid_map_msgs::GridMap msg;

        if (map_.exists(req.layer))
        {
            std::vector<std::string> layers;
            layers.push_back(BASE_LAYER);
            layers.push_back(req.layer);
            res.valid = true;
            map_msg.addDataFrom(map_, true, true, false, layers);
            converter_.toMessage(map_msg, msg);
            res.map = msg;
        }
        else
        {
            res.valid = false;
        }
    }

} // namespace ecs_detection