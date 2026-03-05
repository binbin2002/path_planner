#pragma once

#include <ros/ros.h>
#include <filesystem>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>

#include <std_srvs/Trigger.h>
// #include <zouyu_srvs/ChangeMap.h>
#include <yaml-cpp/yaml.h>


#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

struct NodeConfig
{
    std::string map_topic = "/map";
    std::string map_frame_id = "map";
    std::string metadata_topic = "/map_metadata";
    std::string map_dir = "/home/lion/config/maps";

    // 若为true：按ROS map_server标准解释yaml：occupied_thresh/free_thresh为0~1，且中间区域输出unknown
    // 若为false：保持当前实现：阈值按0~255灰度解释
    bool compat_map_server_yaml = true;
    // 是否对介于free_thresh与occupied_thresh之间区域输出unknown(-1)
    bool output_unknown = true;
    // unknown栅格输出值，nav_msgs/OccupancyGrid 约定为 -1
    int8_t unknown_value = -1;
};

struct NodeState
{
    std::string yaml_name = "";
    nav_msgs::OccupancyGrid current_map;
    nav_msgs::MapMetaData current_metadata;
};

class MapManagerROS
{
public:
    MapManagerROS() = delete;

    MapManagerROS(const MapManagerROS &) = delete;

    MapManagerROS &operator=(const MapManagerROS &) = delete;

    MapManagerROS(ros::NodeHandle& nh);

    void loadParameters();

    void initPublishers();

    void initServices();

    bool loadMap();

    void publishMap();
    
    // bool changeMapService(zouyu_srvs::ChangeMap::Request &req, zouyu_srvs::ChangeMap::Response &res);

    bool reloadMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    
    bool currentMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    

private:
    ros::NodeHandle m_nh;
    ros::Publisher m_map_pub;
    ros::Publisher m_metadata_pub;
    // ros::ServiceServer m_change_map_srv;
    ros::ServiceServer m_reload_map_srv;
    ros::ServiceServer m_current_map_srv;
    NodeConfig m_config;
    NodeState m_state;
};
