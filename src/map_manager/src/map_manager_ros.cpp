#include "map_manager_ros.h"

MapManagerROS::MapManagerROS(ros::NodeHandle &nh) : m_nh(nh)
{
    loadParameters();
    initPublishers();
    initServices();
    if (loadMap())
    {
        ROS_DEBUG("[MapManagerROS] Map loaded");
        publishMap();
    }
}

void MapManagerROS::loadParameters()
{
    ROS_DEBUG("[MapManagerROS] Loading parameters");
    m_nh.param<std::string>("map_topic", m_config.map_topic, "/map");
    m_nh.param<std::string>("map_frame_id", m_config.map_frame_id, "map");
    m_nh.param<std::string>("metadata_topic", m_config.metadata_topic, "/map_metadata");
    m_nh.param<std::string>("map_dir", m_config.map_dir, "/home/lion/config/maps");
    m_nh.param<std::string>("yaml_name", m_state.yaml_name, "map");

    // 与map_server对齐：默认按标准yaml语义(阈值0~1)并输出unknown(-1)
    m_nh.param<bool>("compat_map_server_yaml", m_config.compat_map_server_yaml, true);
    m_nh.param<bool>("output_unknown", m_config.output_unknown, true);

    int unknown_value_param = static_cast<int>(m_config.unknown_value);
    m_nh.param<int>("unknown_value", unknown_value_param, -1);
    if (unknown_value_param < -1)
        unknown_value_param = -1;
    if (unknown_value_param > 100)
        unknown_value_param = 100;
    m_config.unknown_value = static_cast<int8_t>(unknown_value_param);
}

void MapManagerROS::initPublishers()
{
    m_map_pub = m_nh.advertise<nav_msgs::OccupancyGrid>(m_config.map_topic, 1, true);
    m_metadata_pub = m_nh.advertise<nav_msgs::MapMetaData>(m_config.metadata_topic, 1, true);
}

void MapManagerROS::initServices()
{
    // m_change_map_srv = m_nh.advertiseService("change_map", &MapManagerROS::changeMapService, this);
    m_reload_map_srv = m_nh.advertiseService("reload_map", &MapManagerROS::reloadMapService, this);
    m_current_map_srv = m_nh.advertiseService("current_map", &MapManagerROS::currentMapService, this);
}

void MapManagerROS::publishMap()
{
    ros::Time::waitForValid();
    m_state.current_map.header.stamp = ros::Time::now();
    m_state.current_map.info.map_load_time = ros::Time::now();
    m_state.current_map.header.frame_id = m_config.map_frame_id;
    m_map_pub.publish(m_state.current_map);
    m_state.current_metadata.map_load_time = ros::Time::now();
    m_metadata_pub.publish(m_state.current_metadata);
}

//按照ROS map_server的yaml格式，从YAML+图片文件中加载地图数据，并转换为ROS OccupancyGrid消息格式    
bool MapManagerROS::loadMap()
{
    //1.组织并检查yaml文件路径
    std::filesystem::path map_dir(m_config.map_dir);
    std::filesystem::path yaml_file = map_dir / (m_state.yaml_name + ".yaml");
    if (!std::filesystem::exists(yaml_file))
    {
        ROS_WARN("[MapManagerROS] Map file %s does not exist", yaml_file.c_str());
        return false;
    }
    ROS_DEBUG("[MapManagerROS] Loading map from %s", yaml_file.c_str());

    YAML::Node yaml = YAML::LoadFile(yaml_file);

    //2.组织并检查图片文件路径，加载图片数据
    const std::string image_name = yaml["image"].as<std::string>();
    std::filesystem::path image_path(image_name);

    // map_server规则：image字段若为相对路径，则相对yaml文件所在目录解析；若为绝对路径则直接使用
    std::filesystem::path image_file = image_path.is_absolute() ? image_path : (yaml_file.parent_path() / image_path);

    if (!std::filesystem::exists(image_file))
    {
        ROS_WARN("[MapManagerROS] Image file %s does not exist", image_file.c_str());
        return false;
    }

    cv::Mat image = cv::imread(image_file.c_str(), cv::IMREAD_GRAYSCALE);
    if (image.empty())
    {
        ROS_WARN("[MapManagerROS] Failed to load image file %s", image_file.c_str());
        return false;
    }

    //3.读取阈值和选项（map_server标准yaml：occupied_thresh/free_thresh为0~1）
    const double occ_th = yaml["occupied_thresh"].as<double>();
    const double free_th = yaml["free_thresh"].as<double>();

    bool negate = false;
    if (yaml["negate"])
    {
        try
        {
            negate = yaml["negate"].as<bool>();
        }
        catch (const YAML::BadConversion &)
        {
            // 兼容 map_server 常见写法：negate: 0/1
            negate = (yaml["negate"].as<int>() != 0);
        }
    }

    if (occ_th < free_th)
    {
        ROS_WARN("[MapManagerROS] occupied_thresh(%.3f) < free_thresh(%.3f), please check yaml: %s", occ_th, free_th, yaml_file.c_str());
    }

    //4.读取地图元数据，转换坐标系，并初始化OccupancyGrid消息
    std::vector<double> origin = yaml["origin"].as<std::vector<double>>();
    m_state.current_metadata.resolution = yaml["resolution"].as<float>();
    m_state.current_metadata.width = image.cols;
    m_state.current_metadata.height = image.rows;
    m_state.current_metadata.origin.position.x = origin[0];
    m_state.current_metadata.origin.position.y = origin[1];
    m_state.current_metadata.origin.position.z = 0.0;
    tf::Quaternion q = tf::createQuaternionFromYaw(origin[2]);

    m_state.current_metadata.origin.orientation.x = q.x();
    m_state.current_metadata.origin.orientation.y = q.y();
    m_state.current_metadata.origin.orientation.z = q.z();
    m_state.current_metadata.origin.orientation.w = q.w();
    m_state.current_map.info = m_state.current_metadata;
    m_state.current_map.data.resize(m_state.current_metadata.width * m_state.current_metadata.height);
    //5. 把灰度图像素值转换为ROS OccupancyGrid的占用概率值，注意ROS OccupancyGrid的坐标系与图像坐标系的差异（y轴翻转）
    for (int i = 0; i < m_state.current_metadata.height; i++)
    {
        for (int j = 0; j < m_state.current_metadata.width; j++)
        {
            int gray = static_cast<int>(image.at<uchar>(i, j)); // 0~255
            if (negate)
                gray = 255 - gray;

            // map_server语义：灰度越小越占用；先转换为[0,1]占用概率
            const double occ_prob = (255.0 - static_cast<double>(gray)) / 255.0;

            int8_t out;
            if (occ_prob > occ_th)
                out = 100;
            else if (occ_prob < free_th)
                out = 0;
            else
                out = m_config.output_unknown ? m_config.unknown_value : m_config.unknown_value;

            m_state.current_map.data[MAP_IDX(m_state.current_metadata.width, j, m_state.current_metadata.height - i - 1)] = out;
        }
    }

    return true;
}

// bool MapManagerROS::changeMapService(zouyu_srvs::ChangeMap::Request &req, zouyu_srvs::ChangeMap::Response &res)
// {
//     m_state.yaml_name = req.name;
//     if (loadMap())
//     {
//         publishMap();
//         res.success = true;
//         res.message = "Map changed to " + req.name + ".yaml";
//         return true;
//     }
//     res.success = false;
//     res.message = "Failed to change map to " + req.name + ".yaml";
//     return false;
// }

bool MapManagerROS::reloadMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (loadMap())
    {
        publishMap();
        res.success = true;
        res.message = "Map reloaded " + m_state.yaml_name + ".yaml";
        return true;
    }
    res.success = false;
    res.message = "Failed to reload map " + m_state.yaml_name + ".yaml";
    return false;
}

bool MapManagerROS::currentMapService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.success = true;
    res.message = m_state.yaml_name;
    return true;
}
