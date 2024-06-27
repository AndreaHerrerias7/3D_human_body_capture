#include <cstddef>
#include <cstdio>
#include <functional>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <auxiliar.h>

#include "ImageCloud2Correspondences.hpp"
#include "ProcessSensorData.hpp"
#include "Calibration.hpp"
#include "custom_msgs/msg/plane.hpp"
#include "custom_msgs/msg/plane_match.hpp"
#include "custom_msgs/msg/correspondences.hpp"


ImageCloud2Correspondences::ImageCloud2Correspondences() :
        Node("imagecloud2correspondences")
{
    params_.reset(new Parameters());
    std::string path = ament_index_cpp::get_package_share_directory("calib_correspondences") + "/config/parameters.yaml";
    params_->loadFromYAML(path);

//----------------------------------------------------------------------------//

    // Initialize tools from fetched parameters
    tools_.emplace(params_, *this);

    // Initialize synchronizers
    rclcpp::Duration time_diff = rclcpp::Duration::from_seconds(params_->max_time_diff);
    std::string policy;
    
    if (params_->calib_strategy == "standard") 
    {
        for (int i = 1; i < params_->num_sensors; ++i)
        {   
            if (params_->msg_types[0] == "PointCloud2" && params_->msg_types[i] == "Image")
            {
                using PairType = SyncPair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;
                PairType pair(params_->sensor_topics[0], params_->sensor_topics[i], this);
                syncs_.push_back(std::move(pair));
                auto callback = std::bind(&ImageCloud2Correspondences::isOnSync<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>, 
                                            this, 
                                            std::placeholders::_1, 
                                            std::placeholders::_2, 
                                            params_->sensor_topics[0], 
                                            params_->sensor_topics[i]);
                std::get<PairType>(syncs_.back()).sync->registerCallback(callback);
                std::get<PairType>(syncs_.back()).sync->setMaxIntervalDuration(time_diff);
                policy = "Cloud_Image";
            }
            else if (params_->msg_types[0] == "PointCloud2" && params_->msg_types[i] == "PointCloud2")
            {
                using PairType = SyncPair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
                PairType pair(params_->sensor_topics[0], params_->sensor_topics[i], this);
                syncs_.push_back(std::move(pair));
                auto callback = std::bind(&ImageCloud2Correspondences::isOnSync<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>, 
                                            this, 
                                            std::placeholders::_1, 
                                            std::placeholders::_2, 
                                            params_->sensor_topics[0], 
                                            params_->sensor_topics[i]);
                std::get<PairType>(syncs_.back()).sync->registerCallback(callback);
                std::get<PairType>(syncs_.back()).sync->setMaxIntervalDuration(time_diff);
                policy = "Cloud";
            }
            else if (params_->msg_types[0] == "Image" && params_->msg_types[i] == "Image")
            {
                using PairType = SyncPair<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
                PairType pair(params_->sensor_topics[0], params_->sensor_topics[i], this);
                syncs_.push_back(std::move(pair));
                auto callback = std::bind(&ImageCloud2Correspondences::isOnSync<sensor_msgs::msg::Image, sensor_msgs::msg::Image>, 
                                            this, 
                                            std::placeholders::_1, 
                                            std::placeholders::_2, 
                                            params_->sensor_topics[0], 
                                            params_->sensor_topics[i]);
                std::get<PairType>(syncs_.back()).sync->registerCallback(callback);
                std::get<PairType>(syncs_.back()).sync->setMaxIntervalDuration(time_diff);
                policy = "Image";
            }
            std::cout << "Created synchronizer with policy type: " << policy << std::endl;
        }
    }

    else if (params_->calib_strategy == "redundant")
    {
        for (std::size_t i = 0; i < params_->sensor_topics.size(); ++i)
        {   
            for (std::size_t j = i + 1; j < params_->sensor_topics.size(); ++j)
            {
                if (params_->msg_types[i] == "PointCloud2")
                {
                    if (params_->msg_types[j] == "PointCloud2")
                    { //PC - PC
                       using PairType = SyncPair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
                        PairType pair(params_->sensor_topics[i], params_->sensor_topics[j], this);
                        syncs_.push_back(std::move(pair));
                        auto callback = std::bind(&ImageCloud2Correspondences::isOnSync<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>, 
                                                    this, 
                                                    std::placeholders::_1, 
                                                    std::placeholders::_2, 
                                                    params_->sensor_topics[i], 
                                                    params_->sensor_topics[j]);
                        std::get<PairType>(syncs_.back()).sync->registerCallback(callback);
                        std::get<PairType>(syncs_.back()).sync->setMaxIntervalDuration(time_diff);
                        policy = "Cloud"; 
                    }
                    else if (params_->msg_types[j] == "Image")
                    { // PC - IMG
                        using PairType = SyncPair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;
                        PairType pair(params_->sensor_topics[i], params_->sensor_topics[j], this);
                        syncs_.push_back(std::move(pair));
                        auto callback = std::bind(&ImageCloud2Correspondences::isOnSync<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>, 
                                                    this, 
                                                    std::placeholders::_1, 
                                                    std::placeholders::_2, 
                                                    params_->sensor_topics[i], 
                                                    params_->sensor_topics[j]);
                        std::get<PairType>(syncs_.back()).sync->registerCallback(callback);
                        std::get<PairType>(syncs_.back()).sync->setMaxIntervalDuration(time_diff);
                        policy = "Cloud_Image";
                    }
                    else
                        std::cerr << params_->msg_types[j] << "is not a valid message type." << std::endl;
                }
                else if (params_->msg_types[i] == "Image")
                {
                    if (params_->msg_types[j] == "PointCloud2")
                    { //IMG - PC (should not happen because of order of senors)
                        std::cerr << "Please order the sensors in the parameters file. Put PointCloud2 sensors before putting Image sensors." << std::endl;
                    }
                    else if (params_->msg_types[j] == "Image")
                    { // IMG - IMG
                        using PairType = SyncPair<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
                        PairType pair(params_->sensor_topics[i], params_->sensor_topics[j], this);
                        syncs_.push_back(std::move(pair));
                        auto callback = std::bind(&ImageCloud2Correspondences::isOnSync<sensor_msgs::msg::Image, sensor_msgs::msg::Image>, 
                                                    this, 
                                                    std::placeholders::_1, 
                                                    std::placeholders::_2, 
                                                    params_->sensor_topics[i], 
                                                    params_->sensor_topics[j]);
                        std::get<PairType>(syncs_.back()).sync->registerCallback(callback);
                        std::get<PairType>(syncs_.back()).sync->setMaxIntervalDuration(time_diff);
                        policy = "Image";
                    }
                    else
                        std::cerr << params_->msg_types[j] << "is not a valid message type." << std::endl;
                }
                else
                    std::cerr << params_->msg_types[i] << "is not a valid message type." << std::endl;
                
                std::cout << "Created synchronizer (" << i << ", " << j << ") with policy type: " << policy << std::endl;
            }
        }
    }

    publisher_ = this->create_publisher<custom_msgs::msg::Correspondences>("/custom_msgs/msg/correspondences", 10);

    // One element for every sensor pair to count the correspondences
    int N = params_->num_sensors;
    corr_counter.resize(params_->calib_strategy == "redundant" ? (N * (N - 1) / 2) : (N - 1));


    // Visualization (initialize all publishers)
    // std::size_t img_counter = params_->getNumImgSensors();
    // tools_->vis_planes_img_.resize(img_counter);
    // std::size_t pc2_counter = params_->getNumCloudSensors();
    // tools_->vis_planes_pc_.resize(pc2_counter);
    // tools_->vis_marker_.resize(pc2_counter);

    // for (std::size_t i = 0; i < img_counter; ++i)
    //     tools_->vis_planes_img_[i] = this->create_publisher<sensor_msgs::msg::Image>("/visualize/segmented_image" + std::to_string(i), 10);
    // for (std::size_t j = 0; j < pc2_counter; ++j)
    // {
    //     tools_->vis_planes_pc_[j] = this->create_publisher<sensor_msgs::msg::PointCloud2>("/visualize/segmented_cloud" + std::to_string(j), 10);
    //     tools_->vis_marker_[j] = this->create_publisher<visualization_msgs::msg::Marker>("/visualize/cloud_marker" + std::to_string(j), 10);
    // }
}


ImageCloud2Correspondences::~ImageCloud2Correspondences() 
{
    std::size_t id_i, id_j;
    std::cout << "----- Total published correspondences: -----" << std::endl;
    for (std::size_t idx = 0; idx < corr_counter.size(); ++idx) 
    {
        if (corr_counter[idx] > 0) 
        {
            params_->getIndicesFromPairId(idx, id_i, id_j);
            std::cout << "(" << id_i << ", " << id_j << "):\t" << corr_counter[idx] << std::endl;
        }
    }
}


void ImageCloud2Correspondences::pubMessage(Correspondences correspondences,
                                            std::string first_label,
                                            std::string second_label)
{
    custom_msgs::msg::Plane plane1;
    custom_msgs::msg::Plane plane2;
    custom_msgs::msg::PlaneMatch match;
    custom_msgs::msg::Correspondences message;

    for (std::size_t i = 0; i < correspondences.size(); ++i)
    {
        // Prepare Plane message
        plane1.d = correspondences[i].first.d;
        plane1.n.x = correspondences[i].first.n[0];
        plane1.n.y = correspondences[i].first.n[1];
        plane1.n.z = correspondences[i].first.n[2];

        plane2.d = correspondences[i].second.d;
        plane2.n.x = correspondences[i].second.n[0];
        plane2.n.y = correspondences[i].second.n[1];
        plane2.n.z = correspondences[i].second.n[2];

        // Put planes into Match
        match.first = plane1;
        match.second = plane2;

        message.correspondences.push_back(match);
    }
    message.first_label = first_label;
    message.second_label = second_label;
    std::cout << "Successfully published " << correspondences.size() << " plane correspondences." << std::endl;
    std::cout << "Topics: " << first_label << ", " << second_label << std::endl << std::endl;
    publisher_->publish(message);

    std::size_t first_id = params_->sensor_id[first_label];
    std::size_t second_id = params_->sensor_id[second_label];
    ++corr_counter[params_->getPairId(first_id, second_id)];
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<ImageCloud2Correspondences>();
    rclcpp::spin(ros_node);
    rclcpp::shutdown();
    return 0;
}