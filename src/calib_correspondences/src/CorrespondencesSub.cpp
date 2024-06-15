#include <cstdio>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <auxiliar.h>

#include "CorrespondencesSub.hpp"
#include "Types.hpp"
#include "Calibration.hpp"


CorrespondencesSub::CorrespondencesSub() : Node("correspondences_subscriber")
{
    subscription_ = this->create_subscription<custom_msgs::msg::Correspondences>(
        "/custom_msgs/msg/correspondences", 10, std::bind(&CorrespondencesSub::rcvMessage, this, _1));
    params_.reset(new Parameters());

    std::string path = ament_index_cpp::get_package_share_directory("calib_correspondences") + "/config/parameters.yaml";
    params_->loadFromYAML(path);

    // Creat one field for every possible 2-sensor-combination
    std::size_t num_sensors = params_->sensor_topics.size();
    all_correspondences.resize(num_sensors*(num_sensors - 1) / 2);  

    // Initialize map to identify every sensor with a std::size_t
    params_->genSensorIdMap();
}


CorrespondencesSub::~CorrespondencesSub() 
{
    startCalib();
}


void CorrespondencesSub::rcvMessage(const custom_msgs::msg::Correspondences &msg)
{
    PlaneMatch match;
    Plane p1, p2;
    Correspondences correspondences;

    std::size_t id_first = params_->sensor_id[msg.first_label];
    std::size_t id_second = params_->sensor_id[msg.second_label];

    for(std::size_t i = 0; i < msg.correspondences.size(); ++i)
    {
        // Extract data from first plane
        p1.d = msg.correspondences[i].first.d;
        p1.n[0] = msg.correspondences[i].first.n.x;
        p1.n[1] = msg.correspondences[i].first.n.y;
        p1.n[2] = msg.correspondences[i].first.n.z;

        // From second
        p2.d = msg.correspondences[i].second.d;
        p2.n[0] = msg.correspondences[i].second.n.x;
        p2.n[1] = msg.correspondences[i].second.n.y;
        p2.n[2] = msg.correspondences[i].second.n.z;

        // Put back together into match
        if (id_first > id_second)
        {
            match.first = p2;
            match.second = p1;
        } 
        else if (id_first < id_second)
        {
            match.first = p1;
            match.second = p2;
        }
        else std::cerr << "Sensor labels in custom_msgs::msg::Correspondences msg must be different." << std::endl;

        correspondences.push_back(match);

    }
    std::size_t index = params_->getPairId(id_first, id_second);
    all_correspondences[index].insert(all_correspondences[index].end(), correspondences.begin(), correspondences.end()); 
    
    if (all_correspondences[index].size() + correspondences.size() > params_->lim_correspondences)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Limit of " << params_->lim_correspondences << " correspondences reached for (" << id_first << ", " << id_second << ").");
    }
    else 
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Recieved " << msg.correspondences.size()<< " correspondences for sensor pair (" << id_first << ", " << id_second << ").");
    }
    
}


void CorrespondencesSub::startCalib()
{
    if (params_->verbose)
    {
        std::cout << std::endl << "----- Total correspondences: -----" << std::endl;
        std::size_t id_first, id_second;
        for (std::size_t i = 0; i < all_correspondences.size(); ++i) 
        {
            params_->getIndicesFromPairId(i, id_first, id_second);
            std::cout << std::endl << "(" << id_first << ", " << id_second << "): \t" << all_correspondences[i].size() << std::endl;
        }
    }

    Calibration calibration(params_, all_correspondences);

    // Run the different calibrations
    std::vector<Eigen::Matrix4f> results = calibration.runAllCalibrations();
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<CorrespondencesSub>();
    rclcpp::spin(ros_node);
    rclcpp::shutdown();
    return 0;
}