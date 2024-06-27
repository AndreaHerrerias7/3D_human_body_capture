#ifndef Z_IMAGE_CLOUD_2_CORRESPONDENCES_HPP
#define Z_IMAGE_CLOUD_2_CORRESPONDENCES_HPP

#include <cstdio>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "Parameters.hpp"
#include "ProcessSensorData.hpp"
#include "Types.hpp"
#include "custom_msgs/msg/correspondences.hpp"
#include "SyncPair.hpp"

using CloudSyncPair = SyncPair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
using ImageSyncPair = SyncPair<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
using CloudImageSyncPair = SyncPair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Class for handling a ROS2 node that processes point cloud and image data.
 * 
 * This class subscribes to multiple point cloud and/or iamge topics and
 * synchronizes/pairs them in the specified way (depending on the parameters):
 * - If `calib_strategy` is set to `'standard'` it creates a synchronizer for every 
 * sensor with the first one (which is considered as reference).
 * - If `calib_strategy` is set to `'redundant'` a synchronizer for every possible 
 * sensor pair among the given sensors is created.
 * In the callback (that means if synchronized sensor data for a specific sensor
 * pair is available), the planes are extracted from the image/point cloud. 
 * All coinciding plane pairs are published in a custom message format 
 * `custom_msgs::msg::correspondences` to the same-named topic 
 * `/custom_msgs/msg/correspondences`.
 */
class ImageCloud2Correspondences : public rclcpp::Node {

public:
    /**
     * @brief Constructor for ImageCloud2Correspondences.
     */
    ImageCloud2Correspondences();

    /**
     * @brief Destructor for ImageCloud2Correspondences with output info.
     */
    ~ImageCloud2Correspondences();

    std::optional<ProcessSensorData> tools_;    /**< Helper object that is initialized after fetching the parameters. */
    
private:

    ParametersPtr params_;                      /**< Pointer to parameters. */
    std::vector<std::size_t> corr_counter;      /**< Vector to count correspondences for each pair. */

    std::vector<std::variant<CloudSyncPair, ImageSyncPair, CloudImageSyncPair>> syncs_; /**< Vector of synchronizers for every PointCloud2/Image pair. */
    rclcpp::Publisher<custom_msgs::msg::Correspondences>::SharedPtr publisher_;         /**< Correspondences publisher. */

    /**
     * @brief Callback function for synchronized point cloud and image data.
     * 
     * This callback is registered when synchronized point cloud and/or image 
     * data are received.
     * It processes the data and publishes the correspondences.
     * 
     * @param msg1 Pointer to the first message.
     * @param msg2 Pointer to the second message.
     * @param topic1 Topic of the first message.
     * @param topic2 Topic of the second message.
     */
    
    template <typename Msg1, typename Msg2>
    void isOnSync(std::shared_ptr<const Msg1> msg1, std::shared_ptr<const Msg2> msg2, std::string topic1, std::string topic2) 
    {
        ObservationPair obs2;
        obs2.first = msg1;
        obs2.second = msg2;

        PlanesList p1, p2;
        tools_->segmentPlanes(msg1, p1, params_->sensor_id[topic1]);
        if (!p1.empty())
            tools_->segmentPlanes(msg2, p2, params_->sensor_id[topic2]);

        std::cout << std::endl;

        Correspondences correspondences = tools_->findFromPlanesList(obs2, topic1, topic2, p1, p2, false);
        
        if (correspondences.size() > 0)
            this->pubMessage(correspondences, topic1, topic2);
    }

    /**
     * @brief Publishes correspondences.
     * 
     * This method publishes the correspondences between point cloud and image data.
     * 
     * @param correspondences Correspondences between the point cloud and image data.
     * @param depth_label Label for the point cloud data.
     * @param image_label Label for the image data.
     */
    void pubMessage(Correspondences correspondences,
                    std::string depth_label,
                    std::string image_label);
            
};

#endif