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
 * - If `calib_strategy` is set to `'star'` it creates a synchronizer for every 
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
    
private:

    ParametersPtr params_;                      /**< Pointer to parameters. */
    std::optional<ProcessSensorData> tools_;    /**< Helper object that is initialized after fetching the parameters. */


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
     * @param depth_label Label for the first message.
     * @param image_label Label for the second message.
     */
    
    template <typename Msg1, typename Msg2>
    void isOnSync(std::shared_ptr<const Msg1> msg1, std::shared_ptr<const Msg2> msg2, std::string label1, std::string label2) 
    {
        // std::cout << "Message synchronized." << std::endl;
        ObservationPair obs2;
        obs2.first = msg1;
        obs2.second = msg2;

        PlanesList p1, p2;
        // auto start1 = std::chrono::high_resolution_clock::now();
        tools_->segmentPlanes(msg1, p1);
        // auto end1 = std::chrono::high_resolution_clock::now();
        // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
        // std::cout << "Time taken first: " << duration1.count() << " milliseconds, got " << p1.size() << " planes" << std::endl;
        // for (size_t i = 0; i < p1.size(); ++i)
        //     p1[i].print();

        // auto start2 = std::chrono::high_resolution_clock::now();
        tools_->segmentPlanes(msg2, p2);
        // auto end2 = std::chrono::high_resolution_clock::now();
        // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2);
        // std::cout << "Time taken second: " << duration2.count() << " milliseconds, got " << p2.size() << " planes" << std::endl;
        // for (size_t j = 0; j < p2.size(); ++j)
        //     p2[j].print();

        std::cout << std::endl;

        Correspondences correspondences = tools_->findFromPlanesList(obs2, label1, label2, p1, p2, false);

        if (correspondences.size() > 0)
            this->pubMessage(correspondences, label1, label2);
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