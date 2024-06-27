#ifndef Z_PROCESS_SENSOR_DATA_HPP
#define Z_PROCESS_SENSOR_DATA_HPP

// (NEW) 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

#include "Parameters.hpp"
#include "Types.hpp"

/**
 * @brief Class for processing a pair sensor messages.
 * 
 * This class contains tools that are helpful, when a pair of two temporally 
 * synchronized messages is to be process to find the coinciding planes 
 * between them.
 */
class ProcessSensorData
{
public:
    /**
     * @brief Constructor for ProcessPointClouds class.
     * 
     * Constructs the objects from a pointer to a parameters struct,
     * which holds configuration settings for the processing algorithms.
     * 
     * @param params Pointer to parameters object.
     * @param node ROS2 node pointer for viualization purposes
     */
    ProcessSensorData(const ParametersPtr &params, rclcpp::Node& node);

    /**
     * @brief Segment planes in a point cloud.
     * 
     * This member function can be called for an incoming point cloud message
     * and segments all planes considering the configuration set in the
     * parameters given during construction. The list of detected planes is
     * stored as list of `Plane` objects in `planes`. \n
     * In case of no detected planes the list stays empty.
     * 
     * @param msg_cloud Pointer to the input point cloud message.
     * @param planes List to store segmented planes.
     * @param id Sensor id of the corresponding sensor (for visualization)
     */
    void segmentPlanes(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_cloud,
                       PlanesList &planes,
                       std::size_t id);

    /**
     * @brief Find checkerboard planes in an image.
     * 
     * This member function can be called for an incoming image message
     * and segments all planes that can be detected from checkerboards 
     * considering the configuration set in the parameters given during 
     * construction.  \n 
     * The list of detected planes is stored as list of `Plane` objects in 
     * `planes`. In case of no detected planes the list stays empty.
     * 
     * @param msg_cloud Pointer to the input image message.
     * @param planes List to store segmented planes.
     * @param id Sensor id of the corresponding sensor (for visualization)
     */
    void segmentPlanes(sensor_msgs::msg::Image::ConstSharedPtr msg_image,
                       PlanesList &planes,
                       std::size_t id) ;
                       
    /**
     * @brief Find correspondences between two sets of planes.
     * 
     * Once a pair of two point clouds has been segmented by 
     * `ProcessPointClouds::segmentPlanes()`, the results can be used to search
     * the output, the `PlanesList` vectors `p1` and `p2` using the initial
     * estimation of the sensor positions given in the parameters.  \n
     * The pairs of planes found among the two lists are returned in a list of 
     * pairs of planes (`Correspondences`).
     * 
     * @param obs2 Pair of point cloud messages representing observations from two sensors.
     * @param topic1 Topic of the first message in the `obs2` pair.
     * @param topic2 Topic of the second message in the `obs2` pair.
     * @param p1 List of planes from sensor 1.
     * @param p2 List of planes from sensor 2.
     * @param use_initial_est Flag to indicate whether to use initial estimation for correspondence search.
     * @return Correspondences found between the planes of the two sensors.
     */
    Correspondences findFromPlanesList(const ObservationPair &obs2, 
                                       std::string topic1,
                                       std::string topic2,
                                       const PlanesList &p1, 
                                       const PlanesList &p2, 
                                       bool use_initial_est = true);


protected:

    /**
     * @brief Publish a marker for a plane in a point cloud for visualization.
     * 
     * This function publishes a marker that is supposed to be interpreted as 
     * normal vector of a plane to visualize it in RViz2 subscribing to the 
     * corresponding topic.
     * 
     * @param n Vector (usually normal vector of a plane) that is to visualize.
     * @param offset Offset point wheree the vector should start.
     * @param id Sensor id indicating the point cloud the plane was found in.
     * @param r, g, b (optional) color for the vector, set to white by default. 
     */
    void publishPlaneMarker(Eigen::Vector3f n, Eigen::Vector3f offset, std::size_t id, int r = 255, int g = 255, int b = 255);

    /**
     * @brief Delete all markers on the topic the marker is published to.
     * 
     * This function is used to clean up the RViz2 view. Each time a new point
     * cloud is segmented the old markers should disappear.
     * 
     * @param id ID of the sensor the point cloud was taken from.
     */
    void deleteAllMarkers(std::size_t id);


    const ParametersPtr params_;    /**< Pointer to parameters object */
    std::size_t outliers;           /**< Number of outliers */

    rclcpp::Node& node_;                    /**< ROS node for visualization */  
    std::vector<int> marker_id_counter;     /**< Counter to assign every marker message a unique id when publishing */
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> vis_planes_pc_;    /**< Publisher to visualize the detected planes in point clouds */
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> vis_planes_img_;         /**< Publisher to visualize the detected planes in images */
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> vis_marker_;     /**< Markers for the point cloud planes that a correspondence in another message was found for. */

};

#endif
