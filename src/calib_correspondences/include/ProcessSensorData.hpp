#ifndef Z_PROCESS_SENSOR_DATA_HPP
#define Z_PROCESS_SENSOR_DATA_HPP

// (NEW) 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

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
     */
    ProcessSensorData(const ParametersPtr &params);

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
     */
    void segmentPlanes(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_cloud,
                       PlanesList &planes);

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
     */
    void segmentPlanes(sensor_msgs::msg::Image::ConstSharedPtr msg_image,
                       PlanesList &planes) ;
                       
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
     * @param p1 List of planes from sensor 1.
     * @param p2 List of planes from sensor 2.
     * @param use_initial_est Flag to indicate whether to use initial estimation for correspondence search.
     * @return Correspondences found between the planes of the two sensors.
     */
    Correspondences findFromPlanesList(const ObservationPair &obs2, 
                                       std::string label1,
                                       std::string label2,
                                       const PlanesList &p1, 
                                       const PlanesList &p2, 
                                       bool use_initial_est = true);

protected:
    const ParametersPtr params_; /**< Pointer to parameters object */
    std::size_t outliers; /**< Number of outliers */
    pcl::visualization::PCLVisualizer::Ptr viewer; /**< Pointer to PCL visualizer used for visualizing the segmentation. */
};

#endif
