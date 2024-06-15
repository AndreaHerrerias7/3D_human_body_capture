#ifndef Z_TYPES_HPP
#define Z_TYPES_HPP

// STL
#include <vector>
#include <variant>

// ROS
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

// Eigen
#include <Eigen/Dense>

/**
 * @brief Class representing a plane in 3D space.
 * 
 * This class represents a plane in the Hesse normal form, i. e. it is
 * determined by its normal vector n and the minimal distance to the origin d 
 * in meters.
 */
class Plane
{
public:
    Eigen::Vector3f n; /**< Normal vector of the plane */
    float d; /**< Distance from the origin along the normal vector */

    /**
     * @brief Default constructor.
     */
    Plane()
        : n(Eigen::Vector3f::Zero()), d(0.f)
    {   }

    /**
     * @brief Parameterized constructor.
     * 
     * @param n Normal vector of the plane.
     * @param d Distance from the origin along the normal vector.
     */
    Plane(const Eigen::Vector3f &n, float d)
        : n(n), d(d)
    {    }

    /**
     * @brief Print plane data
     * 
     * This method prints a planes normal vector and distance, e. g. for
     * debugging purposes.
     */
    void print() 
    {
        std::cout << "n = " << n[0] << ", " << n[1] << ", " << n[2] << ")" << std::endl;
        std::cout << "d = " << d << std::endl;
    }
};

/** @typedef PlanesList
 *  @brief Alias for a list of Plane objects.
 */
typedef std::vector<Plane> PlanesList;

/** @typedef PlaneMatch
 *  @brief Alias for a pair of matched Plane objects.
 */
typedef std::pair<Plane,Plane> PlaneMatch;

/** @typedef Correspondences
 *  @brief Alias for a list of corresponding Plane pairs.
 * 
 * Considering a point cloud pair of two sensors, this type stores all matching
 * pairs of planes found between the two data clouds.
 */
typedef std::vector<PlaneMatch> Correspondences;

/** @typedef CorrespondenceVec
 *  @brief Alias for a list of Correspondences lists.
 * 
 * Every element of the vector holds the Correspondences of a pair of two
 * sensors. Keep in mind that not every element has to be used, depending on the
 * calibration strategy.
 * To determine which element of the vector belongs to which sensor pair, use
 * the helper functions in `Parameters`.
 * 
 */
typedef std::vector<Correspondences> CorrespondenceVec;

/** @typedef ObservationPair
 *  @brief Alias for a pair of ConstSharedPtr to different type of messages.
 * 
 * With this type it is possible to pair two recieved messages conveniently in
 * one variable. They can be of type \n 
 * - sensor_msgs::msg::PointCloud2::ConstSharedPtr
 * - sensor_msgs::msg::Image::ConstSharedPtr
 * 
 */
using ObservationVariant = std::variant<sensor_msgs::msg::PointCloud2::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr>;
using ObservationPair = std::pair<ObservationVariant, ObservationVariant>;

#endif
