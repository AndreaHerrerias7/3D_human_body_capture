#ifndef Z_PARAMETERS_HPP
#define Z_PARAMETERS_HPP

//STL
#include <map>
#include <string>
#include <vector>
#include <auxiliar.h>

//Eigen
#include <eigen3/Eigen/Dense>

//Boost
#include <boost/smart_ptr/shared_ptr.hpp>


/**
 * @brief Store and manage all parameters used along the calibration process.
 *
 * This class provides a convenient way to manage various parameters used
 * in different nodes of the calibration. It includes options for:
 * - Verbosity and visualization
 * - Image matching parameters
 * - Segmentation parameters
 * - Calibration parameters
 * - Sensor topics and initial poses
 */
class Parameters {
public:
    /**
     * @brief Default constructor for the Parameters class.
     * 
     * This constructor initializes members with default values. This is "" for
     * strings, 0.0 for doubles and false for booleans.
     */
    Parameters();

    /**
     * @brief Add a sensor pose to the internal data structures.
     *
     * This function takes a sensor's `topic`, rotation vector `r`, and translation 
     * vector `t`.
     * It converts them to Eigen vectors and creates a 4x4 SE(3) pose matrix.
     * The function then stores the pose matrix and sensor topic in the 
     * `init_pose` and `sensor_topics` member variables, respectively. On top
     * of that it adds the sensors message type to the associated vector
     * `msg_types`.
     *
     * @param topic The topic associated with the sensor.
     * @param msg_type The message type that the sensor sends.
     * @param r The rotation vector (3 elements).
     * @param t The translation vector (3 elements).
     */
    void addSensorPose(std::string topic, std::string msg_type, std::vector<double> r, std::vector<double> t);

    /**
     * @brief Fetch all parameters from YAML file
     *
     * This function fetches all parameters used to fill the `Parameters` object
     * from the given file in `filename`.
     *
     * @param filename
     */
    void loadFromYAML(const std::string& filename);

    /**
     * @brief Generate a map between sensor topics and their corresponding indices.
     *
     * This function iterates through all sensor topics and creates a map
     * where the key is the sensor topic and the value is its corresponding index
     * in the `sensor_topics` and `init_pose` vectors.
     */
    void genSensorIdMap();

    /**
     * @brief Calculate a unique vector index for a given sensor pair based on 
     * their indices.
     * 
     * This function takes two sensor indices (`id_i` and `id_j`) and computes a
     * unique index for every 2-sensor-combination.
     *
     * @param id_i The index of the first sensor.
     * @param id_j The index of the second sensor.
     * @return The unique pair ID for the given sensor pair.
     */
    std::size_t getPairId(std::size_t id_i, std::size_t id_j);

    /**
     * @brief Retrieve the sensor indices from a given pair ID.
     * 
     * This function is the counter part to `getPairId`. For a given 
     * vector_index it calculates the two corresponding sensor topics that, 
     * using `getPairId`, would return the vector_index and saves the IDs in 
     * ´id_i´ and ´id_j´.
     *
     * @param vector_index The pair ID for which to retrieve sensor indices.
     * @param id_i [OUT] The index of the first sensor.
     * @param id_j [OUT] The index of the second sensor.
     */
    void getIndicesFromPairId(std::size_t vector_index, std::size_t& id_i, 
        std::size_t& id_j);

    /**
     * @brief Retrieve the number of image sensors (cameras) in the system.
     *
     * @return number of image sensors (cameras) in the system
     */
    std::size_t getNumImgSensors();

    /**
     * @brief Retrieve the number of depth sensors (RGB-D, LiDAR) in the system.
     *
     * @return number of depth sensors in the system
     */
    std::size_t getNumCloudSensors();


    bool verbose;                   
    
    // Image matching
    double max_time_diff;           /**< Max time difference for image pair consideration */ 

    // Segmentation of point clouds
    bool visualize;        
    double sigma_s;                 /**< Allowed spatial extent for similar points */
    double sigma_r;                 /**< Allowed intensity difference for similar points */
    double max_change_factor;       /**< Max relative depth change for same surface points */
    int smoothing_size;             /**< Number of nearest neighbour points for normal vector computation */
    double inliers_ratio;           /**< Number of points required to form a plane */
    double angular_th;              /**< Max angular difference for points in a plane (radians) */
    double distance_th;             /**< Max perpendicular distance for points in a plane */

    // Segmentation of RGB images
    double fx;                      /**< focal length of the camera along the y-axis */
    double fy;                      /**< focal length of the camera along the y-axis */
    double cx;                      /**< X-coordinate of the principal point/optical center of the camera in pixels */
    double cy;                      /**< Y-coordinate of the principal point/optical center of the camera in pixels */
    std::vector<double> D;          /**< Distortion vector (5 components) of the used RGB camera(s) */
    int checkerboardWidth;          /**< Number of inner corners of the checkerboard(s) used for RGB calibration in first dimension */
    int checkerboardHeight;         /**< Number of inner corners of the checkerboard(s) used for RGB calibration in second dimension */
    double squareSize;              /**< Side length of checkerboard squares in meters */

    // Calibration
    double max_angle;                   /**< Max angle difference for normal vectors in different coordinate systems */
    double max_distance;                /**< Max distance for points considered part of the same plane in different coordinate systems */
    std::string calib_strategy;         /**< Calibration strategy ("standard" or "redundant") */
    std::string algorithm;              /**< Optimization algorithm ("std", "lm", or "lmr") */
    bool error_analysis;                /**< Runs the calibration multiple times (25) with a randomly selected sample of `lim_correspondences` correspondences (for error analysis) */
    std::size_t lim_correspondences;    /**< Limits the values of considered correspondences for the calibration (only for error convergence analysis) */
    
    // Sensor data
    int num_sensors;                                    /**< Number of sensors in the system */
    std::vector<std::string> sensor_topics;             /**< Storing all sensors' topics */
    std::vector<std::string> msg_types;                 /**< Storing all sensors' message types (`Image` or `PointCloud2`)*/
    std::vector<Eigen::Matrix4f> init_pose;             /**< Initial pose of all sensors in a 4x4 matrix using the same order as `sensor_topics` */
    std::map< std::string, std::size_t > sensor_id;     /**< Maps all topics to a number */
};

typedef boost::shared_ptr<Parameters> ParametersPtr;

#endif