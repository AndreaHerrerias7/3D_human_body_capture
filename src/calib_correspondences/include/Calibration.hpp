#ifndef Z_CALIB_CORRESPONDENCES_HPP
#define Z_CALIB_CORRESPONDENCES_HPP

#include <string>
#include <vector>
#include <map>
#include <cstddef>

#include "Parameters.hpp"
#include "Types.hpp"

/**
 * @brief Class for performing the calibration operations.
 */
class Calibration
{
public:
    /**
     * @brief Constructor for the Calibration class.
     * 
     * @param params Pointer to the Parameters object with the values provided 
     *  by the launch file (only calibration and sensor parameters are used).
     * @param corr_vec Vector of correspondences between different sensors.
     */
    Calibration(const ParametersPtr &params, CorrespondenceVec& corr_vec);
    
    /**
     * @brief Run calibrations according to `calib_strategy` and `algorithm`.
     * 
     * If `calib_strategy` is set to "standard", the calibration is run for every
     * `params->sensor_id` combination 0-i (i = 1, ..., number of sensors - 1),
     * i. e. all sensors are calibrated with respect to the reference sensor 
     * with ID 0. Mind that this requires data for every of these combinations 
     * in `corr_vec`. \n
     * For example in case of a 4-sensor system this requires correspondences 
     * entries in `corr_vec` in the fields associated with \n
     * 0 - 1 \n
     * 0 - 2 \n
     * 0 - 3 \n
     * where sensor 0 is the reference. \n
     * If `calib_strategy` is set to "standard", the calibration is run for every 
     * possible 2-sensor combination in the system. This is kind of redundant,
     * but might be interesting for future purposes. Apparently, to run this 
     * type of calibration, `corr_vec` needs entries in every field (i. e. for 
     * all sensor combinations). \n
     * The choice of the algorithm can be \n
     *          'std':          non-iterative \n
	 *			'lm':		    Levenberg-Marquardt \n
	 *			'lm_robust':	Levenberg-Marquardt (robust version) \n
     *			'gn':		    Gauss-Newton \n
	 *			'gn_robust':   	Gauss-Newton (robust version)
     * 
     * @return All transformation matrices obtained in the calibration.
     */
    std::vector<Eigen::Matrix4f> runAllCalibrations();

    /**
     * @brief Run a calibration operation between two sensors.
     * 
     * This method runs just one calibration between the sensors with 
     * `params->sensor_id` `rel_id` and `ref_id`. It requires data in the
     * field of `corr_vec` associated to this sensor combination.
     * 
     * @param rel_id Relative ID of the target frame.
     * @param ref_id Reference ID of the source frame.
     * @return Transformation matrix.
     */
    Eigen::Matrix4f runCalibrationSTD(std::size_t rel_id, std::size_t ref_id);

    /**
     * @brief Run a calibration between two sensors using Gauss-Newton 
     *  optimization.
     * 
     * This method runs just one calibration between the sensors with 
     * `params->sensor_id` `rel_id` and `ref_id` using the Gauss-Newton 
     * algorithm. \n
     * It expects data in the element of `corr_vec` associated to this sensor 
     * combination, but is not necessary (sensor combination is skipped in case
     * no data is provided).
     * 
     * @param id_i ID of the first frame.
     * @param id_j ID of the second frame.
     * @param robust Use robust estimation (optional, default: true).
     * @return Transformation matrix.
     */
    Eigen::Matrix4f runCalibrationGN(std::size_t id_i, std::size_t id_j, 
        bool robust = true);

    /**
     * @brief Run a calibration between two sensors using Levenberg-Marquardt
     *  optimization.
     * 
     * This method runs just one calibration between the sensors with 
     * `params->sensor_id` `rel_id` and `ref_id` using the Levenberg-Marquardt 
     * algorithm. \n
     * It requires data in the element of `corr_vec` associated to this sensor 
     * combination.
     * 
     * @param id_i ID of the first frame.
     * @param id_j ID of the second frame.
     * @param robust Use robust estimation (optional, default: true).
     * @return Transformation matrix.
     */
    Eigen::Matrix4f runCalibrationLM(std::size_t id_i, std::size_t id_j, 
        bool robust = true);

    Eigen::Matrix4d Tini; /**< Initial estimation iterative solutions */
    CorrespondenceVec& all_matches; /**< Vector in which every element contains Correspondences for a 2-sensor-combination. */
    
protected:
    ParametersPtr params_; /**< Pointer to the Parameters object */

    /**
     * @brief Perform rotation calibration between two frames.
     * 
     * This method is used internally by `runCalibrationSTD` to perform the 
     * calibration of the rotation matrix of the sensor positions of `id_i` and 
     * `id_j`.
     * 
     * @param id_i ID of the first frame.
     * @param id_j ID of the second frame.
     * @return Rotation matrix.
     */
    Eigen::Matrix3f calibrateRotation(std::size_t id_i, std::size_t id_j);

    /**
     * @brief Perform translation calibration between two frames.
     * 
     * This method is used internally by `runCalibrationSTD` to perform the 
     * calibration of the translation vector of the sensor positions of `id_i` 
     * and `id_j`.
     * 
     * @param id_i ID of the first frame.
     * @param id_j ID of the second frame.
     * @return Translation vector.
     */
    Eigen::Vector3f calibrateTranslation(std::size_t id_i, std::size_t id_j);

    /**
     * @brief Compute the robust kernel function for robust estimation.
     * 
     * @param e Error value.
     * @return Robust kernel value.
     */
    double robustKernel(double e);
};

#endif