#ifndef Z_CORRESPONDENCES_SUB_HPP
#define Z_CORRESPONDENCES_SUB_HPP

// #include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/correspondences.hpp"
#include "Types.hpp"
#include "Parameters.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Class for subscribing to correspondences messages.
 * 
 * This class can create a ROS2 node that subscribes to the topic 
 * `/custom_msgs/msg/correspondences` and stores the recieved data in the member
 * `all_correspondences`. 
 * To stop the reading process and therefore the accumulation of data, press
 * <kbd>Ctrl</kbd>+<kbd>C</kbd>. Then the calibration is run according to the
 * parameters fetched from the launch file.
 * 
 */
class CorrespondencesSub : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the CorrespondencesSub class.
     */
    CorrespondencesSub();

    /**
     * @brief Destructor for the CorrespondencesSub class.
     * 
     * This destructor automatically starts the calibration. It is called as
     * the user interrupts the data stream by <kbd>Ctrl</kbd>+<kbd>C</kbd>.
     */
    ~CorrespondencesSub();

private:
    CorrespondenceVec all_correspondences; /**< Vector to store `Correspondences` for every sensor combination */
    ParametersPtr params_; /**< Pointer to the Parameters object fetching all values automatically from the launch file. */

    rclcpp::Subscription<custom_msgs::msg::Correspondences>::SharedPtr subscription_; /**< Subscription to the messages from topic `/custom_msgs/msg/correspondences`. */

    /**
     * @brief Callback function to receive correspondences messages.
     * 
     * This callback function reacts to an incoming message by converting it to
     * a `Correspondences` object and then stores the contained plane matches
     * in the matching element of `all_correspondences`, that belongs to the
     * sensor combination the data comes from.
     * 
     * @param msg The received correspondences message in custom format.
     */
    void rcvMessage(const custom_msgs::msg::Correspondences &msg);
    
    /**
     * @brief Start the calibration process.
     * 
     * This member function is called internally by the destructor to start the
     * calibration when the user interrupts the data reading process by
     * pressing <kbd>Ctrl</kbd>+<kbd>C</kbd>. To specify the calibration change
     * the parameters `calib_strategy` and `algorithm` in the corresponding 
     * launch file.
     */
    void startCalib();
};

#endif
