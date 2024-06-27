#include "Types.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Class template to create a synchronizer for two ROS2 message formats.
 * 
 * This helper template class creates a synchronizer for the two data types 
 * given in the arguments `Msg1` and `Msg2`, synchronizing with an
 * `ApproximateTime` policy.
 * 
 * @tparam Msg1 The first message type of the pair.
 * @tparam Msg2 The second message type of the pair.
 */
template <typename Msg1, typename Msg2>
class SyncPair 
{
public:
    /// Define the synchronization policy using approximate time for Msg1 and Msg2.
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Msg1, Msg2>;

    /**
     * @brief Constructor for SyncPair.
     * 
     * Initializes the subscribers for the given topics and sets up the 
     * synchronizer.
     * 
     * @param topic1 The name of the first topic to subscribe to.
     * @param topic2 The name of the second topic to subscribe to.
     * @param node A pointer to the `rclcpp::Node` instance.
     */
    SyncPair(std::string topic1, std::string topic2, rclcpp::Node *node)  
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        sub1 = std::make_shared<message_filters::Subscriber<Msg1>>(node, topic1, qos_profile);
        sub2 = std::make_shared<message_filters::Subscriber<Msg2>>(node, topic2, qos_profile);
        sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), *sub1, *sub2);
    }

    /// Shared pointer to the synchronizer that uses the defined SyncPolicy.
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    /// Shared pointer to the subscriber for the first topic.
    std::shared_ptr<message_filters::Subscriber<Msg1>> sub1;

    /// Shared pointer to the subscriber for the second topic.
    std::shared_ptr<message_filters::Subscriber<Msg2>> sub2;
};
