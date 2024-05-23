#ifndef SUBSCRIBER_INFO_HPP
#define SUBSCRIBER_INFO_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>



/**
 * @class SubscriberInfo class
 * @brief Creating dynamic subscriber for multiple robots.
 * @ref https://answers.ros.org/question/199678/how-to-deal-with-a-large-number-of-possibly-variable-topic-names/
*/
class SubscriberInfo
{
    private:

    public:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscriber;
        
        bool _process;
};

#endif