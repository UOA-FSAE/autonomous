#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <vector>
#include <memory>
#include <functional>
#include <string>

constexpr const char* MISSION_ANNOUNCE_TOPIC_NAME = "MissionSonic";
constexpr size_t MISSION_ANNOUNCE_QOS = 10;

class SubscriptionWrapperBase
{
public:
    virtual ~SubscriptionWrapperBase() = default;
};

template<typename MsgT>
class SubscriptionWrapper : public SubscriptionWrapperBase
{
public:
    SubscriptionWrapper(
        rclcpp::Node::SharedPtr node,
        const std::string& topic,
        std::function<void(typename MsgT::SharedPtr)> callback,
        const std::vector<int>& mission_array,
        size_t qos)
        : node_(node),
          topic_(topic),
          callback_(callback),
          qos_(qos),
          mission_array_(mission_array)
    {
        control_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
            MISSION_ANNOUNCE_TOPIC_NAME,
            MISSION_ANNOUNCE_QOS,
            std::bind(&SubscriptionWrapper::on_mission_change, this, std::placeholders::_1)
        );

        if (std::find(mission_array_.begin(), mission_array_.end(), 0) != mission_array_.end()) {
            subscription_ = node_->create_subscription<MsgT>(topic_, qos_, callback_);
        }
    }

private:
    void on_mission_change(const std_msgs::msg::Int8::SharedPtr msg)
    {
        int mission = msg->data;
        bool should_subscribe = std::find(mission_array_.begin(), mission_array_.end(), mission) != mission_array_.end();

        if (should_subscribe && subscription_ == nullptr) {
            subscription_ = node_->create_subscription<MsgT>(topic_, qos_, callback_);
        } else if (!should_subscribe && subscription_ != nullptr) {
            subscription_.reset();
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::string topic_;
    std::function<void(typename MsgT::SharedPtr)> callback_;
    size_t qos_;
    std::vector<int> mission_array_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr control_sub_;
    typename rclcpp::Subscription<MsgT>::SharedPtr subscription_;
};

template<typename MsgT>
std::shared_ptr<SubscriptionWrapper<MsgT>> ManageSubscription(
    rclcpp::Node::SharedPtr node,
    const std::string& topic,
    std::function<void(typename MsgT::SharedPtr)> callback,
    const std::vector<int>& mission_array,
    size_t qos_profile = 10)
{
    return std::make_shared<SubscriptionWrapper<MsgT>>(node, topic, callback, mission_array, qos_profile);
}