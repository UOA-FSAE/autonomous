#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "mission_control_cpp/subscription_manager.hpp"

class MissionControl : public rclcpp::Node
{
public:
    MissionControl() : Node("MissionControl")
    {
        this->declare_parameter<int>("current_mission", 0);
        current_mission_ = this->get_parameter("current_mission").as_int();

        mission_publisher_ = this->create_publisher<std_msgs::msg::Int8>(
            MISSION_ANNOUNCE_TOPIC_NAME, MISSION_ANNOUNCE_QOS);

        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MissionControl::on_param_change, this, std::placeholders::_1));
    }

private:
    int current_mission_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr mission_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

    rcl_interfaces::msg::SetParametersResult on_param_change(
        const std::vector<rclcpp::Parameter>& params)
    {
        for (const auto& param : params)
        {
            if (param.get_name() == "current_mission")
            {
                current_mission_ = param.as_int();
                publish_signals();
                RCLCPP_INFO(this->get_logger(), "Mission changed to %d", current_mission_);
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void publish_signals()
    {
        std_msgs::msg::Int8 msg;
        msg.data = current_mission_;
        mission_publisher_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
