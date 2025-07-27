#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DummyPublisher : public rclcpp::Node
{
public:
    DummyPublisher() : Node("dummy_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("DUMMY_TOPIC", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DummyPublisher::publish_dummy, this)
        );
    }

private:
    void publish_dummy()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello from dummy publisher! " + std::to_string(++count_);
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyPublisher>());
    rclcpp::shutdown();
    return 0;
}
