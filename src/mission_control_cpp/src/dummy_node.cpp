#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mission_control_cpp/subscription_manager.hpp"

class DummyNode : public rclcpp::Node  // Remove enable_shared_from_this<DummyNode>
{
public:
  DummyNode()
  : Node("dummy_node")
  {
    // Don't call shared_from_this() here!
  }

  void init()
  {
    // Explicitly get shared_ptr<DummyNode> from rclcpp::Node's shared_from_this()
    auto self = std::static_pointer_cast<DummyNode>(this->rclcpp::Node::shared_from_this());

    subscription_wrapper_ = ManageSubscription<std_msgs::msg::String>(
      self,
      "DUMMY_TOPIC",
      std::bind(&DummyNode::dummy_callback, this, std::placeholders::_1),
      {0, 2, 4});
  }

  void dummy_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

private:
  std::shared_ptr<SubscriptionWrapper<std_msgs::msg::String>> subscription_wrapper_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
