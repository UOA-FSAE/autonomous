#include <chrono>
#include <memory>
#include <thread>
#include <iostream>

#include "sl/Camera.hpp"

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moa_msgs/msg/detections.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class ZedLaunchNode : public rclcpp::Node
{
public:
  ZedLaunchNode(sl::Camera& _zed)
  : Node("cone_detection_node"), zed(_zed)
  {
    // Initialize the camera pose
    cam_w_pose.pose_data.setIdentity();

    // Create the publishers
    cone_detection_publisher = this->create_publisher<moa_msgs::msg::Detections>("cone_detection", 10);
    car_position_publisher = this->create_publisher<geometry_msgs::msg::Pose>("car_position", 10);
    car_marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("car_marker", 10);
    tf_broadcaster = this->create_publisher<geometry_msgs::msg::TransformStamped>("tf_broadcaster", 10);

    // Start the threads
    std::thread t1(&ZedLaunchNode::cone_detection_loop, this);

    std::thread t2(&ZedLaunchNode::car_position, this);

    t1.join();
  }

private:
    // ZED camera object
    sl::Camera& zed;
    sl::Pose cam_w_pose;
    bool camera_running = true;

    // Publishers
    rclcpp::Publisher<moa_msgs::msg::Detections>::SharedPtr cone_detection_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr car_position_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr car_marker_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_broadcaster;
        
    void cone_detection_loop();
    void car_position();
};