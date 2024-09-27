#include <chrono>
#include <memory>
#include <thread>
#include <iostream>

#include "sl/Camera.hpp"

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moa_msgs/msg/detections.hpp"
#include "sensor_msgs/msg/image.hpp"

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
    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

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
    bool visualisation = true;
    std::string model_name = "cone_detection_model.engine";
    float CONF_THRESH = 0.8;
    float NMS_THRESH = 0.5;

    // Publishers
    rclcpp::Publisher<moa_msgs::msg::Detections>::SharedPtr cone_detection_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr car_position_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
        
    void cone_detection_loop();
    void car_position();
};