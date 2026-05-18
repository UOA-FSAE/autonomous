/*
Wrapper-driven perception node for the ZED wrapper.

This node subscribes to the wrapper's RGB image and registered point cloud topics, runs the
existing YOLO cone detector on the incoming camera frames, and converts each 2D detection into
a 3D point by sampling the registered point cloud.

It publishes the same `zed/cone_detection` detections topic used by the custom direct ZED node,
so the rest of the planning stack can remain unchanged.
*/

#include <cmath>
#include <mutex>
#include <string>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fsae_interfaces/msg/detections.hpp>

#include "yolo.hpp"

#define CONF_THRESH 0.40
#define MAX_CONE_DISTANCE_M 25.0

class WrapperPerceptionNode : public rclcpp::Node
{
public:
  WrapperPerceptionNode()
  : Node("zed_wrapper_cone_detection")
  {
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/zed/zed_node/rgb/color/rect/image");
    pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/zed/zed_node/point_cloud/cloud_registered");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/zed/zed_node/odom");
    model_name_ = this->declare_parameter<std::string>("model_name", "cone_detection_model.engine");

    RCLCPP_INFO(get_logger(), "Wrapper perception subscribing image: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Wrapper perception subscribing cloud: %s", pointcloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Wrapper perception subscribing odom: %s", odom_topic_.c_str());

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&WrapperPerceptionNode::imageCallback, this, std::placeholders::_1));

    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 5,
      std::bind(&WrapperPerceptionNode::pointCloudCallback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&WrapperPerceptionNode::odomCallback, this, std::placeholders::_1));

    cone_detection_publisher_ = this->create_publisher<fsae_interfaces::msg::Detections>("zed/cone_detection", 10);
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("zed/image", 10);
    car_position_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("zed/car_position", 10);
    car_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("zed/car_velocity", 10);

    RCLCPP_INFO(get_logger(), "Initializing YOLO model: %s", model_name_.c_str());
    if (detector_.init(model_name_) != 0) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize YOLO model '%s'", model_name_.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "YOLO model initialized successfully");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 30 == 0) {
      RCLCPP_DEBUG(get_logger(), "[DEBUG] Processing frame %d, encoding: %s, size: %ux%u", frame_count, msg->encoding.c_str(), msg->width, msg->height);
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image_bgr;
    // ZED wrapper publishes BGRA, force explicit conversion
    if (msg->encoding == "bgra8") {
        cv::cvtColor(cv_ptr->image, image_bgr, cv::COLOR_BGRA2BGR);
    } else if (msg->encoding == "bgr8") {
        image_bgr = cv_ptr->image.clone();  // Force a copy
    } else {
        RCLCPP_ERROR(get_logger(), "Unexpected encoding: %s", msg->encoding.c_str());
        return;
    }

    // Verify it's actually 3-channel BGR
    if (image_bgr.channels() != 3) {
        RCLCPP_ERROR(get_logger(), "Image still has %d channels after conversion!", image_bgr.channels());
        return;
    }

    if (image_bgr.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty wrapper image frame");
      return;
    }

    auto detections = detector_.run(image_bgr, static_cast<int>(msg->height), static_cast<int>(msg->width), CONF_THRESH);
    if (frame_count % 30 == 0) {
      RCLCPP_INFO(get_logger(), "[YOLO] Frame %d: detected %lu cones", frame_count, detections.size());
    }
    fsae_interfaces::msg::Detections detections_msg;

    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (last_odom_) {
        detections_msg.car_pose = last_odom_->pose.pose;
      }
    }

    for (const auto & detection : detections) {
      int px = std::lround((detection.box.x1 + detection.box.x2) / 2.0f);
      int py = std::lround(detection.box.y2);
      geometry_msgs::msg::Point point;
      if (!getPointFromCloud(px, py, point)) {
        continue;
      }

      float dist = std::hypot(point.x, point.y);
      if (dist > MAX_CONE_DISTANCE_M) {
        continue;
      }

      switch (detection.label) {
        case 0:
          detections_msg.blue.push_back(point);
          break;
        case 4:
          detections_msg.yellow.push_back(point);
          break;
        default:
          detections_msg.big_orange.push_back(point);
          break;
      }

      cv::rectangle(image_bgr,
                    cv::Point(static_cast<int>(detection.box.x1), static_cast<int>(detection.box.y1)),
                    cv::Point(static_cast<int>(detection.box.x2), static_cast<int>(detection.box.y2)),
                    cv::Scalar(0, 255, 0), 2);
      char label_text[64];
      std::snprintf(label_text, sizeof(label_text), "cls=%d %.2f", detection.label, detection.prob);
      cv::putText(image_bgr, label_text,
                  cv::Point(static_cast<int>(detection.box.x1), static_cast<int>(detection.box.y1) - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    }

    if (!detections_msg.blue.empty() || !detections_msg.yellow.empty() || !detections_msg.big_orange.empty()) {
      RCLCPP_DEBUG(get_logger(), "Publishing detections: %lu blue, %lu yellow, %lu orange", detections_msg.blue.size(), detections_msg.yellow.size(), detections_msg.big_orange.size());
      cone_detection_publisher_->publish(detections_msg);
    }

    sensor_msgs::msg::Image::SharedPtr overlay_msg;
    try {
      overlay_msg = cv_bridge::CvImage(msg->header, "bgr8", image_bgr).toImageMsg();
      image_publisher_->publish(*overlay_msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to publish overlay image: %s", e.what());
    }

    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (last_odom_) {
      car_position_publisher_->publish(last_odom_->pose.pose);
      geometry_msgs::msg::Vector3 velocity;
      velocity.x = last_odom_->twist.twist.linear.x;
      velocity.y = last_odom_->twist.twist.linear.y;
      velocity.z = last_odom_->twist.twist.linear.z;
      car_velocity_publisher_->publish(velocity);
    }
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    latest_pointcloud_ = msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_ = msg;
  }

  bool getPointFromCloud(int u, int v, geometry_msgs::msg::Point & point)
  {
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    if (!latest_pointcloud_) {
      return false;
    }

    const auto & cloud = *latest_pointcloud_;
    if (cloud.is_bigendian) {
      return false;
    }
    if (u < 0 || v < 0 || u >= static_cast<int>(cloud.width) || v >= static_cast<int>(cloud.height)) {
      return false;
    }

    int x_offset = getFieldOffset(cloud, "x");
    int y_offset = getFieldOffset(cloud, "y");
    int z_offset = getFieldOffset(cloud, "z");
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      return false;
    }

    size_t point_index = static_cast<size_t>(v) * cloud.width + static_cast<size_t>(u);
    size_t data_offset = point_index * cloud.point_step;
    const uint8_t * ptr = &cloud.data[data_offset];

    float x = *reinterpret_cast<const float *>(ptr + x_offset);
    float y = *reinterpret_cast<const float *>(ptr + y_offset);
    float z = *reinterpret_cast<const float *>(ptr + z_offset);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }

    point.x = x;
    point.y = y;
    point.z = z;
    return true;
  }

  int getFieldOffset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == field_name) {
        return field.offset;
      }
    }
    return -1;
  }

  std::string image_topic_;
  std::string pointcloud_topic_;
  std::string odom_topic_;
  std::string model_name_;

  Yolo detector_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  rclcpp::Publisher<fsae_interfaces::msg::Detections>::SharedPtr cone_detection_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr car_position_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr car_velocity_publisher_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  std::mutex pointcloud_mutex_;
  std::mutex odom_mutex_;
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WrapperPerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
