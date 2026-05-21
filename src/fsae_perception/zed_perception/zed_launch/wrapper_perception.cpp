#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <fsae_interfaces/msg/detections.hpp>

#include "yolo.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#define CONF_THRESH 0.40f
#define DIST_FAR_M 25.0f

class WrapperPerceptionNode : public rclcpp::Node
{
public:
  WrapperPerceptionNode()
  : Node("zed_wrapper_cone_detection")
  {
    image_topic_ = this->declare_parameter<std::string>(
      "image_topic", "/zed/zed_node/rgb/color/rect/image");
    depth_topic_ = this->declare_parameter<std::string>(
      "depth_topic", "/zed/zed_node/depth/depth_registered");
    camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info");
    odom_topic_ = this->declare_parameter<std::string>(
      "odom_topic", "/zed/zed_node/odom");
    model_name_ = this->declare_parameter<std::string>(
      "model_name", "cone_detection_model.engine");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&WrapperPerceptionNode::imageCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, 10,
      std::bind(&WrapperPerceptionNode::depthCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&WrapperPerceptionNode::cameraInfoCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&WrapperPerceptionNode::odomCallback, this, std::placeholders::_1));

    cone_detection_pub_ = this->create_publisher<fsae_interfaces::msg::Detections>("zed/cone_detection", 10);
    car_position_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("zed/car_position", 10);
    car_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("zed/car_velocity", 10);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("zed/image", 10);

    RCLCPP_INFO(get_logger(), "Initializing YOLO model: %s", model_name_.c_str());
    if (detector_.init(model_name_) != 0) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize YOLO model '%s'", model_name_.c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Wrapper perception node ready.");
    RCLCPP_INFO(get_logger(), "  image:  %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  depth:  %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  info:   %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  odom:   %s", odom_topic_.c_str());
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "Depth cv_bridge error: %s", e.what());
      return;
    }
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_ = cv_ptr->image.clone();
    depth_received_ = true;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(intrinsics_mutex_);
    if (intrinsics_received_) return;
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    intrinsics_received_ = true;
    RCLCPP_INFO(get_logger(), "Camera intrinsics received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                fx_, fy_, cx_, cy_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    auto & pos = msg->pose.pose.position;
    auto & ori = msg->pose.pose.orientation;

    car_pose_.position.x = pos.x;
    car_pose_.position.y = pos.y;

    // Extract yaw from quaternion, store in orientation.w to match old node convention
    float yaw = std::atan2(
      2.0f * (ori.w * ori.z + ori.x * ori.y),
      1.0f - 2.0f * (ori.y * ori.y + ori.z * ori.z));
    car_pose_.orientation.w = yaw;

    car_velocity_.x = msg->twist.twist.linear.x;
    car_velocity_.y = msg->twist.twist.linear.y;
    car_velocity_.z = msg->twist.twist.linear.z;

    odom_received_ = true;

    car_position_pub_->publish(car_pose_);
    car_velocity_pub_->publish(car_velocity_);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "Image cv_bridge error: %s", e.what());
      return;
    }

    auto detections = detector_.run(
      cv_ptr->image, static_cast<int>(msg->height), static_cast<int>(msg->width), CONF_THRESH);

    // BGR colors per YOLO class: 0=blue cone, 4=yellow cone, else=orange
    static const cv::Scalar COLOR_BLUE(255, 100, 0);
    static const cv::Scalar COLOR_YELLOW(0, 255, 255);
    static const cv::Scalar COLOR_ORANGE(0, 140, 255);

    cv::Mat depth_for_vis;
    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      if (depth_received_) depth_for_vis = latest_depth_.clone();
    }

    cv::Mat vis = cv_ptr->image.clone();
    for (const auto & det : detections) {
      cv::Rect rect(
        static_cast<int>(det.box.x1), static_cast<int>(det.box.y1),
        static_cast<int>(det.box.x2 - det.box.x1),
        static_cast<int>(det.box.y2 - det.box.y1));

      cv::Scalar color;
      if (det.label == 0) color = COLOR_BLUE;
      else if (det.label == 4) color = COLOR_YELLOW;
      else color = COLOR_ORANGE;

      cv::rectangle(vis, rect, color, 2);

      char text[128];
      if (!depth_for_vis.empty()) {
        int cu = std::max(0, std::min((static_cast<int>(det.box.x1) + static_cast<int>(det.box.x2)) / 2, depth_for_vis.cols - 1));
        int cv_y = std::max(0, std::min((static_cast<int>(det.box.y1) + static_cast<int>(det.box.y2)) / 2, depth_for_vis.rows - 1));
        float d = depth_for_vis.at<float>(cv_y, cu);
        if (std::isfinite(d) && d > 0.0f)
          sprintf(text, "Class %d - %.0f%% | %.1fm", det.label, det.prob * 100.0f, d);
        else
          sprintf(text, "Class %d - %.0f%%", det.label, det.prob * 100.0f);
      } else {
        sprintf(text, "Class %d - %.0f%%", det.label, det.prob * 100.0f);
      }

      int baseline = 0;
      cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
      int tx = rect.x;
      int ty = std::min(rect.y + 1, vis.rows);
      cv::rectangle(vis, cv::Rect(tx, ty, label_size.width, label_size.height + baseline), {0, 0, 255}, -1);
      cv::putText(vis, text, cv::Point(tx, ty + label_size.height),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 255, 255}, 1);
    }
    auto img_msg = cv_bridge::CvImage(msg->header, "bgr8", vis).toImageMsg();
    image_pub_->publish(*img_msg);

    if (!depth_received_ || !intrinsics_received_ || !odom_received_) return;

    cv::Mat depth_local;
    float fx, fy, cx, cy;
    geometry_msgs::msg::Pose pose_local;

    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      depth_local = latest_depth_.clone();
    }
    {
      std::lock_guard<std::mutex> lock(intrinsics_mutex_);
      fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    }
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      pose_local = car_pose_;
    }

    fsae_interfaces::msg::Detections detections_msg;
    detections_msg.car_pose = pose_local;

    for (const auto & det : detections) {
      float u = (det.box.x1 + det.box.x2) / 2.0f;
      float v = (det.box.y1 + det.box.y2) / 2.0f;

      int ui = std::max(0, std::min(static_cast<int>(u), depth_local.cols - 1));
      int vi = std::max(0, std::min(static_cast<int>(v), depth_local.rows - 1));

      float depth = depth_local.at<float>(vi, ui);
      if (!std::isfinite(depth) || depth <= 0.0f || depth > DIST_FAR_M) continue;

      // Back-project from optical frame (X right, Y down, Z forward)
      // to body frame (X forward, Y left, Z up)
      float opt_x = (u - cx) * depth / fx;
      float opt_y = (v - cy) * depth / fy;

      geometry_msgs::msg::Point p;
      p.x = depth;      // forward
      p.y = -opt_x;     // left
      p.z = -opt_y;     // up (unused by downstream but correct)

      switch (det.label) {
        case 0:
          detections_msg.blue.push_back(p);
          break;
        case 4:
          detections_msg.yellow.push_back(p);
          break;
        default:
          detections_msg.big_orange.push_back(p);
          break;
      }
    }

    if (!detections_msg.blue.empty() && !detections_msg.yellow.empty()) {
      cone_detection_pub_->publish(detections_msg);
    }
  }

  std::string image_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string odom_topic_;
  std::string model_name_;

  Yolo detector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<fsae_interfaces::msg::Detections>::SharedPtr cone_detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr car_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr car_velocity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  std::mutex depth_mutex_;
  cv::Mat latest_depth_;
  bool depth_received_ = false;

  std::mutex intrinsics_mutex_;
  float fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  bool intrinsics_received_ = false;

  std::mutex odom_mutex_;
  geometry_msgs::msg::Pose car_pose_;
  geometry_msgs::msg::Vector3 car_velocity_;
  bool odom_received_ = false;
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
