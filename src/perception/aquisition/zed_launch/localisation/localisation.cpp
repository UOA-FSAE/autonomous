#include <iostream>
#include <cmath>

#include "sl/Camera.hpp"
#include "../zed_launch/zed_launch.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

void ZedLaunchNode::car_position() {
    
    geometry_msgs::msg::Pose car_pose;
    visualization_msgs::msg::MarkerArray car_marker_array;
    geometry_msgs::msg::TransformStamped map;

    int id = 0;
    while (camera_running) {
        car_pose.position.x = cam_w_pose.getTranslation().tx;
        car_pose.position.y = cam_w_pose.getTranslation().tz;
        car_pose.position.z = cam_w_pose.getTranslation().ty;

        float ox = cam_w_pose.getOrientation().ox;
        float oy = cam_w_pose.getOrientation().oy;
        float oz = cam_w_pose.getOrientation().oz;
        float ow = cam_w_pose.getOrientation().ow;

        float yaw = atan2(2.0f * (ow * oy + ox * oz), 1.0f - 2.0f * (oy * oy + oz * oz));
        float yaw_deg = yaw * 180.0f / M_PI;

        if (yaw_deg < 0) {
            yaw_deg += 360;
        }
        
        car_pose.orientation.w = yaw_deg;

        car_position_publisher->publish(car_pose);

        map.header.frame_id = "map";
        map.child_frame_id = "base_link";
        // non moving transform
        map.transform.translation.x = 0;
        map.transform.translation.y = 0;
        map.transform.translation.z = 0;
        map.transform.rotation.x = 0;
        map.transform.rotation.y = 0;
        map.transform.rotation.z = 0;
        map.transform.rotation.w = 1;
        map.header.stamp = rclcpp::Time();

        tf_broadcaster->publish(map);
        
        visualization_msgs::msg::Marker car_marker;

        car_marker.header.frame_id = "map";
        car_marker.header.stamp = rclcpp::Time();

        car_marker.ns = "car";
        car_marker.id = id++;
        car_marker.type = visualization_msgs::msg::Marker::CUBE;
        car_marker.action = visualization_msgs::msg::Marker::ADD;

        car_marker.pose = car_pose;

        car_marker.scale.x = 40.0;
        car_marker.scale.y = 40.0;
        car_marker.scale.z = 40.0;

        car_marker.color.r = 0.0f;
        car_marker.color.g = 1.0f;
        car_marker.color.b = 0.0f;

        car_marker.color.a = 1.0;

        car_marker_array.markers.push_back(car_marker);

        car_marker_publisher->publish(car_marker_array);

        std::this_thread::sleep_for(200ms);
    }
}