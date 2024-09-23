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
        car_pose.position.x = cam_w_pose.getTranslation().tx / 1000.0;
        car_pose.position.y = cam_w_pose.getTranslation().ty / 1000.0;

        float ox = cam_w_pose.getOrientation().ox;
        float oy = cam_w_pose.getOrientation().oy;
        float oz = cam_w_pose.getOrientation().oz;
        float ow = cam_w_pose.getOrientation().ow;

        float yaw = atan2(2.0f * (ow * oz + ox * oy), 1.0f - 2.0f * (oy * oy + oz * oz));
        
        car_pose.orientation.w = yaw;

        car_position_publisher->publish(car_pose);

        std::this_thread::sleep_for(100ms);
    }
}