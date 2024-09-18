#include <iostream>

#include "sl/Camera.hpp"
#include "../zed_launch/zed_launch.hpp"

#include "geometry_msgs/msg/pose.h"

void ZedLaunchNode::car_position() {
    
    geometry_msgs::msg::Pose car_pose;
    while (camera_running) {
        zed.getPosition(cam_w_pose, sl::REFERENCE_FRAME::WORLD);
        car_pose.position.x = cam_w_pose.getTranslation().tx;
        car_pose.position.y = cam_w_pose.getTranslation().tz;
        car_pose.position.z = cam_w_pose.getTranslation().ty;

        float ox = cam_w_pose.getOrientation().ox;
        float oy = cam_w_pose.getOrientation().oy;
        float oz = cam_w_pose.getOrientation().oz;
        float ow = cam_w_pose.getOrientation().ow;

        float yaw = atan2(2.0f * (ow * oy + ox * oz), 1.0f - 2.0f * (oy * oy + oz * oz));
        float yaw_deg = yaw * 180.0f / M_PI;

        car_pose.orientation.w = yaw_deg;

        car_position_publisher->publish(car_pose);

        std::this_thread::sleep_for(200ms);
    }
}