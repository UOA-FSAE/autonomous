#include <iostream>

#include "sl/Camera.hpp"
#include "../zed_launch/zed_launch.hpp"

#include "geometry_msgs/msg/pose.h"

void ZedLaunchNode::car_position() {
    
    geometry_msgs::msg::Pose car_pose;
    while (camera_running) {
        car_pose.position.x = cam_w_pose.getTranslation().tx;
        car_pose.position.y = cam_w_pose.getTranslation().ty;
        car_pose.position.z = cam_w_pose.getTranslation().tz;
        car_pose.orientation.x = cam_w_pose.getOrientation().ox;
        car_pose.orientation.y = cam_w_pose.getOrientation().oy;
        car_pose.orientation.z = cam_w_pose.getOrientation().oz;

        car_position_publisher->publish(car_pose);

        std::this_thread::sleep_for(200ms);
    }
}