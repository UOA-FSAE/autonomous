#include <iostream>
#include <cmath>
#include <mutex>

#include "sl/Camera.hpp"
#include "../zed_launch/zed_launch.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// mtx is defined in cone_detection.cpp and guards cam_w_pose across threads.
extern std::mutex mtx;

void ZedLaunchNode::car_position() {

    geometry_msgs::msg::Pose car_pose;

    while (camera_running) {
        // Lock before reading cam_w_pose: cone_detection_loop() writes it
        // under the same mutex, so reading without the lock is a data race.
        float tx, ty, ox, oy, oz, ow;
        mtx.lock();
        tx = cam_w_pose.getTranslation().tx;
        ty = cam_w_pose.getTranslation().ty;
        ox = cam_w_pose.getOrientation().ox;
        oy = cam_w_pose.getOrientation().oy;
        oz = cam_w_pose.getOrientation().oz;
        ow = cam_w_pose.getOrientation().ow;
        mtx.unlock();
 
        car_pose.position.x = tx / 1000.0;
        car_pose.position.y = ty / 1000.0;

        float yaw = atan2(2.0f * (ow * oz + ox * oy), 1.0f - 2.0f * (oy * oy + oz * oz));
        car_pose.orientation.w = yaw;

        car_position_publisher->publish(car_pose);

        std::this_thread::sleep_for(100ms);
    }
}