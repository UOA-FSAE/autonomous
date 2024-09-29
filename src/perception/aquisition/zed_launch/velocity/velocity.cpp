#include <iostream>
#include <cmath>

#include "sl/Camera.hpp"
#include "../zed_launch/zed_launch.hpp"

#include "geometry_msgs/msg/vector3.hpp" 

void ZedLaunchNode::car_velocity() {

    geometry_msgs::msg::Vector3 car_velocity;
    sl::SensorsData sensor_data;
    sl::SensorsData::IMUData imu_data;

    car_velocity.x = 0;
    car_velocity.y = 0;
    car_velocity.z = 0;

    while (camera_running) {
        std::this_thread::sleep_for(10ms);

        zed.getSensorsData(sensor_data, sl::TIME_REFERENCE::IMAGE);
        imu_data = sensor_data.imu;

        car_velocity.x += imu_data.linear_acceleration.x * 0.01;
        car_velocity.y += imu_data.linear_acceleration.y * 0.01;

        car_velocity_publisher->publish(car_velocity);
    
    }
}