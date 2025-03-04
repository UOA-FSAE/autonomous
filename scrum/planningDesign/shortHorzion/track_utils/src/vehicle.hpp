#pragma once

#include "DataTypes.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace planning {

/**
 * @brief A data class for storing details about a vehicles position and 
 * 
 */
struct Vehicle {
    Point pos;
    Angle bearing; 
    double velocity;
    Angle angular_velocity;


    //timestamp of the last update
    rclcpp::Time lastUpdateTime;

    //Default constructor
    Vehicle(): pos(0.0, 0.0), bearing(0.0), velocity(0.0), angular_velocity(0.0) {}

    /*
     * @brief update the vehicle's state from posestamped message 
     * 
     * @param poseMsg The pose message containing position and orientation data
     */
    void updateFromPose(const geometry_msgs::msg::PoseStamped& poseMsg){
        //update position
        pos.x = poseMsg.pose.position.x;
        pos.y = poseMsg.pose.position.y;

        //extract bearing from quaternion
        tf2::Quaternion q(
            poseMsg.pose.orientation.x,
            poseMsg.pose.orientation.y,
            poseMsg.pose.orientation.z,
            poseMsg.pose.orientation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        //to degrees
        bearing.setRadians(yaw);
        lastUpdateTime = rclcpp::Time(poseMsg.header.stamp);
        
    }

    /**
     * @brief 
     * 
     * @return ** void 
     */
    void updateFromVelocity(){
        double vx = twistMsg.twist.linear.x;
        double vy = twistMsg.twist.linear.y;
        velocity = std::sqrt(vx* vx + vy * vy);
        angular_velocity = twistMsg.twist.angular.z;

        lastUpdateTime = rclcpp::Time(twistMsg.header.stamp);

    }
    
    const Point& getPostion const{
        return pos;
    }

    /**
     * @brief Get the current bearing/heading of the vehicle
     * 
     * @return const Angle& Reference to the bearing
     */
    const Angle& getHeading() const {
        return bearing;
    }

    /**
     * @brief Get the current velocity magnitude
     * 
     * @return double The velocity in m/s
     */
    double getVelocity() const {
        return velocity;
    }

    /**
     * @brief Get the current angular velocity
     * 
     * @return double The angular velocity in rad/s
     */
    double getAngularVelocity() const {
        return angular_velocity;
    }

    /**
     * @brief Get the timestamp of the last update
     * 
     * @return const rclcpp::Time& Reference to the timestamp
     */
    const rclcpp::Time& getLastUpdateTime() const {
        return lastUpdateTime;
    }
};


}