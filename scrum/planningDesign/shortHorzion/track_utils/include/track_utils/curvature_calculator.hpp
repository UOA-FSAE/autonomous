#ifndef TRACK_UTILS_CURVATURE_CALCULATOR_HPP_
#define TRACK_UTILS_CURVATURE_CALCULATOR_HPP_

#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <cmath>
#include <optional>

namespace track_utils {

constexpr double CURVATURE_THRESHOLD = 0.5;

std::optional<double> calculateCurvature(const geometry_msgs::msg::Point& p1,
                                         const geometry_msgs::msg::Point& p2,
                                         const geometry_msgs::msg::Point& p3);


std::optional<double> calculateBearingDifference(const geometry_msgs::msg::Point& left_cone,
                                                 const geometry_msgs::msg::Point& right_cone,
                                                 const geometry_msgs::msg::Point& current_position,
                                                 const geometry_msgs::msg::Point& previous_position);

std::optional<geometry_msgs::msg::Point> detectCornerStart(
    const std::vector<geometry_msgs::msg::Point>& upcoming_points);



}// namespace track_utils
#endif  // TRACK_UTILS_CURVATURE_CALCULATOR_HPP_
