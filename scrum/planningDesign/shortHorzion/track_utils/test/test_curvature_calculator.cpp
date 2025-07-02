#include "track_utils/curvature_calculator.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <vector>

int main() {
    using geometry_msgs::msg::Point;

    std::cout << "=== Testing Curvature Calculation ===" << std::endl;
    // Test for calculateCurvature (existing test case)
    Point p1;
    p1.x = 0.0;
    p1.y = 0.0;

    Point p2;
    p2.x = 1.0;
    p2.y = 0.0;

    Point p3;
    p3.x = 0.0;
    p3.y = 1.0;

    auto curvature = track_utils::calculateCurvature(p1, p2, p3);
    if (curvature.has_value()) {
        std::cout << "Curvature: " << curvature.value() << std::endl;
    } else {
        std::cout << "Curvature: Undefined" << std::endl;
    }

    std::cout << "\n=== Testing Bearing Difference ===" << std::endl;
    // Test for calculateBearingDifference
    Point left_cone;
    left_cone.x = 0.0;
    left_cone.y = 0.0;

    Point right_cone;
    right_cone.x = 2.0;
    right_cone.y = 0.0;

    Point current_position;
    current_position.x = 1.0;
    current_position.y = 1.0;

    Point previous_position;
    previous_position.x = 0.0;
    previous_position.y = 1.0;

    auto bearing_difference = track_utils::calculateBearingDifference(
        left_cone, right_cone, current_position, previous_position);

    if (bearing_difference.has_value()) {
        std::cout << "Bearing Difference: " << bearing_difference.value() << " degrees" << std::endl;
    } else {
        std::cout << "Bearing Difference: Undefined" << std::endl;
    }

    std::cout << "\n=== Testing Corner Detection ===" << std::endl;
    // Test cases for corner detection
    std::vector<Point> track_points;

    // Test Case 1: Straight line followed by curve:
    std::cout << "\nTest Case 1: Straight line followed by curve" << std::endl;
    track_points.clear();

    // Add straight line points
    for (int i = 0; i < 3; ++i) {
        Point p;
        p.x = i * 1.0;
        p.y = 0.0;
        track_points.push_back(p);
        std::cout << "Added straight point " << i << ": (" << p.x << ", " << p.y << ")" << std::endl;
    }

    // Add curve points
    Point curve1;
    curve1.x = 3.0;
    curve1.y = 0.5;
    track_points.push_back(curve1);
    std::cout << "Added curve point 1: (" << curve1.x << ", " << curve1.y << ")" << std::endl;

    Point curve2;
    curve2.x = 3.5;
    curve2.y = 1.0;
    track_points.push_back(curve2);
    std::cout << "Added curve point 2: (" << curve2.x << ", " << curve2.y << ")" << std::endl;

    std::cout << "Total points in track: " << track_points.size() << std::endl;

    // Check curvature values before corner detection
    for (size_t i = 0; i < track_points.size() - 2; ++i) {
        auto test_curvature = track_utils::calculateCurvature(
            track_points[i],
            track_points[i + 1],
            track_points[i + 2]
        );
        std::cout << "Curvature at point " << i << ": " 
                << (test_curvature.has_value() ? std::to_string(test_curvature.value()) : "undefined")
                << std::endl;
    }

    auto corner_start = track_utils::detectCornerStart(track_points);
    if (corner_start.has_value()) {
        std::cout << "Corner detected at point: ("
                << corner_start.value().x << ", "
                << corner_start.value().y << ")" << std::endl;
    } else {
        std::cout << "No corner detected" << std::endl;
    }

    // Test Case 2: All straight line points (should not detect corner)
    std::cout << "\nTest Case 2: Straight line only" << std::endl;
    track_points.clear();
    for (int i = 0; i < 5; ++i) {
        Point p;
        p.x = i * 1.0;
        p.y = 0.0;
        track_points.push_back(p);
    }

    corner_start = track_utils::detectCornerStart(track_points);
    if (corner_start.has_value()) {
        std::cout << "Corner detected at point: ("
                  << corner_start.value().x << ", "
                  << corner_start.value().y << ")" << std::endl;
    } else {
        std::cout << "No corner detected" << std::endl;
    }

    // Test Case 3: Sharp corner (should definitely detect)
    std::cout << "\nTest Case 3: Sharp corner" << std::endl;
    track_points.clear();
    
    // First two points straight
    Point s1;
    s1.x = 0.0;
    s1.y = 0.0;
    track_points.push_back(s1);
    
    Point s2;
    s2.x = 1.0;
    s2.y = 0.0;
    track_points.push_back(s2);
    
    // Sharp turn
    Point s3;
    s3.x = 1.0;
    s3.y = 1.0;
    track_points.push_back(s3);
    
    Point s4;
    s4.x = 0.0;
    s4.y = 1.0;
    track_points.push_back(s4);

    corner_start = track_utils::detectCornerStart(track_points);
    if (corner_start.has_value()) {
        std::cout << "Corner detected at point: ("
                  << corner_start.value().x << ", "
                  << corner_start.value().y << ")" << std::endl;
    } else {
        std::cout << "No corner detected" << std::endl;
    }

    // Test Case 4: Insufficient points
    std::cout << "\nTest Case 4: Insufficient points" << std::endl;
    track_points.clear();
    track_points.push_back(s1);
    track_points.push_back(s2);  // Only 2 points

    corner_start = track_utils::detectCornerStart(track_points);
    if (corner_start.has_value()) {
        std::cout << "Corner detected at point: ("
                  << corner_start.value().x << ", "
                  << corner_start.value().y << ")" << std::endl;
    } else {
        std::cout << "No corner detected (expected for insufficient points)" << std::endl;
    }

    return 0;
}