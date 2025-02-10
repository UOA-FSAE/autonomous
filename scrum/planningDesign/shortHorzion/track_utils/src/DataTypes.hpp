#pragma once

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

//includes 
#include <iostream>
#include <cmath>

// Done
namespace planning {
/**
 * @brief Class representing a point
 * 
 */

 
class Angle {
private:
    double degrees;

public:
    // Constructor
    Angle(double deg = 0.0) : degrees(deg) {}

    // Set angle in degrees
    void setDegrees(double deg) {
        degrees = deg;
    }

    // Get angle in degrees
    double getDegrees() const {
        return degrees;
    }

    // Set angle in radians
    void setRadians(double rad) {
        degrees = rad * (180.0 / M_PI);
    }

    // Get angle in radians
    double getRadians() const {
        return degrees * (M_PI / 180.0);
    }

    // Normalize the angle to be within [0, 360) degrees
    void normalize() {
        degrees = fmod(degrees, 360.0);
        if (degrees < 0) {
            degrees += 360.0;
        }
    }

    Angle difference(Angle &other) {
        double angle = getDegrees();
        angle -= other.getDegrees();
        return Angle(angle);
    }

    void scale(double scalar) {
        degrees = degrees*scalar;
    }

    // Print the angle
    void print() const {
        std::cout << "Angle: " << degrees << " degrees (" << getRadians() << " radians)" << std::endl;
    }

    friend std::ostream& operator<<(std::ostream& os, const Angle& obj) {
        os << "Angle: {degrees: " << obj.getDegrees() << "°, radians: " << obj.getRadians() << "}";
        return os;
    }
};

struct Point {
    double x;
    double y;

    Point(double x = 0, double y = 0) : x(x), y(y) {};

    double distanceTo(const Point& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
};

struct InertialPose {
    Point pos;
    double curvature = 0;
    Angle bearing = 0;
public:
    InertialPose(Point& point) : pos(point) {};
    InertialPose(Point& point, double curvature, Angle bearing) : pos(point), curvature(curvature), bearing(bearing) {};
};


}

#endif // DATA_TYPES_H
