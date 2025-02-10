#pragma once
#ifndef TRACK_HPP
#define TRACK_HPP

#include <vector>
#include <map>
#include <memory>
// #include "planning.hpp"
#include "cone.hpp"
#include "DataTypes.hpp"
#include "constants.hpp"
#include "vehicle.hpp"
#include <optional>

#include <cmath> 

using namespace planning;

// tested
// implemented
// wip
// declared
// todo

namespace planning {

    constexpr double PI = M_PI;
    constexpr double TWO_PI = 2.0 * M_PI;
    
        
class Track{

    protected:
        std::map<uint16_t, std::shared_ptr<Cone>> coneMap;
        std::vector<InertialPose> centerPoints;
        Point newCenterPoint;
        Point initialCenterPoint;

    public:
        Track() = default;
        virtual ~Track() = 0;

        virtual std::pair<InertialPose, InertialPose> getEnd() = 0;
        virtual std::pair<InertialPose, InertialPose> getStart() = 0;

        std::vector<std::shared_ptr<Cone>> getLocalCones(const Point& position, const uint8_t range) const; // declared
        std::vector<InertialPose> getLocalCenterPoints(const Point& point, const double range) const; // declared
        std::pair<std::optional<InertialPose>, std::optional<InertialPose>> getNearestCenterPoints(const Point point) const; // declared

        // double getLocalCurvature(Point position, uint8_t range) const;
        double getCurvature(const std::tuple<Point, Point, Point>& centerPoints) const; // implemented
        std::optional<double> getCurvature(const Point& position) const ; // declared
        std::optional<double> getCurvature(const Point& position, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const; // declared
        double getCurvature(const Point& position, const std::vector<InertialPose>& nearestCenterPoints) const; // declared
        std::optional<double> getLocalCurvature(const Point& position, const uint8_t range) const; // declared

        Angle getBearing(const Point& position) const; // declared
        Angle getBearing(const Point& position, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const;  // declared
        std::optional<Angle> Track::getLocalBearing(const Point& position, const uint8_t radius) const; // declared

        Angle getDifferenceInBearing(const Vehicle& vehicle) const; 

        double getROCofCurvature(const std::pair<InertialPose, InertialPose>&) const;
        double getROCofCurvature(const Point&) const;
        double getROCofCurvature(const InertialPose&) const;

        void insertCone(const std::shared_ptr<Cone> cone);                                  //implemented
        void insertCenterPoint(const Point& point);                                         //implemented
        std::vector<Point> triangulateCenterPoints() const;                                 //TODO
        std::vector<Point> matchCenterPoints(std::vector<Point>&& points) const;            //TODO
        void nearestNeighbourSort(std::vector<Point>& points);                              //implemented
        void nearestNeighbourInsert(Point& point);                                          //implemented
        size_t getNearestNeighbourInsertionPoint(const Point& point) const;                 //TODO    

        double getProgression(const Vehicle vehicle) const;     //TODO NOT declared and implemented
        double getProgression(const Point position) const;      //TODO NOT declared and implemented

        /* 

        + Track::getCurvatureROCAt(Point): double
        + Track::extrapolateTrack(distance: double): std::vector<InertialPose>
        */
    };

}




#endif // TRACK_HPP