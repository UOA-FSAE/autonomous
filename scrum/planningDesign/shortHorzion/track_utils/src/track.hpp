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
        bool closedLoop = false;

    public:
        Track() = default;
        virtual ~Track() = 0;

        virtual std::pair<Cone, Cone> getEnd() const = 0;
        virtual std::pair<Cone, Cone> getStart() const = 0;

        void setClosedLoop();
                                    
        std::vector<std::shared_ptr<Cone>> getLocalCones(const Point& position, const uint8_t range) const; // implemented and tested
        std::vector<InertialPose> getLocalCenterPoints(const Point& point, const double range) const; // implemented and tested
        std::pair<std::optional<InertialPose>, std::optional<InertialPose>> getNearestCenterPoints(const Point& point) const; //implemented and tested

        double getCurvature(const std::tuple<Point, Point, Point>& centerPoints) const; // implemented and tested
        std::optional<double> getCurvature(const Point& position) const ; // implemented
        double getCurvature(const Point& position, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const; // implemented
        double getCurvature(const Point& position, const std::vector<InertialPose>& localCenterPoints) const; // implemented
        std::optional<double> getLocalCurvature(const Point& position, const uint8_t range) const; //implemented

        Angle getBearing(const std::tuple<Point, Point, Point>& points) const;
        std::optional<Angle> getBearing(const Point& position) const; // implemented
        Angle getBearing(const Point& position, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const;  // implemented
        std::optional<Angle> getLocalBearing(const Point& position, const uint8_t radius) const; // implemented

        std::optional<Angle> getDifferenceInBearing(const Vehicle& vehicle) const; //implemented

        double getROCofCurvature(const std::pair<InertialPose, InertialPose>&) const;       //implemented
        double getROCofCurvature(const Point&) const;                                       //implemented

        void insertCone(const std::shared_ptr<Cone> cone);                                  //implemented
        void initialiseCenterPoint(std::vector<Point>&& points, bool is_closed);             //implemented 
        std::vector<Point> triangulateCenterPoints() const;                                 // TODO: Done - Winola to transfer 
        std::vector<Point> matchCenterPoints(std::vector<Point>&& points) const;            //implemented
        void nearestNeighbourSort(std::vector<Point>& points);                              //implemented
        void nearestNeighbourInsert(const Point& point);                                          //implemented
        size_t getNearestNeighbourInsertionPoint(const Point& point) const;                 //implemented    

        double calculateProgression(const Point& position) const;                           //implemented
        double getProgression(const Vehicle vehicle);                                       //implemented
        double getProgression(const Point position);                                        //implemented

        /* 

        + Track::getCurvatureROCAt(Point): double
        + Track::extrapolateTrack(distance: double): std::vector<InertialPose>
        */
    };

}




#endif // TRACK_HPP