#include "track.hpp"
#include "DataTypes.hpp"  
#include "constants.hpp"
#include "util.hpp"
#include <algorithm>
#include <limits>
#include <cstring>
#include <complex>

namespace planning {
    
class InvalidVectorLengthException : public std::exception {
public:
    InvalidVectorLengthException(const char* message) : msg_(message) {}
    virtual const char* what() const noexcept override {
        return msg_;
    }
private:
    const char* msg_;
};


Track::~Track() {
    // No special cleanup required
};

/**
 * @brief Sets the track as a closed loop (i.e., track forms a complete circuit)
 * 
 * This affects how track progression and nearest neighbor calculations are performed,
 * as points at the end of the track connect back to the beginning.
 * 
 */
void Track::setClosedLoop() {
    closedLoop = true;
}

//DONE
/**
 * @brief returns the cones that are within a radius 'range' from a point. 
 *
 * WARNING computationally intensive
 * 
 * @param position 
 * @param range 
 * @return std::vector<std::shared_ptr<Cone>> 
 */
[[nodiscard]]
std::vector<std::shared_ptr<Cone>> Track::getLocalCones(const Point& position, const uint8_t range) const
{
    std::vector<std::shared_ptr<Cone>> localCones;
    for (auto& [id, cone_p] : coneMap) {
        if (position.distanceTo(cone_p->getPos()) <= range) {
            localCones.push_back(cone_p);
        }
    }
    return localCones;
}  


/**
 * @brief returns a vector of center points that are within a set range
 * 
 * @param point the queried location
 * @param range distance within which a center point should be within from a point
 * @return std::pair<InertialPose> 
 */
[[nodiscard]]
std::vector<InertialPose> Track::getLocalCenterPoints(const Point& point, const double range) const
{
    std::vector<InertialPose> localCenterPoints;

    //Iterate through all the centrepoints in the track
    for (const auto& centerPoint : centerPoints){
         
        //calculating distance between the queried point and the centrepoint
        double distance = point.distanceTo(centerPoint.pos);

        //checking if the distance is within the range
        if (distance <= static_cast<double>(range)){
            localCenterPoints.push_back(centerPoint);
        }    
   }

   return localCenterPoints;
    
}


// /**
//  * @brief returns a pair of center points that are the closest to a given point
//  * and on either side of the point. 
//  * 
//  * @param point the queried location
//  * @param range distance within which a center point should be within from a point
//  * @return std::pair<InertialPose> 
//  */
[[nodiscard]]
std::pair<std::optional<InertialPose>, std::optional<InertialPose>> Track::getNearestCenterPoints(const Point& point) const {
    // Edge case: Empty track
    if (centerPoints.empty()) {
        return {std::nullopt, std::nullopt};
    }

    // Edge case: Single point track
    if (centerPoints.size() == 1) {
        return {centerPoints[0], std::nullopt};
    }

    // Find the closest center point
    size_t closestIdx = 0;
    double minDist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < centerPoints.size(); ++i) {
        double dist = point.distanceTo(centerPoints[i].pos);
        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
        }
    }

    // Handle open loop cases
    if (!closedLoop) {
        // If closest point is first point, check if point is before track
        if (closestIdx == 0){
            //if queried point lies between frist two centerpoints, return both centerpoints
            if(in_opposing_or_normal_direction(point, centerPoints[0].pos, centerPoints[1].pos)) {
                return {centerPoints[0], centerPoints[1]};
            }
            //else if both centrepoints lie on the same side, means queried point is before first centerpoint, so return just the first centre point
            return {std::nullopt, centerPoints[0]};
        }
        
        // If closest point is last point, check if point is after track
        if (closestIdx == centerPoints.size() - 1){
            //if the last centrepoint and second to last centrepoing lie in either side of the queried point, bot should be returned 
            if(in_opposing_or_normal_direction(point, centerPoints.back().pos, centerPoints[centerPoints.size() - 2].pos)) {
                return {centerPoints[centerPoints.size() - 2], centerPoints.back()};
            }
            //else if the lie in the same direction relative wo queried point, queried point must be positioned after the last cenrepoint. so only return the last cp
            return {centerPoints.back(), std::nullopt};
        }
    }
    // These are all closed loop scenarios

    if (closedLoop){

        if(closestIdx == 0){
             //if queried point lies between frist two centerpoints, return both centerpoints
            if(in_opposing_or_normal_direction(point, centerPoints[0].pos, centerPoints[1].pos)) {
                return {centerPoints[0], centerPoints[1]};
            }
            // if not, then point must lie between the last centrepoint in the vector and the first
            return {centerPoints.back(), centerPoints[0] };
        }

        if (closestIdx == centerPoints.size() - 1){
             //if the last centrepoint and second to last centrepoint lie in either side of the queried point, both should be returned 
            if(in_opposing_or_normal_direction(point, centerPoints.back().pos, centerPoints[centerPoints.size() - 2].pos)) {
                return {centerPoints[centerPoints.size() - 2], centerPoints.back()};
            }
            //else if the lie in the same direction relative wo queried point, queried point must be positioned after the last cenrepoint, and befoire the first cp, so return the latter two
            return {centerPoints.back(), centerPoints[0]};
        }
    }

    // Get indices for adjacent points
    size_t prevIdx = (closestIdx == 0) ? (closedLoop ? centerPoints.size() - 1 : 0) : closestIdx - 1;
    size_t nextIdx = (closestIdx == centerPoints.size() - 1) ? (closedLoop ? 0 : centerPoints.size() - 1) : closestIdx + 1;

    // Check previous point
    if (in_opposing_or_normal_direction(point, centerPoints[closestIdx].pos, centerPoints[prevIdx].pos)) {
        return {centerPoints[prevIdx], centerPoints[closestIdx]};
    }
    
    // Check next point
    if (in_opposing_or_normal_direction(point, centerPoints[closestIdx].pos, centerPoints[nextIdx].pos)) {
        return {centerPoints[closestIdx], centerPoints[nextIdx]};
    }

    // If no opposing points found, return closest point and nearest neighbor
    double distToPrev = point.distanceTo(centerPoints[prevIdx].pos);
    double distToNext = point.distanceTo(centerPoints[nextIdx].pos);
    
    if (distToPrev < distToNext) {
        return {centerPoints[prevIdx], centerPoints[closestIdx]};
    } else {
        return {centerPoints[closestIdx], centerPoints[nextIdx]};
    }
}

/**
 * @brief gets the curvature of the middle (2nd) of 3 consecutive points
 * 
 * @param InertialPoses 
 * @return std::optional<double> 
 */
[[nodiscard]]
double Track::getCurvature(const std::tuple<Point, Point, Point>& centerPoints) const
{
    Point p1 = std::get<0>(centerPoints);
    Point p2 = std::get<1>(centerPoints);
    Point p3 = std::get<2>(centerPoints);

    // Length of sides of the traingle formed by the three points
    double a = std::hypot(p2.x - p1.x, p2.y - p1.y);
    double b = std::hypot(p3.x - p2.x, p3.y - p2.y);
    double c = std::hypot(p3.x - p1.x, p3.y - p1.y);

    //semi perimeter and area of triangle
    double s = (a + b + c) / 2.0;
    double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

    if (area == 0.0) {  
        return 0; 
    }

    //Heron's formula to calcuate radius
    double radius = (a * b * c) / (4.0 * area);
    double curvature = 1.0 / radius;

    return curvature;
}


/**
 * @brief returns the weighted average of the curvature of the nearest center points
 * @param point 
 * @return double 
 */
[[nodiscard]]
std::optional<double> Track::getCurvature(const Point& position) const
{
    std::pair<std::optional<InertialPose>, std::optional<InertialPose>> nearestCenterPoints = getNearestCenterPoints(position);

    if (nearestCenterPoints.first && nearestCenterPoints.second) {
        const auto& firstPoint = nearestCenterPoints.first.value();
        const auto& secondPoint = nearestCenterPoints.second.value();
        
        // Use interpolation when both points are available
        return interpolate_curvature(
            firstPoint.pos, firstPoint.curvature,
            secondPoint.pos, secondPoint.curvature,
            position
        );
    } else if (nearestCenterPoints.first) {
        // Use the curvature of the first point when only it is available
        return nearestCenterPoints.first.value().curvature;
    } else if (nearestCenterPoints.second) {
        // Use the curvature of the second point when only it is available
        return nearestCenterPoints.second.value().curvature;
    } else {
        // Return nullopt when no nearby points are found
        return std::nullopt;
    }
}

/**
 * @brief 
 * 
 * @param point F
 * @return double 
 */
[[nodiscard]]
double Track::getCurvature(const Point& position, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const
{
    return interpolate_curvature(nearestCenterPoints.first.pos, nearestCenterPoints.first.curvature, nearestCenterPoints.second.pos, nearestCenterPoints.second.curvature, position);
}

/**
 * @brief returns the weighted 
 * 
 * @param point 
 * @throws InvalidVectorLengthException
 * @return double 
 */
[[nodiscard]]
double Track::getCurvature(const Point& position, const std::vector<InertialPose>& localCenterPoints) const {
    if (localCenterPoints.size() < 2) {
        // char size[5] = {((char) localCenterPoints.size())};
        // char* errorMsg = strcat("localCenterPoints.size(): ", size);
        // throw InvalidVectorLengthException(errorMsg);
        std::cerr << "invalid vector length" << std::endl;
    }
    return interpolate_curvature(localCenterPoints[0].pos, localCenterPoints[0].curvature, localCenterPoints[1].pos, localCenterPoints[1].curvature, position);
}


// WIP
/**
 * @brief gets the curvature 
 * 
 * @param position 
 * @param range 
 * @return std::optional<double> 
 */
 [[nodiscard]]
std::optional<double> Track::getLocalCurvature(const Point& position, const uint8_t range) const
{
    // center points should already be sorted.
    auto localCenterPoints = getLocalCenterPoints(position, range);
    
    if (localCenterPoints.size() < 2) {
        return std::nullopt; // Need at least 3 points to calculate curvature
    }

    // calculate curvature
    return interpolate_curvature(localCenterPoints[0].pos, localCenterPoints[0].curvature, localCenterPoints[1].pos, localCenterPoints[1].curvature, position);
}


//DONE
/**
 * @brief Inserts a new cone into the track's cone map
 * 
 * @param cone Inserts a new cone into the track's cone map
 * @throws Prints error message if cone is null or if cone ID already exists
 */
void Track::insertCone(const std::shared_ptr<Cone> cone)
{
    if (cone) {
        int coneID = cone->getId();
        auto result = coneMap.emplace(coneID, cone);
        if (!result.second) {
            std::cerr << "Error: Cone with ID " << coneID << " already exists." << std::endl;
        }
    } else {
       std::cerr << "Error: Trying to insert a null cone." << std::endl; 
    }
}


/**
 * @brief Inserts a new center point into the track's centerline, determine
 * where is placed (ordered in the vectore), calcualte the curvature of it, and
 * the bearning so that the point becomes an inertial pose (having info about the curvature, bearning etc)
 * c
 * 
 * 
 * @param point : coordinates to be added as a center point
 */
void Track::initialiseCenterPoint(std::vector<Point>&& points, bool is_closed) {
    // Validate input
    if (points.size() < 3) {
        throw std::invalid_argument("At least 3 points are required to initialize a track.");
    }

    // Sort points using nearest neighbor algorithm
    nearestNeighbourSort(points);

    // Clear existing center points
    centerPoints.clear();

    // Process points
    for (size_t i = 0; i < points.size(); ++i) {
        Point a = points[(i == 0 && is_closed) ? points.size() - 1 : i - 1];
        Point b = points[i];
        Point c = points[(i == points.size() - 1 && is_closed) ? 0 : i + 1];

        std::tuple<Point, Point, Point> point_tuple = {a, b, c};
        Angle bearing = getBearing(point_tuple);
        double curvature = Track::getCurvature(point_tuple);

        centerPoints.emplace_back(b, curvature, bearing);
    }
}

// /**
//  * @brief 
//  * 
//  * Uses triangulation algorithm to generate optimal center points based on track boundaries
//  * 
//  * @return std::vector<Point> vector of triangulated center points
//  */
std::vector<Point> Track::triangulateCenterPoints() const
{
    return {};
}


/**
 * @brief returns a vector points that didn't match 
 * with any currently stored center points. Ie the difference of the two sets of points. 
 * 
 * @param points Vector of points to be matched with the track
 * @return std::vector<Point> : Vector of matched and aligned center points
 */
std::vector<Point> Track::matchCenterPoints(std::vector<Point>&& points) const
{
    // If there are no existing center points, return input points
    if (centerPoints.empty()){
        return points;
    }

    //if input poins empty, reutrn empty vector
    if(points.empty()){
        return std::vector<Point>();
    }

    std::vector<Point> unmatchedPoints;
    const double MATCHING_THRESHOLD = 2.0;

    //for each input point check if it matches with existing centerpoint
    for (const auto& point : points) {
        bool matched = false;
        
        for (const auto& centerPoint : centerPoints){
            if (point.distanceTo(centerPoint.pos) <= MATCHING_THRESHOLD){
                matched = true;
                break;
            }
        }

        if (!matched){
            unmatchedPoints.push_back(point);
        }
    }

    return unmatchedPoints;

}


//DONE
/**
 * @brief Sorts points in a vector using nearest neighbor algorithm
 * 
 *  Reorganizes points so that each point is followed by its nearest unvisited neighbor,
 * creating a continuous path through all points.
 * 
 * @param points : Vector of points to be sorted (modified in place)
 */
void Track::nearestNeighbourSort(std::vector<Point>& points)
{
    if (points.empty()){
        return;
    }

    std::vector<Point> sorted;

    
    sorted.push_back(points.front());
    points.erase(points.begin());
    
    while (!points.empty()) {
        auto nearestIt = std::min_element(points.begin(), points.end(), [&](const Point& a, const Point& b) {
            return sorted.back().distanceTo(a) < sorted.back().distanceTo(b);
        });
        sorted.push_back(*nearestIt);
        points.erase(nearestIt);
    }
   // points = sorted;

   //I made this modification to update original points instead of the local copy
   std::swap(points, sorted);
}


/**
 * @brief Inserts a new point into the center line using nearest neighbor algorithm
 * 
 * Finds the optimal insertion point between existing center points and calculates
 * appropriate curvature and bearing values for the new point.
 * 
 * @param point : Point to be inserted into the center line
 */
void Track::nearestNeighbourInsert(const Point& point)
{   
    // get insertion location
    size_t index = getNearestNeighbourInsertionPoint(point);

    // if (index == 0 || index >= centerPoints.size()){
    //     double curvature = 0;
    //     double bearing = 0;
    //     InertialPose newCenterPoint{point, curvature, bearing};
    //     centerPoints.insert(centerPoints.begin() + index, newCenterPoint);
    //     return;
    // }

    // get curvature
    InertialPose cp1 = centerPoints[index-1];
    InertialPose cp2 = centerPoints[index];
    std::pair<InertialPose, InertialPose> cps = {cp1, cp2};
    
    std::optional<double> curvature_o = getCurvature(point, cps);

    double curvature;

    if (curvature_o) {
        curvature = curvature_o.value();
    }

    // get bearing
    Angle bearing = getBearing(point, cps);

    InertialPose newCenterPoint{point, curvature, bearing};
    // instantiate a InertialPos
    centerPoints.insert(centerPoints.begin() + index, newCenterPoint);
}

/**
 * @brief Given a Point, find the best insertion index in centerPoints so that the order remains as continuous as possible.
 * 
 * @param point 
 * @return size_t 
 */
size_t Track::getNearestNeighbourInsertionPoint(const Point& point) const{

    //insert at the beginning of the list if center points list is empty
    if (centerPoints.empty()){
        return 0;
    }
    
    //if point is closest to the inital center point, isert at the start
    if(point.distanceTo(initialCenterPoint) < point.distanceTo(centerPoints.front().pos)){
        return 0;
    }
    //if point is closest to the new center point, isert at the end
    if (point.distanceTo(newCenterPoint) < point.distanceTo(centerPoints.back().pos)){
        return centerPoints.size();
    }

    // make use of allPoints and offset insertion index accordingly

    size_t closestIndex = 0;
    double minDistance = std::numeric_limits<double>::max();

    // Find closest center point that exists
    for (size_t i = 0; i < centerPoints.size(); i++){
        double distance = point.distanceTo(centerPoints[i].pos);
        if (distance < minDistance){
            minDistance = distance;
            closestIndex = i;
        }
    }

    double prevDistance = centerPoints[closestIndex -1].pos.distanceTo(point);   
    double nextDistance = (closestIndex + 1 < centerPoints.size())
                            ? centerPoints[closestIndex + 1].pos.distanceTo(point)
                            : std::numeric_limits<double>::max();
    
    return (prevDistance < nextDistance) ? closestIndex : closestIndex + 1;
}


/**
 * @brief retrive local points and return the weighted average of their bearings
 * 
 * @return Angle 
 */
[[nodiscard]]
std::optional<Angle> Track::getLocalBearing(const Point& position, uint8_t radius) const {

    // Retrieve center points within the given radius
    auto centerPoints = getLocalCenterPoints(position, radius);
    
    if (centerPoints.size() < 2) {
        return std::nullopt; // Not enough points to compute a meaningful bearing
    }

    // Find the two nearest center points
    std::pair<InertialPose, InertialPose> nearestCenterPoints{centerPoints[0], centerPoints[0]};
    double minDist1 = std::numeric_limits<double>::max();
    double minDist2 = std::numeric_limits<double>::max();

    bool firstAssigned = false;
    bool secondAssigned = false;

    for (const auto& cp : centerPoints){
        double dist = position.distanceTo(cp.pos);
        if (dist < minDist1) {
            minDist2 = minDist1;
            if (firstAssigned) {
                nearestCenterPoints.second = nearestCenterPoints.first;
                secondAssigned = true;
            }
            minDist1 = dist;
            nearestCenterPoints.first = cp;
            firstAssigned = true;
        } else if (dist < minDist2) {
            minDist2 = dist;
            nearestCenterPoints.second = cp;
            secondAssigned = true;
        }
    }

    // Ensure both points are assigned before computing the bearing
    if (!firstAssigned || !secondAssigned) {
        return std::nullopt;
    }

    // Compute the weighted average of the bearings
    return std::optional<Angle>(getBearing(position, nearestCenterPoints));

}   

/**
 * @brief get the differnece between the heading of the vehichle and the bearing of its current position on the track
 * 
 * @param vehicle 
 * @return Angle 
 */
[[nodiscard]]
std::optional<Angle> Track::getDifferenceInBearing(const Vehicle& vehicle) const
{
    //find nearest points from vehicle's position
    //we should now have vehicles poijnts and two nearest center points
    std::optional<Angle> trackBearingOpt = getBearing(vehicle.pos);

    if (!trackBearingOpt) {
        return std::nullopt;
    }

    //can probably use getbearing to find the bearing at the vehicle's position
    Angle vehicleHeading = vehicle.bearing;
    //find the difference between the bearing of the vehcile and it's position.
    return trackBearingOpt.value().difference(vehicleHeading);

}

/**
 * @brief Calculates rate of change of curvature between two center points
 * 
 * @param centerPoints Pair of InertialPose objects representing consecutive track points
 * @return double :  Rate of change of curvature between the points
 */
[[nodiscard]]
double Track::getROCofCurvature(const std::pair<InertialPose, InertialPose>& centerPoints) const 
{
    //extract curvatures of the points
    double kappa1 = centerPoints.first.curvature;
    double kappa2 = centerPoints.second.curvature;

    // calculate arc length between points
    double ds = centerPoints.first.pos.distanceTo(centerPoints.second.pos);

    //prevention of div by 0
    if (ds < 1e-6){
        return 0.0;
    }

    //calculare ROC (dk/ds): spatial derivative of curvature wrt arc length
    return (kappa2 - kappa1) / ds;

}

template<typename T>
struct Circle {
    Point center;
    double radius; 
    Circle(std::pair<std::complex<double>, double>& pair) {
        center = pair.first;
        radius = pair.second;
    }
};
Angle Track::getBearing(const std::tuple<Point, Point, Point>& points) const {

    // find center and radius of a circle that intersects the 3 points

    std::complex<double> z1 = {std::get<0>(points).x, std::get<0>(points).y};
    std::complex<double> z2 = {std::get<1>(points).x, std::get<1>(points).y};
    std::complex<double> z3 = {std::get<2>(points).x, std::get<2>(points).y};    
    std::pair<std::complex<double>, double> circle = circle_from_3_points<double>(z1, z2, z3);
        
    std::complex<double> center = circle.first;
    double radius = circle.second; 

    std::complex<double> z = (z2-center); 
    Angle bearing = std::atan2(-z.imag(),z.real()); //(x,y) ---> (-y,x)
    return bearing;
}   

/**
 * @brief Calculate the bearing at a given position by finding nearest center points
 * @param position Point to calculate bearing at
 * @return Angle 
 */
[[nodiscard]]
std::optional<Angle> Track::getBearing(const Point& position) const
{
    // obtaining nearest center points to position
    auto nearestPoints = getNearestCenterPoints(position);
    //if we don't have valid center pioints then return 0 angle
    
    if(!nearestPoints.first || ! nearestPoints.second){
        if (nearestPoints.first) {
            return nearestPoints.first.value().bearing;
        } else if (nearestPoints.second) {
            return nearestPoints.second.value().bearing;
        } else {
            return std::nullopt;
        }
    }
    
    std::pair<InertialPose, InertialPose> centerPointPair = {
        nearestPoints.first.value(),
        nearestPoints.second.value()
    };

    return getBearing(position, centerPointPair);
}

/**
 * @brief Calculate the bearing at a position given two nearest center points
 * 
 * @param position Point to calculate bearing at
 * @param nearestCenterPoints Pair of nearest center points with their poses
 * @return Angle Weighted average bearing based on distances to center points
 */
[[nodiscard]]
Angle Track::getBearing(const Point& position, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const {
    
    //excracting points from nearestCenterPoints
    const auto& point1 = nearestCenterPoints.first.pos;
    const auto& point2 = nearestCenterPoints.second.pos;

    //Calcualting the distances from position to each centerpoint
    double distanceToPoint1 = position.distanceTo(point1);
    double distanceToPoint2 = position.distanceTo(point2);

    const double epsilon = 1e-6;

    //weights based on inverse distnace
    double weight1 = 1.0/ (distanceToPoint1 + epsilon);
    double weight2 = 1.0 / (distanceToPoint2 + epsilon);
    
    //normalise wieghts to sum to 1
    double totalWeight = weight1 + weight2;
    weight1 /= totalWeight;
    weight2 /= totalWeight;

    //Extracting the bearings at each point
    Angle bearing1 = nearestCenterPoints.first.bearing;
    Angle bearing2 = nearestCenterPoints.second.bearing;

    //Calculate weighted average bearing 
    double averageDegrees = bearing1.getDegrees() * weight1 + bearing2.getDegrees() * weight2;

    //create and normalise result
    Angle result;
    result.setDegrees(averageDegrees);
    result.normalize();

    return result;

}

/**
 * @brief Calculates rate of change of curvature at a specific position
 * 
 * Finds nearest center points and interpolates the rate of change of curvature
 * at the given position.
 * 
 * @param position Point at which to calculate rate of change of curvature
 * @return double Interpolated rate of change of curvature at the position
 */
[[nodiscard]]
double Track::getROCofCurvature(const Point& position) const
{
    //get nearest center points
    auto nearestPoints = getNearestCenterPoints(position);

    //check if we have valid centerpoints:
    if (!nearestPoints.first || !nearestPoints.second) {
        return 0.0;  
    }

    // Create pair of InertialPoses
    std::pair<InertialPose, InertialPose> centerPointPair = {
        nearestPoints.first.value(),
        nearestPoints.second.value()
    };
    
    //basic ROC between the two nearest centerpoints
    double baseROC = getROCofCurvature(centerPointPair);
    
    //caluclate weight for interpolation
    double dist1 = position.distanceTo(centerPointPair.first.pos);
    double dist2 = position.distanceTo(centerPointPair.second.pos);
    double totalDist = dist1 + dist2;

    if (totalDist < 1e-6) {
        return baseROC;
    }

    // Weight the ROC based on relative distances
    // When closer to first point, ROC is more influenced by the approaching change
    // When closer to second point, ROC is more influenced by the current change
    double weight2 = dist1 / totalDist;  
    double weight1 = 1.0 - weight2;

    //return ROC of curvature at the point influenced by weight
    return baseROC * (weight1 + weight2);    
}

/**
 * @brief Helper function to calcualte total progression for a given point
 * 
 * @param position 
 * @return double 
 */
[[nodiscard]]
double Track::calculateProgression(const Point& position) const {
    
    // variable to hold the total distance, determine intial point (i.e
    // initalCenterPoint e first element in the centerPoints vector)
    double totalDistance = 0.0;

    Point startPoint = (initialCenterPoint.x != 0 && initialCenterPoint.y != 0) ? initialCenterPoint : centerPoints[0].pos;

    //iterate over the centerpoints and sum the distances along the way
    for (size_t i = 1; i < centerPoints.size(); i++){
        const Point& prevPoint = centerPoints[i - 1].pos;
        const Point& currentPoint = centerPoints[i].pos;
        
        double segmentDistance = prevPoint.distanceTo(currentPoint);
        
        //check if pos is between prevPoint and currentPoint
        double prevDist = prevPoint.distanceTo(position);
        double currentDist = currentPoint.distanceTo(position);
        
        // If the position is between prevPoint and currentPoint, calculate the partial distance
        if (prevDist <= currentDist) {
            totalDistance += prevDist;
            totalDistance += segmentDistance;
            break; 
        }
        
    }
    return totalDistance;
}


/**
 * @brief Use helper function to calculate the distance travelled by the vehicle
 * from the intial center point
 * 
 * @param vehicle 
 * @return double 
 */
double Track::getProgression(const Vehicle vehicle) {
    return calculateProgression(vehicle.pos);  
}

/**
 * @brief Use helper function to calculate the distance along the trakc a
 * speicifc point is from the intial center point
 * 
 * @param position 
 * @return double 
 */
double Track::getProgression(const Point position) {
    return calculateProgression(position);  
}


} //end of planning namespace