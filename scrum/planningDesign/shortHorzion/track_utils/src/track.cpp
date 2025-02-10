#include "track.hpp"
#include "DataTypes.hpp"  
#include "constants.hpp"
#include <algorithm>

namespace planning {
    
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


/**
 * @brief returns a pair of center points that are the closest to a given point
 * and on either side of the point. 
 * 
 * @param point the queried location
 * @param range distance within which a center point should be within from a point
 * @return std::pair<InertialPose> 
 */
 [[nodiscard]]
std::pair<std::optional<InertialPose>, std::optional<InertialPose>> Track::getNearestCenterPoints(const Point point) const {


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
 * @brief 
 * 
 * @param point 
 * @return double 
 */
[[nodiscard]]
std::optional<double> Track::getCurvature(const Point& position) const
{
    std::vector<planning::InertialPose>  localCenterPoints = getLocalCenterPoints(position, track_utils::CONE_SPACING);

    return 0.0;
    

}

/**
 * @brief 
 * 
 * @param point 
 * @return double 
 */
[[nodiscard]]
std::optional<double> Track::getCurvature(const Point& position, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const
{
    return 0.0;
}

/**
 * @brief returns the weighted 
 * 
 * @param point 
 * @return double 
 */
[[nodiscard]]
double Track::getCurvature(const Point& position, const std::vector<InertialPose>& nearestCenterPoints) const {

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
    auto centerPoints = getLocalCenterPoints(position, range);
    
    if (centerPoints.size() < 3) {
        return std::nullopt; // Need at least 3 points to calculate curvature
    }


    // TODO make the average of all points
    std::tuple<Point, Point, Point> firstThreePoints = {
        centerPoints[0].pos, centerPoints[1].pos, centerPoints[2].pos
    };

    // calculate curvature
    return getCurvature(firstThreePoints);
}


//DONE
/**
 * @brief 
 * 
 * @param cone 
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


//DONE
/**
 * @brief 
 * 
 * @param point 
 *//**
 * @brief 
 * 
 * @param point 
 */
void Track::insertCenterPoint(const Point& point)
{
    centerPoints.emplace_back(point);
}

/**
 * @brief 
 * 
 * @return std::vector<Point> 
 */
std::vector<Point> Track::triangulateCenterPoints() const
{}

/**
 * @brief 
 * 
 *
 * 
 * @param points 
 * @return std::vector<Point> 
 */
std::vector<Point> Track::matchCenterPoints(std::vector<Point>&& points) const
{
    
}


//DONE
/**
 * @brief 
 * 
 * @param points 
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
    points = sorted;
}


/**
 * @brief 
 * 
 * @param point 
 */
void Track::nearestNeighbourInsert(Point& point)
{   
    // get insertion location
    size_t index = getNearestNeighbourInsertionPoint(point);

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

    // collect all points
    std::vector<Point> allPoints(centerPoints.size() + 2);
    std::transform(centerPoints.begin(), centerPoints.end(), allPoints.begin(), [](const InertialPose& centerPoint) { return centerPoint.pos; });

    allPoints.insert(allPoints.begin(), initialCenterPoint); // insert intial point
    allPoints.insert(allPoints.end(), newCenterPoint); // append end points

    // TODO make use of allPoints and offset insertion index accordingly

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

    //Finding best insetion index based on distance between the points
    //first case: if it si closest to the first point
    if(closestIndex ==0){
        return 1;
    }

    size_t insertIndex = closestIndex;
    double prevDistance = centerPoints[closestIndex -1].pos.distanceTo(point);   
    double nextDistance = (closestIndex + 1 < centerPoints.size())
                            ? centerPoints[closestIndex + 1].pos.distanceTo(point)
                            : std::numeric_limits<double>::max();
    
    //insert betweemn the two closest neighbours
    if(prevDistance < nextDistance){
        insertIndex = closestIndex;
    } else{
        insertIndex = closestIndex + 1;
    }
}


/**
 * @brief returns a weighted average of 2 center point's bearings based on
 * their proximity to a queried point
 *  
 * @param neareastCenterPoint the closest 2 center points
 * @param queriedPoint the point we are getting the bearing for
 * 
 * @return Angle 
 */
 [[nodiscard]]
Angle Track::getBearing(const Point& queriedPoint, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const {
    // Extract points from nearestCenterPoints
    const auto& point1 = nearestCenterPoints.first.pos;
    const auto& point2 = nearestCenterPoints.second.pos;

    // Calculate distances from queriedPoint to each centerpoint
    double distanceToPoint1 = queriedPoint.distanceTo(point1);
    double distanceToPoint2 = queriedPoint.distanceTo(point2);
    
    // Calculate the bearings from the queriedPoint to each center point
    Angle bearingToPoint1;
    bearingToPoint1.setRadians(std::atan2(
        point1.y - queriedPoint.y,
        point1.x - queriedPoint.x
    ));
    bearingToPoint1.normalize();

    Angle bearingToPoint2;
    bearingToPoint2.setRadians(std::atan2(
        point2.y - queriedPoint.y,
        point2.x - queriedPoint.x
    ));
    bearingToPoint2.normalize();

    // Weights calculated based on the inverse of distance
    double weight1 = 1.0 / (distanceToPoint1 + 1e-6);
    double weight2 = 1.0 / (distanceToPoint2 + 1e-6);
    
    // Normalize the weights
    double totalWeight = weight1 + weight2;
    weight1 /= totalWeight;
    weight2 /= totalWeight;
    
    // Calculate the weighted average of bearings
    Angle weightedBearing;
    weightedBearing.setDegrees(
        bearingToPoint1.getDegrees() * weight1 + 
        bearingToPoint2.getDegrees() * weight2
    );
    weightedBearing.normalize();
    
    return weightedBearing;
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
 * @brief 
 * 
 * @param vehicle 
 * @return Angle 
 */
[[nodiscard]]
Angle Track::getDifferenceInBearing(const Vehicle& vehicle) const
{}

/**
 * @brief 
 * 
 * @return double 
 */
[[nodiscard]]
double Track::getROCofCurvature(const std::pair<InertialPose, InertialPose>& centerPoints) const 
{}

/**
 * @brief 
 * 
 * @return Angle 
 */
[[nodiscard]]
Angle Track::getBearing(const Point& position) const
{
    //from a point, what centrepoint(s) it is nearest to

}

/**
 * @brief 
 * 
 * @return Angle 
 */
[[nodiscard]]
Angle Track::getBearing(const Point& position, const std::pair<InertialPose, InertialPose>& nearestCenterPoints) const {

}

/**
 * @brief 
 * 
 * @return double 
 */
[[nodiscard]]
double Track::getROCofCurvature(const Point& position) const
{}



} //end of planning namespace