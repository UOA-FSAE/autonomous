#include "util.hpp"
namespace planning {

/**
 * @brief determine whether common_vertex point queried_head is behind or on the line segment defined by points common_vertex and projection_head
 * 
 * @param common_vertex :one point in segemnt
 * @param projection_head :other point in question
 * @param queried_head :point queried
 * @return true 
 * @return false 
 */
bool in_opposing_or_normal_direction(const Point& common_vertex, const Point& projection_head, const Point& queried_head) {
    Point line_of_projection = projection_head - common_vertex;
    Point projecting_line = queried_head - common_vertex;
    
    double dot = projecting_line.dot(line_of_projection);
    
    return dot <= 0; // Only check if it's behind or perpendicular
}

/**
 * Determines if point `c` is behind the line segment `line_of_projection`.
 * 
 * A point is considered "behind" the line segment if it lies on the opposite side
 * of the line segment as the direction from `a` to `b`. In aprticular Used to make sure that getnearestCenterpoints
 * will always returntwo cp's on either side of queried point. If the points form
 * an accute angle the points are considered lying in the same direction. If the
 * points form a obtuse angle they are considerd lying in the oposite directions.
 * 
 * @param common_vertex: The starting point of the line segment.
 * @param fixed_head The head of the line segment beginning from common_vertex.
 * @param queried_head The head of the line segment in which to determine
 * the relative location.
 * @return True if `queried_vertex` is behind the line segment `line_of_projection`, false otherwise.
 */
bool in_opposing_direction(const Point& common_vertex, const Point& fixed_head, const Point& queried_head) {
    //TODO: Siva has changed this too: same reasoning as above
    
    Point line_of_projection = fixed_head - common_vertex; // Direction vector ab
    Point projecting_line = queried_head - common_vertex; // Vector ac
    
    // return v.dot(d) < 0; // Check projection

    double dot = line_of_projection.dot(projecting_line); // cp to determine rel position
    return dot < 0;
}

// Function to get the magnitude of AC's component along AB
double projection_magnitude(const Point& a, const Point& b, const Point& c) {
    
    
    Point ab = b - a; // Vector AB
    Point ac = c - a; // Vector AC
    
    double ab_magnitude = ab.mag(); // Magnitude of AB
    if (ab_magnitude == 0) return 0; // Prevent division by zero

    // Calculate the projection magnitude: (ac . ab) / |ab|
    return ac.dot(ab) / ab_magnitude;
}

// Compute Euclidean distance between two points
double distance(const Point& p1, const Point& p2) {
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// Lagrange interpolation of curvature using arc length
// double interpolate_curvature(const Point& A, double kappa_A, 
//                              const Point& B, double kappa_B, 
//                              const Point& C) {
//     // Compute arc-length distances
//     double s_A = 0; // Reference point
//     double s_B = A.distanceTo(B);
//     double s_C = A.distanceTo(C);

//     // Lagrange interpolation formula using arc-length
//     return kappa_A * (s_C - s_B) / (s_A - s_B) + 
//            kappa_B * (s_C - s_A) / (s_B - s_A);
// }
// Lagrange interpolation of curvature using arc length and handling edge cases properly
double interpolate_curvature(const Point& A, double kappa_A, 
                             const Point& B, double kappa_B, 
                             const Point& C) {
    // Handle the special case: A and B are the same point
    if (A.distanceTo(B) < 1e-10) {
        return kappa_A;
    }

    // Compute arc-length distances correctly handling points before A
    double s_A = 0; // Reference point
    double s_B = A.distanceTo(B);
    
    // Instead of just distance, use projection to handle points before A correctly
    double proj_magnitude = projection_magnitude(A, B, C);
    double s_C = proj_magnitude * s_B / A.distanceTo(B);
    
    // Check if C is behind A (in the opposite direction of B)
    if (in_opposing_direction(A, B, C)) {
        // If C is behind A, we need to make s_C negative
        s_C = -A.distanceTo(C);
    } else {
        // If C is in the same direction as B, use the projection
        s_C = proj_magnitude;
    }
    
    // Lagrange interpolation formula using arc-length
    return kappa_A * (s_C - s_B) / (s_A - s_B) + 
           kappa_B * (s_C - s_A) / (s_B - s_A);
}


}