// #include "track_utils/include/track_utils/curvature_calculator.hpp"

// namespace track_utils {

// // Function to calculate curvature given three points
// std::optional<double> calculateCurvature(
//     const geometry_msgs::msg::Point& p1,
//     const geometry_msgs::msg::Point& p2,
//     const geometry_msgs::msg::Point& p3) {

//     // Length of sides of the traingle formed by the three points
//     double a = std::hypot(p2.x - p1.x, p2.y - p1.y);
//     double b = std::hypot(p3.x - p2.x, p3.y - p2.y);
//     double c = std::hypot(p3.x - p1.x, p3.y - p1.y);

//     //semi perimeter and area of triangle
//     double s = (a + b + c) / 2.0;
//     double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

//     if (area == 0.0) {  
//         return std::nullopt; 
//     }

//     //Heron's formula to calcuate radius
//     double radius = (a * b * c) / (4.0 * area);
//     double curvature = 1.0 / radius;

//     return curvature;
// }

// // Function to calculate bearing difference between cones and positions in degrees
// std::optional<double> calculateBearingDifference(
//     const geometry_msgs::msg::Point& left_cone,
//     const geometry_msgs::msg::Point& right_cone,
//     const geometry_msgs::msg::Point& current_position,
//     const geometry_msgs::msg::Point& previous_position) {

//     //vector from left cone to right cone
//     double x_cone_diff = right_cone.x - left_cone.x;
//     double y_cone_diff = right_cone.y - left_cone.y;

//     //finding cone vector's normal 
//     double normal_x = y_cone_diff;
//     double normal_y = -(x_cone_diff);
//     double normal_magnitude = std::hypot(normal_x, normal_y);

//     if(normal_magnitude == 0.0){
//         return std::nullopt;
//     }

//     //convert cone's normal vector to unit vector
//     normal_x /= normal_magnitude;
//     normal_y /= normal_magnitude;

//     //calculate car's heading
//     double heading_x = current_position.x - previous_position.x;
//     double heading_y = current_position.y - previous_position.y;
//     double heading_magnitude = std::hypot(heading_x, heading_y);
    
//     if (heading_magnitude == 0.0){
//         return std::nullopt;
//     }

//     //convert heading vector to unit vector
//     heading_x /= heading_magnitude;
//     heading_y /= heading_magnitude;

//     //calculate dot product and limit range
//     double dot_product = normal_x * heading_x + normal_y * heading_y;
//     dot_product = std::clamp(dot_product, -1.0, 1.0);

//     //angle between two unit vectors
//     double angle_radians = std::acos(dot_product);

//     //return angle in degrees
//     return angle_radians * (180/M_PI);

// }

// std::optional<geometry_msgs::msg::Point> detectCornerStart(
//     const std::vector<geometry_msgs::msg::Point>& upcoming_points) {

//     if (upcoming_points.size() < 3){
//         return std::nullopt;
//     }

//     for (size_t i = 0; i < upcoming_points.size() -2; i++){
        
//         auto curvature = calculateCurvature(
//             upcoming_points[i],
//             upcoming_points[i+1],
//             upcoming_points[i+2]
//         );

//         if (curvature.has_value() && std::abs(curvature.value()) > CURVATURE_THRESHOLD){
//             return upcoming_points[i];
//         }
        
//     }

//     return std::nullopt;

// }

// //time from how fast we can decelerate
// //different time parameters e.g. delay for info from camera, delay for computing, delay for taking action
// double calculateArcLength(double time, double speed){
//     return time * speed;
// }

// double determineDistanceTraveled() {
//     actionDelay = 1; // time to carry out corrective actions some function of speed.

//     constexpr double const_time_delay = PERCEPTION_DELAY + RESPONSE_DELAY;
//     double variable_time_delay = 
// }

// //Function toi calculate corner end point and final bearinng

// }  // namespace track_utils
