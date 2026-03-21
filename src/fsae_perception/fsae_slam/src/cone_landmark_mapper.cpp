/**
 * Cone Landmark Mapper — maintains a persistent map of cone positions on the track.
 *
 * Subscribes to /cone_detection (fsae_interfaces/Detections) which provides the
 * car's pose and lists of blue/yellow cones in the camera's local frame.
 *
 * For each detection frame:
 *   1. Transforms cone positions from the car's local frame to the global frame.
 *   2. Matches each observed cone to the nearest known landmark.
 *   3. If matched: refines the landmark's position via a Kalman filter.
 *   4. If unmatched: inserts a new landmark into the map.
 *   5. Periodically prunes low-confidence landmarks.
 *
 * Publishes:
 *   /left_track  (fsae_interfaces/Track) — refined left boundary cone positions
 *   /right_track (fsae_interfaces/Track) — refined right boundary cone positions
 *
 * See reference_cone_landmark_mapper.py for the original Python implementation.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fsae_interfaces/msg/detections.hpp>
#include <fsae_interfaces/msg/track.hpp>

class ConeLandmarkMapper : public rclcpp::Node {
public:
  ConeLandmarkMapper() : Node("cone_landmark_mapper") {
    // Subscriber: raw cone detections from perception
    cone_sub_ = this->create_subscription<fsae_interfaces::msg::Detections>(
        "cone_detection", rclcpp::SensorDataQoS(),
        [this](const fsae_interfaces::msg::Detections::SharedPtr msg) {
          this->cones_callback(msg);
        });

    // Publishers: refined track boundaries
    left_pub_ = this->create_publisher<fsae_interfaces::msg::Track>("left_track", 10);
    right_pub_ = this->create_publisher<fsae_interfaces::msg::Track>("right_track", 10);

    RCLCPP_INFO(this->get_logger(), "ConeLandmarkMapper initialised — waiting for detections");
  }

private:
  void cones_callback(const fsae_interfaces::msg::Detections::SharedPtr /*msg*/) {
    // TODO: implement landmark matching + Kalman update
    // See reference_cone_landmark_mapper.py for the algorithm
  }

  rclcpp::Subscription<fsae_interfaces::msg::Detections>::SharedPtr cone_sub_;
  rclcpp::Publisher<fsae_interfaces::msg::Track>::SharedPtr left_pub_;
  rclcpp::Publisher<fsae_interfaces::msg::Track>::SharedPtr right_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeLandmarkMapper>());
  rclcpp::shutdown();
  return 0;
}
