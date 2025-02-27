#include main.hpp

#include "rclcpp/rclcpp.hpp"
#include "track_utils/src/cone.hpp"
#include "track_utils/src/track.hpp"



    //Zane

    //Jonty

    //Winola

    //Siva

    //Pang

    //Adrian

    //Mahmoud

    //Waldo


// need namespace!!

enum state_t {
  UNINITIALIZED,
  INITIALIZED,
  TERMINATED
}

enum event_type {
  ACCEL,
  SKIDPAD,
  AUTOCROSS,
}

enum side_t{
  LEFT,
  RIGHT
}

struct current_event{ // use case?
  event_type event_type;
}


class TrajectoryPlanner(Node) {
  using namespace std::chrono_literals;
private:
  std::unique_ptr<planning::Track> raceTrack;
  state_t state = UNINITIALIZED;


public:
  TrajectoryPlanner() {
    
  }

    // 
    event_manager_subscription = this->create_subscription<moa_msgs::msg::Track>(
      "", 10, std::bind(&TrajectoryPlanner::event_manager_subscription, this, _1));

    // callback binding for perception cone information
    left_track_subscription = this->create_subscription<moa_msgs::msg::Track>(
      "", 10, std::bind(&TrajectoryPlanner::left_track_subscription, this, _1));
    right_track_subscription = this->create_subscription<moa_msgs::msg::Track>(
      "", 10, std::bind(&TrajectoryPlanner::right_track_subscription, this, _1));

    // 
    trajectory_publisher = this->create_publisher<std_msgs::msg::Trajectory>("planned_trajectory", 10);

    auto trajectory_timer_ = this.create_wall_timer(500ms, std::bind(&TrajectoryPlanner::tracjectory_callback, this));
    
    // callback binding for vehicle information


    // publisher binding for center points
    event_manager_subscripton(std::msgs& msg) {
      if 
      raceTrack = std::make_unique<planning::Track>();
    }

    // publisher  binding for the trajectory
        // state machine based on track conditions

        // center line following in straight areas of the track.

        // identifying a corner which needs special handling
        // initiate a corner
    void tracjectory_publish_callback() {
      


      trajectory_publisher.publish();
    }

    // event_manager_subscription
    void event_manager_subscription(const moa_msgs::msg::current_event& msg) {
      switch (msg.event_type)
      {
      case ACCEL:

        break;
      
      case SKIDPAD:
        /* code */
        break;
      
      case AUTOCROSS:
        /* code */
        break;
      
      default:
        break;
      }

    }

    // callback for perception cone information
    void track_update_callback(const moa_msgs::msg::Track& msg, const side_t side) {
      // msg:
      // geometry_msgs/Point[] cones 
      // float64 x
      // float64 y
      // float64 z

      // left blue, right yellow, need to populate intrinsic cone prop, or write another constructor for planning::Cone without it

      // alternative: create another enum for intrinsic properties so we don't need to make_shared<IntrinsicConeProp>
      // when we actually need intrinsic cone prop info we can write a lambda func to process the switch cases inline

      // big orange are start / finish 
      planning::Cone convert(const moa_msgs::msg::Cone& cone) {
        (side == LEFT) ? return Cone(planning::Point(cone.x, cone.y), planning::Conetype::BLUE) : return Cone(planning::Point(cone.x, cone.y), planning::Conetype::YELLOW);
      }

      // convert from geomtry_msg to planning version of Cone.
      std::vector<planning::Cone> cones_formatted(msg.cones.size());

      // Using std::transform to map the values
      std::transform(source.begin(), source.end(), cones_formatted.begin(), convert);

      //Track need function that takes raw cone message, takes those coords,
      //compares with any existing cones, append new cones if needed, then
      //re-triangulate centre line
      raceTrack.____(cones_formatted);

      // give 
    }

    void left_track_subscription(const moa_msgs::msg::Track& msg) {
      
    }
    void right_track_subscription(const moa_msgs::msg::Track& msg) {
      
    }

    // callback for vehicle information


    // publisher for center points


    // publisher for the trajectory





}














int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPlanner>());
  rclcpp::shutdown();
  return 0;
}

