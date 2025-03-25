#include main.hpp

// ros messages
#include "rclcpp/rclcpp.hpp"
#include "track_utils/src/cone.hpp"
#include "track_utils/src/track.hpp"
#include "track_utils/src/vehicle.hpp"

//
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/uint16.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

// Zane

// Jonty

// Winola

// Siva

// Pang

// Adrian

// Mahmoud

// Waldo

namespace planning::ConeType = conetype;

enum state_t
{
	UNINITIALIZED,
	INITIALIZED,
	TERMINATED
}

enum event_type
{
	ACCEL,
	SKIDPAD,
	AUTOCROSS,
}

enum side_t
{
	LEFT,
	RIGHT
}

struct current_event
{ // use case?
	event_type event_type;
}

struct CarHeuristics
{											// update car heuristics function/callback, use wall timer (e.g update interval). calls other helper funtcions to get these
	double lateral_position;				// helper function, finds the projection of the car onto the centerline
	double car_position_bearing_difference; // write helper function that uses get difference in bearing
	double car_velocity;					// should be provided by a ros topic (find which one).
}

// TODO: need separate function that uses car heuristics to generate the trajectory

class TrajectoryPlanner(Node)
{
	using namespace std::chrono_literals;

	std::unique_ptr<planning::Track> raceTrack;
	state_t state = UNINITIALIZED;

	const std::map<ConeType, IntrinsicConeProp> coneProp {{conetype::BIG_ORANGE, IntrinsicConeProp(0.285, 0.505)}, {conetype::SMALL_ORANGE, IntrinsicConeProp(0.228, 325)}, {conetype::BLUE, IntrinsicConeProp(0.228, 325)}, {conetype::YELLOW, IntrinsicConeProp(0.228, 325)}};
	CarHeuristics car_heuristics;
	Vehicle vehicle;

	this->declare_parameter("autonomous_event_type", "accel"); // use lower case for param inputs
	this->declare_parameter("enable_centerpoints", "false");

	auto mAutonomousEventType = this->get_parameter("autonomous_event_type").as_string();

public:
	TrajectoryPlanner() : Node("minimal_param_node")
	{
		// initialise raceTrack based on parameter input
		this->update_race_event_type();
	}

private:
	// autonomous event type

	// car position and velocity subscribers
	auto car_position_subscription = this->create_subscription<>;

	// subscriptions
	auto car_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/vehicle/pose", 10, // TODO: Change the topic as required, this is just a placeholder!
		std::bind(&TrajectoryPlanner::carPoseCallback, this, _1););

	auto car_velocity_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
		"/vehicle/velocity", 10, // TODO: Change the topic as required, this is just a placeholder!
		std::bind(&TrajectoryPlanner::carVelocityCallback, this, _1););

	auto event_manager_subscription = this->create_subscription<moa_msgs::msg::Track>(
		"", 10, std::bind(&TrajectoryPlanner::event_manager_subscription, this, _1));

	// callback binding for perception cone information
	auto left_track_subscription = this->create_subscription<moa_msgs::msg::Track>(
		"", 10, std::bind(&TrajectoryPlanner::left_track_subscription, this, _1));

	auto right_track_subscription = this->create_subscription<moa_msgs::msg::Track>(
		"", 10, std::bind(&TrajectoryPlanner::right_track_subscription, this, _1));

	auto pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
		"/car_pose", 10, std::bind(&TrajectoryPlanner::poseCallback, this, std::placeholders::_1));

	// publishers
	auto trajectory_publisher = this->create_publisher<std_msgs::msg::Trajectory>("planned_trajectory", 10);
	auto centerpoint_publisher = this->create_publisher<std_msgs::msg::Track>("Centerpoints", 10);

	auto update_vehicle_heuristic_timer_ = this.create_wall_timer(500ms, std::bind(&TrajectoryPlanner::trajectory_publish_callback, this));
	auto centerpoints_timer_ = this.create_wall_timer(500ms, std::bind(&TrajectoryPlanner::centerpoints_publish_callback, this));

	// TODO: callback binding for vehicle information

	// carPose callback that updates the
	
	// Car veholcity callback that updates the velocity of the vehicle object 

	// TODO: publisher  binding for the trajectory
	//  state machine based on track conditions
	/**
 * @brief Updates car heuristics based on current vehicle state and track
 */
void updateCarHeuristics() {
    if (!raceTrack) {
        RCLCPP_WARN(this->get_logger(), "Cannot update car heuristics: race track not initialized");
        return;
    }
    
    // Get nearest center points
    auto nearestPoints = raceTrack->getNearestCenterPoints(vehicle.pos);
    
    if (nearestPoints.first && nearestPoints.second) {
        // Calculate lateral position (distance from centerline)
        const Point& p1 = nearestPoints.first.value().pos;
        const Point& p2 = nearestPoints.second.value().pos;
        
        // Vector from p1 to p2 (track direction)
        double track_dx = p2.x - p1.x;
        double track_dy = p2.y - p1.y;
        double track_length = std::sqrt(track_dx * track_dx + track_dy * track_dy);
        
        if (track_length > 0) {
            // Normalize track direction vector
            double norm_track_dx = track_dx / track_length;
            double norm_track_dy = track_dy / track_length;
            
            // Vector from p1 to vehicle
            double vehicle_dx = vehicle.pos.x - p1.x;
            double vehicle_dy = vehicle.pos.y - p1.y;
            
            // Project vehicle vector onto track direction
            double proj = vehicle_dx * norm_track_dx + vehicle_dy * norm_track_dy;
            
            // Vector from track to vehicle (perpendicular component)
            double perp_x = vehicle_dx - proj * norm_track_dx;
            double perp_y = vehicle_dy - proj * norm_track_dy;
            
            // Lateral position is the length of the perpendicular vector
            car_heuristics.lateral_position = std::sqrt(perp_x * perp_x + perp_y * perp_y);
            
            // Determine if vehicle is to the left or right of track
            // Cross product sign determines which side
            double cross = norm_track_dx * vehicle_dy - norm_track_dy * vehicle_dx;
            if (cross < 0) {
                car_heuristics.lateral_position = -car_heuristics.lateral_position;
            }
        }
        
        // Update bearing difference
        auto bearingDiff = raceTrack->getDifferenceInBearing(vehicle);
        if (bearingDiff) {
            car_heuristics.car_position_bearing_difference = bearingDiff.value().getDegrees();
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Unable to find nearest center points for vehicle position");
    }
    
    // Velocity is already updated in carVelocityCallback
}
	// center line following in straight areas of the track.

	// identifying a corner which needs special handling


	// initiate a corner
	void trajectory_publish_callback()
	{
		trajectory_helper();

		moa_msg::trajectory;
		trajectory_publisher.publish();
	}

	// TODO: helper to update heuristics about the car relative to the track.
	void trajectory_helper(const planning::Track &track, const Vehicle &vehicle)
	{
		// update lateral position of the car in the track. Have a struct that
		// stores that 'state data'

		// Get nearest center points on the track
		auto nearestPoints = track.getNearestCenterPoints(vehicle.pos);

		if (!nearestPoints.first || !nearestPoints.second)
		{
			std::cerr << "Error: Unable to determine nearest center points.\n";
			return;
		}

		// Extract the relevant points
		planning::Point p1 = nearestPoints.first.value().pos;
		planning::Point p2 = nearestPoints.second.value().pos;
		
		// Compute lateral position (perpendicular distance to track segment)
		double dx = p2.x - p1.x;
		double dy = p2.y - p1.y;
		double lengthSquared = dx * dx + dy * dy;
		
		// angular displacement of the car's bearing to the track

		//subscribe to topic that gives the car's velocity
	}

	// event_manager_subscription
	// void event_manager_subscription(const moa_msgs::msg::current_event& msg) {
	// 	switch (msg.event_type) 	
	// 	{
	// 	case ACCEL:
	// 		raceTrack = std::make_unique<planning::AccelTrack>();
	// 		break;
		
	// 	case SKIDPAD:
	// 		raceTrack = std::make_unique<planning::SkidPadTrack>();
	// 		/* code */
	// 		break;
		
	// 	case AUTOCROSS:
	// 		raceTrack = std::make_unique<planning::AutocrossTrack>();
	// 		/* code */
	// 		break;
		
	// 	default:
	// 		break;
	// 	}
	// }
	
	// uses autonomous_event_type parameter to initialize Track raceTrack
	event_type update_race_event_type() {
		event_type event_type_from_param;
	
		switch (mAutonomousEventType)
		{
		case "accel":
			event_type_from_param = ACCEL;
			raceTrack = std::make_unique<planning::AccelTrack>();
			break;
		case "skidpad":
			event_type_from_param = SKIDPAD;
			raceTrack = std::make_unique<planning::SkidPadTrack>();
			break;		
		case "autocross":
			event_type_from_param = AUTOCROSS;
			raceTrack = std::make_unique<planning::AutocrossTrack>();
			break;
		}
		RCLCPP_INFO(get_logger(), "initalized %s track type", mAutonomousEventType.c_str());		
		return event_type_from_param;
	}

	//TODO: callback for perception cone information
	void track_update_callback(const moa_msgs::msg::Track& msg, const side_t side) {
		// Track: 	
		// geometry_msgs/Point[] cones 

		// geometry_msgs/Point:
		// float64 x
		// float64 y
		// float64 z

		// left blue, right yellow, need to populate intrinsic cone prop, or write
		// another constructor for planning::Cone without it

		// confirm cones would have ids, if existing id: updte coord, 
		// if else new id: insert new Cone to map
		std::map<uint16_t, planning::Cone> cones(msg.cones.size());
		
		void consume(const geometry_msgs::Point& cone) { 
		
			auto match = cones.find(cone.id);
			if (match != cones.end())
			{
				match->second.setPos(planning::Point(cone.x, cone.y));
			} else {
				if (side == LEFT) {
					cones.insert({cone.id, Cone(planning::Point(cone.x, cone.y), planning::Conetype::BLUE)});
				} else {					
					cones.insert({cone.id, Cone(planning::Point(cone.x, cone.y), planning::Conetype::YELLOW)});
				}	        
			}
		}

		for (geometry_msgs::Point conePoint : msg) {
			consume(conePoint);		
		}
	}

	void lefttrack_subscription(const moa_msgs::msg::Track& msg) {
		track_update_callback(msg, LEFT);
	}
	void right_track_subscription(const moa_msgs::msg::Track& msg) {
		track_update_callback(msg, RIGHT);
	}

	// callback for vehicle information (Jonty) : uopdating the struct using the
	// 3 functions below

	//function for car position 
 
	//function for car bearing differnce
	double updateBearingDiff(vehicle){
		return track.getBearingDifference(vehicle);	
	}

	void centerpoints_publish_callback() {	
		if (this->get_parameter("enable_centerpoints").as_bool()){
		
            //track msg to hold centerpoints
			moa_msgs::msg::Track centerpoints_msg;
			
            //access racetrack centerpoints,
			const auto& track_centerpoints = racetrack->getCenterPoints();
			 
			//populate msg with centerpoints
			centerpoints_msg.cones.reserve(track_centerpoints.size());
			
            for (const auto& point : track_centerpoints){
				moa_msgs::msg::Point center_point;			
				center_point.x = point.pos.x;
				center_point.y = point.pos.y;
				
				centerpoints_msg.cones.push_back(center_point);
				
				//set header information
				centerpoints_msg.header.stamp = this->now();
				centerpoints_msg.header.frame_id = "map";
			}	
			//Publish centerpoints
				RCLCPP_INFO(this->get_logger(), "publishing %zu centerpoints", centerpoints_msg.cones.size());
				centerpoint_publisher->publish(centerpoints_msg);
        }
	}	
	
} //end of TrajectoryPlanner
 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrajectoryPlanner>());
	rclcpp::shutdown();
	return 0;
}
