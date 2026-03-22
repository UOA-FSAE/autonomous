/*
OVERVIEW:
  This header defines the `ZedLaunchNode` class, the ROS 2-facing wrapper around the
  ZED camera pipeline. It declares all publishers, shared camera state, and worker-loop
  entry points used by the implementation.

KEY RESPONSIBILITIES:
  1. ROS 2 Interface Definition: Declares a node that exposes cone detections, vehicle pose,
    vehicle velocity, and image data onto ROS topics.
  2. Runtime Wiring: Stores references to the ZED SDK camera object and internal pose state
    so the `.cpp` implementation can continuously read and publish sensor outputs.
  3. Parallel Work Declaration: Declares independent processing loops for detection,
    localisation, and velocity estimation that are launched on separate threads.

DEPENDENCIES:
  - ZED SDK (`sl::Camera`, `sl::Pose`)
  - ROS 2 C++ API (`rclcpp::Node`, `rclcpp::Publisher`)
  - ROS messages (`fsae_interfaces`, `geometry_msgs`, `sensor_msgs`)
*/

#include <chrono>
#include <memory>
#include <thread>
#include <iostream>

#include "sl/Camera.hpp"

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "fsae_interfaces/msg/detections.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

class ZedLaunchNode : public rclcpp::Node // Creates your custom class and tells it to inherit all the standard abilities of a ROS 2 Node.
{
public:
  ZedLaunchNode(sl::Camera& _zed) // Constructor
  : Node("cone_detection_node"), zed(_zed) // Initializer list. It officially names your node cone_detection_node in the ROS network, and saves the camera you passed in into a private variable called zed.
  {
    /*
    Initialize the camera pose the starting position of the car to (0,0,0) with no rotation.
    cam_w_pose: An object of type sl::Pose (from the ZED SDK), which represents the camera's position 
      and orientation in the world coordinate system.
    .pose_data: A member within the sl::Pose object that holds the actual numerical data for the pose
    .setIdentity(): Sets the pose_data to an identity transformation. In the context of 3D transformations, 
      an identity transformation means:
        Position: The object is at the origin (0, 0, 0) of the coordinate system.
        Orientation: The object has no rotation.
    */
    cam_w_pose.pose_data.setIdentity();

    /*
    this->create_publisher<MessageType>("topic_name", queue_size);
    Queue size of 10 means it will keep the last 10 messages.
    */
    cone_detection_publisher = this->create_publisher<fsae_interfaces::msg::Detections>("zed/cone_detection", 10);
    car_position_publisher = this->create_publisher<geometry_msgs::msg::Pose>("zed/car_position", 10);
    car_velocity_publisher = this->create_publisher<geometry_msgs::msg::Vector3>("zed/car_velocity", 10);
    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("zed/image", 10);

    /*
    Launch dedicated worker threads in parallel so cone detection, localisation,
    and velocity estimation can run concurrently.
    */
    std::thread t1(&ZedLaunchNode::cone_detection_loop, this);
    std::thread t2(&ZedLaunchNode::car_position, this);
    std::thread t3(&ZedLaunchNode::car_velocity, this);

    /*
    t1.join() tells the constructor, "Wait right here until t1 finishes its job." Might be a problem because t1 is 
    an infinite loop (it runs as long as the camera is on), the constructor never finishes. It never hits the }.
    */
    t1.join();
  }

private:
    // This is a reference to the actual ZED camera object (sl::Camera) that was initialized and opened in the main function.
    sl::Camera& zed; // By using a reference (&), the ZedLaunchNode doesn't own or copy the camera object; it simply gets a way to interact with the existing camera instance. 

    // This member stores the camera's current pose (position and orientation) in the world coordinate frame.
    sl::Pose cam_w_pose;

    // Runtime flags/config values consumed by the worker loops.
    bool camera_running = true;
    bool visualisation = true;
    std::string model_name = "cone_detection_model.engine"; // Custom object detection model.

    /*
    Up in the constructor, we configured the topic names and queue sizes for your publishers.
    These lines are where those publishers are actually stored in the computer's memory. 
    By keeping them private, it ensures that only this specific node is allowed to broadcast 
    messages on those topics.
    */
    rclcpp::Publisher<fsae_interfaces::msg::Detections>::SharedPtr cone_detection_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr car_position_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr car_velocity_publisher;

    // Declaring functions.
    void cone_detection_loop();
    void car_position();
    void car_velocity();
};