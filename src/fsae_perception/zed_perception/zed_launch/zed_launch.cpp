/*
Apache License 2.0.
Stereolabs (the company that makes the ZED camera) wrote the original 
"skeleton" of this code as an example or tutorial:
    Copyright 2022 Stereolabs

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
OVERVIEW:
  This is the primary entry point for the FSAE vision pipeline. It is responsible for 
  initializing the hardware (ZED 2i camera) and spinning up the ROS 2 node that handles 
  the entire perception stack. 
KEY RESPONSIBILITIES:
  1. Hardware Initialization: Secures a CUDA context for the ZED camera before the neural 
  network loads, preventing GPU memory collisions.
  2. ZED Configuration: Sets the camera to ULTRA depth mode, defines the coordinate system 
  (Right-Handed Z-Up), and enables positional tracking.
  3. Custom Object Detection: Disables the ZED's internal AI and configures it to accept 
  custom 2D bounding boxes (which will be provided by our YOLO TensorRT model).
  4. Node Spin-up: Hands the configured camera object to the `ZedLaunchNode` and starts 
  the ROS 2 event loop.

DEPENDENCIES: 
  - ZED SDK (sl::Camera)
  - rclcpp (ROS 2 C++ client library)
  - zed_launch.hpp (Node definitions and thread loops)
*/

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "zed_launch.hpp"
// Imports the ZED SDK library so you can control the physical camera.
#include "sl/Camera.hpp"

int main(int argc, char * argv[])
{
  /*
  This forces the program to print std::cout messages to your terminal immediately, 
  rather than storing them up in a buffer, to not miss an error message.
  */
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  /*
  This officially starts the ROS 2 system for this specific program. 
  You have to call this before making any ROS nodes.
  */
  rclcpp::init(argc, argv);

  sl::Camera zed;
  sl::InitParameters init_parameters;
  init_parameters.sdk_verbose = true; // Tells the ZED to print lots of helpful debugging info to the terminal.
  init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA; // Tells the ZED to use its most accurate (but computationally heavy) algorithm for calculating 3D depth.
  init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP; // Sets the 3D axis. In this case, X is forward, Y is left, and Z is straight up.
  init_parameters.depth_minimum_distance = 0.5f; // Ignore depth closer than 0.5 m. The ZED 2i's stereo baseline cannot reliably triangulate below this range, and the car body itself would generate noise at very short distances.
  init_parameters.depth_maximum_distance = 25000.0f; // Match DIST_FAR_M: the pipeline already discards cones beyond 25 m in the publishing loop, so computing depth beyond this range wastes GPU cycles and increases the noise floor.
  
  /*
  It allows you to run this exact code on your laptop using a pre-recorded video file 
  instead of having to plug into the physical ZED camera on the car.
  
  RUN MODE SELECTOR: Live Camera vs. Recorded Video
  By default, running this node with a single command (argc == 1) connects 
  directly to the physical ZED camera on the car. 
  
  However, if a second command-line argument is provided (argc > 1, hence it 
  would read inside if-statement) and that argument is an ".svo" file, the 
  program will bypass the physical hardware and read from the recorded video instead. 
  This allows the team to test the vision pipeline offline without needing the actual car.
  */
  if (argc > 1) {
    std::string zed_opt = argv[1]; // Grabs the second word you typed and saves it as a string variable called zed_opt.
    if (zed_opt.find(".svo") != std::string::npos) // .svo stands for Stereolabs Video Object. It's a special 3D video file format recorded by the ZED. This line checks to see if the word you typed contains ".svo".
      init_parameters.input.setFromSVOFile(zed_opt.c_str()); // If it is an SVO file, it tells the camera's initialization settings to read from that file instead of looking for a physical USB connection.
  }

  /*
  By running zed.open() before doing anything else, we forced the ZED SDK to go first. 
  ZED claims its workspace on the GPU, gets comfortable, and then the YOLO model is allowed to 
  load in and claim the remaining space. It's a traffic control measure!

  If the ROS node boots up and allows both the ZED SDK and TensorRT to try and rent a workspace 
  on the GPU at the exact same microsecond, they can trip over each other. They both try to be 
  the "boss" of the GPU, the memory addresses get confused, and the entire program instantly 
  crashes (a Segmentation Fault).
  */
  auto returned_state = zed.open(init_parameters); // Commands the ZED SDK to boot up and claim the GPU memory.
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Camera Open " << returned_state << ", exit program." << std::endl;
      return EXIT_FAILURE;
  }

  /*
  Enable positional tracking. This turns on the ZED's internal IMU (gyroscope/accelerometer) and visual odometry.
  This is how the detectionsMsg.car_pose gets its data later on.
  */
  zed.enablePositionalTracking();

  /*
  The ZED camera actually has its own built-in AI for finding people and cars. This block turns off the internal AI and tells the ZED to expect outside help from our YOLO model.
  */
  sl::ObjectDetectionParameters detection_parameters;
  detection_parameters.enable_tracking = true; // Tells the ZED SDK to track objects across multiple frames (so if a cone is partially blocked for a split second, it doesn't instantly forget it).
  detection_parameters.enable_segmentation = false; // We only need bounding boxes, not pixel-perfect outlines (segmentation masks) of the cones. Turning this off saves massive amounts of computing power.
  detection_parameters.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS; // Tells the ZED: "Do not run your internal AI. I am going to feed you custom 2D boxes."
  returned_state = zed.enableObjectDetection(detection_parameters); // Applies these settings to the camera.
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    std::cerr << "enableObjectDetection " << returned_state << ", exit program." << std::endl;
    zed.close();
    return EXIT_FAILURE;
  }

  /*
  It creates your custom ROS 2 node (ZedLaunchNode), passes it the perfectly configured zed camera object, 
  and spin() tells ROS to keep this node running indefinitely until someone hits Ctrl+C.
  */
  rclcpp::spin(std::make_shared<ZedLaunchNode>(zed));

  rclcpp::shutdown();

  return 0;
}

