// Copyright 2022 Stereolabs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "sl/Camera.hpp"

#include "zed_launch.hpp"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize any global resources needed by the middleware and the client library.
  // This will also parse command line arguments one day (as of Beta 1 they are not used).
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  /// Opening the ZED camera before the model deserialization to avoid cuda context issue
  sl::Camera zed;
  sl::InitParameters init_parameters;
  init_parameters.sdk_verbose = true;
  init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
  init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP; 
  

  if (argc > 1) {
    std::string zed_opt = argv[1];
    if (zed_opt.find(".svo") != std::string::npos)
      init_parameters.input.setFromSVOFile(zed_opt.c_str());
  }

  // Open the camera
  auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Camera Open " << returned_state << ", exit program." << std::endl;
      return EXIT_FAILURE;
  }
  
  zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE, sl::VIDEO_SETTINGS_VALUE_AUTO);
  zed.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, 5);
  zed.setCameraSettings(sl::VIDEO_SETTINGS::CONTRAST, 7);
  zed.setCameraSettings(sl::VIDEO_SETTINGS::SATURATION, 7);

  // Enable positional tracking
  zed.enablePositionalTracking();

  // Custom OD
  sl::ObjectDetectionParameters detection_parameters;
  detection_parameters.enable_tracking = true;
  detection_parameters.enable_segmentation = false; // designed to give person pixel mask with internal OD
  detection_parameters.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
  returned_state = zed.enableObjectDetection(detection_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    std::cerr << "enableObjectDetection " << returned_state << ", exit program." << std::endl;
    zed.close();
    return EXIT_FAILURE;
  }

  // Create a node.
  rclcpp::spin(std::make_shared<ZedLaunchNode>(zed));

  rclcpp::shutdown();

  return 0;
}

