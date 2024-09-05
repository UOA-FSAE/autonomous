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
#include <chrono>
#include <cmath>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"

#include "yolo.hpp"
#include "sl/Camera.hpp"

#include <NvInfer.h>

using namespace nvinfer1;
#define NMS_THRESH 0.4
#define CONF_THRESH 0.3

#include <rclcpp/rclcpp.hpp>

#include "zed_components/zed_camera_component.hpp"
#include "std_msgs/msg/string.hpp"

std::vector<sl::uint2> cvt(const BBox &bbox_in) {
    std::vector<sl::uint2> bbox_out(4);
    bbox_out[0] = sl::uint2(bbox_in.x1, bbox_in.y1);
    bbox_out[1] = sl::uint2(bbox_in.x2, bbox_in.y1);
    bbox_out[2] = sl::uint2(bbox_in.x2, bbox_in.y2);
    bbox_out[3] = sl::uint2(bbox_in.x1, bbox_in.y2);
    return bbox_out;
}

class ConeDetectionNode : public rclcpp::Node
{
public:
  ConeDetectionNode(sl::Camera *zed)
  : Node("cone_detection_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("cone_detection", 10);
    cone_detection_loop(zed);
  }

private:
  void cone_detection_loop(sl::Camera *zed)
  {
    auto camera_config = zed->getCameraInformation().camera_configuration;
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404));
    auto camera_info = zed->getCameraInformation(pc_resolution).camera_configuration;

    // Creating the inference engine class
    std::string engine_name = "cone_detection_model.engine";
    Yolo detector;
    if (detector.init(engine_name)) {
        std::cerr << "Detector init failed!" << std::endl;
        return;
    }

    auto display_resolution = zed->getCameraInformation().camera_configuration.resolution;
    sl::Mat left_sl, point_cloud;
    cv::Mat left_cv;
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    sl::Objects objects;
    sl::Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();

    while (zed->grab() == sl::ERROR_CODE::SUCCESS) {
      // Get image for inference
      zed->retrieveImage(left_sl, sl::VIEW::LEFT);

      // Running inference
      auto detections = detector.run(left_sl, display_resolution.height, display_resolution.width, CONF_THRESH);

      // Get image for display
      left_cv = slMat2cvMat(left_sl);

      // Preparing for ZED SDK ingesting
      std::vector<sl::CustomBoxObjectData> objects_in;
      for (auto &it : detections) {
          sl::CustomBoxObjectData tmp;
          // Fill the detections into the correct format
          tmp.unique_object_id = sl::generate_unique_id();
          tmp.probability = it.prob;
          tmp.label = (int) it.label;
          tmp.bounding_box_2d = cvt(it.box);
          tmp.is_grounded = ((int) it.label == 0); // Only the first class (person) is grounded, that is moving on the floor plane
          // others are tracked in full 3D space                
          objects_in.push_back(tmp);
      }
      // Send the custom detected boxes to the ZED
      zed->ingestCustomBoxObjects(objects_in);

      // Retrieve the tracked objects, with 2D and 3D attributes
      zed->retrieveObjects(objects, objectTracker_parameters_rt);
    }
        
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

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
  sl::Camera *zed;
  sl::InitParameters init_parameters;
  init_parameters.sdk_verbose = true;
  init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
  init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed   

  if (argc > 1) {
    std::string zed_opt = argv[1];
    if (zed_opt.find(".svo") != std::string::npos)
      init_parameters.input.setFromSVOFile(zed_opt.c_str());
  }

  // Open the camera
  auto returned_state = zed->open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Camera Open " << returned_state << ", exit program." << std::endl;
      return EXIT_FAILURE;
  }

  zed->enablePositionalTracking();

  // Custom OD
  sl::ObjectDetectionParameters detection_parameters;
  detection_parameters.enable_tracking = true;
  detection_parameters.enable_segmentation = false; // designed to give person pixel mask with internal OD
  detection_parameters.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
  returned_state = zed->enableObjectDetection(detection_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    std::cerr << "enableObjectDetection " << returned_state << ", exit program." << std::endl;
    zed->close();
    return EXIT_FAILURE;
  }

  // Create a node.
  rclcpp::spin(std::make_shared<ConeDetectionNode>(zed));

  rclcpp::shutdown();

  return 0;
}

