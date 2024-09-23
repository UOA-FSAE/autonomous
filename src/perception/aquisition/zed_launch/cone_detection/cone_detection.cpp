#include <iostream>
#include <chrono>
#include <cmath>
#include <mutex>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"

#include "yolo.hpp"
#include "sl/Camera.hpp"
#include "../zed_launch/zed_launch.hpp"

#include <NvInfer.h>

using namespace nvinfer1;
#define NMS_THRESH 0.4
#define CONF_THRESH 0.8

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moa_msgs/msg/detections.hpp"

std::mutex mtx;

std::vector<sl::uint2> cvt(const BBox &bbox_in) {
    std::vector<sl::uint2> bbox_out(4);
    bbox_out[0] = sl::uint2(bbox_in.x1, bbox_in.y1);
    bbox_out[1] = sl::uint2(bbox_in.x2, bbox_in.y1);
    bbox_out[2] = sl::uint2(bbox_in.x2, bbox_in.y2);
    bbox_out[3] = sl::uint2(bbox_in.x1, bbox_in.y2);
    return bbox_out;
}

void ZedLaunchNode::cone_detection_loop()
  {
    auto camera_config = zed.getCameraInformation().camera_configuration;
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404));
    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;

    // Creating the inference engine class
    std::string engine_name = "cone_detection_model.engine";
    Yolo detector;
    if (detector.init(engine_name)) {
        std::cerr << "Detector init failed!" << std::endl;
        return;
    }

    auto display_resolution = zed.getCameraInformation().camera_configuration.resolution;
    sl::Mat left_sl;
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
    sl::Objects objects;

    while (zed.grab() == sl::ERROR_CODE::SUCCESS) {
        // Get image for inference
        zed.retrieveImage(left_sl, sl::VIEW::LEFT);

        // Running inference
        auto detections = detector.run(left_sl, display_resolution.height, display_resolution.width, CONF_THRESH);

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
        zed.ingestCustomBoxObjects(objects_in);

        // Retrieve the tracked objects, with 2D and 3D attributes
        zed.retrieveObjects(objects, objectTracker_parameters_rt);

        // Publish the detected objects
        moa_msgs::msg::Detections detectionsMsg;

        int count = 0;
        for (sl::ObjectData& obj : objects.object_list) {
            geometry_msgs::msg::Point p;
            switch (obj.raw_label) {
                case 0:
                    p.x = obj.position[0] / 1000.0;
                    p.y = obj.position[1] / 1000.0;

                    if (sqrt(p.x * p.x + p.y * p.y) < 6.0) {
                        detectionsMsg.blue.push_back(p);
                    }

                    break;
                case 4:
                    p.x = obj.position[0] / 1000.0;
                    p.y = obj.position[1] / 1000.0;

                    if (sqrt(p.x * p.x + p.y * p.y) < 6.0) {
                        detectionsMsg.yellow.push_back(p);
                    }
                    
                    break;
                default:
                    p.x = obj.position[0] / 1000.0;
                    p.y = obj.position[1] / 1000.0;

                    if (sqrt(p.x * p.x + p.y * p.y) < 6.0) {
                        detectionsMsg.big_orange.push_back(p);
                    }

                    break;
            }
        }
        
        mtx.lock();

        zed.getPosition(cam_w_pose, sl::REFERENCE_FRAME::WORLD);
        detectionsMsg.car_pose.position.x = cam_w_pose.getTranslation().tx / 1000.0;
        detectionsMsg.car_pose.position.y = cam_w_pose.getTranslation().ty / 1000.0;

        float ox = cam_w_pose.getOrientation().ox;
        float oy = cam_w_pose.getOrientation().oy;
        float oz = cam_w_pose.getOrientation().oz;
        float ow = cam_w_pose.getOrientation().ow;

        float yaw = atan2(2.0f * (ow * oz + ox * oy), 1.0f - 2.0f * (oy * oy + oz * oz));

        detectionsMsg.car_pose.orientation.w = yaw;

        if (detectionsMsg.yellow.size() > 0 && detectionsMsg.blue.size() > 0) {
            cone_detection_publisher->publish(detectionsMsg);
        }

        mtx.unlock();
    }

    mtx.lock();

    camera_running = false;

    mtx.unlock();
}
