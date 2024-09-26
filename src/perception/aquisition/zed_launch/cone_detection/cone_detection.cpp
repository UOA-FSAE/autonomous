#include <iostream>
#include <chrono>
#include <cmath>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"

#include "yolo.hpp"
#include "sl/Camera.hpp"
#include "../zed_launch/zed_launch.hpp"

#include <NvInfer.h>

using namespace nvinfer1;
#define NMS_THRESH 0.7
#define CONF_THRESH 0.8

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moa_msgs/msg/detections.hpp"
#include "sensor_msgs/msg/image.hpp"

std::mutex mtx;

static void draw_objects(cv::Mat const& image,
                         cv::Mat &res,
                         sl::Objects const& objs,
                         std::vector<std::vector<int>> const& colors)
{
    res = image.clone();
    cv::Mat mask{image.clone()};
    for (sl::ObjectData const& obj : objs.object_list) {
        size_t const idx_color{obj.id % colors.size()};
        cv::Scalar const color{cv::Scalar(colors[idx_color][0U], colors[idx_color][1U], colors[idx_color][2U])};

        cv::Rect const rect{static_cast<int>(obj.bounding_box_2d[0U].x),
                            static_cast<int>(obj.bounding_box_2d[0U].y),
                            static_cast<int>(obj.bounding_box_2d[1U].x - obj.bounding_box_2d[0U].x),
                            static_cast<int>(obj.bounding_box_2d[2U].y - obj.bounding_box_2d[0U].y)};
        cv::rectangle(res, rect, color, 2);

        char text[256U];
        sprintf(text, "Class %d - %.1f%%", obj.raw_label, obj.confidence);
        if (obj.mask.isInit() && obj.mask.getWidth() > 0U && obj.mask.getHeight() > 0U) {
            const cv::Mat obj_mask = slMat2cvMat(obj.mask);
            mask(rect).setTo(color, obj_mask);
        }

        int baseLine{0};
        cv::Size const label_size{cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine)};

        int const x{rect.x};
        int const y{std::min(rect.y + 1, res.rows)};

        cv::rectangle(res, cv::Rect(x, y, label_size.width, label_size.height + baseLine), {0, 0, 255}, -1);
        cv::putText(res, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 255, 255}, 1);
    }
    cv::addWeighted(res, 0.5, mask, 0.8, 1, res);
}

std::vector<sl::uint2> cvt(const BBox &bbox_in) {
    std::vector<sl::uint2> bbox_out(4);
    bbox_out[0] = sl::uint2(bbox_in.x1, bbox_in.y1);
    bbox_out[1] = sl::uint2(bbox_in.x2, bbox_in.y1);
    bbox_out[2] = sl::uint2(bbox_in.x2, bbox_in.y2);
    bbox_out[3] = sl::uint2(bbox_in.x1, bbox_in.y2);
    return bbox_out;
}

cv::Rect get_rect(BBox box) {
    return cv::Rect(round(box.x1), round(box.y1), round(box.x2 - box.x1), round(box.y2 - box.y1));
}

void ZedLaunchNode::cone_detection_loop()
  {
    auto camera_config = zed.getCameraInformation().camera_configuration;
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404));
    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration;

    // Creating the inference engine class
    // std::string engine_name = "cone_detection_yolo10l.engine";
    std::string engine_name = "cone_detection_model.engine";
    // std::string engine_name = "cone_detection_yolo9c.engine";
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

        if (visualisation) {
            // publish image
            cv::Mat left_cv;
            left_cv = slMat2cvMat(left_sl);
            draw_objects(left_cv, left_cv, objects, CLASS_COLORS);
            sensor_msgs::msg::Image::SharedPtr imageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", left_cv).toImageMsg();
            image_publisher->publish(*imageMsg);
        }

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
