/*
OVERVIEW:
This file contains the core logic for the custom object detection pipeline. It acts as the bridge between our custom neural network 
(YOLO running via TensorRT) and the ZED SDK's 3D spatial tracking, all wrapped inside a continuous ROS 2 loop.

KEY RESPONSIBILITIES:
    1. YOLO Inference: Grabs 2D images from the ZED camera and passes them to the TensorRT neural network to detect bounding boxes around cones.
    2. Sensor Fusion (3D Tracking): Takes those 2D bounding boxes and feeds them back into the ZED SDK. The ZED then calculates the 3D depth 
       of those boxes and tracks them over time (giving them unique IDs and velocities).
    3. Visualization: Draws the bounding boxes, confidence scores, and class labels onto the camera stream and publishes it as a ROS image topic 
       for debugging (e.g., viewing in RViz).
    4. Data Extraction & ROS Publishing: Extracts the 3D coordinates of the tracked cones, filters them by distance (ignoring cones further than 
       6 meters), categorizes them by color (blue, yellow, orange), grabs the car's current pose, and publishes everything as a single 
       fsae_interfaces::msg::Detections message.

DEPENDENCIES:
    - ZED SDK (sl::Camera, sl::Objects): For retrieving images and 3D spatial data.
    - OpenCV (cv::Mat): For drawing bounding boxes and rendering text.
    - TensorRT (NvInfer.h, yolo.hpp): For running the high-speed YOLO object detection model on the GPU.
    - ROS 2 & Messages: rclcpp, sensor_msgs/Image, geometry_msgs/Pose, and our custom fsae_interfaces/Detections.
*/

#include <iostream>
#include <chrono>
#include <cmath>
#include <mutex> // For thread safety when accessing shared data.
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"

#include "yolo.hpp"
#include "sl/Camera.hpp"
#include "../zed_launch/zed_launch.hpp"

#include <NvInfer.h> // Brings in NVIDIA TensorRT, which is the high-performance engine that runs our YOLO model on the GPU.

using namespace nvinfer1;
#define CONF_THRESH 0.8 // Threshold for the YOLO model. If the AI is less than 80% confident that it sees a cone, it will ignore it, reducing false positives.
#define NMS_THRESH 0.4 // NMS stands for Non-Maximum Suppression. If the AI draws two overlapping boxes on the same cone, 
                      // this threshold tells the code to combine/delete the weaker box if they overlap by more than 40%.

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "fsae_interfaces/msg/detections.hpp"
#include "sensor_msgs/msg/image.hpp"

/*
Creates a global "lock". Before modifying shared data (like the camera's pose or shutting down the camera), a thread will 
"lock" this mutex. If another thread tries to access it, it has to wait in line until the first thread "unlocks" it.
*/
std::mutex mtx;

/*
This function is solely responsible for visual debugging. If we have a monitor plugged in (or are viewing the ROS image topic), 
this is the code that draws the colored boxes and text over the cones.

This function takes the raw image from the camera and the list of cones tracked by the ZED SDK. For every cone it sees, 
it figures out its location on the 2D screen, draws a colored rectangle around it, and writes the class (e.g., blue cone, yellow cone) 
and the AI's confidence score above it. It also uses some clever image blending (cv::addWeighted) to make the graphics look 
semi-transparent and professional.
*/
static void draw_objects(cv::Mat const& image,
                         cv::Mat &res, // This will be the output image with drawings on it.
                         sl::Objects const& objs, // This is the list of detected and tracked cones from the ZED SDK, which includes their 2D bounding boxes and confidence scores.
                         std::vector<std::vector<int>> const& colors) // This is a predefined list of colors (e.g., blue, yellow, orange) that we use to draw the boxes.
{
    /*
    Creates two exact copies of the camera frame. One will hold the solid lines (res), and one will act as a layer for 
    transparent fill colors (mask).
    */

    /*
    .clone() is a built-in function provided by the OpenCV cv::Mat class. By default, if we just say res = image, C++ tries 
    to be highly efficient. It won't actually copy the millions of pixels; it will just make res point to the exact same memory 
    location as image. Calling image.clone() forces the computer to allocate brand new memory and copy every single pixel 
    over one by one. It creates a completely independent "deep copy." Now, if we draw on the clone, the original image is perfectly safe.
    */
    res = image.clone();
    cv::Mat mask{image.clone()};
    for (sl::ObjectData const& obj : objs.object_list) { // Loop that goes through every single object (cone) the ZED camera is currently tracking.
        /*
        Calculates a color index for the bounding box based on the ZED camera's unique 3D tracking ID (obj.id). Uses the modulo operator 
        to ensure the ID cleanly wraps around within the limits of the colors list. This guarantees that a specific physical cone retains 
        the exact same bounding box color across multiple camera frames, which is critical for visually verifying that the 3D tracker is 
        maintaining a stable lock on the object.
        */
        size_t const idx_color{obj.id % colors.size()};
        cv::Scalar const color{cv::Scalar(colors[idx_color][0U], colors[idx_color][1U], colors[idx_color][2U])}; // Converts the RGB color from the list into a cv::Scalar format that OpenCV uses for drawing.
        
        /*
        The ZED SDK provides the bounding box as four specific corner coordinates in 2D space. OpenCV, however, requires rectangles 
        to be defined by a top-left X/Y starting coordinate, a width, and a height. This block performs the math to calculate the 
        width and height, resulting in a standard OpenCV cv::Rect.
        */
        cv::Rect const rect{static_cast<int>(obj.bounding_box_2d[0U].x), // x = x0
                            static_cast<int>(obj.bounding_box_2d[0U].y), // y = y0
                            static_cast<int>(obj.bounding_box_2d[1U].x - obj.bounding_box_2d[0U].x), // width = x1 - x0
                            static_cast<int>(obj.bounding_box_2d[2U].y - obj.bounding_box_2d[0U].y)}; // height = y2 - y0
        /*
        Commands OpenCV to draw a hollow rectangle onto the res image using the calculated dimensions, the assigned tracking color, 
        and a line thickness of 2 pixels.
        */
        cv::rectangle(res, rect, color, 2);
        
        /*
        Creates and formats the text string that floats above the bounding box. It uses obj.raw_label to display the actual physical 
        color classification that the YOLO AI detected (e.g., 0 for blue, 4 for yellow), and obj.confidence to display the AI's 
        percentage of certainty.
        */
        char text[256U];
        sprintf(text, "Class %d - %.1f%%", obj.raw_label, obj.confidence);

        /*
        Sometimes, AI models don't just draw boxes; they create a "stencil" or "silhouette" of the object, called a Mask. 
        The code first checks if the camera actually successfully created a stencil for this specific cone. If the stencil 
        exists, it translates the stencil into a format our drawing tools can read, and then uses that stencil to spray-paint 
        our color perfectly over the cone.
        */
        if (obj.mask.isInit() && obj.mask.getWidth() > 0U && obj.mask.getHeight() > 0U) { // Checks if the mask is valid and has dimensions greater than 0.
            const cv::Mat obj_mask = slMat2cvMat(obj.mask); // So, we use a helper function to translate the ZED silhouette (sl::Mat) into a standard OpenCV image (cv::Mat).
            mask(rect).setTo(color, obj_mask); // We select the region (rect) on the mask and fill it with the chosen color, but only where the stencil (obj_mask) allows - so the color appears exactly in the shape of the cone.
        }

        /*
        In typography, letters like 'g', 'p', 'y', and 'q' have "tails" that dip below the normal line of text. This is called the baseline. 
        We create an empty integer here so OpenCV has a place to write down the measurement of those tails.
        */
        int baseLine{0};
        /*
        We say: "If I use this specific font, at a size of 0.4, with a thickness of 1, how many pixels wide and tall will the string 
        "Class 0 - 85%" be?" OpenCV calculates this, saves the width and height into label_size, and updates our baseLine variable with 
        the length of the letter tails.
        */
        cv::Size const label_size{cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine)};

        /*
        We want the text to line up perfectly with the left side of our bounding box. So, we set our starting X coordinate to the 
        exact X coordinate of the bounding box (rect.x).
        We want the text to sit just above the top edge of the bounding box (rect.y + 1). However, if the bounding box is right at 
        the top of the screen, this could put our text off-screen. To prevent this, we use std::min to ensure that the Y coordinate 
        of the text is never less than 0 (the top edge of the screen). By adding 1 pixel to rect.y, we also create a small gap between 
        the top edge of the bounding box and the text, improving readability.
        */
        int const x{rect.x};
        int const y{std::min(rect.y + 1, res.rows)};

        // Actually drawing the elements and stamping the actual letters onto the red box.
        cv::rectangle(res, cv::Rect(x, y, label_size.width, label_size.height + baseLine), {0, 0, 255}, -1);
        cv::putText(res, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4, {255, 255, 255}, 1);
    }
    /*
    Right now, the computer is holding two separate canvases:
    res: The main image.
    mask: The transparency layer.
    cv::addWeighted is a math equation that smashes them together.
    */
    cv::addWeighted(res, 0.5, mask, 0.8, 1, res);
}

/*
It translates YOLO's version of a bounding box into the ZED Camera's version of a bounding box. Our YOLO AI is very minimalist. 
When it draws a box, it only gives us 2 coordinates: the Top-Left corner (x1, y1) and the Bottom-Right corner (x2, y2). However, 
the ZED SDK is very strict; it refuses to accept just 2 corners. It demands a list of all 4 corners in a clockwise order. This 
function calculates the missing 2 corners and packages them into the format ZED demands.
*/
std::vector<sl::uint2> cvt(const BBox &bbox_in) {
    std::vector<sl::uint2> bbox_out(4);
    bbox_out[0] = sl::uint2(bbox_in.x1, bbox_in.y1);
    bbox_out[1] = sl::uint2(bbox_in.x2, bbox_in.y1);
    bbox_out[2] = sl::uint2(bbox_in.x2, bbox_in.y2);
    bbox_out[3] = sl::uint2(bbox_in.x1, bbox_in.y2);
    return bbox_out;
}
 /*
 It takes a bounding box generated by our YOLO AI and translates it into a format that OpenCV (our drawing tool) can understand.
 */
cv::Rect get_rect(BBox box) {
    return cv::Rect(round(box.x1), round(box.y1), round(box.x2 - box.x1), round(box.y2 - box.y1));
}

/*
This is the heart of the entire perception system. It is the main run loop that executes continuously on a dedicated thread
for as long as the camera is active. Everything in this function is designed to execute as fast as possible, once per camera
frame, in a strict sequence:

    Step 1 - Setup:     Initializes the camera resolution handles and loads the YOLO TensorRT engine from disk into GPU memory.
    Step 2 - Grab:      Waits for the camera hardware to deliver a new frame. This is the "clock tick" of the whole system.
    Step 3 - Infer:     Passes the raw 2D image to YOLO to get a list of bounding boxes around cones.
    Step 4 - Fuse:      Hands those 2D bounding boxes back to the ZED SDK, which combines them with stereo depth data to
                        produce real 3D world-space positions and assign persistent tracking IDs to each cone.
    Step 5 - Visualize: If the visualisation flag is set, draws the boxes onto the frame and publishes it as a ROS image topic.
    Step 6 - Classify:  Sorts detected 3D cones by color, discards anything beyond 6 meters, and packs them into a ROS message.
    Step 7 - Publish:   Safely locks shared data, reads the car's 3D pose, extracts the yaw heading, and fires off the message.
*/
void ZedLaunchNode::cone_detection_loop()
  {
    /*
    Before the main loop starts, we configure the camera resolution. The ZED camera is physically capable of running
    at very high resolutions, but that would be unnecessarily expensive for our pipeline. We deliberately cap it here
    at a maximum of 720x404 pixels, giving us a solid balance between detection quality and processing speed.
    We then query the camera a second time (camera_info) using that capped resolution. This second query gives us the
    camera's calibration parameters (like focal length and optical center) that are mathematically correct for this
    specific resolution - these are needed internally by the ZED SDK to correctly interpret depth.
    */
    auto camera_config = zed.getCameraInformation().camera_configuration; // Query the camera's full native configuration to find out its maximum supported resolution.
    sl::Resolution pc_resolution(std::min((int) camera_config.resolution.width, 720), std::min((int) camera_config.resolution.height, 404)); // Cap the resolution to 720x404. std::min ensures we never request a resolution higher than what the hardware supports.
    auto camera_info = zed.getCameraInformation(pc_resolution).camera_configuration; // Re-query the camera using the capped resolution to get intrinsic calibration parameters that are valid at this specific resolution.

    /*
    Load the TensorRT inference engine and initialize the YOLO detector. The engine_name is the path to our
    pre-compiled .engine file - a binary that TensorRT baked specifically for the GPU it was compiled on. It
    contains the entire neural network, already optimized and ready to run. If detector.init() returns a
    non-zero value, something went wrong: the file might be missing, corrupt, or compiled for a different GPU.
    In any of those cases, there is no point continuing, so we bail out immediately.
    */
    std::string engine_name = model_name; // model_name is a class member set at node startup, holding the absolute path to the .engine file on disk.
    Yolo detector; // Create the YOLO inference object. At this point it is just an empty shell; no model has been loaded onto the GPU yet.
    if (detector.init(engine_name)) { // detector.init() reads the .engine file and uploads the neural network into GPU memory. Returns 0 on success, non-zero on failure.
        std::cerr << "Detector init failed!" << std::endl;
        return; // If the model fails to load, the entire detection loop cannot function. Exit immediately.
    }

    /*
    Declare the containers that will be reused on every single frame inside the main loop. By allocating them
    here, outside the loop, the OS only sets aside memory for them once. If we declared them inside the loop,
    the program would constantly allocate and deallocate memory hundreds of times per second, which is slow
    and can cause memory fragmentation.

        display_resolution:              The camera's full native resolution, used when drawing the visualisation overlay.
        left_sl:                         A ZED-format image buffer. Every frame, zed.retrieveImage() overwrites this
                                         with the latest raw camera image.
        objectTracker_parameters_rt:     Runtime settings for the ZED's object tracker (e.g., confidence cutoffs).
                                         Left as SDK defaults here, but the variable must exist to call retrieveObjects().
        objects:                         The container the ZED SDK fills with its list of tracked, 3D-positioned cones
                                         after each sensor fusion cycle.
    */
    auto display_resolution = zed.getCameraInformation().camera_configuration.resolution; // The camera's native display resolution (uncapped) - used to run YOLO at the highest quality for the visualisation overlay.
    sl::Mat left_sl; // An empty ZED image buffer. Every iteration of the loop, zed.retrieveImage() will overwrite this with the newest frame.
    sl::ObjectDetectionRuntimeParameters objectTracker_parameters_rt; // Runtime parameters for the ZED's built-in 3D object tracker. We use SDK defaults and do not override any settings.
    sl::Objects objects; // The output container for the ZED's sensor fusion results. After retrieveObjects(), this holds every tracked cone's 3D position, velocity, and persistent ID.

    /*
    The main perception loop. zed.grab() is a blocking call - it pauses execution and waits until the camera
    hardware has a brand new frame ready to process. Think of it as the "heartbeat" of the entire perception
    system: the whole pipeline runs at exactly the rate the camera produces frames. The loop only continues
    as long as zed.grab() returns SUCCESS. If the camera disconnects, errors out, or is told to stop, it
    returns a different error code, the condition becomes false, and we fall through to the cleanup code below.
    */
    while (zed.grab() == sl::ERROR_CODE::SUCCESS) {

        /*
        Retrieve the raw color image from the left camera lens. The ZED is actually a stereo camera with two
        lenses, and it can output many different "views": left RGB, right RGB, a depth map, a confidence map, etc.
        We specifically request sl::VIEW::LEFT because YOLO was trained on standard single 2D color images,
        and the left lens is the one that matches that convention. The image is written directly into
        our pre-allocated left_sl buffer, overwriting whatever was there from the previous frame.
        */
        zed.retrieveImage(left_sl, sl::VIEW::LEFT); // Grab the left camera's RGB frame and overwrite left_sl with the latest data.

        /*
        Run YOLO inference on the captured frame. Under the hood, Yolo::run() handles the full GPU pipeline:
        it reformats the ZED image into a tensor, copies it onto the GPU, fires a forward pass through the
        TensorRT neural network, and then reads the output detections back from the GPU. The result is a
        list of bounding boxes in 2D pixel-space, each containing a class label, a confidence score, and
        the four corner coordinates of the box around the cone.
        */
        auto detections = detector.run(left_sl, display_resolution.height, display_resolution.width, CONF_THRESH); // Run the YOLO model on the current frame. CONF_THRESH silently discards any detection where the model is less than 80% confident, before they even reach us.

        /*
        Translate YOLO's output into the format the ZED SDK expects. The ZED has its own ingestion pipeline
        and refuses to accept YOLO's raw output directly. We must repackage each detection into a
        sl::CustomBoxObjectData struct - filling in a unique ID, the confidence score, the class label,
        the 4-corner bounding box, and a "grounded" flag that tells the ZED whether to track the object
        on the ground plane or in full 3D space. We collect all the translated detections into the
        objects_in vector, ready to feed into the ZED.
        */
        std::vector<sl::CustomBoxObjectData> objects_in; // An empty list that will hold all translated detections for this frame, formatted for the ZED SDK.
        for (auto &it : detections) { // Loop over every bounding box that YOLO returned for this frame.
            sl::CustomBoxObjectData tmp; // Create a blank ZED detection container for this specific cone.
            tmp.unique_object_id = sl::generate_unique_id(); // Assign a freshly generated UUID to this raw detection. The ZED tracker uses this to correlate detections across frames and maintain persistent cone identities.
            tmp.probability = it.prob; // Pass the YOLO confidence score to the ZED so it can weigh how reliable this detection is during sensor fusion.
            tmp.label = (int) it.label; // Cast the YOLO class label (e.g., 0 for blue, 4 for yellow) to an integer as the ZED SDK requires.
            tmp.bounding_box_2d = cvt(it.box); // Convert YOLO's 2-corner box format (top-left + bottom-right) into the ZED's required 4-corner clockwise format using our cvt() helper.
            tmp.is_grounded = true; // All classes are cones on a flat race track, so every detection should be anchored to the ground plane for accurate depth estimation.
            objects_in.push_back(tmp); // Add the fully populated detection to our list.
        }

        /*
        Sensor fusion: hand the 2D detections to the ZED and get back the 3D results. This two-step process
        is the core of what makes the ZED powerful. zed.ingestCustomBoxObjects() feeds our pixel-space
        bounding boxes into the ZED's stereo depth pipeline. The ZED then triangulates the 3D position of
        each box using its two lenses. zed.retrieveObjects() gives us back the fused result: each cone
        now has a definitive 3D world-space position (x, y, z in millimeters), a persistent tracking ID
        that survives across frames (even if YOLO misses it briefly), and a velocity estimate.
        */
        zed.ingestCustomBoxObjects(objects_in); // Hand all translated 2D detections to the ZED SDK to begin depth fusion and 3D tracking.
        zed.retrieveObjects(objects, objectTracker_parameters_rt); // Pull the ZED's fused output: the same cones now enriched with 3D world positions, persistent IDs, and velocity estimates.

        /*
        Optional visualisation: if the visualisation flag was set at node startup, we draw all the bounding
        boxes and labels onto the current frame and publish it as a ROS image topic. This gives developers
        a live view of exactly what the AI sees and is tracking, which is invaluable for debugging on a
        display or remotely via Foxglove.
        */
        if (visualisation) {
            cv::Mat left_cv;
            left_cv = slMat2cvMat(left_sl); // Convert the ZED's proprietary sl::Mat format into a standard OpenCV cv::Mat so we can use OpenCV's drawing functions on it.
            draw_objects(left_cv, left_cv, objects, CLASS_COLORS); // Overlay all bounding boxes, class labels, and confidence scores from the ZED's tracked objects onto the image.
            sensor_msgs::msg::Image::SharedPtr imageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgra8", left_cv).toImageMsg(); // Wrap the annotated OpenCV image into a ROS Image message. "bgra8" declares the pixel encoding: Blue, Green, Red, Alpha channels, 8 bits per channel.
            image_publisher->publish(*imageMsg); // Publish the annotated image to the ROS camera topic for live monitoring in RViz or Foxglove.
        }

        /*
        Build the ROS Detections message for this frame. We create a fresh, empty message and then loop
        over every cone the ZED is currently tracking. For each cone, we convert its position from
        millimeters (the ZED SDK's native unit) to meters, compute its straight-line distance from the
        car (Pythagorean theorem on x and y), and discard it if it's further than 6 meters away. Cones
        beyond that distance are unreliable due to depth noise and are outside the planning horizon anyway.
        Each surviving cone is then sorted into the correct color bucket (blue, yellow, or big_orange)
        based on its YOLO class label.
        */
        fsae_interfaces::msg::Detections detectionsMsg; // Create a blank Detections message that will be populated and published at the end of this frame's iteration.

        int count = 0; // Unused counter, retained for potential future use (e.g., logging detection totals).
        for (sl::ObjectData& obj : objects.object_list) { // Iterate over every cone the ZED 3D tracker is currently tracking.
            geometry_msgs::msg::Point p; // A temporary point struct to hold this cone's converted x/y position in meters.
            switch (obj.raw_label) { // Branch on the YOLO class label to determine which color bucket this cone belongs to.
                case 0: // Label 0 = Blue cone (left boundary of the track).
                    p.x = obj.position[0] / 1000.0; // Convert X position from millimeters to meters by dividing by 1000.
                    p.y = obj.position[1] / 1000.0; // Convert Y position from millimeters to meters.

                    if (sqrt(p.x * p.x + p.y * p.y) < 6.0) { // Pythagorean distance check: only keep this cone if it is within 6 meters of the car (the origin).
                        detectionsMsg.blue.push_back(p); // Cone is close enough - add its 3D position to the blue cone list in the message.
                    }

                    break;
                case 4: // Label 4 = Yellow cone (right boundary of the track).
                    p.x = obj.position[0] / 1000.0; // Convert X position from millimeters to meters.
                    p.y = obj.position[1] / 1000.0; // Convert Y position from millimeters to meters.

                    if (sqrt(p.x * p.x + p.y * p.y) < 6.0) { // Same 6-meter distance filter applied to yellow cones.
                        detectionsMsg.yellow.push_back(p); // Add to the yellow cone list.
                    }
                    
                    break;
                default: // Any other label is treated as a large orange cone, used to mark the start/finish line or chicanes.
                    p.x = obj.position[0] / 1000.0; // Convert X position from millimeters to meters.
                    p.y = obj.position[1] / 1000.0; // Convert Y position from millimeters to meters.

                    if (sqrt(p.x * p.x + p.y * p.y) < 6.0) { // Same 6-meter distance filter applied consistently to all cone colors.
                        detectionsMsg.big_orange.push_back(p); // Add to the big orange cone list.
                    }

                    break;
            }
        }
        
        /*
        Extract the car's current pose and publish the completed message. We lock the mutex here because
        cam_w_pose and camera_running are shared between this thread and the main thread. Without locking,
        the main thread could try to read or write these variables at the exact same moment we are using
        them, causing a data race and undefined behavior.

        The ZED provides its 3D orientation as a quaternion (ox, oy, oz, ow). A quaternion is a compact
        mathematical way to represent a rotation in 3D space using 4 numbers. It has no gimbal lock issues,
        which makes it the industry standard. However, our car drives on a flat track, so its full 3D
        orientation can be summarized with just one angle - yaw, which is the left/right heading. We
        mathematically extract yaw from the quaternion using the standard Euler angle derivation formula.

        We store the single yaw float in orientation.w as a convenient single-field container. We
        only publish the message if both a blue and a yellow cone are visible, ensuring the planning
        stack always receives a message that describes both sides of the track.
        */
        mtx.lock(); // Acquire the mutex lock before touching any data shared with other threads.

        zed.getPosition(cam_w_pose, sl::REFERENCE_FRAME::WORLD); // Ask the ZED's internal visual odometry system for the camera's current position and orientation, expressed in the fixed global world frame.
        detectionsMsg.car_pose.position.x = cam_w_pose.getTranslation().tx / 1000.0; // Extract the car's X position and convert from millimeters to meters.
        detectionsMsg.car_pose.position.y = cam_w_pose.getTranslation().ty / 1000.0; // Extract the car's Y position and convert from millimeters to meters.

        // Extract all four components of the orientation quaternion from the ZED's pose estimate.
        float ox = cam_w_pose.getOrientation().ox; // X component of the quaternion - encodes the contribution of rotation around the world X axis.
        float oy = cam_w_pose.getOrientation().oy; // Y component of the quaternion - encodes the contribution of rotation around the world Y axis.
        float oz = cam_w_pose.getOrientation().oz; // Z component of the quaternion - encodes the contribution of rotation around the world Z (vertical) axis.
        float ow = cam_w_pose.getOrientation().ow; // W (scalar) component of the quaternion. Together with the other three, it fully defines the 3D rotation with no ambiguity.

        float yaw = atan2(2.0f * (ow * oz + ox * oy), 1.0f - 2.0f * (oy * oy + oz * oz)); // Standard quaternion-to-Euler formula to extract yaw (the heading angle, i.e. rotation around the vertical Z axis). The car can only rotate left/right on flat ground, so yaw is the only rotation our planning stack needs.

        detectionsMsg.car_pose.orientation.w = yaw; // Store the extracted yaw angle in orientation.w. We repurpose this single float field as a container since we only need one angle, not a full quaternion, in the downstream planning code.

        if (detectionsMsg.yellow.size() > 0 && detectionsMsg.blue.size() > 0) { // Only publish if we can see at least one cone on each side of the track. A message with only one boundary color would give the planner an incomplete and potentially dangerous picture.
            cone_detection_publisher->publish(detectionsMsg); // Publish the fully populated Detections message (3D cone positions + car pose) to the ROS topic for the SLAM and path planning nodes to consume.
        }

        mtx.unlock(); // Release the mutex so the main thread can access shared data again.
    }

    /*
    We reach here only when zed.grab() stops returning SUCCESS - meaning the camera has been disconnected,
    encountered a hardware fault, or been commanded to stop. The loop has exited and this detection thread
    is about to die. We need to safely signal this to the rest of the system by setting camera_running to
    false. We lock the mutex first because the main thread may be reading this flag at any time, and writing
    to a shared variable without a lock is a data race.
    */
    mtx.lock(); // Acquire the mutex before modifying the shared camera_running flag.

    camera_running = false; // Notify all other threads that the camera has stopped and this detection loop has fully exited.

    mtx.unlock(); // Release the mutex so the main thread can read the updated camera_running flag and respond accordingly.
}
