/*
OVERVIEW:
  This header defines all the data structures, helper types, and the Yolo class interface
  that form the foundation of our TensorRT-based object detection pipeline. It is the
  "contract" that yolo.cpp implements and that cone_detection.cpp consumes.

KEY RESPONSIBILITIES:
  1. Data Structures: Defines BBox and BBoxInfo, the plain data containers that carry a
     single detection (bounding box corners, class label, confidence score) from YOLO's
     raw output all the way through to cone_detection.cpp where they are ingested by the ZED SDK.
  2. Model Version Abstraction: Defines the YOLO_MODEL_VERSION_OUTPUT_STYLE enum so the
     same Yolo class can handle the different output tensor layouts produced by YOLOv5/v8
     versus YOLOv6 without duplicating logic.
  3. Dynamic Dimension Support: Defines the OptimDim struct, used only during the one-time
     engine build step to tell TensorRT what input resolution to optimize the network for.
  4. Yolo Class Interface: Declares the three lifecycle methods (build_engine, init, run)
     and encapsulates all GPU memory handles, buffer pointers, and TensorRT objects so
     that cone_detection.cpp never has to touch raw CUDA or TensorRT APIs directly.

DEPENDENCIES:
  - NvInfer.h (TensorRT): ICudaEngine, IExecutionContext, IRuntime.
  - sl/Camera.hpp (ZED SDK): sl::Mat for receiving camera frames, sl::Resolution for reporting inference size.
  - OpenCV: cv::Mat used internally for image pre-processing.
  - cuda_utils.h / logging.h / utils.h: Project-local helpers for CUDA error checking, TensorRT logging,
    and image preprocessing (preprocess_img, slMat2cvMat).
*/

#ifndef YOLO_HPP
#define YOLO_HPP

#include <NvInfer.h>
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"

/*
Identifies which YOLO generation produced the .engine file we loaded, because YOLOv5/v6/v8
each arrange their output tensor differently. The Yolo class detects which style is in use
during init() and stores the result here so run() can parse the raw GPU output correctly.
*/
enum class YOLO_MODEL_VERSION_OUTPUT_STYLE {
    YOLOV6,     // Output tensor is arranged as [1 x num_detections x (5 + num_classes)], i.e. each row is one detection.
    YOLOV8_V5   // Output tensor is arranged as [1 x (4 + num_classes) x num_detections], i.e. each column is one detection (transposed relative to v6).
};

/*
The minimal bounding box representation. Two corners (top-left and bottom-right) are 
enough to uniquely define any axis-aligned rectangle, and this is the format used 
internally throughout the inference and NMS pipeline.
*/
struct BBox {
    float x1, y1, x2, y2; // (x1, y1) = top-left corner; (x2, y2) = bottom-right corner, all in pixel coordinates on the original image.
};

/*
Bundles a single YOLO detection into one self-contained object. This is the atomic unit
of data that flows from Yolo::run() all the way to cone_detection.cpp where it gets
converted into a ZED CustomBoxObjectData and fed into the sensor fusion pipeline.
*/
struct BBoxInfo {
    BBox box;   // Pixel-space bounding box corners on the original camera image.
    int label;  // Integer class ID assigned by the YOLO model (e.g., 0 = blue cone, 4 = yellow cone), matching the labels the model was trained on.
    float prob; // Confidence score in [0, 1]. Only detections above CONF_THRESH (0.8) in cone_detection.cpp ever reach the ZED SDK.
};

/*
A tiny string-splitting utility used exclusively by OptimDim::setFromString() below.
It tokenizes a string by a delimiter and returns each part as a vector of strings.
For example, split_str("images:1x3x512x512", ":") returns {"images", "1x3x512x512"}.
*/
inline std::vector<std::string> split_str(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr(pos_start, pos_end - pos_start); // Extract the substring between the last delimiter and the current one.
        pos_start = pos_end + delim_len; // Advance the search start position past the delimiter we just found.
        res.push_back(token);
    }

    res.push_back(s.substr(pos_start)); // Push the final token (everything after the last delimiter).
    return res;
}


/*
Used only during the offline one-time engine compilation step (Yolo::build_engine). 
TensorRT needs to know the exact input tensor shape to optimize the network for. This 
struct parses that shape from a human-readable command-line string like "images:1x3x640x640"
into the Dims4 structure TensorRT requires. It is not used at all during normal car operation
since we always load the pre-built .engine file directly in Yolo::init().
*/
struct OptimDim {
    nvinfer1::Dims4 size;   // The 4D tensor shape [batch, channels, height, width] that TensorRT will optimize for.
    std::string tensor_name; // The name of the input tensor in the ONNX graph (typically "images").

    /*
    Parses a string of the form "images:1x3x512x512" and populates the size and
    tensor_name fields. Returns false on success, true on failure (non-standard
    convention inherited from the original Stereolabs sample code).
    */
    bool setFromString(std::string &arg) {
        // "images:1x3x512x512"
        std::vector<std::string> v_ = split_str(arg, ":"); // Split on ":" to separate the tensor name from the dimension string.
        if (v_.size() != 2) return true; // If we didn't get exactly two parts (name + dims), the format is invalid.

        std::string dims_str = v_.back(); // The dimension string is always the second part, e.g. "1x3x512x512".
        std::vector<std::string> v = split_str(dims_str, "x"); // Split the dimension string on "x" to get individual dimension values.

        size.nbDims = 4; // All YOLO inputs are 4-dimensional: [batch, channels, height, width].
        // assuming batch is 1 and channel is 3
        size.d[0] = 1; // Batch size is always 1; we process one frame at a time.
        size.d[1] = 3; // RGB channel count is always 3.

        /*
        The user might provide 2, 3, or 4 dimension values. We handle all cases:
          2 values: just height and width were given (e.g., "512x512").
          3 values: channels, height, width were given (e.g., "3x512x512").
          4 values: full specification including batch (e.g., "1x3x512x512").
        In all cases we only care about the last two values (height and width).
        */
        if (v.size() == 2) {
            size.d[2] = stoi(v[0]); // height
            size.d[3] = stoi(v[1]); // width
        } else if (v.size() == 3) {
            size.d[2] = stoi(v[1]); // height
            size.d[3] = stoi(v[2]); // width
        } else if (v.size() == 4) {
            size.d[2] = stoi(v[2]); // height
            size.d[3] = stoi(v[3]); // width
        } else return true; // Any other count is malformed input.

        if (size.d[2] != size.d[3]) std::cerr << "Warning only squared input are currently supported" << std::endl; // Our pipeline only supports square input tensors (e.g., 640x640). Non-square inputs will work but may produce stretched detections.

        tensor_name = v_.front(); // The tensor name is always the first part of the split, e.g. "images".
        return false; // Return false to indicate successful parsing (non-standard convention).
    }
};

/*
The Yolo class is the self-contained GPU inference engine. It owns all TensorRT and CUDA
resources, and exposes a clean three-method interface to the rest of the codebase:

  build_engine(): A static one-time tool to compile an ONNX model into an optimized TensorRT
                  .engine binary. This is only called offline on a developer machine, never
                  during a competition run. The resulting .engine file is then baked into the
                  car's filesystem.

  init():         Loads the pre-compiled .engine file from disk into GPU memory and allocates
                  all the CUDA input/output buffers needed to run inference. This is called once
                  at the start of cone_detection_loop() in cone_detection.cpp.

  run():          The hot path. Called every single camera frame. Takes the raw ZED camera image,
                  preprocesses it, fires it through the GPU neural network, and parses the raw
                  output tensor back into a clean list of BBoxInfo detections. The result is
                  immediately consumed by cone_detection.cpp to feed the ZED sensor fusion pipeline.
*/
class Yolo {
public:
    Yolo();
    ~Yolo();

    // Compiles a raw ONNX model file into an optimized TensorRT .engine binary. This is a heavy offline operation (can take minutes) and only needs to be run once per model/GPU combination.
    static int build_engine(std::string onnx_path, std::string engine_path, OptimDim dyn_dim_profile);

    // Loads the pre-compiled .engine file into GPU memory and allocates all CUDA buffers. Must be called once before any calls to run(). Returns 0 on success.
    int init(std::string engine_path);

    // Runs a full inference cycle on a single camera frame and returns a list of detected bounding boxes. This is called every frame inside cone_detection_loop().
    std::vector<BBoxInfo> run(sl::Mat left_sl, int orig_image_h, int orig_image_w, float thres);
    std::vector<BBoxInfo> run(const cv::Mat &image_bgr, int orig_image_h, int orig_image_w, float thres);

    // Returns the resolution the neural network was compiled to accept (e.g., 640x640). Used externally to configure the ZED camera stream size to match.
    sl::Resolution getInferenceSize() {
        return sl::Resolution(input_width, input_height);
    }

private:

    cv::Mat left_cv_rgb; // Internal staging buffer. The ZED delivers frames in BGRA format; we convert to BGR here before passing to the neural network preprocessor.

    float nms = 0.4; // The NMS (Non-Maximum Suppression) overlap threshold. If two bounding boxes overlap by more than 40% of their combined area, the weaker one is merged or discarded. Matches NMS_THRESH in cone_detection.cpp.

    /*
    TensorRT binding indices and names. When TensorRT loads an ONNX model it assigns
    integer "binding indices" to each tensor (0 for input, 1 for output). We store both
    the name and the index so we can look them up by name at init time and then use the
    faster integer index during every run() call.
    */
    std::string input_binding_name = "images"; // The name of the input tensor in the ONNX graph. "images" is the standard name used by the Ultralytics YOLO exporter.
    std::string output_name = "classes"; // The name of the output tensor in the ONNX graph.
    int inputIndex, outputIndex; // Integer binding indices assigned by TensorRT at init time. Used to build the buffer pointer array that enqueueV2() requires.

    /*
    Neural network geometry. These are populated during init() by inspecting the loaded
    engine's binding dimensions. They define the exact size of the CPU and GPU memory
    buffers we need to allocate.
    */
    size_t input_width = 0, input_height = 0, batch_size = 1; // Input tensor spatial dimensions and batch size. Batch size is always 1 for our real-time pipeline.
    // Yolov6 1x8400x85 //  85=5+80=cxcy+cwch+obj_conf+cls_conf //https://github.com/DefTruth/lite.ai.toolkit/blob/1267584d5dae6269978e17ffd5ec29da496e503e/lite/ort/cv/yolov6.cpp#L97
    // Yolov8/yolov5 1x84x8400
    size_t out_dim = 8400, out_class_number = 0, out_box_struct_number = 4; // https://github.com/ultralytics/yolov3/issues/750#issuecomment-569783354
    // out_dim = the number of candidate anchor boxes the model considers per frame (8400 for a 640x640 input).
    // out_class_number = how many object categories were trained. Set to 0 here; overwritten by init() from the loaded model's output tensor shape.
    // out_box_struct_number = how many values describe each box's geometry (4 for YOLOv8: cx, cy, w, h; 5 for YOLOv6 which adds an objectness score).
    size_t output_size = 0; // Total number of floats in the output tensor = out_dim * (out_class_number + out_box_struct_number). Computed during init() once the model is loaded.

    YOLO_MODEL_VERSION_OUTPUT_STYLE yolo_model_version; // Detected during init() by inspecting the output tensor shape. Tells run() which output parsing branch to execute.

    /*
    CPU-side (host) and GPU-side (device) memory buffers.
    The "h_" prefix = host (CPU RAM). The "d_" prefix = device (GPU VRAM).
    The input pipeline is: camera → h_input → [cudaMemcpyAsync] → d_input → TensorRT GPU kernel.
    The output pipeline is: TensorRT GPU kernel → d_output → [cudaMemcpyAsync] → h_output → parsed BBoxInfo list.
    Both host buffers are plain float arrays allocated with new[] in init().
    Both device buffers are CUDA pointers allocated with cudaMalloc() in init().
    */
    float *h_input, *h_output; // CPU-side float arrays that stage data before uploading to (h_input) and after downloading from (h_output) the GPU.
    float *d_input, *d_output; // GPU VRAM pointers. TensorRT reads from d_input and writes its raw detections to d_output.

    /*
    The three core TensorRT objects that represent a loaded and ready-to-run neural network.
    runtime   → The deserialization factory. Reads the binary .engine file and rebuilds the network.
    engine    → The compiled network, loaded into GPU memory. Holds the model weights and optimized execution graph.
    context   → The execution state machine. Created from the engine; needed to actually enqueue an inference job.
    stream    → A CUDA stream. Using a dedicated stream lets inference run asynchronously without blocking the CPU,
                and ensures memory transfers and GPU kernels execute in the correct order.
    */
    nvinfer1::IRuntime* runtime;
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    cudaStream_t stream;

    bool is_init = false; // Guard flag set to true only after init() completes successfully. The destructor checks this before attempting to free GPU resources, preventing crashes on early-exit failure paths.


};

#endif /* YOLO_HPP */

