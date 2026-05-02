/*
OVERVIEW:
  This file contains the full implementation of the Yolo class declared in yolo.hpp. It is
  the low-level GPU engine that sits underneath the perception pipeline. cone_detection.cpp 
  calls into this file for every single camera frame and gets back a list of bounding boxes
  ready to be fused with ZED depth data.

KEY RESPONSIBILITIES:
  1. NMS (Non-Maximum Suppression): Implements the algorithm that eliminates duplicate 
     bounding boxes when YOLO draws multiple overlapping boxes around the same cone. 
     Uses a "Weighted NMS" variant that blends the surviving boxes together for a more 
     accurate final position, rather than simply deleting the weaker duplicate.
  2. Engine Build (Offline): Implements Yolo::build_engine(), which reads a raw ONNX model 
     file, compiles it into an optimized TensorRT .engine binary, and saves it to disk. This 
     is a one-time developer tool - it is never called during a competition run.
  3. Engine Init (Startup): Implements Yolo::init(), which loads the pre-compiled .engine 
     file from disk into GPU memory, inspects the model's tensor dimensions to auto-detect 
     whether it is YOLOv5/v8 or YOLOv6 format, and allocates all the CPU and GPU buffers 
     required for inference.
  4. Inference Loop (Per Frame): Implements Yolo::run(), which preprocesses the camera image,
     copies it to the GPU, fires the neural network forward pass, reads the raw output back 
     from the GPU, decodes the output tensor into BBoxInfo structs, and applies NMS to produce 
     the final clean list of cone detections returned to cone_detection.cpp.

DEPENDENCIES:
  - yolo.hpp: Class declaration, BBox/BBoxInfo structs, YOLO_MODEL_VERSION_OUTPUT_STYLE enum.
  - NvOnnxParser.h (TensorRT): Only used in build_engine() to parse the .onnx file.
  - utils.h: preprocess_img() (letterbox resize), slMat2cvMat() (ZED→OpenCV conversion).
  - cuda_utils.h: CUDA_CHECK macro for CUDA API error checking.
  - logging.h: TensorRT Logger class (gLogger).
*/

#include "yolo.hpp"
#include "NvOnnxParser.h"

using namespace nvinfer1;

static Logger gLogger; // A global TensorRT logger instance. TensorRT requires a logger to be passed to every builder and runtime factory call. Being static means it lives for the entire duration of the program.

/*
A small helper that constrains an integer value to the closed range [min, max]. 
Used when converting YOLO's raw predicted box coordinates back into pixel space to 
ensure a box corner can never be placed outside the boundaries of the image frame.
*/
inline int clamp(int val, int min, int max) {
    if (val <= min) return min;
    if (val >= max) return max;
    return val;
}


#define WEIGHTED_NMS // Enables the Weighted NMS variant. When defined, overlapping boxes are not simply deleted - instead the surviving box's coordinates are updated to a confidence-weighted average of all the merged boxes, producing a more precise final bounding box.

/*
Filters a raw list of YOLO detections down to the final non-overlapping set. When YOLO 
processes an image it can produce hundreds of candidate bounding boxes per cone. NMS 
is the algorithm that reduces all those duplicates to a single box per cone.

The algorithm works as follows:
  1. Sort all detections from highest to lowest confidence.
  2. Greedily walk through the sorted list. Keep a detection only if it does not 
     overlap too heavily (above nmsThresh) with any box already in the "kept" list.
  3. With WEIGHTED_NMS enabled: instead of hard-deleting near-duplicate boxes, 
     collect them as weighted candidates. After the greedy pass, update each kept 
     box's corners to a confidence-weighted average of itself and all its collected 
     near-duplicates, pulling the final box toward the true object center.
*/
std::vector<BBoxInfo> nonMaximumSuppression(const float nmsThresh, std::vector<BBoxInfo> binfo) {
    /*
    A lambda (anonymous function) that calculates the 1D overlap between two line 
    segments. It is used twice by computeIoU: once for the horizontal axis and once 
    for the vertical axis. The trick of swapping x1/x2 if x1 > x2 ensures the function 
    always treats the left-most interval as interval 1, making the overlap math simple.
    */
    auto overlap1D = [](float x1min, float x1max, float x2min, float x2max) -> float {
        if (x1min > x2min) {
            std::swap(x1min, x2min); // Ensure interval 1 always starts to the left of interval 2.
            std::swap(x1max, x2max);
        }
        return x1max < x2min ? 0 : std::min(x1max, x2max) - x2min; // If the intervals don't touch at all, return 0. Otherwise the overlap is the distance from where interval 2 starts to where the earlier-ending interval finishes.
    };

    /*
    Calculates the Intersection over Union (IoU) between two bounding boxes. IoU is the 
    standard metric to measure how much two rectangles overlap. It is computed as:
        IoU = (area of intersection) / (area of union)
    A value of 0 means no overlap. A value of 1 means the boxes are identical.
    If two boxes for the same cone have an IoU above nmsThresh, one of them will be 
    suppressed (or merged in Weighted NMS).
    */
    auto computeIoU = [&overlap1D](BBox& bbox1, BBox & bbox2) -> float {
        float overlapX = overlap1D(bbox1.x1, bbox1.x2, bbox2.x1, bbox2.x2); // Horizontal overlap in pixels.
        float overlapY = overlap1D(bbox1.y1, bbox1.y2, bbox2.y1, bbox2.y2); // Vertical overlap in pixels.
        float area1 = (bbox1.x2 - bbox1.x1) * (bbox1.y2 - bbox1.y1); // Area of the first box in pixels squared.
        float area2 = (bbox2.x2 - bbox2.x1) * (bbox2.y2 - bbox2.y1); // Area of the second box in pixels squared.
        float overlap2D = overlapX * overlapY; // The intersection area (the rectangle where the two boxes actually overlap).
        float u = area1 + area2 - overlap2D; // The union area = sum of both areas minus the intersection (which was double-counted).
        return u == 0 ? 0 : overlap2D / u; // IoU = intersection / union. Guard against division by zero if both boxes have zero area.
    };

    // Sort all input detections from highest to lowest confidence score so we always start the greedy selection with the most trustworthy detection.
    std::stable_sort(binfo.begin(), binfo.end(), [](const BBoxInfo& b1, const BBoxInfo & b2) {
        return b1.prob > b2.prob; });

    std::vector<BBoxInfo> out; // The final output list of kept (surviving) bounding boxes.

#if defined(WEIGHTED_NMS)
    std::vector<std::vector < BBoxInfo> > weigthed_nms_candidates; // A parallel list of the same size as `out`. For each kept box in `out`, this stores a list of the near-duplicate boxes that were merged into it.
#endif

    /*
    The core greedy NMS loop. For each detection `i` (in descending confidence order),
    check it against every box already in `out`. If it overlaps too heavily with any of 
    them, it is suppressed (keep = false). If it survives all checks it is added to `out`.
    */
    for (auto& i : binfo) {
        bool keep = true; // Assume this detection should be kept until a heavy overlap is found.

#if defined(WEIGHTED_NMS)
        int j_index = 0; // Tracks which index in `out` we are comparing against, so we can add near-duplicates to the correct candidate list.
#endif

        for (auto& j : out) {
            if (keep) {
                float overlap = computeIoU(i.box, j.box); // Compute how much this candidate overlaps with a kept box.
                keep = overlap <= nmsThresh; // If the overlap exceeds the threshold, mark this detection for suppression.
#if defined(WEIGHTED_NMS)
                if (!keep && fabs(j.prob - i.prob) < 0.52f) // add label similarity check
                    weigthed_nms_candidates[j_index].push_back(i); // The box is suppressed but its confidence is close enough to the kept box (within 0.52) that it qualifies as a weighted candidate for positional refinement.
#endif
            } else
                break; // Once we know this detection is suppressed, no need to keep checking.

#if defined(WEIGHTED_NMS)  
            j_index++; // Move to the next kept box index.
#endif

        }
        if (keep) {
            out.push_back(i); // This detection survived all overlap checks - add it to the final kept list.
#if defined(WEIGHTED_NMS)
            weigthed_nms_candidates.emplace_back(); // Create an empty candidate list for this newly kept box.
            weigthed_nms_candidates.back().clear(); // Ensure it starts empty (belt-and-suspenders clear after emplace_back).
#endif
        }
    }

#if defined(WEIGHTED_NMS)
    /*
    Weighted coordinate refinement. For each surviving box, compute a confidence-weighted 
    average of its own corner coordinates and those of all near-duplicate boxes that were 
    merged into it. This pulls the box center toward the true object position rather than 
    leaving it anchored to the single highest-confidence detection, which may be slightly off-center.
    */
    for (int i = 0; i < out.size(); i++) {
        // the best confidence
        BBoxInfo& best = out[i]; // Reference to the kept box we are refining. We will overwrite its corner coordinates in-place.
        
        /*
        Initialize the weighted sums using the best (highest-confidence) box itself. 
        Each corner coordinate is multiplied by its confidence score, so higher-confidence 
        detections have a stronger pull on the final averaged position.
        */
        float sum_tl_x = best.box.x1 * best.prob; // Start the weighted sum for the top-left x corner with the best box's contribution.
        float sum_tl_y = best.box.y1 * best.prob;
        float sum_br_x = best.box.x2 * best.prob;
        float sum_br_y = best.box.y2 * best.prob;

        float weight = best.prob; // Total weight accumulator. Will be divided out at the end to normalize the sum into a true weighted average.
        
        // Add the contribution of each near-duplicate box to the running weighted sums.
        for (auto& it : weigthed_nms_candidates[i]) {
            sum_tl_x += it.box.x1 * it.prob; // Each corner is weighted by the detection's confidence score.
            sum_tl_y += it.box.y1 * it.prob;
            sum_br_x += it.box.x2 * it.prob;
            sum_br_y += it.box.y2 * it.prob;
            weight += it.prob; // Accumulate the total weight.
        }

        weight = 1.f / weight; // Invert the total weight once so we can multiply instead of divide (faster arithmetic).
        
        // Write the confidence-weighted average coordinates back over the original box corners.
        best.box.x1 = sum_tl_x * weight;
        best.box.y1 = sum_tl_y * weight;
        best.box.x2 = sum_br_x * weight;
        best.box.y2 = sum_br_y * weight;
    }

#endif

    return out;
}

/*
Default constructor. All member variables that need initialization are given inline default values 
in yolo.hpp (e.g., is_init = false, nms = 0.4), so there is nothing further to do here.
*/
Yolo::Yolo() {
}

/*
Destructor: safely tears down all GPU and CPU resources in the reverse order they were created.
The is_init guard ensures we only try to free resources that were actually allocated. If init()
failed partway through (e.g., the engine file was missing), is_init would still be false and 
we would skip the entire cleanup block, preventing crashes from freeing uninitialized pointers.
*/
Yolo::~Yolo() {
    if (is_init) {

        /*
        Free GPU resources in the correct order - CUDA stream first, then the GPU memory buffers, 
        then the TensorRT objects. TensorRT objects must be destroyed before the runtime that 
        created them, to avoid dangling internal references.
        */
        cudaStreamDestroy(stream); // Destroy the CUDA stream. Any pending async operations are cancelled.
        CUDA_CHECK(cudaFree(d_input)); // Release the GPU VRAM allocated for the input tensor.
        CUDA_CHECK(cudaFree(d_output)); // Release the GPU VRAM allocated for the output tensor.
        // Destroy the engine
        context->destroy(); // Destroy the execution context (the per-inference state machine).
        engine->destroy(); // Destroy the compiled engine (the weight-loaded GPU graph).
        runtime->destroy(); // Destroy the TensorRT runtime (the deserialization factory).

        // Free the CPU-side staging buffers that were allocated with new[] in init().
        delete[] h_input;
        delete[] h_output;
    }
    is_init = false; // Mark as uninitialized so any accidental second destructor call is a no-op.
}

/*
A static, offline-only utility that takes a raw ONNX neural network file, hands it to TensorRT's 
compiler, and writes the resulting optimized .engine binary to disk. This is a developer tool - 
it is run once per model/GPU combination on a development machine or on the car itself before 
competition. The resulting .engine file is what Yolo::init() loads at runtime.

Why compile to an engine? TensorRT analyzes the full ONNX graph and fuses operations, 
selects optimal GPU kernels for the specific GPU model, and quantizes weights to FP16 
if the hardware supports it. The result is an inference run that is 2-10x faster than 
executing the raw ONNX graph at runtime.
*/
int Yolo::build_engine(std::string onnx_path, std::string engine_path, OptimDim dyn_dim_profile) {


    std::vector<uint8_t> onnx_file_content;
    if (readFile(onnx_path, onnx_file_content)) return 1; // Read the entire .onnx file into a memory buffer. If the file cannot be read (missing, wrong path), bail out immediately.

    if ((!onnx_file_content.empty())) {

        ICudaEngine * engine;
        /*
        Build the TensorRT inference engine from the ONNX model. This involves four 
        sequential steps: creating a Builder, creating a Network definition, creating 
        a BuilderConfig (with optional FP16 flag), and then calling buildEngineWithConfig() 
        which does the heavy compute work of compiling the optimized engine.
        */
        std::cout << "Creating engine from onnx model" << std::endl;

        gLogger.setReportableSeverity(Severity::kINFO); // Set the TensorRT logger to print INFO-level messages so we can track the build progress in the terminal.
        auto builder = nvinfer1::createInferBuilder(gLogger); // The Builder is the top-level TensorRT object that orchestrates the entire compilation pipeline.
        if (!builder) {
            std::cerr << "createInferBuilder failed" << std::endl;
            return 1;
        }

        auto explicitBatch = 1U << static_cast<uint32_t> (nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH); // The kEXPLICIT_BATCH flag tells TensorRT that our model's batch size is baked into the ONNX graph (batch=1), rather than being dynamic. This is required for ONNX models exported by modern YOLO frameworks.
        auto network = builder->createNetworkV2(explicitBatch); // Create an empty TensorRT network definition object that the ONNX parser will populate with layers and weights.

        if (!network) {
            std::cerr << "createNetwork failed" << std::endl;
            return 1;
        }

        auto config = builder->createBuilderConfig(); // A configuration object where we set build-time options like FP16 precision and memory limits.
        if (!config) {
            std::cerr << "createBuilderConfig failed" << std::endl;
            return 1;
        }

        /*
        Dynamic input dimension handling. Most YOLO ONNX models are exported with a fixed 
        input size (e.g., 640x640), but some are exported with variable dimensions. If the 
        caller provided a non-empty tensor name, it means they want to tell TensorRT exactly 
        what size to optimize for at build time. We set kMIN, kOPT, and kMAX all to the same 
        value because our pipeline always uses a single fixed resolution.
        */
        if (!dyn_dim_profile.tensor_name.empty()) {

            IOptimizationProfile* profile = builder->createOptimizationProfile(); // An optimization profile tells TensorRT the min/opt/max input shapes to consider when choosing kernels.

            profile->setDimensions(dyn_dim_profile.tensor_name.c_str(), OptProfileSelector::kMIN, dyn_dim_profile.size); // Minimum allowed input size. Set equal to kOPT since we only support one fixed size.
            profile->setDimensions(dyn_dim_profile.tensor_name.c_str(), OptProfileSelector::kOPT, dyn_dim_profile.size); // Optimal input size - TensorRT will optimize GPU kernels specifically for this shape.
            profile->setDimensions(dyn_dim_profile.tensor_name.c_str(), OptProfileSelector::kMAX, dyn_dim_profile.size); // Maximum allowed input size. Same as kMIN and kOPT since we don't support variable resolution.

            config->addOptimizationProfile(profile); // Register the profile with the builder config.
            builder->setMaxBatchSize(1); // Enforce that the engine only ever processes one frame at a time.
        }

        auto parser = nvonnxparser::createParser(*network, gLogger); // Create the ONNX parser. It will read the raw .onnx file bytes and fill the `network` object with all the layers, weights, and connections.
        if (!parser) {
            std::cerr << "nvonnxparser::createParser failed" << std::endl;
            return 1;
        }

        /*
        Parse the ONNX file bytes into the TensorRT network definition. The parser walks 
        the ONNX protobuf and adds one TensorRT layer at a time into the `network` object. 
        After this call, `network` is a complete description of our YOLO model.
        */
        bool parsed = false;
        unsigned char *onnx_model_buffer = onnx_file_content.data(); // Raw pointer to the start of the .onnx file bytes we read from disk.
        size_t onnx_model_buffer_size = onnx_file_content.size() * sizeof (char); // Total size of the buffer in bytes.
        parsed = parser->parse(onnx_model_buffer, onnx_model_buffer_size); // Tell the parser to interpret those bytes as an ONNX model and populate the TensorRT network graph.

        if (!parsed) {
            std::cerr << "onnx file parsing failed" << std::endl;
            return 1;
        }

        if (builder->platformHasFastFp16()) { // Check if the car's GPU physically supports accelerated FP16 (half-precision) arithmetic. All NVIDIA Ampere and Turing GPUs (e.g., Jetson AGX Xavier/Orin) do.
            std::cout << "FP16 enabled!" << std::endl;
            config->setFlag(BuilderFlag::kFP16); // If FP16 is supported, enable it. This halves the memory footprint of the model weights and typically doubles inference throughput with negligible accuracy loss.
        }

        /*
        The actual compilation step. TensorRT analyzes the full network, fuses layers 
        (e.g., Conv+BN+ReLU become a single kernel), selects the fastest GPU kernels 
        for our specific hardware, and produces an ICudaEngine object ready for inference.
        This step can take several minutes on first run.
        */
        engine = builder->buildEngineWithConfig(*network, *config);

        onnx_file_content.clear(); // Free the ONNX file bytes from RAM - they are no longer needed now that the engine is built.

        // write plan file if it is specified        
        if (engine == nullptr) return 1; // If engine compilation failed (e.g., unsupported ONNX op), bail out.
        IHostMemory* ptr = engine->serialize(); // Serialize the in-memory engine into a flat binary blob suitable for writing to disk.
        assert(ptr);
        if (ptr == nullptr) return 1;

        /*
        Write the serialized engine binary to the .engine file path. This is the file 
        that Yolo::init() will read at every subsequent program startup, bypassing this 
        entire expensive compilation process.
        */
        FILE *fp = fopen(engine_path.c_str(), "wb"); // Open the output .engine file for writing in binary mode.
        fwrite(reinterpret_cast<const char*> (ptr->data()), ptr->size() * sizeof (char), 1, fp); // Write the serialized bytes to disk.
        fclose(fp);

        // Clean up all TensorRT build objects. The engine was saved to disk and is no longer needed in memory.
        parser->destroy();
        network->destroy();
        config->destroy();
        builder->destroy();

        engine->destroy(); // Destroy the in-memory engine. Next time the program runs, init() will reload it from the .engine file on disk.

        return 0;
    } else return 1; // onnx_file_content was empty - the file could not be read.


}

/*
Loads the pre-compiled TensorRT .engine file from disk into GPU memory and sets up all 
the input/output buffers needed to run inference. This is called once at the start of 
ZedLaunchNode::cone_detection_loop() and, if successful, leaves the Yolo object ready 
for repeated calls to run() on every camera frame.

The function auto-detects whether the loaded engine is YOLOv5/v8 format or YOLOv6 format 
by inspecting the output tensor's dimension ordering. It then allocates appropriately-sized 
CPU RAM buffers and GPU VRAM buffers for the inference pipeline.
*/
int Yolo::init(std::string engine_name) {

    /*
    Read the entire .engine binary file into a raw byte array. The .engine file is a 
    serialized TensorRT plan - it is not human-readable; it is a GPU-optimized binary 
    that can only be used on the same GPU architecture it was compiled for.
    */
    std::ifstream file(engine_name, std::ios::binary); // Open the file in binary mode (ios::binary) so no newline translation or encoding conversion happens.
    if (!file.good()) {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        return -1; // The .engine file doesn't exist or cannot be opened. This is the most common failure mode (wrong path, file not copied to car).
    }
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end); // Seek to the end of the file to measure its total size.
    size = file.tellg(); // Record the current position (= file size in bytes).
    file.seekg(0, file.beg); // Seek back to the beginning before reading.
    trtModelStream = new char[size]; // Allocate a CPU RAM buffer large enough to hold the entire .engine file.
    if (!trtModelStream) return 1; // Return failure if memory allocation failed.
    file.read(trtModelStream, size); // Read the entire file into the buffer in one go.
    file.close();

    /*
    Deserialize the raw engine bytes back into live TensorRT objects. Deserialization 
    reconstructs the full ICudaEngine from the binary plan, uploading the optimized 
    GPU kernels and weights back into VRAM. The three objects (runtime, engine, context) 
    form a chain: runtime creates the engine, engine creates the context.
    */
    runtime = createInferRuntime(gLogger); // Create the TensorRT runtime factory. This is the entry point for deserialization.
    if (runtime == nullptr) return 1;
    engine = runtime->deserializeCudaEngine(trtModelStream, size); // Feed the raw bytes to the runtime. It reconstructs the ICudaEngine and uploads weights to the GPU. This is the most expensive step in init().
    if (engine == nullptr) return 1;
    context = engine->createExecutionContext(); // Create the execution context - a stateful object that holds the per-inference memory for intermediate activations during the forward pass.
    if (context == nullptr) return 1;

    delete[] trtModelStream; // The engine is now fully resident in GPU memory; the CPU-side byte array is no longer needed.
    if (engine->getNbBindings() != 2) return 1; // Sanity check: all YOLO models should have exactly 2 bindings - one input tensor and one output tensor.


    /*
    Inspect each binding (tensor) in the loaded engine to extract the model's input/output 
    dimensions. TensorRT "bindings" are its name for the data ports of the neural network. 
    We identify whether each binding is the input or output and read off its geometry 
    (height, width, number of anchors, number of classes). We also use the output geometry 
    to auto-detect which YOLO generation format this engine was compiled from.
    */
    const int bindings = engine->getNbBindings();
    for (int i = 0; i < bindings; i++) {
        if (engine->bindingIsInput(i)) {
            // This binding is the input tensor (the camera image going IN to the network).
            input_binding_name = engine->getBindingName(i); // Store the tensor name (typically "images") for future reference.
            Dims bind_dim = engine->getBindingDimensions(i); // Query the tensor's shape: [batch, channels, height, width].
            input_width = bind_dim.d[3]; // d[3] = width (4th dimension in [N, C, H, W] ordering).
            input_height = bind_dim.d[2]; // d[2] = height (3rd dimension).
            inputIndex = i; // Save the binding index for use in the run() buffer pointer array.
            std::cout << "Inference size : " << input_height << "x" << input_width << std::endl;
        }//if (engine->getTensorIOMode(engine->getBindingName(i)) == TensorIOMode::kOUTPUT) 
        else {
            // This binding is the output tensor (the raw detection data coming OUT of the network).
            output_name = engine->getBindingName(i);
            outputIndex = i; // Save the binding index for use in the run() buffer pointer array.
            Dims bind_dim = engine->getBindingDimensions(i);
            size_t batch = bind_dim.d[0];
            if (batch > batch_size) {
                std::cout << "batch > 1 not supported" << std::endl;
                return 1; // We only support single-frame inference. A batch size > 1 would violate our buffer sizing assumptions.
            }
            size_t dim1 = bind_dim.d[1]; // Second output dimension.
            size_t dim2 = bind_dim.d[2]; // Third output dimension.

            /*
            Auto-detect the YOLO output format by comparing the two non-batch dimensions. 
            The two formats differ in how they orient their output tensor:
              YOLOv6 outputs [1 x num_detections x (5 + num_classes)], so dim1 > dim2 (8400 > 85).
              YOLOv8/v5 outputs [1 x (4 + num_classes) x num_detections], so dim2 > dim1 (8400 > 84).
            */
            if (dim1 > dim2) {
                // Yolov6 1x8400x85 //  85=5+80=cxcy+cwch+obj_conf+cls_conf
                out_dim = dim1; // Number of candidate anchors (rows = detections).
                out_box_struct_number = 5; // YOLOv6 has 5 geometric fields: center x, center y, width, height, and an objectness confidence score.
                out_class_number = dim2 - out_box_struct_number; // Remaining fields are per-class scores.
                yolo_model_version = YOLO_MODEL_VERSION_OUTPUT_STYLE::YOLOV6;
                std::cout << "YOLOV6 format" << std::endl;
            } else {
                // Yolov8 1x84x8400
                out_dim = dim2; // Number of candidate anchors (columns = detections, since the matrix is transposed relative to v6).
                out_box_struct_number = 4; // YOLOv8/v5 uses only 4 geometric fields: center x, center y, width, height. Objectness is folded into the class scores.
                out_class_number = dim1 - out_box_struct_number; // Remaining fields are per-class scores.
                yolo_model_version = YOLO_MODEL_VERSION_OUTPUT_STYLE::YOLOV8_V5;
                std::cout << "YOLOV8/YOLOV5 format" << std::endl;
            }
        }
    }

    /*
    Allocate CPU RAM and GPU VRAM buffers sized exactly for one frame of inference. These 
    buffers act as the staging areas for data moving between the CPU and GPU. They are 
    allocated once here in init() and reused on every call to run(), avoiding the massive 
    per-frame overhead of repeated malloc/cudaMalloc calls.
    */
    output_size = out_dim * (out_class_number + out_box_struct_number); // Total number of float values in one output tensor = (detections per frame) × (values per detection).
    h_input = new float[batch_size * 3 * input_height * input_width]; // CPU input buffer: one frame × 3 color channels × height × width floats. This is where the preprocessed image is staged before upload to the GPU.
    h_output = new float[batch_size * output_size]; // CPU output buffer: this is where the GPU's raw detection data is downloaded to after inference.
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    assert(inputIndex == 0); // Sanity check: input binding must be index 0. This mirrors how the engine was compiled.
    assert(outputIndex == 1); // Sanity check: output binding must be index 1.
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&d_input, batch_size * 3 * input_height * input_width * sizeof (float))); // Allocate GPU VRAM for the input tensor. Mirrors h_input in size.
    CUDA_CHECK(cudaMalloc(&d_output, batch_size * output_size * sizeof (float))); // Allocate GPU VRAM for the output tensor. TensorRT writes raw detections here after the forward pass.
    // Create stream
    CUDA_CHECK(cudaStreamCreate(&stream)); // Create a dedicated CUDA stream. Using a stream allows the GPU memory transfers (cudaMemcpyAsync) and the inference kernel (enqueueV2) to be queued and pipelined without blocking the CPU.

    if (batch_size != 1) return 1; // This sample only support batch 1 for now

    is_init = true; // All allocations succeeded. Set the guard flag so the destructor knows it has resources to free.
    return 0;
}

/*
The per-frame inference function. This is the hot path called by ZedLaunchNode::cone_detection_loop()
on every single camera frame. It accepts the raw ZED camera image and returns a clean, NMS-filtered
list of BBoxInfo detections ready for the ZED sensor fusion pipeline.

The function executes four distinct stages:
  1. Preprocessing:  Converts the ZED BGRA image to BGR, letterbox-resizes it to the network's
                     fixed input resolution (e.g., 640x640), and serializes the pixel data into
                     a normalized float tensor in CHW (channels-first) layout.
  2. Inference:      Asynchronously copies the input tensor to the GPU, fires the TensorRT forward
                     pass, and copies the output tensor back to CPU RAM. cudaStreamSynchronize()
                     blocks until all GPU work is done.
  3. Extraction:     Parses the raw output float array into BBoxInfo structs. Handles both YOLOv8/v5
                     and YOLOv6 output formats. Rescales box coordinates back from the network's
                     padded input space to the original camera image's pixel space.
  4. NMS:            Calls nonMaximumSuppression() to eliminate duplicate boxes and return the
                     final, clean detection list.
*/
std::vector<BBoxInfo> Yolo::run(sl::Mat left_sl, int orig_image_h, int orig_image_w, float thres) {
    std::vector<BBoxInfo> binfo; // The output list. Starts empty and is populated during extraction. Returned at the end.

    size_t frame_s = input_height * input_width; // The number of pixels in one channel of the network's input tensor. Used as a stride when indexing the interleaved CHW buffer.

    /*
    Preprocess the camera frame into the normalized float tensor the neural network expects.
    The ZED SDK delivers images in BGRA format (4 channels with an Alpha layer). The neural 
    network was trained on standard 3-channel BGR images. preprocess_img() also performs 
    letterbox resizing: it scales the image to fit within the network's input box while 
    preserving the aspect ratio, then pads the empty space with grey (value 128) rather 
    than stretching the image.
    */
    cv::Mat left_cv_rgba = slMat2cvMat(left_sl); // Convert the ZED SDK's proprietary sl::Mat image into a standard OpenCV cv::Mat so we can use OpenCV's color conversion and resize functions.
    cv::cvtColor(left_cv_rgba, left_cv_rgb, cv::COLOR_BGRA2BGR); // Strip the alpha channel, converting from 4-channel BGRA to 3-channel BGR.
    if (left_cv_rgb.empty()) return binfo; // Guard: if the conversion produced an empty image (e.g., the camera frame was invalid), return an empty list immediately rather than crashing.
    cv::Mat pr_img = preprocess_img(left_cv_rgb, input_width, input_height); // letterbox BGR to RGB. Resize with letterboxing to the network's exact input resolution. The returned image is input_width × input_height with grey padding on the shorter axis.
    
    /*
    Serialize the preprocessed OpenCV image (HWC layout, uint8) into the h_input float array 
    (CHW layout, float32 in [0,1]). Neural networks expect pixels in CHW order (all red values 
    first, then all green, then all blue) and normalized to [0, 1]. OpenCV stores pixels in HWC 
    order (R, G, B interleaved per pixel). This nested loop performs the layout transpose and 
    the uint8 to float normalization simultaneously.
    */
    int i = 0; // Linear index into the flat CHW float array, incremented for each pixel processed.
    int batch = 0; // Batch index. Always 0 since we only process one frame at a time.
    for (int row = 0; row < input_height; ++row) {
        uchar* uc_pixel = pr_img.data + row * pr_img.step; // Get a pointer to the start of this row's pixel data. pr_img.step is the row stride in bytes.
        for (int col = 0; col < input_width; ++col) {
            h_input[batch * 3 * frame_s + i] = (float) uc_pixel[2] / 255.0; // Red channel (index 2 in BGR). Stored at offset 0 in the CHW layout. Dividing by 255 normalizes from [0,255] to [0,1].
            h_input[batch * 3 * frame_s + i + frame_s] = (float) uc_pixel[1] / 255.0; // Green channel (index 1 in BGR). Stored at offset frame_s (one full plane's worth of floats after the red plane).
            h_input[batch * 3 * frame_s + i + 2 * frame_s] = (float) uc_pixel[0] / 255.0; // Blue channel (index 0 in BGR). Stored at offset 2*frame_s (two full planes after the start).
            uc_pixel += 3; // Advance the pixel pointer by 3 bytes (one BGR pixel).
            ++i; // Advance the CHW index by one position.
        }
    }

    /*
    GPU inference. This is the three-step async pipeline:
      1. cudaMemcpyAsync HostToDevice: Queue a DMA transfer that copies h_input from CPU RAM 
         to d_input in GPU VRAM. "Async" means the CPU does not wait; it just queues the command 
         on the CUDA stream and returns immediately.
      2. context->enqueueV2: Queue the neural network forward pass on the GPU. It reads from 
         d_input and writes raw detection floats to d_output. Also async.
      3. cudaMemcpyAsync DeviceToHost: Queue a DMA transfer that copies d_output from GPU VRAM 
         to h_output in CPU RAM. Also async.
      4. cudaStreamSynchronize: This is the single blocking call. The CPU waits here until the 
         CUDA stream has finished executing all three queued operations in order.
    */
    CUDA_CHECK(cudaMemcpyAsync(d_input, h_input, batch_size * 3 * frame_s * sizeof (float), cudaMemcpyHostToDevice, stream)); // Async upload: copy the preprocessed float tensor from CPU RAM to GPU VRAM.

    std::vector<void*> d_buffers_nvinfer(2); // TensorRT's enqueueV2 requires an array of void pointers, one per binding, pointing to the GPU buffers.
    d_buffers_nvinfer[inputIndex] = d_input; // Wire the GPU input buffer pointer to the correct binding slot.
    d_buffers_nvinfer[outputIndex] = d_output; // Wire the GPU output buffer pointer to the correct binding slot.
    context->enqueueV2(&d_buffers_nvinfer[0], stream, nullptr); // Queue the neural network forward pass. TensorRT reads from d_input and writes to d_output asynchronously on the CUDA stream.

    CUDA_CHECK(cudaMemcpyAsync(h_output, d_output, batch_size * output_size * sizeof (float), cudaMemcpyDeviceToHost, stream)); // Async download: queue the copy of TensorRT's raw output from GPU VRAM back to CPU RAM.
    cudaStreamSynchronize(stream); // Block the CPU here until all three async operations above have completed. Only after this call is h_output safe to read.


    /*
    Coordinate system rescaling. The neural network processed the image at a padded, 
    letterboxed resolution (e.g., 640x640). The box coordinates in h_output are therefore 
    in that padded space. We need to reverse the letterboxing to map them back to the original 
    camera pixel coordinates (orig_image_w × orig_image_h).

    scalingFactor: The ratio used during letterboxing (the smaller of width-ratio and height-ratio 
                   so neither dimension was stretched). We invert it to go from network-space back 
                   to image-space.
    xOffset/yOffset: The number of padding pixels added to each axis during letterboxing. We 
                     subtract them before scaling to remove the padding contribution.
    */
    float scalingFactor = std::min(static_cast<float> (input_width) / orig_image_w, static_cast<float> (input_height) / orig_image_h); // The same scale factor that preprocess_img used. It is the minimum ratio so that the image fits within the network's input box without cropping.
    float xOffset = (input_width - scalingFactor * orig_image_w) * 0.5f; // The horizontal padding added on each side by the letterbox. Subtract this before rescaling to remove the grey border contribution.
    float yOffset = (input_height - scalingFactor * orig_image_h) * 0.5f; // The vertical padding added on each side by the letterbox.
    scalingFactor = 1.f / scalingFactor; // Invert the scale factor. We now multiply by it to go from network pixel coordinates back to original image pixel coordinates.
    float scalingFactor_x = scalingFactor; // Renamed for clarity when applied to x coordinates.
    float scalingFactor_y = scalingFactor; // Renamed for clarity when applied to y coordinates.


    /*
    Parse the raw output float array into BBoxInfo structs. The parsing logic branches on 
    which YOLO version format was detected during init(). Both paths do the same logical 
    work (threshold filter → decode box geometry → rescale coordinates → push to binfo) 
    but read the h_output array in different ways due to the transposed tensor layouts.
    */
    switch (yolo_model_version) {
        default:
        case YOLO_MODEL_VERSION_OUTPUT_STYLE::YOLOV8_V5:
        {
            // https://github.com/triple-Mu/YOLOv8-TensorRT/blob/df11cec3abaab7fefb28fb760f1cebbddd5ec826/csrc/detect/normal/include/yolov8.hpp#L343
            auto num_channels = out_class_number + out_box_struct_number; // Total values per detection = 4 geometric fields + one score per class.
            auto num_anchors = out_dim; // Number of candidate detections in this frame (e.g., 8400 for a 640x640 input).
            auto num_labels = out_class_number; // Number of object categories this model was trained on.

            auto& dw = xOffset; // Alias for readability: dw = "delta width" = horizontal padding to subtract.
            auto& dh = yOffset; // Alias for readability: dh = "delta height" = vertical padding to subtract.

            auto& width = orig_image_w; // Original camera frame width in pixels.
            auto& height = orig_image_h; // Original camera frame height in pixels.

            /*
            Read h_output directly via stride arithmetic, avoiding any matrix allocation or
            memory copy. The buffer is laid out as [num_channels x num_anchors] in row-major
            order, so channel c for anchor i is at data[c * num_anchors + i]. This is the
            zero-copy equivalent of constructing a cv::Mat view and then calling .t(), which
            would trigger a full O(num_channels * num_anchors) heap allocation and copy per frame.
            */
            const float* data = static_cast<const float*>(h_output); // Raw read-only pointer into the TensorRT output buffer. No allocation, no copy.
            for (int i = 0; i < num_anchors; i++) {
                // Find the highest-scoring class for this anchor. std::max_element cannot be used
                // here because the class scores for a single anchor are not contiguous in memory
                // (they are separated by num_anchors elements each). A manual loop is required.
                int label = 0;
                float score = data[out_box_struct_number * num_anchors + i]; // Initialise with class 0's score.
                for (int c = 1; c < num_labels; c++) {
                    float s = data[(out_box_struct_number + c) * num_anchors + i];
                    if (s > score) { score = s; label = c; }
                }
                if (score > thres) { // Only process this anchor if its best class confidence exceeds the threshold (0.8).

                    BBoxInfo bbi; // Container for this detection's data.

                    /*
                    Decode the box from center format (cx, cy, w, h) to corner format (x1, y1, x2, y2).
                    First subtract the letterbox padding (dw, dh) to get coordinates in the scaled-image 
                    space, then multiply by scalingFactor to get back to original image pixel space.
                    clamp() ensures box corners never exceed the image boundaries.
                    */
                    float x = data[0 * num_anchors + i] - dw; // cx in padded network space → subtract horizontal padding.
                    float y = data[1 * num_anchors + i] - dh; // cy in padded network space → subtract vertical padding.
                    float w = data[2 * num_anchors + i]; // Box width in network space (not affected by padding offset).
                    float h = data[3 * num_anchors + i]; // Box height in network space.

                    float x0 = clamp((x - 0.5f * w) * scalingFactor_x, 0.f, width); // Left edge: center minus half-width, then scale back to original image pixels, clamped to [0, image_width].
                    float y0 = clamp((y - 0.5f * h) * scalingFactor_y, 0.f, height); // Top edge: center minus half-height, scaled and clamped.
                    float x1 = clamp((x + 0.5f * w) * scalingFactor_x, 0.f, width); // Right edge: center plus half-width, scaled and clamped.
                    float y1 = clamp((y + 0.5f * h) * scalingFactor_y, 0.f, height); // Bottom edge: center plus half-height, scaled and clamped.

                    cv::Rect_<float> bbox; // An OpenCV rectangle (not used further in this path - legacy field retained for compatibility).
                    bbox.x = x0;
                    bbox.y = y0;
                    bbox.width = x1 - x0;
                    bbox.height = y1 - y0;

                    // Populate the BBoxInfo struct with the final rescaled corner coordinates.
                    bbi.box.x1 = x0;
                    bbi.box.y1 = y0;
                    bbi.box.x2 = x1;
                    bbi.box.y2 = y1;

                    if ((bbi.box.x1 > bbi.box.x2) || (bbi.box.y1 > bbi.box.y2)) break; // Sanity check: a valid box must have its top-left corner strictly to the upper-left of its bottom-right corner. If not, the detection is malformed - skip it.

                    bbi.label = label; // Store the winning class index (e.g., 0 = blue cone, 4 = yellow cone).
                    bbi.prob = score; // Store the confidence score of the winning class.

                    binfo.push_back(bbi); // Add this valid detection to the output list.
                }
            }
            break;
        }
        case YOLO_MODEL_VERSION_OUTPUT_STYLE::YOLOV6:
        {
            // https://github.com/DefTruth/lite.ai.toolkit/blob/1267584d5dae6269978e17ffd5ec29da496e503e/lite/ort/cv/yolov6.cpp#L97

            auto& dw_ = xOffset; // Alias for horizontal padding.
            auto& dh_ = yOffset; // Alias for vertical padding.

            auto& width = orig_image_w;
            auto& height = orig_image_h;

            const unsigned int num_anchors = out_dim; // Total candidate detections per frame in YOLOv6 format.
            const unsigned int num_classes = out_class_number;

            /*
            YOLOv6 stores each detection as a flat row of (5 + num_classes) floats in h_output 
            in HWC order (all values for detection 0 first, then detection 1, etc.). We use a 
            raw pointer with manual stride arithmetic instead of an OpenCV matrix transpose here.
            */
            for (unsigned int i = 0; i < num_anchors; ++i) {
                const float *offset_obj_cls_ptr = h_output + (i * (num_classes + 5)); // Point to the start of this detection's data block. Each block is (num_classes + 5) floats wide.
                float obj_conf = offset_obj_cls_ptr[4]; // Index 4 is the objectness confidence score (how likely is there any object here at all, regardless of class). Note: this is always ~1.0 in practice for YOLOv6 outputs.
                float cls_conf = offset_obj_cls_ptr[5]; // Index 5 is the confidence score for class 0 (used as the initial baseline when finding the highest class score below).

                /*
                YOLOv6's raw confidence output is poorly calibrated in the low range (below ~0.1), 
                producing garbage scores that do not reflect true detection quality. This block 
                remaps the confidence from its raw [0, 1] range into a cleaner effective range 
                by subtracting an offset (0.1) and re-normalizing. This prevents low-quality 
                background noise from leaking through the threshold.
                */
                const float conf_offset = 0.1; // Any raw confidence below this offset is treated as noise and collapsed to 0.
                const float input_start = 0;
                const float output_start = input_start;
                const float output_end = 1;
                const float input_end = output_end - conf_offset; // The re-mapping compresses the valid range [0.1, 1.0] onto [0, 1].

                float conf = (obj_conf * cls_conf) - conf_offset; // Combine objectness and class confidence into a single score (as YOLO intends), then subtract the noise floor offset.
                if (conf < 0) conf = 0; // Clamp to 0 - negative confidence has no meaning.
                conf = (conf - input_start) / (input_end - input_start) * (output_end - output_start) + output_start; // Apply the linear remap to spread the valid range back to [0, 1].

                if (conf > thres) { // Only bother decoding this detection if its remapped confidence exceeds the threshold.

                    /*
                    Find the class with the highest score (argmax). We start with class 0 as 
                    the current best and scan all remaining classes. At the end, `label` holds 
                    the index of the winning class and `cls_conf` holds its score.
                    */
                    unsigned int label = 0;
                    for (unsigned int j = 0; j < num_classes; ++j) {
                        float tmp_conf = offset_obj_cls_ptr[j + 5]; // Class scores start at index 5 (after cx, cy, w, h, objectness).
                        if (tmp_conf > cls_conf) {
                            cls_conf = tmp_conf; // New best class score found.
                            label = j; // Update the winning label.
                        }
                    } // argmax

                    BBoxInfo bbi;

                    /*
                    Decode box geometry from center format to corner format, similar to the 
                    YOLOv8/v5 path but reading directly from the raw float array with manual 
                    index offsets instead of a transposed OpenCV matrix.
                    */
                    float cx = offset_obj_cls_ptr[0]; // Box center x in network space.
                    float cy = offset_obj_cls_ptr[1]; // Box center y in network space.
                    float w = offset_obj_cls_ptr[2]; // Box width in network space.
                    float h = offset_obj_cls_ptr[3]; // Box height in network space.
                    float x1 = ((cx - w / 2.f) - (float) dw_) * scalingFactor_x; // Left edge: subtract half-width for top-left corner, subtract padding, scale to original image space.
                    float y1 = ((cy - h / 2.f) - (float) dh_) * scalingFactor_y; // Top edge.
                    float x2 = ((cx + w / 2.f) - (float) dw_) * scalingFactor_x; // Right edge: add half-width for bottom-right corner, subtract padding, scale.
                    float y2 = ((cy + h / 2.f) - (float) dh_) * scalingFactor_y; // Bottom edge.

                    // Clamp corners to image boundaries.
                    bbi.box.x1 = std::max(0.f, x1); // Left edge cannot be negative.
                    bbi.box.y1 = std::max(0.f, y1); // Top edge cannot be negative.
                    bbi.box.x2 = std::min(x2, (float) width - 1.f); // Right edge cannot exceed image width.
                    bbi.box.y2 = std::min(y2, (float) height - 1.f); // Bottom edge cannot exceed image height.

                    if ((bbi.box.x1 > bbi.box.x2) || (bbi.box.y1 > bbi.box.y2)) break; // Malformed box check: skip if corners are inverted.

                    bbi.label = label;
                    bbi.prob = conf; // Use the remapped, combined confidence score.

                    binfo.push_back(bbi);
                }
            }
            break;
        }
    };

    // Apply Non-Maximum Suppression to the raw detection list. This collapses all duplicate and overlapping boxes down to the cleanest single box per cone.
    binfo = nonMaximumSuppression(nms, binfo);

    return binfo; // Return the final, clean list of detections. cone_detection.cpp will consume this to build the ZED CustomBoxObjectData list for sensor fusion.
}
