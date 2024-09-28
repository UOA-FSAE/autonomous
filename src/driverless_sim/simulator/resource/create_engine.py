from ultralytics import YOLO

# Load your custom-trained YOLOv8 model
model = YOLO('src/driverless_sim/simulator/resource/cone_detection.pt')

# Export the model to TensorRT format
model.export(format='engine')  # This will create 'best_prep.engine'