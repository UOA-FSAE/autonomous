from inference.models.utils import get_roboflow_model
import cv2

# Image path
image_path = "cone-2.jpg"

# Roboflow model
model_name = "fsae-autonomous-cone-detection"
model_version = "1"

# Get Roboflow face model (this will fetch the model from Roboflow)
model = get_roboflow_model(
    model_id="{}/{}".format(model_name, model_version),
    # Replace ROBOFLOW API KEY with your Roboflow API Key
    api_key="HRZLnQrwYBnB2XoN2XHQ"
)

# Load image with opencv
frame = cv2.imread(image_path)

# Inference image to find faces
results = model.infer(image=frame, confidence=0.5, iou_threshold=0.5)[0]

for prediction in results.predictions:
    print(prediction)
    x0 = int(prediction.x - prediction.width / 2)
    y0 = int(prediction.y - prediction.height / 2)
    x1 = int(prediction.x + prediction.width / 2)
    y1 = int(prediction.y + prediction.height / 2)

    cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 255, 0), 1)

# Show image
cv2.imshow('Image Frame', frame)
cv2.waitKey(0)  # waits until a key is pressed
cv2.destroyAllWindows()  # destroys the window showing image
