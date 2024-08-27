#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moa_msgs.msg import ConeStamped
from moa_msgs.msg import ConeMapStamped
from moa_msgs.msg import ConeMap
from moa_msgs.msg import Cone

import sys
import numpy as np
import cv2
import pyzed.sl as sl
from threading import Lock, Thread
from time import sleep
from ultralytics import YOLO
import torch


lock = Lock()
exit_signal = False
# detections = []
# image_net = None

# args
weights = "yolov8m.pt"
img_size = 416
conf_thres = 0.2

class Detection(Node):
    def __init__(self):
        super().__init__('detector')
        self.run_signal = False

        # Initialize ZED camera and YOLOv8
        capture_thread = Thread(target=self.torch_thread, kwargs={'weights': weights, 'img_size': img_size, 'conf_thres': conf_thres})
        capture_thread.start()

        print("Initializing Camera...")

        self.zed = sl.Camera()
        input_type = sl.InputType()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.depth_maximum_distance = 50

        self.runtime_params = sl.RuntimeParameters()
        status = self.zed.open(init_params)

        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()

        self.image_left_tmp = sl.Mat()

        print("Initialized Camera")

        positional_tracking_parameters = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(positional_tracking_parameters)

        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True
        self.zed.enable_object_detection(obj_param)

        self.objects = sl.Objects()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

        # Initialize the ROS2 publisher for the ConeMap message
        self.publisher = self.create_publisher(ConeMap, 'cone_map', 10)
        self.timer = self.create_timer(0.1, self.run_detection)
        self.counter = 0

    def xywh2abcd(self, xywh, im_shape):
        output = np.zeros((4, 2))

        x_min = (xywh[0] - 0.5 * xywh[2])
        x_max = (xywh[0] + 0.5 * xywh[2])
        y_min = (xywh[1] - 0.5 * xywh[3])
        y_max = (xywh[1] + 0.5 * xywh[3])

        output[0][0] = x_min
        output[0][1] = y_min

        output[1][0] = x_max
        output[1][1] = y_min

        output[2][0] = x_max
        output[2][1] = y_max

        output[3][0] = x_min
        output[3][1] = y_max
        return output

    def detections_to_custom_box(self, detections, im0):
        output = []
        for i, det in enumerate(detections):
            xywh = det.xywh[0]

            obj = sl.CustomBoxObjectData()
            obj.bounding_box_2d = self.xywh2abcd(xywh, im0.shape)
            obj.label = det.cls
            obj.probability = det.conf
            obj.is_grounded = False
            output.append(obj)
        return output

    def torch_thread(self, weights, img_size, conf_thres=0.2, iou_thres=0.45):
        global image_net, exit_signal, detections

        print("Intializing Network...")

        model = YOLO(weights)

        while not exit_signal:
            if self.run_signal:
                lock.acquire()
                img = cv2.cvtColor(image_net, cv2.COLOR_BGRA2BGR)
                det = model.predict(img, save=False, imgsz=img_size, conf=conf_thres, iou=iou_thres)[0].cpu().numpy().boxes

                detections = self.detections_to_custom_box(det, image_net)
                lock.release()
                self.run_signal = False
            sleep(0.01)

    def run_detection(self):
        self.zed.grab(self.runtime_params)

        lock.acquire()
        self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
        self.image_net = self.image_left_tmp.get_data()
        lock.release()

        self.run_signal = True

        while self.run_signal:
            sleep(0.001)

        lock.acquire()
        self.zed.ingest_custom_box_objects(detections)
        lock.release()
        self.zed.retrieve_objects(self.objects, self.obj_runtime_param)

        all_cones = ConeMap()

        # Commented out the localization cone section
        # all_cones.cones.append(self.get_localization_cone())

        for obj in self.objects.object_list:
            # moved single_cone to inside for obj loop
            single_cone = Cone()
            single_cone.id = obj.id
            single_cone.confidence = obj.confidence
            single_cone.colour = 1

            # Position
            single_cone.pose.pose.position.x = obj.position[0]
            single_cone.pose.pose.position.y = obj.position[2] * -1
            single_cone.pose.pose.position.z = obj.position[1]

            # Dimensions
            single_cone.radius = obj.dimensions[0] / 2
            single_cone.height = obj.dimensions[1]

            # Bounding Box (Assuming `Cone` message has a bounding box attribute)
            single_cone.bounding_box = []  # You may need to define the structure of the bounding box
            for corner in obj.bounding_box:
                point = geometry_msgs.msg.Point()
                point.x = corner[0]
                point.y = corner[2] * -1
                point.z = corner[1]
                single_cone.bounding_box.append(point)

            # Append new cone
            all_cones.cones.append(single_cone)

        self.publisher.publish(all_cones)

        self.counter += 1
        if self.counter == 100:
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
            self.counter = 0

    # don't need to track camera position, but jsut in case
    # def get_localization_cone(self):
    #     localization_cone = Cone()
    #     self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
    #     rotation = self.pose.get_rotation_vector()
    #     translation = self.pose.get_translation(self.py_translation)
    #     x = round(translation.get()[0], 2)
    #     y = -1 * round(translation.get()[2], 2)
    #     w = round(rotation[1], 2)
    #     localization_cone.id = 99999
    #     localization_cone.pose.pose.position.x = x
    #     localization_cone.pose.pose.position.y = y
    #     localization_cone.pose.pose.orientation.w = w
    #
    #     return localization_cone


def main(args=None):
    rclpy.init(args=args)
    cone_detection = Detection()
    rclpy.spin(cone_detection)
    exit_signal = True
    cone_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
