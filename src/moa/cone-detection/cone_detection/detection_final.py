#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moa_msgs.msg import Cone
from moa_msgs.msg import ConeStamped
from moa_msgs.msg import ConeMap
from moa_msgs.msg import ConeMapStamped

import sys
import numpy as np
from OpenGL.GLUT import *
import argparse
import torch
import cv2
import pyzed.sl as sl
import torch.backends.cudnn as cudnn

sys.path.insert(0, './yolov7')
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device
from utils.datasets import letterbox

from threading import Lock, Thread
from time import sleep

global exit_signal,detections, weights, img_size, conf_thres

#Basic arguments of the scripts
weights = "yolov7m.pt"
img_size = 416
conf_thres = 0.4

lock = Lock()
exit_signal = False

class detection(Node):
    def __init__(self):
        self.run_signal = False
        super().__init__('detector')

        # Initialize ZED camera and YOLOv7
        capture_thread = Thread(target=self.torch_thread,kwargs={'weights': weights, 'img_size': img_size, "conf_thres": conf_thres})
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
        # If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
        # positional_tracking_parameters.set_as_static = True
        self.zed.enable_positional_tracking(positional_tracking_parameters)
        
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        obj_param.enable_tracking = True
        self.zed.enable_object_detection(obj_param)
        
        self.objects = sl.Objects()
        self.pose = sl.Pose()
        self.py_translation = sl.Translation()
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

        # ... [Initialize the ROS 2 publisher for DetectedObject message]
        self.publisher = self.create_publisher(ConeMap, 'cone_detection', 10)
        self.timer = self.create_timer(0.5, self.run_detection)

    def img_preprocess(self, img, device, half, net_size):
        net_image, ratio, pad = letterbox(img[:, :, :3], net_size, auto=False)
        net_image = net_image.transpose((2, 0, 1))[::-1] # HWC to CHW, BGR to RGB
        net_image = np.ascontiguousarray(net_image)

        img = torch.from_numpy(net_image).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        if img.ndimension() == 3:
                img = img.unsqueeze(0)
        return img, ratio, pad

    def xywh2abcd(self, xywh, im_shape):
        output = np.zeros((4, 2))

        # Center / Width / Height -> BBox corners coordinates
        x_min = (xywh[0] - 0.5*xywh[2]) * im_shape[1]
        x_max = (xywh[0] + 0.5*xywh[2]) * im_shape[1]
        y_min = (xywh[1] - 0.5*xywh[3]) * im_shape[0]
        y_max = (xywh[1] + 0.5*xywh[3]) * im_shape[0]

        # A ------ B
        # | Object |
        # D ------ C
        
        output[0][0] = x_min
        output[0][1] = y_min
        
        output[1][0] = x_max
        output[1][1] = y_min
        
        output[2][0] = x_min
        output[2][1] = y_max
        
        output[3][0] = x_max
        output[3][1] = y_max
        return output

    def detections_to_custom_box(self, detections, im, im0):
        output = []
        for i, det in enumerate(detections):
            if len(det):
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]] # normalization gain whwh
                
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh

					# Creating ingestable objects for the ZED SDK
                    obj = sl.CustomBoxObjectData()
                    obj.bounding_box_2d = self.xywh2abcd(xywh, im0.shape)
                    obj.label = cls
                    obj.probability = conf
                    obj.is_grounded = False
                    output.append(obj)
        return output

    def torch_thread(self, weights, img_size, conf_thres=0.2, iou_thres=0.45):
        global image_net, exit_signal, detections
	
        print("Intializing Network...")
        
        device = select_device()
        half = device.type != 'cpu'  # half precision only supported on CUDA
        imgsz = img_size
        
        # Load model
        model = attempt_load(weights, map_location=device)  # load FP32
        stride = int(model.stride.max())  # model stride
        imgsz = check_img_size(imgsz, s=stride)  # check img_size
        if half:
            model.half()  # to FP16
        cudnn.benchmark = True

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        
        while not exit_signal:
            #print("looping in another thread")
            if self.run_signal:
                lock.acquire()
                img, ratio, pad = self.img_preprocess(self.image_net, device, half, imgsz)

                pred = model(img)[0]
                det = non_max_suppression(pred, conf_thres, iou_thres)

                # ZED CustomBox format (with inverse letterboxing tf applied)
                detections = self.detections_to_custom_box(det, img, self.image_net)
                lock.release()
                self.run_signal = False
            sleep(0.01)

    def run_detection(self):
        self.zed.grab(self.runtime_params)
        # -- Get the image
        lock.acquire()
        self.zed.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
        self.image_net = self.image_left_tmp.get_data()
        lock.release()
        self.run_signal = True
        print("here")
        # -- Detection running on the other thread
        while self.run_signal:
            #print(run_signal);
            sleep(0.001)
        print("outside loop")
        # Wait for detections
        lock.acquire()
        # -- Ingest detections
        self.zed.ingest_custom_box_objects(detections)
        lock.release()
        self.zed.retrieve_objects(self.objects, self.obj_runtime_param)

        all_cones = ConeMap();
        single_cone = Cone();
        #Get position and rotation as first cone
        self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        rotation = self.pose.get_rotation_vector()
        translation = self.pose.get_translation(self.py_translation)
        #Create messages and send
        for object in self.objects.object_list:
            single_cone.id = object.id
            single_cone.confidence = object.confidence
            #single_cone.colour = int(object.label[0])
            single_cone.colour = 1

            #single_cone.pose.covariance = object.position_covariance
            single_cone.pose.pose.position.x = object.position[0]
            single_cone.pose.pose.position.y = object.position[1]
            single_cone.pose.pose.position.z = object.position[2]
            single_cone.radius = object.dimensions[0]/2
            single_cone.height = object.dimensions[1]
            all_cones.cones.append(single_cone)

        self.publisher.publish(all_cones)
        string_output = "";
        for cone_item in all_cones.cones:
            string_output += "s"
        print(string_output);

def main(args=None):
    rclpy.init(args=args)
    cone_detection = detection()
    rclpy.spin(cone_detection)  # The spin function will run the node until it is shutdown
    exit_signal = True
    cone_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
