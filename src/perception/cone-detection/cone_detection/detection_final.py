#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moa_msgs.msg import ConeStamped
from moa_msgs.msg import ConeMapStamped
from moa_msgs.msg import ConeMap
from moa_msgs.msg import Cone

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
        #self.publisher = self.create_publisher(ConeMap, 'cone_detection', 10)
        self.publisher = self.create_publisher(ConeMap, 'cone_map', 10)
        self.timer = self.create_timer(0.1, self.run_detection)

        self.counter = 0

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
        #print("here")
        # -- Detection running on the other thread
        while self.run_signal:
            #print(run_signal);
            sleep(0.001)
        #print("outside loop")
        # Wait for detections
        lock.acquire()
        # -- Ingest detections
        self.zed.ingest_custom_box_objects(detections)
        lock.release()
        self.zed.retrieve_objects(self.objects, self.obj_runtime_param)

        all_cones = ConeMap();
        single_cone = Cone();
        all_cones.cones.append(self.get_localization_cone())
        #Create messages and send
        for object in self.objects.object_list:
            single_cone.id = object.id
            single_cone.confidence = object.confidence
            #single_cone.colour = int(object.label[0])
            single_cone.colour = 1

            #single_cone.pose.covariance = object.position_covariance
            single_cone.pose.pose.position.x = object.position[0]
            single_cone.pose.pose.position.y = object.position[2] * -1
            single_cone.pose.pose.position.z = object.position[1]
            single_cone.radius = object.dimensions[0]/2
            single_cone.height = object.dimensions[1]
            all_cones.cones.append(single_cone)

        self.publisher.publish(all_cones)
        # self.get_logger().info('Publishing: "%s"' % ConeMap.data)
        string_output = ""
        is_first = True

        # print("######################New Message##########################" + str(self.counter))
        # self.print_cone_information(all_cones.cones[0], True)
        # for cone_item in all_cones.cones:
        #     self.print_cone_information(cone_item, is_first)
        #     is_first = False

        self.counter += 1
        if self.counter == 100:
            torch.cuda.empty_cache()
            torch.cuda.synchronize()
            self.counter = 0

    def get_localization_cone(self):
        localization_cone = Cone();
        #Get position and rotation as first cone
        self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
        rotation = self.pose.get_rotation_vector()
        translation = self.pose.get_translation(self.py_translation)
        x = round(translation.get()[0], 2);
        y = -1 * round(translation.get()[2], 2);
        w = round(rotation[1], 2);
        localization_cone.id = 99999;
        localization_cone.pose.pose.position.x = x
        localization_cone.pose.pose.position.y = y
        localization_cone.pose.pose.orientation.w = w

        return localization_cone


    def print_cone_information(self, cone_input, is_first):
        id_message = ""
        x_message = "x: " + str(cone_input.pose.pose.position.x)
        y_message = "y: " + str(cone_input.pose.pose.position.y)
        z_message = "z: " + str(cone_input.pose.pose.position.z)
        w_message = "w: " + str(cone_input.pose.pose.orientation.w * 180 / np.pi)
        color_message = "color_id: " + str(cone_input.colour)
        radius_message = "radius: " + str(cone_input.radius)
        height_message = "height: " + str(cone_input.height)

        if is_first:
            id_message = "Car location"
        else:
            id_message = str(cone_input.id)

        print("############################")
        print(id_message);
        print(x_message);
        print(y_message);
        print(z_message);
        print(w_message);
        print(color_message);
        print(radius_message);
        print(height_message);




def main(args=None):
    rclpy.init(args=args)
    cone_detection = detection()
    rclpy.spin(cone_detection)  # The spin function will run the node until it is shutdown
    exit_signal = True
    cone_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
