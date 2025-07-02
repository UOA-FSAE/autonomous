#!/usr/bin/env python3
# Python imports
import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, Vector3
from ackermann_msgs.msg import AckermannDrive
from foxglove_msgs.msg import LinePrimitive, Color, SceneEntity, SceneUpdate, ArrowPrimitive, SpherePrimitive, PoseInFrame, PosesInFrame
from builtin_interfaces.msg import Time, Duration
from moa_msgs.msg import Track



class pure_pursuit_algorithm(Node):
    def __init__(self):
        super().__init__("Pure_Pursuit_Controller")
        self.get_logger().info("Pure Pursuit Node Started")

        # Constant to tune (touch me please it makes me feel horny ahhhhhhh!)
        ## Tuning for look ahead distance
        self.look_up_distance = 10
        ## Tuning for matching steering in theory with steering in actual
        self.P = 100
        ## Current speed setting
        self.current_speed = 2

        # Initializer (normally don't touch)
        self.steering_angle = 0
        self.pos = (0,0)      

        # visualisation parameters
        self.declare_parameter('vis', False)  # Enable/Disable visualization
        self.visualization_enabled = self.get_parameter('vis').get_parameter_value().bool_value

        # subscribe to best trajectory
        self.best_trajectory_sub = self.create_subscription(PoseArray, "moa/selected_trajectory", self.selected_trajectory_handler, 5)
        
        self.cone_map_sub = self.create_subscription(Track, "cone_map", self.main_hearback, 5)

        self.create_subscription(Pose, "car_position", self.get_car_position, 5)
        self.cmd_vel_pub = self.create_publisher(AckermannDrive, "/drive", 5)
        self.cmd_vis_pub = self.create_publisher(AckermannDrive, "/drive_vis", 5)
        self.track_point_pub = self.create_publisher(Pose, "moa/track_point", 5)

        if self.visualization_enabled:
            self.viz_pub = self.create_publisher(SceneUpdate, 'control_visualization', 5)
            # selected path
            self.chosen_path = self.create_subscription(AckermannDrive, "/drive_vis", self.show_drive_path, 5)
            self.next_destination = self.create_subscription(Pose, "moa/track_point", self.save_pursue_destination, 5)
            self.cone_map_sub = self.create_subscription(Pose, "car_position", self.get_transformations, 5)

            self.id = 1

        else:
            self.get_logger().info("Visualization is disabled")

        self.get_logger().info("Pure Pursuit Node Started")

    def main_hearback(self, msg: Track):
        # Update car's current location and update transformation matrix
        self.car_pose = self.car_position_pose
        self.position_vector, self.rotation_matrix_l2g, self.rotation_matrix_g2l = self.convert_to_transformation_matrix(self.car_pose.position.x, self.car_pose.position.y, self.car_pose.orientation.w)

        # Before proceed, check whether we have a trajectory input
        if hasattr(self, "trajectory_in_global_frame"):
            # Update destination point to track
            self.update_track_point(self.trajectory_in_global_frame)

            # Get expected steering angle to publish
            self.steering_angle = self.get_steering_angle(self.Pose_to_track_in_global_frame)

            self.steering_angle = self.saturating_steering(self.steering_angle)
            # self.get_logger().info(f"Set steering angle to {self.steering_angle * self.P}")

        else:
            self.steering_angle = 0
            self.get_logger().info("Warning: no trajectory found, will set steering angle to 0!!!!")

        # Publish command for velocity
        self.publish_ackermann()

    def selected_trajectory_handler(self, msg: PoseArray):
        self.trajectory_in_global_frame = msg

    def saturating_steering(self, steering_angle):
        saturation = 10
        if steering_angle > saturation:
            steering_angle = saturation
        elif steering_angle < -1 * saturation:
            steering_angle = -1 * saturation

        return steering_angle
    
    def single_trajectory_generator(self, steering_radius):
        # https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357 is used for
        # bicycle steering
        R = steering_radius
        t_range = np.arange(0, np.pi/2, 0.01);
        trajectory_output = PoseArray();
        if hasattr(self, "position_vector") and hasattr(self, "rotation_matrix_l2g") and hasattr(self, "rotation_matrix_g2l"):
            for individual_t in t_range:
                pose_input = Pose();
                x_pre_trans = np.cos(individual_t) * R - R
                y_pre_trans = abs(np.sin(individual_t) * R)
                post_trans_point = self.apply_transformation(self.position_vector, self.rotation_matrix_l2g, x_pre_trans, y_pre_trans);
                pose_input.position.x = post_trans_point[0][0]
                pose_input.position.y = post_trans_point[1][0]
                trajectory_output.poses.append(pose_input)
        else:
            self.get_logger().info("Warning: Transformation data not acquired, no trajectory produced")
        return trajectory_output
    
    def apply_transformation(self, position_vector, rotation_matrix, point_x, point_y):
        point = np.array([[point_x], [point_y]])
        transformed_point = np.matmul(rotation_matrix, point) + position_vector
        return transformed_point

    def get_transformations(self, msg: Pose):
        # Update car's current location and update transformation matrix
        self.car_pose = msg
        self.position_vector, self.rotation_matrix_l2g, self.rotation_matrix_g2l = self.convert_to_transformation_matrix(self.car_pose.position.x, self.car_pose.position.y, self.car_pose.orientation.w)

    # Coordinate tranformer
    def convert_to_transformation_matrix(self, x: float, y: float, theta: float) -> (
            np.array, np.array, np.array):
        '''Convert state and list_of_cones input into position vector, rotation matrix (DCM) and the matrix of list of cones

        Args:
            x: x position specified in float
            y: y position specified in float
            theta: theta orientation specified in float
            list_of_cones: np.array (numpy array) for matrix of cone positions in 2 by n matrix (n is number of cones recorded in input))

        Returns:
            position_vector: np.array of position vector of cart
            rotation_matrix: np.array DCM matrix to convert reading from local frame into global frame
            list_of_cones: np.array 2 x n matrix of the list of cones measured in global frame

        Raises:
            None
        '''
        position_vector = np.array([[x], [y]])
        rotation_matrix_l2g = np.array(
            [[math.cos(theta), -math.sin(theta)],
             [math.sin(theta), math.cos(theta)]])  # local coordinate to global coordinate matrix
        rotation_matrix_g2l = np.array(
            [[math.cos(theta), math.sin(theta)],
             [-math.sin(theta), math.cos(theta)]])  # global coordinate to local coordinate matrix

        return position_vector, rotation_matrix_l2g, rotation_matrix_g2l


    def get_track_point_in_local_frame(self, track_point_in_global_frame: Pose):
        if (hasattr(self, "position_vector") and hasattr(self, "rotation_matrix_l2g") and hasattr(self, "rotation_matrix_g2l")):
            track_point_in_local_frame = Pose()
            position_input = np.array(
                [[track_point_in_global_frame.position.x], [track_point_in_global_frame.position.y]])
            position_output = np.matmul(self.rotation_matrix_g2l, (position_input - self.position_vector))
            track_point_in_local_frame.position.x = float(position_output[0])
            track_point_in_local_frame.position.y = float(position_output[1])
            return track_point_in_local_frame
        else:
            self.get_logger.info("Warning: Local pose message not transformed to global frame")
            return track_point_in_global_frame


    def get_track_point_in_global_frame(self, track_point_in_local_frame: Pose):
        if (hasattr(self, "position_vector") and hasattr(self, "rotation_matrix_l2g") and hasattr(self, "rotation_matrix_g2l")):
            track_point_in_global_frame = Pose()
            position_input = np.array(
                [[track_point_in_local_frame.position.x], [track_point_in_local_frame.position.y]])
            position_output = np.matmul(self.rotation_matrix_l2g, position_input) + self.position_vector
            track_point_in_global_frame.position.x = float(position_output[0])
            track_point_in_global_frame.position.y = float(position_output[1])
            return track_point_in_global_frame
        else:
            self.get_logger.info("Warning: Local pose message not transformed to global frame")
            return track_point_in_local_frame

    def show_chosen_path(self, msg: PoseArray):
        tcols = Color(r=0.0, g=255.0, b=0.0, a=1.0)
        pts = []
        line_list = [];
        for j in range(len(msg.poses)):
            # get a particular pose
            _ = msg.poses[j].position
            pts.append(_)
        args = {'type': LinePrimitive.LINE_STRIP,
                'pose': Pose(position=Point(x=0.0, y=0.0, z=0.0),
                             orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)),
                'thickness': 2.0,
                'scale_invariant': True,
                'points': pts,
                'color': tcols}
        #line_list.append(LinePrimitive(**args))

        # arrow primitive code if needed
        # args = {'pose': Pose(position=Point(x=1.0,y=0.0,z=0.0), orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=0.0)),
        #         'shaft_length': 1.0,
        #         'shaft_diameter': 0.1,
        #         'head_length': 2.5,
        #         'head_diameter': 0.5,
        #         'color': Color(r=67.0,g=125.0,b=100.0,a=1.0)}
        # msg = ArrowPrimitive(**args)

        # scene entity encapsulates these primitive objects
        sargs = {'timestamp': Time(sec=0,nanosec=0),
                    'frame_id': 'global_frame',
                    'id': f'{self.id}',
                    'lifetime': Duration(sec=3,nanosec=0),
                    'frame_locked': False,
                    'lines': line_list,
                    'spheres': self.next_destination_vis}

        # scene update is a wrapper for scene entity
        scene_update_msg = SceneUpdate(entities=[SceneEntity(**sargs)])

        self.pubviz.publish(scene_update_msg)
        #self.get_logger().info("Published msg")

        self.id += 1

    def show_drive_path(self, msg: AckermannDrive):
        steering_angle = msg.steering_angle
        if steering_angle == 0:
            steering_angle = 1e-9
        steering_radius = 1 / steering_angle
        trajectory_from_steer = self.single_trajectory_generator(steering_radius)
        self.show_chosen_path(trajectory_from_steer)

# Picking and maintaining a track point
    def update_track_point(self, msg: PoseArray): #Main logic
        # Pick new tracking point if no tracking point is selected or old tracking point is no longer visible
        if self.need_new_track_point():
            self.Pose_to_track_in_global_frame = self.get_track_point_in_global_frame(msg)
        self.track_point_pub.publish(self.Pose_to_track_in_global_frame)

    def need_new_track_point(self):
        if not(hasattr(self, "Pose_to_track_in_global_frame")):
            return True
        else:
            Pose_to_track_in_local_frame = self.get_track_point_in_local_frame(self.Pose_to_track_in_global_frame)
            if self.track_point_still_visible(Pose_to_track_in_local_frame):
                return False
            else:
                return True

    def get_track_point_in_global_frame(self, trajectory_in_global_frame: PoseArray):
        if hasattr(self, "position_vector") and hasattr(self, "rotation_matrix_l2g") and hasattr(self, "rotation_matrix_g2l"):
            sorted_trajectory_poses_in_global_frame = PoseArray()
            sorted_trajectory_poses_in_global_frame.poses = sorted(trajectory_in_global_frame.poses, key=lambda item: self.get_distance(self.get_track_point_in_local_frame(item)))
            for track_point_in_global_frame in sorted_trajectory_poses_in_global_frame.poses:
                track_point_in_local_frame = self.get_track_point_in_local_frame(track_point_in_global_frame)
                if self.track_point_distance_above_look_up_distance(track_point_in_local_frame):
                    return track_point_in_global_frame
            return self.car_pose

    def track_point_still_visible(self, track_point_in_local_frame: Pose):
        return track_point_in_local_frame.position.y >= 0

    def track_point_distance_above_look_up_distance(self, track_point_in_local_frame: Pose):
        return self.get_distance(track_point_in_local_frame) >= self.look_up_distance

    def get_distance(self, track_point_in_local_frame: Pose):
        x = track_point_in_local_frame.position.x
        y = track_point_in_local_frame.position.y
        return abs(((x ** 2) + (y ** 2)) ** (1/2))

### Calculate Steering Angle
    def get_steering_angle(self, Pose_to_track_in_global_frame: Pose):
        Pose_to_track_in_local_frame = self.get_track_point_in_local_frame(Pose_to_track_in_global_frame);
        L = self.get_distance(Pose_to_track_in_local_frame) # Length from origin
        x = Pose_to_track_in_local_frame.position.x
        arc_radius = self.get_arc_radius(L,x)
        steering_angle = 1 / arc_radius
        if Pose_to_track_in_local_frame.position.x > 0:
            return abs(steering_angle)
        elif Pose_to_track_in_local_frame.position.x < 0:
            return -1 * abs(steering_angle)
        else:
            return 0.0

    def lateral_distance(self):
        return self.get_track_point_in_local_frame(self.Pose_to_track_in_global_frame).position.x

    def get_arc_radius(self, L, x):
        return L ** 2 / (2 * abs(x))

    def publish_ackermann(self):

        args1 = {"steering_angle": float(self.steering_angle * self.P),
                "steering_angle_velocity": 0.0,
                "speed": float(self.current_speed),
                "acceleration": 0.0,
                "jerk": 0.0}
        msg1 = AckermannDrive(**args1)

        print(msg1)

        args2 = {"steering_angle": float(self.steering_angle),
                "steering_angle_velocity": 0.0,
                "speed": float(self.current_speed),
                "acceleration": 0.0,
                "jerk": 0.0}
        msg2 = AckermannDrive(**args2)
        self.cmd_vel_pub.publish(msg1)
        self.cmd_vis_pub.publish(msg2)
    
    def get_car_position(self, msg:Pose): self.car_position_pose = msg


    def visualize_trajectory(self):
        tcols = Color(r=0.0, g=255.0, b=0.0, a=1.0)  # Green trajectory
        if hasattr(self, "trajectory_in_global_frame"):
            pts = [pose.position for pose in self.trajectory_in_global_frame.poses]
            line = LinePrimitive(type=LinePrimitive.LINE_STRIP, points=pts, color=tcols, thickness=0.2)
            scene_msg = SceneUpdate(
                entities=[SceneEntity(
                    id=f'trajectory_{self.id_counter}',
                    lines=[line],
                    timestamp=Time()
                )]
            )
            self.viz_pub.publish(scene_msg)
            self.id_counter += 1


    def save_pursue_destination(self, msg : Pose):
        tcols = Color(r=255.0, g=255.0, b=0.0, a=1.0)
        args = {'pose': msg,
                'size': Vector3(x=1.0, y=1.0, z=1.0),
                'color': tcols}
        if len(self.next_destination_vis) == 0:
            self.next_destination_vis.append(SpherePrimitive(**args))
        else:
            self.next_destination_vis[0] = SpherePrimitive(**args)

def main(args=None):
    rclpy.init(args=args)

    pure_pursuiter = pure_pursuit_algorithm()

    rclpy.spin(pure_pursuiter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pure_pursuiter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()