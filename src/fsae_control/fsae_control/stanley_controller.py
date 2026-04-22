import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from std_msgs.msg import Header

def angle_mod(x, zero_2_2pi=False, degree=False):
    """
    Angle modulo operation
    Default angle modulo range is [-pi, pi)

    Parameters
    ----------
    x : float or array_like
        A angle or an array of angles. This array is flattened for
        the calculation. When an angle is provided, a float angle is returned.
    zero_2_2pi : bool, optional
        Change angle modulo range to [0, 2pi)
        Default is False.
    degree : bool, optional
        If True, then the given angles are assumed to be in degrees.
        Default is False.

    Returns
    -------
    ret : float or ndarray
        an angle or an array of modulated angle.

    Examples
    --------
    >>> angle_mod(-4.0)
    2.28318531

    >>> angle_mod([-4.0])
    np.array(2.28318531)

    >>> angle_mod([-150.0, 190.0, 350], degree=True)
    array([-150., -170.,  -10.])

    >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
    array([300.])

    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle

class StanleyControl(Node):

    def __init__(self):
        super().__init__('Stanley_Controller')
        self.get_logger().info("Stanley Controller Node Started")
        
        self.declare_parameters(
            namespace='',
            parameters=[
            ('vel', 8.0),
            ]
        )
        
        #Constants
        self.k_stanley = 5.0 #stanley Controller gain
        self.k_speed = 1.0 #speed Controller gain
        self.cam_fron_axle_dist= 1 #[m] Wheel base of vehicle
        self.max_steer = 30.0  # [degrees] max steering angle
        self.target_speed = self.get_parameter('vel').get_parameter_value() #[m/s]
        
        self.trajectory_in_global_frame = PoseArray()
        self.tx = []
        self.ty = []

        #Rate limiting + smoothing to prevent servo jitter and overcurrent
        self.steering_angle = 0.0
        self.last_publish_time = self.get_clock().now()
        self.MIN_PUBLISH_INTERVAL = 0.1  # 10 Hz max publish rate
        self.ALPHA = 0.3               # low-pass filter: weight on new measurement

        #Subscribe for car pose and track
        self.create_subscription(PoseArray, "selected_trajectory", self.selected_trajectory_handler, 5)
        self.create_subscription(Pose, "car_position", self.main_hearback, 5)
        #Publish result
        self.cmd_drive_pub = self.create_publisher(AckermannDrive, "drive", 5)
        self.cmd_vis_pub = self.create_publisher(AckermannDrive, "drive_vis", 5)
        self.cmd_vel_pub = self.create_publisher(AckermannDriveStamped, "cmd_vel", 5)
        self.create_publisher(Pose, "track_point", 5)
    
    def main_hearback(self, msg):
        car_pose = msg
        camera_position = [car_pose.position.x,car_pose.position.y]
        car_yaw = car_pose.orientation.w
        self.car_yaw_corrected = self.normalize_angle(car_yaw-4.71)
        
        # self.target_speed = 10.0
        self.target_speed = self.get_parameter('vel').get_parameter_value().double_value
        
        self.get_logger().info(f"car_yaw: {car_yaw}, car_yaw_corrected: {self.car_yaw_corrected}")

        if len(self.tx) > 1 and len(self.ty) > 1:
            #Get Car front axle center position
            axle_pos = self.get_front_axle_position(camera_position,self.car_yaw_corrected)
            #Get closest point on track and distance error
            cls_point,error_front_axle = self.get_closest_track_point(axle_pos,self.car_yaw_corrected)
            #Compute target yaw - Angle from positive x in radians
            target_yaw = self.cal_target_yaw(cls_point)
            #Compute steering angle
            theta_e = -self.circular_diff(target_yaw, self.car_yaw_corrected)
            theta_d = -(np.arctan2(self.k_stanley * error_front_axle, self.target_speed))

            self.get_logger().info(f"target_yaw: {target_yaw}")
        
            raw_delta = np.clip(math.degrees(theta_e + theta_d), -self.max_steer, self.max_steer)
            # Low-pass filter to smooth out localization noise and prevent jitter
            self.steering_angle = self.ALPHA * raw_delta + (1.0 - self.ALPHA) * self.steering_angle
 
            self.get_logger().info(f"steering angle (filtered): {self.steering_angle:.2f}")
            self.target_speed = self.target_speed
        else:
            self.steering_angle = 0.0
            self.target_speed = 0.0
            self.get_logger().warn("Warning: no trajectory found, will set steering angle to 0!!!!")

        # Rate-limit publishes to 10 Hz max to prevent servo from chasing every noisy position update
        now = self.get_clock().now()
        elapsed = (now - self.last_publish_time).nanoseconds / 1e9
        if elapsed < self.MIN_PUBLISH_INTERVAL:
            return
        self.last_publish_time = now
        self.publish_ackermann()
   

    def selected_trajectory_handler(self, msg):
        self.trajectory_in_global_frame = msg
        self.tx = []
        self.ty = []
        for tp in self.trajectory_in_global_frame.poses:
            self.tx.append(tp.position.x)
            self.ty.append(tp.position.y)

    def publish_ackermann(self):

        args1 = {"steering_angle": float(self.steering_angle),
                "steering_angle_velocity": 0.0,
                "speed": float(self.target_speed),
                "acceleration": 0.0,
                "jerk": 0.0}
        msg1 = AckermannDrive(**args1)


        args2 = {"steering_angle": float(self.steering_angle),
                "steering_angle_velocity": 0.0,
                "speed": float(self.target_speed),
                "acceleration": 0.0,
                "jerk": 0.0}
        msg2 = AckermannDrive(**args2)
        
        args3 = {"header": Header(stamp=self.get_clock().now().to_msg(),frame_id="stanley_controller"), 
                 "drive": msg1}
        msg3 = AckermannDriveStamped(**args3)

        self.get_logger().warn('Sending Angle: ' + str(self.steering_angle))
        
        self.cmd_drive_pub.publish(msg1)
        self.cmd_vis_pub.publish(msg2)
        self.cmd_vel_pub.publish(msg3)

    
    def get_front_axle_position(self,cam_pos,car_yaw):
        axle_pos = [cam_pos[0]+self.cam_fron_axle_dist*np.cos(car_yaw),cam_pos[1]+self.cam_fron_axle_dist*np.sin(car_yaw)]
        return axle_pos

    def get_closest_track_point(self,axle_pos,car_yaw):
        # self.get_logger().info(f"target position: {self.tx}, {self.ty}")
        dx = list(axle_pos[0]-np.asarray(self.tx))
        dy = list(axle_pos[1]-np.asarray(self.ty))
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(car_yaw + np.pi / 2),-np.sin(car_yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
        return target_idx, error_front_axle
    
    def cal_target_yaw(self,cls_point):
        if(cls_point==(len(self.ty)-1)):
            dy = self.ty[cls_point]-self.ty[cls_point-1]
            dx = self.tx[cls_point]-self.tx[cls_point-1]
        else:
            dy = self.ty[cls_point+1]-self.ty[cls_point]
            dx = self.tx[cls_point+1]-self.tx[cls_point]

        target_yaw_op1 = self.normalize_angle(np.arctan2(dy,dx))
        target_yaw_op2 = self.normalize_angle(target_yaw_op1 + np.pi)

        yaw_diff_1 = abs(self.circular_diff(target_yaw_op1, self.car_yaw_corrected))
        yaw_diff_2 = abs(self.circular_diff(target_yaw_op2, self.car_yaw_corrected))
        target_yaw2 = target_yaw_op1 if yaw_diff_1<yaw_diff_2 else target_yaw_op2

        dy_dx = np.gradient(self.ty, self.tx)
        # The gradient at the specified point_index
        rate = dy_dx[cls_point]
        target_yaw = np.arctan(rate)
        if(rate)<0: 
            target_yaw = math.pi+target_yaw
    
    
        return target_yaw2
    
    def circular_diff(self, a, b):
        return angle_mod(a - b)
    
    def normalize_angle(self,angle):
        return angle_mod(angle,zero_2_2pi=True)
    


def main(args=None):
    rclpy.init(args=args)
    stanley_controller = StanleyControl()
    rclpy.spin(stanley_controller)
    stanley_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


