import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from moa_msgs.msg import ConeMap
from ackermann_msgs.msg import AckermannDrive

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
        
        #Constants
        self.k_stanley = 5.0 #stanley Controller gain
        self.k_speed = 1.0 #speed Controller gain
        self.cam_fron_axle_dist= 1 #[m] Wheel base of vehicle
        self.max_steer = np.radians(27.0)  # [rad] max steering angle
        self.target_speed = 150/3.6 #[m/s]

        #Subscribe for car pose and track
        self.create_subscription(PoseArray, "moa/selected_trajectory", self.selected_trajectory_handler, 5)
        self.create_subscription(ConeMap, "cone_map", self.main_hearback, 5)
        #Publish result
        self.cmd_vel_pub = self.create_publisher(AckermannDrive, "/drive", 5)
        self.cmd_vis_pub = self.create_publisher(AckermannDrive, "/drive_vis", 5)
        self.create_publisher(Pose, "moa/track_point", 5)
    
    def main_hearback(self, msg):
        car_pose = msg.cones[0].pose.pose
        camera_position = [car_pose.position.x,car_pose.position.y]
        car_yaw = car_pose.orientation.w

        if hasattr(self, "trajectory_in_global_frame"):
            #Get Car front axle center position
            axle_pos = self.get_front_axle_position(camera_position,car_yaw)
            #Get closest point on track and distance error
            cls_point,error_front_axle = self.get_closest_track_point(axle_pos,car_yaw)
            #Compute target yaw
            target_yaw = self.cal_target_yaw(cls_point)
            #Compute steering angle
            theta_e = self.normalize_angle(target_yaw - car_yaw)
            theta_d = np.arctan2(self.k_stanley * error_front_axle, self.target_speed)
            delta = theta_e + theta_d
            delta = np.clip(delta, -self.max_steer, self.max_steer)
            self.steering_angle = delta
        else:
            self.steering_angle = 0
            self.get_logger().info("Warning: no trajectory found, will set steering angle to 0!!!!")

        # Publish command for velocity
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
        self.cmd_vel_pub.publish(msg1)
        self.cmd_vis_pub.publish(msg2)

    
    def get_front_axle_position(self,cam_pos,car_yaw):
        axle_pos = [cam_pos[0]+self.cam_fron_axle_dist*np.cos(car_yaw),cam_pos[1]+self.cam_fron_axle_dist*np.sin(car_yaw)]
        return axle_pos

    def get_closest_track_point(self,axle_pos,car_yaw):
        dx = list(axle_pos[0]-np.asarray(self.tx))
        dy = list(axle_pos[1]-np.asarray(self.ty))
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(car_yaw + np.pi / 2),-np.sin(car_yaw + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
        return target_idx, error_front_axle
    
    def cal_target_yaw(self,cls_point):
        dy_dx = np.gradient(self.ty, self.tx)
        # The gradient at the specified point_index
        rate= dy_dx[cls_point]
        return np.arctan(rate)
    
    def normalize_angle(angle):
        return angle_mod(angle)
    


def main(args=None):
    rclpy.init(args=args)
    stanley_controller = StanleyControl()
    rclpy.spin(stanley_controller)
    stanley_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
