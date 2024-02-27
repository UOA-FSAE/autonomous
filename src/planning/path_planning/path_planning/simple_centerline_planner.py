
import numpy as np 

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Pose, PoseArray
from moa_msgs.msg import ConeMap, Cone



class center_line_publisher(Node):
    def __init__(self):
        super().__init__("Center_Line_Path")
        self.get_logger().info("Center Line Path Planning Node Started")

        self.view_horizon = 2000

        self.id = 0

        # subscribe to car states and cone map
        self.best_traj_pub = self.create_publisher(PoseArray, "moa/selected_trajectory", 5)
        #self.next_destination = self.create_publisher(Pose, "moa/next_destination", 5)
        self.cone_map = self.create_subscription(ConeMap, "cone_map", self.center_line_publisher, 5)

    # CENTER LINE PUBLISHER
    def center_line_publisher(self, msg: ConeMap):
        # get position, orientation of car, and transformation matrices
        self.x, self.y, self.theta = self.get_position_of_cart(msg)
        self.position_vector, self.rotation_matrix_l2g, self.rotation_matrix_g2l = self.get_transformation_matrix((self.x,self.y), self.theta)

        # get visible boundaries
        self.get_bounds(msg)
        # get center line
        self.cntcoods = self.get_center_line()
        center_line_path = self.pack_to_pose_array(self.cntcoods)

        self.best_traj_pub.publish(center_line_path)

        self.get_logger().info("Center line published")

    def get_bounds(self, msg: ConeMap):
        id = 1
        cones = msg.cones
        # loop through each cone
        self.leftbound = []
        self.rightbound = []
        for i in range(len(cones)):
            if i != 0:
                if self.cone_is_included(cones[i]):
                    x = cones[i].pose.pose.position.x
                    y = cones[i].pose.pose.position.y
                    # blue - left
                    if cones[i].colour == 0:
                        self.leftbound.append([x, y])
                    elif cones[i].colour == 2:
                        self.rightbound.append([x, y])

    def get_center_line(self):
        # the midpoint is the average of the coordinates
        coods = []
        for i in range(min(len(self.leftbound), len(self.rightbound))):
            left_cone = self.leftbound[i]
            right_cone = self.get_opposite_cone_to_left_cone(left_cone)
            coods.append(self.get_avg_point(left_cone[0], left_cone[1], right_cone[0], right_cone[1]))

        return coods

    def get_opposite_cone_to_left_cone(self, left_cone_input):
        sorted_right_bound = sorted(self.rightbound, key = lambda right_cone_input: self.get_point_distance(right_cone_input[0], right_cone_input[1], left_cone_input[0], left_cone_input[1]))
        return sorted_right_bound[0]

    def pack_to_pose_array(self, coordinates):
        output = PoseArray()
        for cood in coordinates:
            output_pose = Pose()
            output_pose.position.x = cood[0]
            output_pose.position.y = cood[1]
            output.poses.append(output_pose)
        return output

    def get_avg_point(self, x1, y1, x2, y2):
        return (((x1 + x2)/2), ((y1 + y2)/2))

    def get_point_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_position_of_cart(self, cone_map):
        localization_data = cone_map.cones[0]
        x = localization_data.pose.pose.position.x
        y = localization_data.pose.pose.position.y
        theta = localization_data.pose.pose.orientation.w
        return x, y, theta

    def get_transformation_matrix(self, position, orientation):
        # theta = position_and_orientation[2] - np.pi/2
        cart_x = position[0]
        cart_y = position[1]
        theta = orientation

        rotation_matrix_l2g = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
        rotation_matrix_g2l = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
        position_vector = np.array([[cart_x], [cart_y]])

        return position_vector, rotation_matrix_l2g, rotation_matrix_g2l

    def get_visible_pose_array_and_destination(self, msg: PoseArray):
        if hasattr(self, "position_vector") and hasattr(self, "rotation_matrix_l2g") and hasattr(self, "rotation_matrix_g2l"):
            visible_poses = PoseArray()
            sorted_visible_poses = PoseArray()
            for individual_pose in msg.poses:
                if self.is_visible(individual_pose) and self.is_not_too_far(individual_pose):
                    visible_poses.poses.append(individual_pose)

            sorted_visible_poses.poses = sorted(visible_poses.poses, key=lambda item: self.get_distance_from_car(item))
            if len(sorted_visible_poses.poses) == 0:
                self.get_logger().info("Warning: no trajectory found!")
                next_destination = Pose()
            else:
                next_destination = sorted_visible_poses.poses[0];
            return visible_poses, next_destination

    def globaL2local(self, pose_in_global : Pose):
        pose_output = Pose()
        position_input = np.array([[pose_in_global.position.x], [pose_in_global.position.y]])
        position_output = np.matmul(self.rotation_matrix_g2l, (position_input - self.position_vector))
        pose_output.position.x = float(position_output[0])
        pose_output.position.y = float(position_output[1])
        return pose_output

    def local2global(self, pose_in_global : Pose):
        pose_output = Pose()
        position_input = np.array([[pose_in_global.position.x], [pose_in_global.position.y]])
        position_output = np.matmul(self.rotation_matrix_l2g, position_input) + self.position_vector
        pose_output.position.x = float(position_output[0])
        pose_output.position.y = float(position_output[1])
        return pose_output

    def is_visible(self, pose: Pose):
        pose_in_local = self.globaL2local(pose)
        return pose_in_local.position.y >= 0

    def is_not_too_far(self, pose: Pose):
        return self.get_distance_from_car(pose) <= self.view_horizon

    def get_distance_from_car(self, pose):
        return self.get_distance_from_origin(self.globaL2local(pose))

    def get_distance_from_origin(self, pose):
        return pose.position.x ** 2 + pose.position.y ** 2

### Cone message visibility and ranging criteria ###
    def cone_is_included(self, cone: Cone):
        # return True # If we don't want to filter out invisible cones for path planning
        return self.cone_is_visible(cone) and self.cone_is_not_too_far(cone)


    def cone_to_pose(self, cone: Cone):
        output_pose = Pose()
        output_pose.position.x = cone.pose.pose.position.x
        output_pose.position.y = cone.pose.pose.position.y
        return output_pose

    def cone_is_visible(self, cone: Cone):
        equivalent_pose_of_cone = self.cone_to_pose(cone)
        return self.is_visible(equivalent_pose_of_cone)

    def cone_is_not_too_far(self, cone: Cone):
        equivalent_pose_of_cone = self.cone_to_pose(cone)
        return self.is_not_too_far(equivalent_pose_of_cone)

    
def main():
    rclpy.init()
    exe = SingleThreadedExecutor()
    center_line_node = center_line_publisher()
    exe.add_node(center_line_node)
    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
