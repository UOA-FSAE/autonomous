#!/usr/bin/python3

import numpy as np
import time
from shapely import LineString, intersection
from shapely import Point as shapelyPoint
from scipy import interpolate 

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Pose, PoseArray, Point
from moa_msgs.msg import Cone, ConeMap, BoundaryStamped, AllTrajectories, AllStates
from ackermann_msgs.msg import AckermannDrive


class path_planning(Node):
    def __init__(self):
        super().__init__("Path_Planning")
        self.get_logger().info("Path Planning Node Started")
        
        # all trajectories publisher 
        self.all_traj_pub = self.create_publisher(AllTrajectories, "moa/trajectories", 5)
        self.all_states_pub = self.create_publisher(AllStates, "moa/states", 5)

        # subscribe to car states and cone map
        self.current_states = self.create_subscription(AckermannDrive, "moa/cur_vel", self.get_current_states, 5)
        self.cone_map_sub = self.create_subscription(ConeMap, "cone_map", self.get_cone_map, 5)

        self.id = 0
        self.timer = self.create_timer(3, self.publish_trajectories)

    # BEST TRAJECTORY PUBLISHER
    def publish_trajectories(self):
        '''
        Publishes all generated trajectories once created every n seconds
        '''
        self.get_logger().info("3 seconds up - generating trajectories")

        # if hasattr(self,"current_speed") and hasattr(self,"cone_map"):
        if hasattr(self,"cone_map"):
    
            # generate trajectories
            paths, states = self.trajectory_generator(self.cone_map)
            
            # publish states and trajectories
            state_list = []
            for i in range(len(states)):
                args = {"steering_angle": float(states[i]),
                        "steering_angle_velocity": 0.0,
                        # "speed": self.current_speed,
                        "speed": 0.0,
                        "acceleration": 0.0,
                        "jerk": 0.0}
                state_list.append(AckermannDrive(**args))
            
            msg = AllStates(id = self.id, states = state_list)
            self.all_states_pub.publish(msg)

            msg1 = AllTrajectories(id = self.id, trajectories = paths)
            self.all_traj_pub.publish(msg1)

            self.id += 1

            return
        
        self.get_logger().info("Attribute cone map and/or current speed not initialised")
        return

    def get_current_states(self, msg:AckermannDrive) -> None: self.current_speed = msg.speed
    
    def get_cone_map(self, msg:ConeMap) -> None: self.cone_map = msg

    # ===========================================================
    # TRAJECTORY GENERATION
    def single_trajectory_generator(self, steering_angle, position_vector, rotation_matrix):
        # https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357 is used for
        # bicycle steering
        L = 1;
        R = L / np.tan(steering_angle);
        t_range = np.arange(0, np.pi/2, 0.01);
        trajectory_output = PoseArray();
        for individual_t in t_range:
            pose_input = Pose();
            x_pre_trans = np.cos(individual_t) * R - R
            y_pre_trans = abs(np.sin(individual_t) * R)
            post_trans_point = self.apply_transformation(position_vector, rotation_matrix, x_pre_trans, y_pre_trans);
            pose_input.position.x = post_trans_point[0][0]
            pose_input.position.y = post_trans_point[1][0]
            trajectory_output.poses.append(pose_input)
        return trajectory_output

    def trajectory_generator(self, cone_map):
        candidate_steering_angle = np.deg2rad(np.arange(-10, 10, 0.4))
        trajectories = []
        position_and_orientation = self.get_position_of_cart(cone_map)
        position_vector, rotation_matrix = self.get_transformation_matrix(position_and_orientation)
        for steering_angle in candidate_steering_angle:
            if steering_angle == 0:
                steering_angle = 1e-6;
            added_trajectory = self.single_trajectory_generator(steering_angle, position_vector, rotation_matrix)
            # Add transformation
            trajectories.append(added_trajectory)
        return trajectories, candidate_steering_angle

    def get_position_of_cart(self, cone_map):
        localization_data = cone_map.cones[0]
        x = localization_data.pose.pose.position.x
        y = localization_data.pose.pose.position.y
        theta = localization_data.pose.pose.orientation.w
        return x, y, theta

    def get_transformation_matrix(self, position_and_orientation):
        theta = position_and_orientation[2] - np.pi/2
        cart_x = position_and_orientation[0]
        cart_y = position_and_orientation[1]

        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
        position_vector = np.array([[cart_x], [cart_y]])

        return position_vector, rotation_matrix

    def apply_transformation(self, position_vector, rotation_matrix, point_x, point_y):
        point = np.array([[point_x], [point_y]])
        transformed_point = np.matmul(rotation_matrix, point) + position_vector
        return transformed_point


class trajectory_optimization(Node):
    def __init__(self):
        super().__init__("trajectory_optimization")
        self.get_logger().info("Trajectory Optimization Node Started")

        # subscribe to all paths and states
        self.all_states_sub = self.create_subscription(AllStates, "moa/states", self.get_states, 5)
        self.all_traj_sub = self.create_subscription(AllTrajectories, "moa/trajectories", self.get_all_trajectories, 5)
        self.current_states = self.create_subscription(AckermannDrive, "moa/cur_vel", self.get_current_states, 5)

        # publish best trajectory
        self.best_traj_pub = self.create_publisher(AckermannDrive, "moa/selected_trajectory", 5)
        # publish deleted trajecotries to same topic
        self.inbound_paths = self.create_publisher(AllTrajectories, 'moa/inbound_trajectories', 5)
        self.inbound_states = self.create_publisher(AllStates, "moa/inbound_states", 5)

        # subscribe to cone map and track boundaries
        self.boundl_sub = self.create_subscription(BoundaryStamped, "track/bound_l", self.get_left_boundary, 5)
        self.boundr_sub = self.create_subscription(BoundaryStamped, "track/bound_r", self.get_right_boundary, 5)

        # ONLY TO TEST TRAJECTORY DELETION (LEAVE COMMENTED)
        self.cone_map = self.create_subscription(ConeMap, "cone_map", self.get_bounds, 1)
        self.once = True


    def get_states(self, msg: AllStates) -> None: self.state_msg = msg
    
    def get_current_states(self, msg: AckermannDrive) -> None: self.current_speed = msg.speed

    def get_left_boundary(self, msg: BoundaryStamped) -> None: self.leftbound = msg.coords
    
    def get_right_boundary(self, msg: BoundaryStamped) -> None: self.rightbound = msg.coords

    def get_bounds(self, msg: ConeMap):
        id = 1
        cones = msg.cones
        # loop through each cone
        self.leftbound = []
        self.rightbound = []
        for i in range(len(cones)):
            if i != 0:
                x = cones[i].pose.pose.position.x
                y = cones[i].pose.pose.position.y
                # blue - left
                if cones[i].colour == 0:
                    self.leftbound.append([x,y])
                elif cones[i].colour == 2:
                    self.rightbound.append([x,y])
            
        # print coordinate lists
        if self.once:
            print(f'xl={[i[0] for i in self.leftbound]}')
            print(f'yl={[i[1] for i in self.leftbound]}')
            print(f'xr={[i[0] for i in self.rightbound]}')
            print(f'yr={[i[1] for i in self.rightbound]}')
            self.once = False
        

    def get_all_trajectories(self, msg: AllTrajectories) -> None: 
        # if hasattr(self,"state_msg") and hasattr(self,"current_speed") and hasattr(self,"leftbound") \
            # and hasattr(self,"rightbound"):
        if hasattr(self,"state_msg"):

            # check ids
            if self.state_msg.id != msg.id:
                self.get_logger().info(f"Ids state:{self.state_msg.id} and trajectory:{msg.id} do not match")
                return

            # get list of paths and states
            states = [self.state_msg.states[i].steering_angle for i in range(len(self.state_msg.states))]
            trajectories = msg.trajectories

            # To test Trajectory deletion with cone map
            self.get_logger().info(f"num paths before deletion = {len(trajectories)}")

            # trajectory deletion
            self.trajectory_deletion(trajectories, states)
            self.get_logger().info(f"num paths after deletion = {len(trajectories)}")

            # optimisation
            best_state = self.optimisation(trajectories,states)
            self.get_logger().info(f"best state is = {best_state}")

            # publish to moa/selected_trajectory
            args = {"steering_angle": float(best_state),
                    "steering_angle_velocity": 0.0,
                    # "speed": self.current_speed,
                    "speed": 0.0,
                    "acceleration": 0.0,
                    "jerk": 0.0}
            ack_msg = AckermannDrive(**args)
            self.best_traj_pub.publish(ack_msg)

            # publish in bound trajectories
            # add center line
            ps = [Pose(position=Point(x=shapelyPoint(P).x, y=shapelyPoint(P).y, z=0.0)) for P in self.cntcoods]
            trajectories.append(PoseArray(poses=ps))
            self.inbound_paths.publish(AllTrajectories(id=msg.id, trajectories=trajectories))
            states_msg = []
            for sta in states:
                sargs = {"steering_angle": sta,
                        "steering_angle_velocity": 0.0,
                        # "speed": self.current_speed,
                        "speed": 0.0,
                        "acceleration": 0.0,
                        "jerk": 0.0}
                states_msg.append(AckermannDrive(**sargs))
            self.inbound_states.publish(AllStates(id=msg.id, states=states_msg))
            
            return
        
        self.get_logger().info("Attribute states, current speed, left and right bounds not initialised")
        return

    # ===========================================================
    # TRAJECTORY DELETION
    def trajectory_deletion(self, trajectories=None, states=None) -> None:
        '''
        Caller for trajectory deletion if there are trajectories

        Inputs
            trajectories (1darray): a list of PoseArrays 
        '''
        if trajectories is None:
            return 
        self.get_logger().info("Trajectory Deletion Started")
        self.get_inbound_trajectories(trajectories,states) 
        
    def get_inbound_trajectories(self, trajectories, states):
        '''
        Deletes trajectories that are on track boundaries

        Inputs
            trajectories (1darray): a list of PoseArrays 
        '''
        rm_inds = []
        
        # if shapely code fails use this below (not 100%)
        # for i, T in enumerate(trajectories):
        #     if i == 25:
        #         print("stop here")
        #     for cpose in T.poses:
        #         x = cpose.position.x 
        #         y = cpose.position.y
        #         # compare points with boundaries
        #         onBound = self.compare_with_boundary(x,y,2)
        #         if onBound:  
        #             rm_inds.append(i)
        #             # trajectories.pop([i for i in range(len(trajectories)) if T==trajectories[i]][0])
        #             break

        # shapely library use
        # line strings for left and right boundaries
        self.leftb_line = LineString([(P[0], P[1]) for P in self.leftbound])
        pts_list = [(P[0], P[1]) for P in self.rightbound]
        # IF DEBUGGING TRAJECTORY DELETION USING FAKE DATA
        pts_list.pop(0)
        self.rightb_line = LineString(pts_list)
        for i, T in enumerate(trajectories):
            trajectory = LineString([(P.position.x,P.position.y) for P in T.poses])

            # check path intersects boundaries
            if trajectory.intersects(self.leftb_line) or trajectory.intersects(self.rightb_line):
                rm_inds.append(i)

        [(trajectories.pop(index), states.pop(index)) for index in list(reversed(rm_inds))]
    
    def compare_with_boundary(self, x, y, tol):
        '''
        Compares a coordinate with the left and right boundary coordinates of the track map

        inputs
            x (float): x position of coordinate
            y (float): y position of coordinate
        return
            (boolean): True if point on either boundary
        
        * Assumes same number of points are given for left and right boundary
        '''
        lxyt = []
        rxyt = []
        for i in range(len(self.rightbound)):
            blx, bly = self.leftbound[i]
            brx, bry = self.rightbound[i]
            # on left boundary
            if i != 0:
                # lxyt = abs(x-blx) <= 1e-1 and abs(y-bly) <= 1e-1
                lxyt.append(np.sqrt(((x-blx)**2 + (y-bly)**2)))
            else:
                lxyt.append(np.inf)
            # on right boundary
            # rxyt = abs(x-brx) <= 1e-1 and abs(y-bry) <= 1e-1
            rxyt.append(np.sqrt(((x-brx)**2 + (y-bry)**2)))
        # check if near boundary
        if min(lxyt) <= tol or min(rxyt) <= tol:
            return True
        else:
            return False

    # ===========================================================
    # OPTIMISATION
    def optimisation(self, trajectories=None, states=None):
        if trajectories is None:
            return
        self.get_logger().info("Optimisation Started")
        best_state = self.get_best_trajectory_state(trajectories, states)

        return best_state

    def get_best_trajectory_state(self, trajectories, states):
        tdist = np.zeros(len(trajectories))
        tlen = np.zeros(len(trajectories))
        # get track width
        # wdth = self.get_track_width()
        # get center line
        cntLine, self.cntcoods = self.get_center_line()
        for i in range(len(trajectories)):
            plist = trajectories[i].poses
            # tlens[i] = self.get_trajectory_length(trajectories[i])
            trajectory = LineString([(P.position.x,P.position.y) for P in trajectories[i].poses])
            # endpt = Point([plist[len(plist)//2].position.x, plist[len(plist)//2].position.y])
            tdist[i] = self.get_geometry_distance(trajectory, cntLine)
        # check which one is close to center and largest
        # tdist = abs(tdist - (wdth / 2))

        state = self.get_trajectory_state(states, tdist)

        return state
    
    def get_track_width(self):
        return self.rightb_line.distance(self.leftb_line)
    
    def get_center_line(self):
        # the midpoint is the average of the coordinates
        coods = []
        for i in range(min(len(self.leftbound),len(self.rightbound))):
            x1 = self.leftbound[i][0]
            y1 = self.leftbound[i][1]
            x2 = self.rightbound[i][0]
            y2 = self.rightbound[i][1]
            coods.append(self.get_avg_point(x1,y1,x2,y2))

        # perform extrapolation to extend line
        f = interpolate.interp1d([i[0] for i in coods],[i[1] for i in coods], kind='cubic', fill_value='extrapolate')
        # check which direction 
        into_future_pts = 6
        into_future_dist = 0
        if coods[-1][0] > coods[-2][0]:
            # positive/right
            update = 1
        elif coods[-1][0] < coods[-2][0]:
            # negative/left
            update = -1
        elif abs(coods[-1][0] - coods[-2][0]) <= 1e-3:
            # straight 
            into_future_dist = 0
            into_future_pts = 0
        
        if into_future_pts != 0:
            xnew = []
            into_future_dist += update
            for i in range(into_future_pts):
                xnew.append(coods[-1][0] + into_future_dist)
                into_future_dist += update
        
            # get new pts
            ynew = f(xnew)
            # add to coods
            for i in range(into_future_pts):
                coods.append((xnew[i],ynew[i]))

        return LineString(coods), coods
    
    def get_avg_point(self, x1, y1, x2, y2):
        return (((x1 + x2)/2), ((y1 + y2)/2))

    def get_geometry_distance(self, geometry, cntLine):
        # this is to calculate trajectory length (DECAP)
        # poses = trajectory.poses
        # p0 = np.array([poses[0].position.x, poses[0].position.y])
        # p1 = np.array([poses[1].position.x, poses[1].position.y])
        # intL = np.linalg.norm(p1-p0,ord=2)
        # arc_length = (len(poses)-1) * intL
        
        return geometry.distance(cntLine)

    def get_trajectory_state(self, states, tdist: np.array):
        # to get second best index
        # mv = min(np.setdiff1d(tdist,min(tdist)))
        # idx = [i for i, t in enumerate(tdist) if t == mv][0]

        idx = np.argmin(tdist)
        return states[idx]

    def DEBUG_generate_trajectories(self, n):
        # list of all trajectories and states (for now just steering angle in rads)
        all_traj = []
        all_states = -np.random.random(n) + np.random.random(n)
        # make n pose arrays
        x = y = z = 0.0
        for i in range(n):
            # contains poses for ith pose array
            temp = []
            # make m poses with random coordinates
            for j in range(3):
                pose = Pose()
                pose.position.x, pose.position.y = x, y
                # pose_array.poses = pose
                temp.append(pose)
                # calculate new x, y, z sqrt((x2-x1)^2+(y2-y1)^2) = length with x2 unknown
                x = np.sqrt(0.1**2) + x
            pose_array = PoseArray()
            pose_array.poses = temp
            # append pose array to all trajectories
            all_traj.append(pose_array)

        return all_traj, all_states


class trajectory_following(Node):
    def __init__(self):
        super().__init__("Trajectory_Following")
        self.get_logger().info("Trajectory Following Node Started")

        # publish p-controlled trajectory
        self.p_controlled_pub = self.create_publisher(AckermannDrive, "cmd_vel", 5)
        # subscribe to best trajectory
        self.best_traj_sub = self.create_subscription(AckermannDrive, "moa/selected_trajectory", self.get_best_state, 5)
        self.current_states_sub = self.create_subscription(AckermannDrive, "moa/cur_vel", self.get_current_states, 5)

    def get_current_states(self, msg: AckermannDrive) -> None: 
        self.current_speed = msg.speed
        self.current_angle = msg.steering_angle

    def get_control_error(self, csa, dsa): return dsa-csa

    def get_best_state(self, msg: AckermannDrive):
        self.get_logger().info(f"chosen recieved state is = {msg.steering_angle}")

        # COMMENT THIS WHEN MERGING INTO NIGHTLY
        self.current_angle = self.current_speed = 0.0

        if hasattr(self,"current_speed") and hasattr(self,"current_angle"):

            error = self.get_control_error(self.current_angle, msg.steering_angle)
            # constant gain multiplier
            p_gain = 0.5

            # new steering angle output
            chosen_state = self.current_angle + error * p_gain

            # publish msg to /moa/drive
            args = {"steering_angle": float(chosen_state),
                    "steering_angle_velocity": 0.0,
                    "speed": self.current_speed,
                    "acceleration": 0.0,
                    "jerk": 0.0}
            
            msg = AckermannDrive(**args)
            self.p_controlled_pub.publish(msg)
            self.get_logger().info(f"P-controlled state published = {chosen_state}")

            return
        
        self.get_logger().info("Attributes current speed and angle not initialised")
        return


def main():
    rclpy.init()
    exe = SingleThreadedExecutor()

    # path planning
    nd1 = path_planning()
    # trajectory optimisation
    nd2 = trajectory_optimization()
    # trajectory following
    nd3 = trajectory_following()

    exe.add_node(nd1)
    exe.add_node(nd2)
    exe.add_node(nd3)

    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()








