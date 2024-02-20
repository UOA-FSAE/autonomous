#!/usr/bin/python3
import numpy as np
from shapely import LineString
from shapely import Point as shapelyPoint
from scipy import interpolate 

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Pose, PoseArray, Point
from moa_msgs.msg import Cone, ConeMap, BoundaryStamped, AllTrajectories, AllStates
from ackermann_msgs.msg import AckermannDrive


class trajectory_optimization(Node):
    def __init__(self):
        super().__init__("trajectory_optimization")
        self.get_logger().info("Trajectory Optimization Node Started")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug', True)
            ]
        )

        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        # subscribe to all paths and states
        self.all_states_sub = self.create_subscription(AllStates, "moa/states", self.get_states, 5)
        self.all_traj_sub = self.create_subscription(AllTrajectories, "moa/trajectories", self.get_all_trajectories, 5)
        self.current_states = self.create_subscription(AckermannDrive, "moa/cur_vel", self.get_current_states, 5)

        # publish best trajectory
        self.best_traj_pub = self.create_publisher(AckermannDrive, "moa/selected_trajectory", 5)
        # self.best_traj_pub = self.create_publisher(PoseArray, "moa/selected_trajectory", 5)
        # publish non-deleted trajecotries
        self.inbound_paths = self.create_publisher(AllTrajectories, 'moa/inbound_trajectories', 5)
        self.inbound_states = self.create_publisher(AllStates, "moa/inbound_states", 5)

        # subscribe to track boundaries
        self.boundl_sub = self.create_subscription(BoundaryStamped, "track/bound_l", self.get_left_boundary, 5)
        self.boundr_sub = self.create_subscription(BoundaryStamped, "track/bound_r", self.get_right_boundary, 5)

        if self.debug:
            self.get_logger().info("Debug mode on")
            self.cone_map = self.create_subscription(ConeMap, "cone_map", self.get_bounds, 5)
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
        if self.debug:
            self.current_speed = 0.0

        if hasattr(self,"state_msg") and hasattr(self,"current_speed") and hasattr(self,"leftbound") \
            and hasattr(self,"rightbound"):
            # check ids
            if self.state_msg.id != msg.id:
                self.get_logger().info(f"Ids state:{self.state_msg.id} and trajectory:{msg.id} do not match")
                return

            # get list of paths and states
            states = [self.state_msg.states[i].steering_angle for i in range(len(self.state_msg.states))]
            trajectories = msg.trajectories

            self.get_logger().info(f"num paths before deletion = {len(trajectories)}")
            # trajectory deletion
            self.trajectory_deletion(trajectories, states)
            self.get_logger().info(f"num paths after deletion = {len(trajectories)}")

            # optimisation
            best_state = self.optimisation(trajectories, states)
            self.get_logger().info(f"best state is = {best_state}")

            # publish to moa/selected_trajectory
            args = {"steering_angle": float(best_state),
                    "steering_angle_velocity": 0.0,
                    "speed": self.current_speed,
                    "acceleration": 0.0,
                    "jerk": 0.0}
            ack_msg = AckermannDrive(**args)
            self.best_traj_pub.publish(ack_msg)

            # publish in-bound trajectories
            # add center line
            ps = [Pose(position=Point(x=shapelyPoint(P).x, y=shapelyPoint(P).y, z=0.0)) for P in self.cntcoods]
            trajectories.append(PoseArray(poses=ps))
            self.inbound_paths.publish(AllTrajectories(id=msg.id, trajectories=trajectories))

            # publish in-bound states
            states_msg = []
            for sta in states:
                sargs = {"steering_angle": sta,
                        "steering_angle_velocity": 0.0,
                        "speed": self.current_speed,
                        "acceleration": 0.0,
                        "jerk": 0.0}
                states_msg.append(AckermannDrive(**sargs))
            self.inbound_states.publish(AllStates(id=msg.id, states=states_msg))
            
            return
        
        self.get_logger().info("Attributes states/current speed/left and right bounds not initialised")
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

        # shapely library 
        # line strings for left and right boundaries
        self.leftb_line = LineString([(P[0], P[1]) for P in self.leftbound])

        pts_list = [(P[0], P[1]) for P in self.rightbound]
        if self.debug:
            pts_list.pop(0)
        self.rightb_line = LineString(pts_list)

        for i, T in enumerate(trajectories):
            trajectory = LineString([(P.position.x, P.position.y) for P in T.poses])

            # check path boundary intersection
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
        # trajectory info
        tdist = np.zeros(len(trajectories))
        tlen = np.zeros(len(trajectories))

        # get track width
        # wdth = self.get_track_width()
        # get center line
        cntLine, self.cntcoods = self.get_center_line()

        for i in range(len(trajectories)):
            # tlens[i] = self.get_trajectory_length(trajectories[i])
            trajectory = LineString([(P.position.x, P.position.y) for P in trajectories[i].poses])
            tdist[i] = self.get_geometry_distance(trajectory, cntLine)
        print(tdist);

        # check which one is close to center and largest
        # tdist = abs(tdist - (wdth / 2))

        state = self.get_trajectory_state(states, tdist)

        return state
    
    def get_track_width(self):
        return self.rightb_line.distance(self.leftb_line)
    
    def get_center_line(self):
        # the midpoint is the average of the coordinates
        coods = []
        print(len(self.leftbound));
        print(len(self.rightbound));
        for i in range(min(len(self.leftbound),len(self.rightbound))):
            x1 = self.leftbound[i][0]
            y1 = self.leftbound[i][1]
            x2 = self.rightbound[i][0]
            y2 = self.rightbound[i][1]
            coods.append(self.get_avg_point(x1,y1,x2,y2))

        # perform extrapolation to extend line
        f = interpolate.interp1d([i[0] for i in coods], [i[1] for i in coods], kind='cubic', fill_value='extrapolate')

        into_future_pts = 6
        into_future_dist = 0

        # trajectory deletion
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

    def get_trajectory_state(self, states, tdist: np.array): return states[np.argmin(tdist)]

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

def main():
    rclpy.init()
    exe = SingleThreadedExecutor()
    node = trajectory_optimization()
    exe.add_node(node)
    exe.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()








