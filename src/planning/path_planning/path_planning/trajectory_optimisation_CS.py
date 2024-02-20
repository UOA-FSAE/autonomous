#!/usr/bin/python3
import numpy as np
from shapely import LineString
from shapely import Point as shapelyPoint
from scipy import interpolate 

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray, Point
from moa_msgs.msg import Cone, ConeMap, BoundaryStamped, AllTrajectories, AllStates
from ackermann_msgs.msg import AckermannDrive
from builtin_interfaces.msg import Time


class trajectory_optimization(Node):
    def __init__(self):
        super().__init__("trajectory_optimisation")
        self.get_logger().info("Trajectory Optimisation Node Started")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug', True)
            ]
        )

        # attributes
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value
        self._once = False

        # subscribers
        self.create_subscription(AllStates, "moa/states", self.set_states, 10)
        self.create_subscription(AllTrajectories, "moa/trajectories", self.delete_optimise_trajectories, 10)
        self.create_subscription(AckermannDrive, "moa/cur_vel", self.set_current_speed, 10)
        self.create_subscription(BoundaryStamped, "track/bound_l", self.set_left_boundary, 10)
        self.create_subscription(BoundaryStamped, "track/bound_r", self.set_right_boundary, 10)
        # only debugging
        self.create_subscription(ConeMap, "cone_map", self.set_boundaries, 10)

        # publishers
        # self.best_trajectory_publisher = self.create_publisher(AckermannDrive, "moa/selected_trajectory", 10)
        self.best_trajectory_publisher = self.create_publisher(PoseArray, "moa/selected_trajectory", 10)
        self.within_boundary_trajectories_publisher = self.create_publisher(AllTrajectories, 'moa/inbound_trajectories', 10)
        self.within_boundary_states_publisher = self.create_publisher(AllStates, "moa/inbound_states", 10)


    def set_states(self, msg: AllStates) -> None: self._state_msg = msg

    def set_current_speed(self, msg: AckermannDrive) -> None: self._current_speed = msg.speed

    def set_left_boundary(self, msg: BoundaryStamped) -> None: self._leftboundary = msg.coords

    def set_right_boundary(self, msg: BoundaryStamped) -> None: self._rightboundary = msg.coords

    def set_boundaries(self, msg: ConeMap):
        if self._debug:
            cones = msg.cones
            # loop through each cone
            self._leftboundary = []
            self._rightboundary = []
            for i in range(len(cones)):
                if i != 0:
                    x = cones[i].pose.pose.position.x
                    y = cones[i].pose.pose.position.y
                    # blue - left
                    if cones[i].colour == 0:
                        self._leftboundary.append([x,y])
                    elif cones[i].colour == 2:
                        self._rightboundary.append([x,y])
                
            # print coordinate lists
            if self._once:
                print("------------------coordinates------------------")
                print(f'xl={[i[0] for i in self._leftboundary]}')
                print(f'yl={[i[1] for i in self._leftboundary]}')
                print(f'xr={[i[0] for i in self._rightboundary]}')
                print(f'yr={[i[1] for i in self._rightboundary]}')
                print("------------------------------------------------")
                self._once = False
        

    def delete_optimise_trajectories(self, msg: AllTrajectories): 
        '''deletes trajectories then optimises to find the best path for car'''

        if self._debug: self._current_speed = 0.0
        self.get_logger().info(f"all states: {hasattr(self,'_state_msg')}"
                               f" | current speed: {hasattr(self,'_current_speed')}" \
                               f" | left boundaries: {hasattr(self,'_leftboundary')}"\
                               f" | right boundaries: {hasattr(self,'_rightboundary')}")

        if hasattr(self,"_state_msg") and hasattr(self,"_current_speed") \
            and hasattr(self,"_leftboundary") and hasattr(self,"_rightboundary"):
            # check ids
            if self._state_msg.id != msg.id:
                self.get_logger().info(f"Ids state:{self._state_msg.id} and trajectory:{msg.id} do not match")
                return

            # set trajectories and states as a list
            # states = [self._state_msg.states[i].steering_angle for i in range(len(self._state_msg.states))]
            states = [command.steering_angle for command in self._state_msg.states]
            trajectories = msg.trajectories

            self.get_logger().info(f"number of paths before deletion = {len(trajectories)}")
            self.trajectory_deletion(trajectories, states)
            self.get_logger().info(f"number of paths after deletion = {len(trajectories)}")
            best_trajectory_idx = self.optimisation(trajectories, states)
            self.get_logger().info(f"best state is = {best_trajectory_idx}")

            # publish best trajectory
            args = {"header": Header(stamp=Time(sec=0,nanosec=0), frame_id='best_trajectory'),
                    "poses": trajectories[best_trajectory_idx].poses}
            posearray_msg = PoseArray(**args)
            self.best_trajectory_publisher.publish(posearray_msg)

            # publish valid (within boundaries) trajectories including center line
            ps = [Pose(position=Point(x=shapelyPoint(P).x, y=shapelyPoint(P).y, z=0.0)) for P in self._center_line_coordinates]
            trajectories.append(PoseArray(poses=ps))
            alltrajectories_msg = {
                "id": msg.id,
                "trajectories": trajectories
            }
            self.within_boundary_trajectories_publisher.publish(AllTrajectories(**alltrajectories_msg))

            # publish within boundary trajectory states
            states_msg = []
            for sta in states:
                sargs = {"steering_angle": sta,
                        "steering_angle_velocity": 0.0,
                        "speed": self._current_speed,
                        "acceleration": 0.0,
                        "jerk": 0.0}
                states_msg.append(AckermannDrive(**sargs))
            self.within_boundary_states_publisher.publish(AllStates(id=msg.id, states=states_msg))
            
            return

        return

    # ===========================================================
    # TRAJECTORY DELETION
    def trajectory_deletion(self, trajectories, states):
        '''Caller for trajectory deletion if there are trajectories'''

        self.get_logger().info("Trajectory Deletion Started")
        self.set_within_boundary_trajectories(trajectories, states) 
        
    def set_within_boundary_trajectories(self, trajectories, states):
        '''Deletes trajectories that are on track boundaries'''
        remove_trajectories_indices = []
        
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
        self._left_boundary_linestring = LineString([(P[0], P[1]) for P in self._leftboundary])

        pts_list = [(P[0], P[1]) for P in self._rightboundary]
        if self._debug:
            pts_list.pop(0)
        self._right_boundary_linestring = LineString(pts_list)

        for i, T in enumerate(trajectories):
            trajectory = LineString([(P.position.x, P.position.y) for P in T.poses])

            # check intersection
            if trajectory.intersects(self._left_boundary_linestring) \
                or trajectory.intersects(self._right_boundary_linestring):
                remove_trajectories_indices.append(i)

        [(trajectories.pop(index), states.pop(index)) for index in list(reversed(remove_trajectories_indices))]
    
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
    def optimisation(self, trajectories, states):
        '''returns the best trajectory'''

        self.get_logger().info("Optimisation Started")
        return self.get_best_trajectory_state(trajectories, states) 

    def get_best_trajectory_state(self, trajectories, states):
        '''finds the best trajectory'''

        trajectory_distances = np.zeros(len(trajectories))

        # get track width
        # wdth = self.get_track_width()
        # get center line
        center_linestring, self._center_line_coordinates = self.get_center_line()

        for i in range(len(trajectories)):
            trajectory = LineString([(P.position.x, P.position.y) for P in trajectories[i].poses])
            trajectory_distances[i] = self.get_geometry_distance(trajectory, center_linestring)
        print(trajectory_distances);

        # check which one is close to center and largest
        # tdist = abs(tdist - (wdth / 2))

        return self.get_best_trajectory_index(states, trajectory_distances)
    
    def get_track_width(self):
        return self.rightb_line.distance(self.leftb_line)
    
    def get_center_line(self):
        '''approximates the track's center line'''
        # the midpoint is the average of the coordinates
        coods = []
        print(len(self._leftboundary));
        print(len(self._rightboundary));
        for i in range(min(len(self._leftboundary),len(self._rightboundary))):
            x1 = self._leftboundary[i][0]
            y1 = self._leftboundary[i][1]
            x2 = self._rightboundary[i][0]
            y2 = self._rightboundary[i][1]
            coods.append(self.get_avg_point(x1,y1,x2,y2))

        # perform extrapolation to extend line
        func = interpolate.interp1d([P[0] for P in coods], [P[1] for P in coods], kind='cubic', fill_value='extrapolate')

        into_future_points = 6
        into_future_distance = 0

        # track direction 
        if coods[-1][0] > coods[-2][0]:
            # positive/right
            direction = 1
        elif coods[-1][0] < coods[-2][0]:
            # negative/left
            direction = -1
        elif abs(coods[-1][0] - coods[-2][0]) <= 1e-3:
            # straight 
            into_future_distance = 0
            into_future_points = 0
        
        if into_future_points != 0:
            xnew = []
            into_future_distance += direction
            for i in range(into_future_points):
                xnew.append(coods[-1][0] + into_future_distance)
                into_future_distance += direction
            # get new pts
            ynew = func(xnew)
            # add approximated coordinates to center line
            for i in range(into_future_points):
                coods.append((xnew[i],ynew[i]))

        return LineString(coods), coods
    
    def get_avg_point(self, x1, y1, x2, y2):
        return (((x1 + x2)/2), ((y1 + y2)/2))

    def get_geometry_distance(self, geometry1:LineString, geometry2:LineString):
        '''calculates the eucledian distance bewteen two shapely geometries'''
        # this is to calculate trajectory length using only poses/points (DECAP)
        # poses = trajectory.poses
        # p0 = np.array([poses[0].position.x, poses[0].position.y])
        # p1 = np.array([poses[1].position.x, poses[1].position.y])
        # intL = np.linalg.norm(p1-p0,ord=2)
        # arc_length = (len(poses)-1) * intL
        
        return geometry1.distance(geometry2)

    def get_best_trajectory_index(self, states, trajectory_distances: np.array): return np.argmin(trajectory_distances)

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








