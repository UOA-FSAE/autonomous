#!/usr/bin/python3
import numpy as np
from shapely import LineString, MultiPoint
from shapely import Point as shapelyPoint
from scipy import interpolate 

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Header, Float32, Int16, Int32MultiArray
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
        self._once = True

        # subscribers
        self.create_subscription(AllStates, "moa/states", self.set_states, 10)
        self.create_subscription(AllTrajectories, "moa/trajectories", self.delete_optimise_trajectories, 10)
        self.create_subscription(AckermannDrive, "moa/cur_vel", self.set_current_speed, 10)
        self.create_subscription(BoundaryStamped, "track/bound_l", self.set_left_boundary, 10)
        self.create_subscription(BoundaryStamped, "track/bound_r", self.set_right_boundary, 10)
        # only debugging
        self.create_subscription(ConeMap, "cone_map", self.set_boundaries, 10)
        self.best_steering_angle_pub = self.create_publisher(Float32, "moa/selected_steering_angle", 10)

        # publishers
        # self.best_trajectory_publisher = self.create_publisher(AckermannDrive, "moa/selected_trajectory", 10)
        self.best_trajectory_publisher = self.create_publisher(PoseArray, "moa/selected_trajectory", 10)
        self.within_boundary_trajectories_publisher = self.create_publisher(AllTrajectories, 'moa/inbound_trajectories', 10)
        self.best_trajectory_index = self.create_publisher(Int16, "moa/best_trajectory_index", 10)
        self.out_of_bounds_indicies = self.create_publisher(Int32MultiArray, "moa/out_of_bounds", 10)


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
                with open('/home/tanish/autonomous/src/planning/path_planning/path_planning/bound_coods', 'w') as fh:
                        xl=[i[0] for i in self._leftboundary]
                        yl=[i[1] for i in self._leftboundary]
                        xr=[i[0] for i in self._rightboundary]
                        yr=[i[1] for i in self._rightboundary]
                        for P in xl:
                            fh.write("{} ".format(P))
                        fh.write("\n")
                        for P in yl:
                            fh.write("{} ".format(P))
                        fh.write("\n")
                        for P in xr:
                            fh.write("{} ".format(P))
                        fh.write("\n")
                        for P in yr:
                            fh.write("{} ".format(P))
                        fh.write("\n")
                        fh.close()
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
                # return

            # set trajectories and states as a list
            # states = [self._state_msg.states[i].steering_angle for i in range(len(self._state_msg.states))]
            states = [command.steering_angle for command in self._state_msg.states]
            trajectories = msg.trajectories

            self.get_logger().info(f"number of paths before deletion = {len(trajectories)}")
            self.trajectory_deletion(trajectories, states)
            self.get_logger().info(f"number of paths after deletion = {len(trajectories)}")
            best_trajectory_idx = self.optimisation(trajectories, states)

            # check for no trajectories
            if best_trajectory_idx == None:
                self.get_logger().info("no valid trajectories found")
                return
            self.get_logger().info(f"best state is = {states[best_trajectory_idx]}")

            # publish best trajectory
            args = {"header": Header(stamp=Time(sec=0,nanosec=0), frame_id='best_trajectory'),
                    "poses": trajectories[best_trajectory_idx].poses}
            posearray_msg = PoseArray(**args)
            self.best_trajectory_publisher.publish(posearray_msg)

            # publish valid (within boundaries) trajectories including center line
            ps = [Pose(position=Point(x=P[0], y=P[1], z=0.0)) for P in self._center_line_coordinates]
            trajectories.append(PoseArray(poses=ps))
            alltrajectories_msg = {
                "id": msg.id,
                "trajectories": trajectories
            }
            self.within_boundary_trajectories_publisher.publish(AllTrajectories(**alltrajectories_msg))

            # publish within boundary trajectory states
            # states_msg = []
            # for sta in states:
            #     sargs = {"steering_angle": sta,
            #             "steering_angle_velocity": 0.0,
            #             "speed": self._current_speed,
            #             "acceleration": 0.0,
            #             "jerk": 0.0}
            #     states_msg.append(AckermannDrive(**sargs))
            # self.within_boundary_states_publisher.publish(AllStates(id=msg.id, states=states_msg))

            # publish best steering angle
            self.best_steering_angle_pub.publish(Float32(data=states[best_trajectory_idx]))

            self.get_logger().info("msg published")
            
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

        # track width 
        track_width = self.get_track_width()

        for i in range(len(trajectories)):
            # trajectory line string
            trajectory = LineString([(P.position.x, P.position.y) for P in trajectories[i].poses])
            trajectory_length = trajectory.length

            if i == len(trajectories)/2 - 1:
                print('the long one')

            # check intersection
            if trajectory.intersects(self._left_boundary_linestring):
                tmp = trajectory.intersection(self._left_boundary_linestring)
                idx = 0
                if type(tmp) is MultiPoint:
                    tmp_x = tmp.centroid.x
                else:
                    tmp_x = tmp.x
                for j, P in enumerate(trajectories[i].poses):
                    if tmp_x < P.position.x:
                        idx += 1
                    else:
                        if idx < 2:
                            # idx += 1
                            remove_trajectories_indices.append(i)
                        break
                if idx != 0:
                    trajectories[i].poses = trajectories[i].poses[:idx]

            elif trajectory.intersects(self._right_boundary_linestring):
                tmp = trajectory.intersection(self._right_boundary_linestring)
                idx = 0
                if type(tmp) is MultiPoint:
                    tmp_x = tmp.centroid.x
                else:
                        tmp_x = tmp.x
                for j, P in enumerate(trajectories[i].poses):
                    if tmp_x > P.position.x:
                        idx += 1
                    else:
                        if idx < 2:
                            # idx += 1
                            remove_trajectories_indices.append(i)
                        break
                if idx != 0:
                    trajectories[i].poses = trajectories[i].poses[:idx]
        
        # publish indicies
        self.out_of_bounds_indicies.publish(Int32MultiArray(data=remove_trajectories_indices))

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

        trajectory_distances = np.ones(len(trajectories)) * np.inf
        trajectory_lengths = np.zeros(len(trajectories))

        # get track width
        width = self.get_track_width()
        # get center line
        center_linestring, self._center_line_coordinates = self.get_center_line()

        # decide average from centerline to trajectory or vice versa
        to_center_line = False

        for i in range(len(trajectories)):
            # trajectory linestring 
            if len(trajectories[i].poses) > 1:
                trajectory = self.get_shapely_linestring(trajectories[i].poses)
                trajectory_length = trajectory.length
            else:
                trajectory_length = np.inf    

            if len(trajectories[i].poses) < 2:
                to_center_line = True

            if to_center_line:
                arg = trajectories[i]
                arg2 = center_linestring
            else:
                arg = trajectory
                arg2 = self._center_line_coordinates

            # average trajectory distance from center line
            average_distance = self.get_average_distance_to_center_trajectory(arg, arg2, to_center_line)
            # average_distance = trajectory.distance(center_linestring)

            # if trajectory.length < width/2:
            #     average_distance = np.inf

            trajectory_distances[i] = average_distance
            trajectory_lengths[i] = trajectory_length

        return self.get_best_trajectory_index(trajectory_distances, trajectory_lengths, states)
    
    def get_average_distance_to_center_trajectory(self, trajectory, center_linestring, toCenterLine=True):
        # if distance to centerline (from trajecotry) or other way round
        if toCenterLine:
            num_points = len(trajectory.poses)
        else:
            num_points = len(center_linestring)

        total_distance = np.zeros(num_points)

        for j in range(num_points):
            if toCenterLine:
                ps = trajectory.poses[j]
                point = self.get_shapely_point(ps.position.x, ps.position.y)
                total_distance[j] = point.distance(center_linestring)
            else:
                ps = center_linestring[j]
                point = self.get_shapely_point(ps[0], ps[1])
                # TRAJECTORY MUST BE A LINESTRING NOT A POSEARRAY
                total_distance[j] = point.distance(trajectory)
        
        return np.mean(total_distance)
    
    def get_shapely_point(self, x, y) -> shapelyPoint:
        return shapelyPoint(x,y)
    
    def get_shapely_linestring(self, poses) -> LineString:
        return LineString([(P.position.x, P.position.y) for P in poses])

    def get_track_width(self):
        return self._right_boundary_linestring.distance(self._left_boundary_linestring)
    
    def get_center_line(self):
        '''approximates the track's center line'''
        # the midpoint is the average of the coordinates
        coods = []
        for i in range(min(len(self._leftboundary),len(self._rightboundary))):
            x1 = self._leftboundary[i][0]
            y1 = self._leftboundary[i][1]
            x2 = self._rightboundary[i][0]
            y2 = self._rightboundary[i][1]
            coods.append(self.get_avg_point(x1,y1,x2,y2))

        # perform extrapolation to extend line
        func = interpolate.interp1d([P[0] for P in coods], [P[1] for P in coods], kind='cubic', fill_value='extrapolate')

        into_future_points = 0
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

    def get_geometry_distance(self, geometry1, geometry2):
        '''calculates the eucledian distance bewteen two shapely geometries'''
        # this is to calculate trajectory length using only poses/points (DECAP)
        # poses = trajectory.poses
        # p0 = np.array([poses[0].position.x, poses[0].position.y])
        # p1 = np.array([poses[1].position.x, poses[1].position.y])
        # intL = np.linalg.norm(p1-p0,ord=2)
        # arc_length = (len(poses)-1) * intL
        
        return geometry1.distance(geometry2)
        # return frechet_distance(geometry1, geometry2)

    def get_best_trajectory_index(self, trajectory_distances: np.array, trajectory_lengths: np.array, steering_angles): 
        try:
            objective_function = trajectory_distances + np.abs(steering_angles) + (trajectory_lengths/1.5)
            # ratios = trajectory_distances/trajectory_lengths
            idx = int(np.argmin(abs(objective_function)))
            # sorted_indices = np.argsort(ratios)
            # idx = int(sorted_indices[1])
            print(f"idx,trajectory_distance,steering_angle,score = {idx},{trajectory_distances[idx]},{steering_angles[idx]},{abs(objective_function)[idx]}")
            # print(f"dist: {trajectory_distances[idx]}")
            self.best_trajectory_index.publish(Int16(data=idx))
            return idx
        except ValueError:
            return None

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








