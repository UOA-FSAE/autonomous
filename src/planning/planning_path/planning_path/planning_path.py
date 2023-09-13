#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose, PoseArray

# class stationary_velocities(Node):
#     # parameter functions
#     def Ffy(self, Df, Cf, Bf, vy, vx, w, delta, lf):
#         # calculate angle alpha and force
#         alphaf = -np.arctan( ((w * lf) + vy) / vx ) + delta
#         ffy = Df * np.sin( Cf * np.arctan(Bf * alphaf) )
#         return ffy
#
#     def Fry(self, Dr, Cr, Br, vy, vx, w, lr):
#         # calculate angle alpha and force
#         alphar = np.arctan( ((w * lr) - vy) / vx )
#         fry = Dr * np.sin( Cr * np.arctan(Br * alphar) )
#         return fry
#
#     def differential(self, t, y, p):
#         '''
#         Defines the differential equation for the dynamic car model
#
#         Arguments:
#             t: time
#             y: vector/array of state variables:
#                 y = [vy,w]
#             p: vector/array of parameters for the derivative:
#                 p = []
#         '''
#         # unpack state function values
#         vy, w = y
#         # unpack parameters
#         vx, delta, mass, inertia, lf, lr = p
#
#         # calculate the value for the system of non-linear ODEs: f = (vy',w')
#         Df = Cf = Bf = Dr = Cr = Br = 1
#         ffy = self.Ffy(Df, Cf, Bf, vy, vx, w, delta, lf)
#         fry = self.Fry(Dr, Cr, Br, vy, vx, w, lr)
#         f = [( fry + (ffy * np.cos(delta)) - (mass * vx * w) ) / mass,
#              ( (ffy * lf * np.cos(delta)) - (fry * lr) ) / inertia]
#
#         return f
#
#     # get current car state using ackermen msg
#     def subscribe_ackermann(self):
#         self.ackermann_subscriber = self.create_subscription(AckermannDrive, "/cmd_vel", self.get_initials, 1)
#
#     def get_initials(self, msg:AckermannDrive):
#         # get required fields
#         self.stag = msg.steering_angle
#         self.stag_vel = msg.steering_angle_velocity
#         self.speed = msg.speed
#
#         # compute lateral velocity
#         vy = self.speed * np.sin(self.stag)
#
#         return [vy, self.stag_vel]
#
#     def get_cog(self, bw,bw2,l2):
#         '''
#         Calculates the coordinates of the center of mass of the bicycle model
#
#         Arguments:
#             bw: front/back axle weight
#             bw2: middle axle weight
#             l2: middle axle distance (in meters)
#
#         returns:
#             xcg: distance from back to center of mass
#         '''
#         xcg = (((bw2*l2)/2) + ((22+bw)*l2)) / ((62.5+bw)+(22+bw)+bw2)
#         return xcg
#
#     def __init__(self):
#         super().__init__("stationary_velocities")
#
#         # create Vx uniform range
#         self.Vx = np.arange(0.5, 3.5, 0.25)
#         # crate Vx drifing uniform range
#         self.Vxd = np.linspace(1.5, 2.25, len(Vx), endpoint=True)
#         # create a uniform set of steering angles
#         self.stag = np.linspace(-0.3, 0.3, num=20, endpoint=True)
#         # combine the uniform and drifting velocities
#         self.vxf = np.unique(np.sort(np.append(Vx,Vxd)))
#
#         # Numerically Integrate Odes
#         # initial and final time (in seconds)
#         self.t0 = 0
#         self.tf = 1
#         # get and pack initial conditions
#         self.yinit = self.subscribe_ackermann() # initial y = (vy0, w0)
#
#         # additional parameters for derivative function
#         self.mass = 84.5 # car mass (in kgs)
#         # calculate the distances from CoG to right/left wheel center
#         self.bw = self.bw2 = 1
#         self.l2 = 1.125
#         self.xcg = self.get_cog(self.bw,self.bw2,self.l2)
#         self.lf = self.l2-self.xcg # distance from CoG to front tire
#         self.lr = -self.xcg # distance from CoG to rear tire
#         print(self.xcg,self.lr,self.lf)
#         # The moment of inertia about the z axis on the CoG is the sum of the moment of inertia of all subcomps.
#         # so Iz = 2*Iwheel + Iframe  | for simplicity assume each component is a solid cylinder
#         # thus the formula for a thin frame is ml^2/12 and for a solid cylinder about z is mr^2/4 + ml^2/12
#         self.m_frame = 1
#         self.m_wheel = 1
#         self.r_wheel = 0.1425
#         self.l_wheel = 0.11 # along the y axis
#         self.l_frame = 0.8 # along the x axis
#         self.Iw = (self.m_wheel*self.r_wheel**2)/4 + (self.m_wheel*self.l_wheel**2)/12
#         self.Ib = (self.m_frame*self.l_frame**2)/12
#         self.inertia = 2*self.Iw + self.Ib # inertia of car?
#         print(self.inertia)
#
#         # integrate for lateral and angular velocity for each forward velocity and steering angle
#         self.t, self.y = [],[]
#         for vx in self.vxf:
#             for i, delta in enumerate(self.stag):
#                 # pack up new parameters
#                 p = [vx, delta, self.mass, self.inertia, self.lf, self.lr]
#                 # TODO - figure out what iteration value you should use - should all be same assuming constant velocity?
#                 self.sol = solve_ivp(self.differential, (self.t0,self.tf), self.yinit, args=(p,))
#                 self.t.append(self.sol.t)
#                 self.y.append(self.sol.y)
#                 print(f"for {vx}m/s and {delta}rads lat was {self.y[i][0][-1]}m/s and omega was {self.y[i][1][-1]}rads/s")
#
#         # plot steering angle vs lateral velocity
#         self.temp = []
#         self.fig = plt.figure()
#         for i in range(len(self.stag)):
#             self.temp.append(y[i][0][-1])
#         plt.plot(self.stag,self.temp,"-ok")
#         plt.show()
#         plt.savefig("src/planning/planning_path/planning_path/plot output")

class path_planning(Node):
    def __int__(self):
        super().__int__("path_planning")
        self.optimisation()

    # ===========================================================
    # TRAJECTORY GENERATION

    # ===========================================================
    # TRAJECTORY DELETION

    # ===========================================================
    # OPTIMISATION
    def optimisation(self):
        # get trajectories
        all_traj, all_states = self.generate_trajectories(5)
        # get best trajectory states
        opt_traj = self.get_best_trajectory_states(all_traj, all_states)

        return opt_traj

    def get_best_trajectory_states(self, trajectories, trajectories_states):
        # create an array of all trajectory lengths
        tlens = np.zeros(len(trajectories))
        # loop through each trajectory
        for i in range(len(trajectories)):
            # get length of trajectory i
            tlens[i] = self.get_trajectory_length(trajectories[i])
        # get state of largest trajectory
        states = self.get_trajectory_state(trajectories_states,np.argmax(tlens))

        return states

    def get_trajectory_length(self, trajectory: PoseArray):
        # get radius of arc using any point
        pt = trajectory[0]
        rad = self.get_arc_radius(pt)
        # get angle of arc
        pt2 = trajectory[-1]
        ang = self.get_arc_angle(pt,pt2,rad)
        # calculate trajectory length using arc formula
        arc_length = (2*np.pi*rad*ang)/360

        return arc_length

    def get_arc_radius(self,p1):
        # radius of an arc can be determined using circle parametric equation
        # x^2 + Y^2 = R^2
        x = p1.position.x
        y = p1.position.y
        rad = np.sqrt((x**2 + y**2))

        return rad

    def get_arc_angle(self,p1,p2,radius):
        # calculate arc angle using derived equilateral triangle formula
        div = (0.5 * np.linalg.norm((p2-p1),ord=2)) / radius
        ang = 2 * np.sin(div)

        return ang

    def get_trajectory_state(self,trajectories_states,max_length):
        return trajectories_states[max_length]

    # FOR DEBUGGING
    def generate_trajectories(self, n):
        # list of all trajectories and states (for now just steering angle in rads)
        all_traj = []
        all_states = np.random.random(n) + np.random.random(n)
        # make n pose arrays
        for i in range(n):
            pose_array = PoseArray()
            # make m poses with random coordinates
            for j in range(3):
                pose = Pose()
                pose.position.x = pose.position.y = pose.position.z = i+j
                pose_array[j] = pose
            # append pose array to all trajectories
            all_traj.append(pose_array)

        return all_traj, all_states

def main():
    rclpy.init()
    node = path_planning()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt as e:
        print(f"node spin error: {e}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()








