#!/usr/bin/python3
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive

# parameter functions
def Ffy(Df, Cf, Bf, vy, vx, w, delta, lf):
    # calculate angle alpha and force
    alphaf = -np.arctan( ((w * lf) + vy) / vx ) + delta
    ffy = Df * np.sin( Cf * np.arctan(Bf * alphaf) )
    return ffy

def Fry(Dr, Cr, Br, vy, vx, w, lr):
    # calculate angle alpha and force
    alphar = np.arctan( ((w * lr) - vy) / vx )
    fry = Dr * np.sin( Cr * np.arctan(Br * alphar) )
    return fry

def differential(t, y, p):
    '''
    Defines the differential equation for the dynamic car model

    Arguments:
        t: time
        y: vector/array of state variables:
            y = [vy,w]
        p: vector/array of parameters for the derivative:
            p = []
    '''
    # unpack state function values
    vy, w = y
    # unpack parameters
    vx, delta, mass, inertia, lf, lr = p

    # calculate the value for the system of non-linear ODEs: f = (vy',w')
    Df = Cf = Bf = Dr = Cr = Br = 1
    ffy = Ffy(Df, Cf, Bf, vy, vx, w, delta, lf)
    fry = Fry(Dr, Cr, Br, vy, vx, w, lr)
    f = [( fry + (ffy * np.cos(delta)) - (mass * vx * w) ) / mass,
         ( (ffy * lf * np.cos(delta)) - (fry * lr) ) / inertia]

    return f

# get current car state using ackermen msg
class current_state(Node):
    def __init__(self):
        print("Node initialized")
        super().__init__("current_state")
        self.ackermann_subscriber = self.create_subscription(AckermannDrive, "/cmd_vel", self.callback, 1)

    def callback(self, msg:AckermannDrive):
        print(msg)
        self.stag = msg.steering_angle
        self.stag_vel = msg.steering_angle_velocity
        self.speed = msg.speed

    def get_initials(self):
        # compute lateral velocity 
        vy = self.speed * np.sin(self.stag)
        
        return [vy, self.stag_vel]

if __name__ == "__main__":
    # create Vx uniform range
    Vx = np.arange(0.5, 3.5, 0.25)
    # crate Vx drifing uniform range
    Vxd = np.linspace(1.5, 2.25, len(Vx), endpoint=True)
    # create a uniform set of steering angles
    stag = np.linspace(-0.3, 0.3, num=20, endpoint=True)
    # combine the uniform and drifting velocities
    vxf = np.unique(np.sort(np.append(Vx,Vxd)))

    # Numerically Integrate Odes
    # initial and final time (in seconds)
    t0 = 0
    tf = 1
    # get and pack initial conditions
    rclpy.init()
    node = current_state()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt as e:
        print(f"node spin error: {e}")
    yinit = node.get_initials() # initial y = (vy0, w0)
    node.destroy_node() 
    rclpy.shutdown()

    # additional parameters for derivative function
    mass = 84.5 # car mass (in kgs)
    lf = 3 # distance from CoG to front tire
    lr = 1 # distance from CoG to rear tire
    inertia = 10 # inertia of car?

    # integrate for lateral and angular velocity for each forward velocity and steering angle
    t, y = [],[]
    for vx in [1.5]:
        for i, delta in enumerate(stag):
            # pack up new parameters
            p = [vx, delta, mass, inertia, lf, lr]
            sol = solve_ivp(differential, (t0,tf), yinit, args=(p,))
            t.append(sol.t)
            y.append(sol.y)
            print(f"for {vx}m/s and {delta}rads lat was {y[i][0][-1]}m/s and omega was {y[i][1][-1]}rads/s")

    # plot steering angle vs lateral velocity
    temp = []
    fig = plt.figure()
    for i in range(len(stag)):
        temp.append(y[i][0][-1])
    plt.plot(stag,temp,"-ok")
    plt.show()
    plt.savefig("src/planning/planning_path/planning_path/plot output")








