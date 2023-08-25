import numpy as np
from scipy.integrate import solve_ivp

# create Vx uniform range
Vx = np.arange(0.5, 3.5, 0.25)
# crate Vx drifing uniform range
Vxd = np.linspace(1.5, 2.25, len(Vx), endpoint=True)
# create a uniform set of steering angles
stag = np.linspace(-0.3, 0.3, num=5, endpoint=True)
# combine the uniform and drifting velocities
vxf = np.unique(np.sort(np.append(Vx,Vxd)))

# parameter functions
def Fry():
    alphar = np.arctan(((omega*lr)-vy)/vx)
    fry = Dr*np.sin(Cr*np.arctan(Br*alphar))
    # return(fry)
    pass

def Ffy():
    alphaf = -np.arctan(((omega * lr) + vy) / vx) + sa
    ffy = Df * np.sin(Cf * np.arctan(Bf * alphaf))
    # return(ffy)
    pass

# vy derivative
def Vyd(t, y, m, delta, vx, omega):
    # get parameters
    fry = Fry()
    ffy = Ffy()
    vyd = (fry + (ffy*np.cos(delta)) - (m*vx*omega))/m
    # return(vyd)
    pass

# angular derivative
def Wd(t, y, m, delta, vx, omega, lf, lr, inertia):
    wd = ((Ffy()*lf*np.cos(delta)) - (Fry()*lr))/inertia
    # return(wd)
    pass

# get current car state (kalman filter?)


# numerically integrate odes
# initial and final time (in seconds)
t0 = 0
tf = 2
# initial lateral and angular velocity
y0 = np.nan # kalman filter - lateral
y02 = np.nan # kalman filter - angular
# additional parameters for derivative function
mass = 0 # car mass
lf = 0 # distance from CoG to front tire
lr = 0 # distance from CoG to rear tire
inertia = 0 # inertia of car?

# integrate for lateral and angular velocity for each forward velocity and steering angle
for vx in vxf:
    for delta in stag:
        sol = solve_ivp(Vyd,[t0,tf],y0,args=(mass,delta,vx,omega))
        sol2 = solve_ivp(Wd,[t0,tf],y02,args=(mass,delta,vx,omega,lf,lr,inertia))








