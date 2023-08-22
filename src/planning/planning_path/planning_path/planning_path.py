import numpy as np

# create Vx uniform range
Vx = np.arange(0.5, 3.5, 0.25)
# crate Vx drifing uniform range
Vxd = np.linspace(1.5, 2.25, len(Vx), endpoint=True)
# create a uniform set of steering angles
stag = np.linspace(-0.3, 0.3, num=5, endpoint=True)

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
def Vyd(t, m, delta, args):
    # get parameters
    fry = Fry()
    ffy = Ffy()
    vyd = (fry+(ffy*np.cos(sa))-m*vx*omega)/m
    # return(vyd)
    pass

# angular derivative
def Wd(t, m, delta, args):
    wd = ((Ffy()*lf*np.cos(sa))-(Fry()*lr))/inertia
    # return(wd)
    pass

# get current car state (kalman filter?)









