import path_planning.shortest_path.TrackHelpers as TrackHelpers
from path_planning.shortest_path.TrackHelpers import np

def getMaxTractionForce(μ,mass):
    """traction force is the static friction between tires and ground"""
    return μ*mass*9.81

def getTractionVelocity3p(p1, p2, p3, traction_force, mass):
    """
    numerical traction velocity estimate using limit of number of sides of a polygon approaching infinity is a circle
    """
    a = TrackHelpers.getDistance(p1, p2)
    b = TrackHelpers.getDistance(p2, p3)
    c = TrackHelpers.getDistance(p3, p1)
    # collinear
    if abs((a+b)-c) <= 1e-3:
        return np.inf, np.inf
    else:
        s = (a + b + c) / 2
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        r = (a * b * c) / (4 * area)
        v_traction = np.sqrt(traction_force*r/mass)

    return v_traction, r

def minmaxAccelerationVelocity(v2, distance, α, α_d):
    
    min_velocity = np.sqrt(max(0, v2**2-2*α*distance))
    max_velocity = np.sqrt(v2**2 + 2*α_d*distance)
    
    return min_velocity, max_velocity

def noughtTo60(nought_to_60):
    """
    Converts nought to 60 to linear acceleration
    """
    return 60/nought_to_60 * 0.44704
