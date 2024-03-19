
# imports
import numpy as np
import matplotlib.pyplot as plt

def getDistance(p1:np.array,p2:np.array):
    return getMagnitude((p2-p1))

def getMagnitude(vector:np.array): return np.sqrt(sum(vector**2))


def getVector(p1:np.array, p2:np.array, unit:bool):
    """perform forward/backward difference"""
    # compute rise and run: dx = x2-x1, dy = y2-y1
    vector = p2-p1

    if unit:
        return vector / getMagnitude(vector)
    
    return vector

def getRotatedVector(angle, vector:np.array):
    # rotation matrix
    rotate = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    return np.dot(rotate,vector)


# function getVectorLineIntersection(v1::Vector, p1::Vector,v2::Vector, p2::Vector)
#     # given two vectors and a point on each, return the coordinate of the intersection
    
#     # Extract components of vectors and points
#     vx1, vy1 = v1
#     vx2, vy2 = v2
#     px1, py1 = p1
#     px2, py2 = p2
    
#     # Calculate the intersection point
#     det = (vx1 * vy2 - vy1 * vx2)
    
#     if det == 0
#         error("Lines are parallel, no unique intersection point.")
#     end
    
#     t =(vy2*(px2-px1)-vx2*(py2-py1))/det
#     p = [vx1,vy1]*t + [px1,py1]
    
#     return p
# end

def getBoundaries(center_point:np.array, distance:np.array, unit_center:np.array):
    """computer boundaries"""
    inner = getInnerBoundary(center_point, distance[0], unit_center)
    outer, p_vector = getOuterBoundary(center_point, distance[1], unit_center)

    return inner, outer, p_vector


def getInnerBoundary(center_point:np.array, distance, vector:np.array):
    """computer right getOuterBoundary"""
    # transform by 90 degrees - counter clockwise negative
    angle = (-90.0/180) * np.pi
    rotated_vector = getRotatedVector(angle, vector)
    # dx = vector[1]*cos(-90*pi/180) - vector[2]*sin(-90*pi/180)
    # dy = vector[1]*sin(-90*pi/180) + vector[2]*cos(-90*pi/180)
    # new coordinate relative to center point
    coordinates = center_point + distance * rotated_vector

    return coordinates


def getOuterBoundary(center_point:np.array, distance, vector:np.array):
    """computer left boundary"""
    # transform by 90 degrees - clockwise positive
    angle = (90.0/180) * np.pi
    rotated_vector = getRotatedVector(angle, vector)
    # dx = vector[1]*cos(90*pi/180) - vector[2]*sin(90*pi/180)
    # dy = vector[1]*sin(90*pi/180) + vector[2]*cos(90*pi/180)
    # new coordinate relative to center point
    coordinates = center_point + distance * rotated_vector
    
    return coordinates, rotated_vector


# function getSteeringAngle(p1::Vector, p2::Vector)
#     # horizontal angle 
#     angle = atand(abs((p2-p1)[1]) / abs((p2-p1)[2]))
    
#     return angle
# end

# function getMaxTracktionForce(μ,N)
#     # max tracktion force is the static friction between tires and ground

#     return μ*N
# end

# function getRadiusOfCurvature()
#     # this is rubbish - please change 
#     return abs(randn()*5)
# end

# function getMaxAngleVelocity(mass::AbstractFloat, angle::AbstractFloat, μ::AbstractFloat)
#     # max velocity is tracktion limit which is Ft = mv^2/r
#     # rearrange for velocity, v = sqrt(Ft*r / m)
#     Ft = getMaxTracktionForce(μ,mass)
#     r = getRadiusOfCurvature()

#     return sqrt((Ft*r)/mass)
# end


def Plot(nodes:bool, vector_list:np.array, label:str):
    if nodes:
        all_x = [P._xy[0] for P in vector_list]
        all_y = [P._xy[1] for P in vector_list]
    else:
        all_x = [P[0] for P in vector_list]
        all_y = [P[1] for P in vector_list]

    plt.plot(all_x, all_y, label=label, lw=2)
    
    return all_x, all_y

