#%% md
# Triangulating cones as points
#%%
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
import numpy as np
from itertools import combinations

#Delaunay code provided by Winola

#change this to input from ConeMap
# cones = np.array([[2,0],[-1,2],
#                   [3,2],[4.5,3.5],
#                   [0.5,3.5],[-2,-2],
#                   [2,-2],[-2,-4],
#                   [2,-4],[-2,0]])
cones = np.array([
    # Start straight
    [0, 0],    # left
    [2, 0],    # right
    [0, 10],
    [2, 10],
    [0, 20],
    [2, 20],

    # Right turn (90 deg)
    [0, 30],
    [2, 30],
    [3, 31],
    [5, 31],

    # Upper straight
    [6, 30],
    [8, 30],
    [10, 30],
    [12, 30],

    # S-curve (chicane)
    [13, 29],
    [15, 27],
    [16, 25],
    [15, 23],
    [13, 21],

    [12, 20],
    [10, 20],

    # Hairpin (tight left turn)
    [9, 19],
    [7, 18],
    [5, 18],
    [3, 19],

    # Return straight
    [2, 18],
    [0, 18],
    [-1, 17],
    [-1, 15],
    [-1, 13],
    [0, 11]
])

tri = Delaunay(cones)

plt.triplot(cones[:,0], cones[:,1], tri.simplices.copy())
plt.plot(cones[:,0], cones[:,1], 'o')
plt.show()
#%% md
# You can sort of see the shape of the track, of course a real track won't look as "neat" as this one. Now we need to get rid of the triangles outside the track because we don't want them when calculating centre points.
# 
# If we utilize the colour attribute of each cone, then you can see that triangles outside the track will have all three vertices as the same colour. This means we need to use a more complex data structure
#%%
class Cone:
    def __init__(self, coordinate, colour):
        self.coordinate = np.array(coordinate)
        self.colour = colour



# cones2 = [Cone([2,0], 'b'), Cone([-1,2],'y'),
#           Cone([3,2],'b'), Cone([4.5,3.5],'b'),
#           Cone([0.5,3.5],'y'), Cone([-2,-2],'y'),
#           Cone([2,-2],'b'), Cone([-2,-4],'y'),
#           Cone([2,-4],'b'), Cone([-2,0],'y')]
cones2 = [
    # Start straight
    Cone([0, 0], 'y'), Cone([2, 0], 'b'),
    Cone([0, 10], 'y'), Cone([2, 10], 'b'),
    Cone([0, 20], 'y'), Cone([2, 20], 'b'),

    # Right turn (90 deg)
    Cone([0, 30], 'y'), Cone([2, 30], 'b'),
    Cone([3, 31], 'y'), Cone([5, 31], 'b'),

    # Upper straight
    Cone([6, 30], 'y'), Cone([8, 30], 'b'),
    Cone([10, 30], 'y'), Cone([12, 30], 'b'),

    # S-curve (chicane)
    Cone([13, 29], 'y'), Cone([15, 27], 'b'),
    Cone([16, 25], 'b'), Cone([15, 23], 'b'),
    Cone([13, 21], 'y'),

    Cone([12, 20], 'y'), Cone([10, 20], 'b'),

    # Hairpin (tight left turn)
    Cone([9, 19], 'y'), Cone([7, 18], 'y'),
    Cone([5, 18], 'b'), Cone([3, 19], 'b'),

    # Return straight
    Cone([2, 18], 'y'), Cone([0, 18], 'b'),
    Cone([-1, 17], 'y'), Cone([-1, 15], 'y'),
    Cone([-1, 13], 'y'), Cone([0, 11], 'b')
]

cone_coords = np.array([cone.coordinate for cone in cones2])
tri2 = Delaunay(cone_coords)

valid_simplices = []
#uncomment for visualisation. OR COMMENT IT OUT FOR PERFORMANCE
# for simplex in tri2.simplices:
#     # Get the colors of the vertices of the simplex
#     colours = {cones2[vertex].colour for vertex in simplex}
#     # # If the simplex has a mix of colors, keep it
#     if len(colours) > 1:
#         valid_simplices.append(simplex)

plt.triplot(cone_coords[:, 0], cone_coords[:, 1], valid_simplices)
plt.plot(cones[:,0], cones[:,1], 'o')
plt.show()

#%% md
# Now we can get the centre line of the track. Note that we want to use edges with different coloured end points, because edges with the same coloured end points are the track boundaries.
#%%
centre_points= []
for simplex in tri2.simplices:
    edges = [
            (cones2[simplex[0]], cones2[simplex[1]]),
            (cones2[simplex[1]], cones2[simplex[2]]),
            (cones2[simplex[2]], cones2[simplex[0]])
        ]

    for edge in edges:
        if edge[0].colour != edge[1].colour:
            centre_points.append((edge[0].coordinate + edge[1].coordinate) / 2)

plt.triplot(cone_coords[:, 0], cone_coords[:, 1], valid_simplices)
plt.scatter(cone_coords[:, 0], cone_coords[:, 1], c=["blue" if cone.colour == "b" else "yellow" for cone in cones2])
centre_points = np.array(centre_points)
plt.scatter(centre_points[:, 0], centre_points[:, 1], c='red', marker='x', label='Centre Points')
plt.show()

#REMOVE PLT.SHOWS AND PRINTS, ONLY SEND THE CENTRE_POINTS TO THE PATH PLANNING SCRIPT. OTHERWISE (EVEN BETTER TO JUST MAKE BOTH SCRIPTS ONE)
print(centre_points)
#centre_points[6][0] to access values (this example would access the x value of the 7th point)