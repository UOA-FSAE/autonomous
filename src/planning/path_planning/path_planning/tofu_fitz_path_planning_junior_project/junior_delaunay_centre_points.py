#%% md
# Triangulating cones as points
#%%
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
import numpy as np
from itertools import combinations

#Delaunay code provided by Winola
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

import test_and_scratch as tas
cones2 = tas.tester_oval(100)
cones2 = [Cone(cone[0], cone[1]) for cone in cones2]

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

#CONEVERSION (IMPORTANT?)
centre_points = np.array(centre_points)

#plotting
plt.triplot(cone_coords[:, 0], cone_coords[:, 1], valid_simplices)
plt.scatter(cone_coords[:, 0], cone_coords[:, 1], c=["blue" if cone.colour == "b" else "yellow" for cone in cones2])
plt.scatter(centre_points[:, 0], centre_points[:, 1], c='red', marker='x', label='Centre Points')
plt.show()

#REMOVE PLT.SHOWS AND PRINTS, ONLY SEND THE CENTRE_POINTS TO THE PATH PLANNING SCRIPT. OTHERWISE (EVEN BETTER TO JUST MAKE BOTH SCRIPTS ONE)
print(centre_points)
#centre_points[6][0] to access values (this example would access the x value of the 7th point)