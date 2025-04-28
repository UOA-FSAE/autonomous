import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay

#just testing (and scratching):
import test_and_scratch
# Example input: list of ([x, y], 'colour')

class Cone:
    def __init__(self, coordinate, colour):
        self.coordinate = np.array(coordinate)
        self.colour = colour

def GimmeCanD(incoming_cones, extra_bs=False, prop=False, prop_amount=0.5):

    # Convert incoming data into Cone objects
    cones = [Cone(coord, colour) for coord, colour in incoming_cones]

    # Prepare data for Delaunay triangulation
    cone_coords = np.array([cone.coordinate for cone in cones])
    tri = Delaunay(cone_coords)

    # Find centre points between cones of different colours
    centre_points = []
    deluaunay_cone_pairs = []
    yellow_cones = []
    blue_cones = []

    if not prop:
        for simplex in tri.simplices:
            edges = [
                (cones[simplex[0]], cones[simplex[1]]),
                (cones[simplex[1]], cones[simplex[2]]),
                (cones[simplex[2]], cones[simplex[0]])
            ]
            for edge in edges:
                if edge[0].colour != edge[1].colour:
                    centre = (edge[0].coordinate + edge[1].coordinate) / 2
                    centre_points.append(centre)
                    deluaunay_cone_pairs.append([edge[0].coordinate, edge[1].coordinate])
                    if edge[0].colour == 'b':
                        blue_cones.append(edge[0])
                        yellow_cones.append(edge[1])
                    else:
                        blue_cones.append(edge[1])
                        yellow_cones.append(edge[0])

    else:
        prop_points = []
        for simplex in tri.simplices:
            edges = [
                (cones[simplex[0]], cones[simplex[1]]),
                (cones[simplex[1]], cones[simplex[2]]),
                (cones[simplex[2]], cones[simplex[0]])
            ]
            for edge in edges:
                if edge[0].colour != edge[1].colour:
                    centre = (edge[0].coordinate + edge[1].coordinate) / 2
                    if not any(np.array_equal(centre, point) for point in centre_points):
                        centre_points.append(centre)
                        deluaunay_cone_pairs.append([edge[0].coordinate, edge[1].coordinate])
                        
                        prop_point = (edge[1].coordinate - centre) * prop_amount + centre
                        prop_points.append(prop_point)#ok...

        prop_points = np.array(prop_points)

    centre_points = np.array(centre_points)

    if extra_bs:
        plt.figure(figsize=(12, 7))
        # Plot only edges between cones of different colors
        for simplex in tri.simplices:
            edges = [
                (cones[simplex[0]], cones[simplex[1]]),
                (cones[simplex[1]], cones[simplex[2]]),
                (cones[simplex[2]], cones[simplex[0]])
            ]
            for edge in edges:
                if edge[0].colour != edge[1].colour:  # Only plot edges with different colored cones
                    plt.plot([edge[0].coordinate[0], edge[1].coordinate[0]],
                            [edge[0].coordinate[1], edge[1].coordinate[1]],
                            color='gray')  # Plot edge in gray
        # Scatter the cones by color (yellow for left, blue for right)
        plt.scatter([cone.coordinate[0] for cone in cones if cone.colour == 'y'], 
                    [cone.coordinate[1] for cone in cones if cone.colour == 'y'], 
                    color='yellow', label='Left Cones')

        plt.scatter([cone.coordinate[0] for cone in cones if cone.colour == 'b'], 
                    [cone.coordinate[1] for cone in cones if cone.colour == 'b'], 
                    color='blue', label='Right Cones')
        # Plot centre points
        plt.scatter(centre_points[:, 0], centre_points[:, 1], color='red', marker='x', label='Centre Points')
        if prop:
           plt.scatter(prop_points[:, 0], prop_points[:, 1], color='green', marker='x', label='Prop Points') 

        plt.axis('equal')
        plt.legend()
        plt.title('Track with Only Edges Between Different Coloured Cones')
        plt.show()

        # Send centre_points to next part of your code
        for i in range(len(centre_points)):
            print(f"{centre_points[i]}, from averaging: {deluaunay_cone_pairs[i]}")


    return centre_points, blue_cones, yellow_cones, deluaunay_cone_pairs

# incoming_cones = test_and_scratch.tester_oval(100)
# GimmeCanD(incoming_cones, True, True, 0.5)
