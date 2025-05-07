import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay

#just testing (and scratching):
import test_and_scratch
# Example input: list of ([x, y], 'colour')

tridict = {}
tridict[0] = [0, 1]
tridict[1] = [1, 2]
tridict[2] = [2, 0]

class Cone:
    def __init__(self, coordinate, colour='x', timestamp=0):
        self.coordinate = np.array(coordinate)
        self.colour = colour
        self.timestamp = timestamp

class PointWithIndex:
    def __init__(self, coordinate, index):
        self.coordinate = coordinate
        self.index = index

def GimmeCanD(incoming_cones, extra_bs=False, prop=False, prop_amount=0.5):

    # # Convert incoming data into Cone objects
    cones = [Cone(coord, colour, timestamp) for coord, colour, timestamp in incoming_cones]

    # plt.figure(figsize=(12, 7))
    # for i in range(len(centre_points)):
    #     plt.scatter(centre_points[i, 0], centre_points[i, 1], color='red', marker='x', label='Centre Points')
    #     plt.scatter(cones[i][0][0], cones[i][0][1], color='purple', marker='o', label='Cones')
    # plt.axis('equal')
    # plt.legend()
    # plt.title('Bruh Graph with Segments')
    # plt.show()

    # Prepare data for Delaunay triangulation
    cone_coords = np.array([cone.coordinate for cone in cones])
    tri = Delaunay(cone_coords)

    # Find centre points between cones of different colours
    centre_points = []
    yellow_cones = []
    blue_cones = []

    if not prop:
        for simplex in tri.simplices:
            edges = [
                (cones[simplex[0]], cones[simplex[1]]),
                (cones[simplex[1]], cones[simplex[2]]),
                (cones[simplex[2]], cones[simplex[0]])
            ]
            for i in range(len(edges)):
                edge = edges[i]
                if edge[0].colour != edge[1].colour:
                    centre = (edge[0].coordinate + edge[1].coordinate) / 2
                    if PointWithIndex(centre, simplex[[i][0]]) not in centre_points[-5:]:
                        avg_timestamp = (edge[1].timestamp + edge[0].timestamp) / 2
                        centre_points.append(Cone(centre, timestamp=avg_timestamp))
                        if edge[0].colour == 'b':
                            blue_cones.append(Cone(edge[0].coordinate, timestamp=edge[0].timestamp))
                            yellow_cones.append(Cone(edge[1].coordinate, timestamp=edge[1].timestamp))
                        else:
                            blue_cones.append(Cone(edge[1].coordinate, timestamp=edge[1].timestamp))
                            yellow_cones.append(Cone(edge[0].coordinate, timestamp=edge[0].timestamp))

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
                        
                        prop_point = (edge[1].coordinate - centre) * prop_amount + centre
                        prop_points.append(prop_point)#ok...

        prop_points = np.array(prop_points)
    
    centre_points = np.array(
    list({
        tuple(pt.coordinate): None
        for pt in sorted(centre_points, key=lambda cone: cone.timestamp)
        }.keys())
    )
    blue_cones = np.array(
    list({
        tuple(pt.coordinate): None
        for pt in sorted(blue_cones, key=lambda cone: cone.timestamp)
        }.keys())
    )
    yellow_cones = np.array(
    list({
        tuple(pt.coordinate): None
        for pt in sorted(yellow_cones, key=lambda cone: cone.timestamp)
        }.keys())
    )
    
    # centre_points = np.array(centre_points)
    # blue_cones = np.array(blue_cones)
    # yellow_cones = np.array(yellow_cones)

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

    # plt.figure(figsize=(12, 7))
    # for i in range(len(centre_points)):
    #     plt.scatter(centre_points[i, 0], centre_points[i, 1], color='red', marker='x', label='Centre Points')
    #     plt.scatter(blue_cones[i, 0], blue_cones[i, 1], color='blue', marker='o', label='Centre Points')
    #     plt.scatter(yellow_cones[i, 0], yellow_cones[i, 1], color='yellow', marker='o', label='Centre Points')
    # plt.axis('equal')
    # plt.legend()
    # plt.title('Bruh Graph with Segments')
    # plt.show()

    return centre_points, blue_cones, yellow_cones

# incoming_cones = test_and_scratch.tester_oval(100)
# GimmeCanD(incoming_cones)
