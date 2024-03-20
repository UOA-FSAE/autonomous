# imports
import pandas as pd
import os

import TrackHelpers
from TrackHelpers import plt, np
from CoreModels import Bracket, Node

def importTrack(trackname:str, hasboundaries:bool):
    """
    import named track from racetrack-database (see comment below)

    Inputs:
        trackname (string): name of the track
        hasboundaries (bool): whether to compute & include boundaries
    
    Outpus:
        (vector): list of center/boundary points as list of (x,y)
    """

    ''' Reference:
    racetrack-database, Alexander Heilmeier, https://github.com/TUMFTM/racetrack-database
    '''

    # import track data
    df = pd.read_csv(f"{os.path.dirname(__file__)}/Tracks/racetrack-database/tracks/{trackname}.csv")
    df.columns = ["x_m","y_m","w_tr_right_m","w_tr_left_m"]

    # get number of rows
    num_rows = len(df.x_m)

    # check for boundaries
    if not hasboundaries:
        outer = []
        inner = []
    else:
        # COMPUTE BOUNDARIES
        # center coordinates, track widths and lap distance
        center_points = [0]*num_rows
        track_widths = [0]*num_rows
        lap_distance = [0]*num_rows

        # inner - left, outer - right, perpendicual vector (inner to outer)
        inner = [0]*num_rows
        outer = [0]*num_rows
        p_vector = [0]*num_rows

        
        # concatenate each center point coordinate as a list
        for i in range(num_rows):
            center_points[i] = [df.x_m[i], df.y_m[i]]
            track_widths[i] = [df.w_tr_right_m[i], df.w_tr_left_m[i]]

        # compute boundary for initial center point
        # to np.array
        center_points = np.array(center_points)
        track_widths = np.array(track_widths)
        lap_distance = np.array(lap_distance)

        # set lap_distance of first element to 0
        lap_distance[0] = 0

        vector = TrackHelpers.getVector(center_points[-1], center_points[-2], True)
        inner[0], outer[0], p_vector[0] = TrackHelpers.getBoundaries(center_points[0], track_widths[0], vector)

        for i in range(1,num_rows-1):
            # normalized vector in direction of center line
            vector = TrackHelpers.getVector(center_points[i-1], center_points[i+1], True)
    
            # calculate lap_distance
            lap_distance[i] = TrackHelpers.getDistance(center_points[i], center_points[i-1])+lap_distance[i-1]

            # compute boundaries
            inner[i], outer[i], p_vector[i] = TrackHelpers.getBoundaries(center_points[i], track_widths[i], vector)   

        # compute boundary and final lap distance
        vector = TrackHelpers.getVector(center_points[-2], center_points[1], True) 
        lap_distance[-1] = TrackHelpers.getDistance(center_points[-1], center_points[-2])+lap_distance[-2]
        inner[-1], outer[-1], p_vector[-1] = TrackHelpers.getBoundaries(center_points[-1], track_widths[-1], vector)

        # Note there is a discontinuity between the last and first points as the distance of the first point is 0 and the last is the sum of subsequent points
        # track_distance = TrackHelpers.getDistance(center_points[1], center_points[end])+lap_distance[end]


    # final dataframe
    trackdf = pd.DataFrame({
        "outer": outer,
        "inner": inner,
        "cline": center_points.tolist(),
        "p_vector": p_vector
        })

    # plotting
    p = plt.figure()

    # center line
    TrackHelpers.Plot(False, trackdf.cline, "center line")
    # inner boundary
    TrackHelpers.Plot(False, trackdf.inner, "inner boundary")
    # outer boundary
    TrackHelpers.Plot(False, trackdf.outer, "outer boundary")

    p.savefig(f"{os.path.dirname(__file__)}/Track images/{trackname}.png", dpi=600) 
    plt.legend()
    plt.show()

    return trackdf

# function interpolateTrack(df::DataFrame; distances_array::AbstractArray = [], interval=nothing,)
#     """
#     Inputs: 
#         df: DataFrame of full track data including existing boundary calculations
#         distances_array: array of distances along the track to be interpolated at 
#     Outputs:
#         interpolated_trackdf: interpolated track

#     """
#     if isnothing(interval)==false
#         distances_array = LinRange(0,last(df.lap_distance),ceil(Int64,last(df.lap_distance)/interval))
#     end
    
#     # get number of rows of df and lap_distances
#     new_numrows = length(distances_array)
#     # println([distance for distance in distances_array[1:20]])
    
#     # create empty interpolated lists
#     new_cline = Vector{}(undef, new_numrows)
#     new_inner = Vector{}(undef, new_numrows)
#     new_outer = Vector{}(undef, new_numrows)
#     new_p_vector = Vector{}(undef, new_numrows)

#     track_length = last(df.lap_distance)

#     # current index of interpolated track element
#     j = 1
#     prev_row = first(df)
#     if isapprox(first(distances_array),prev_row.lap_distance)
#         new_cline[1] = prev_row.cline
#         new_inner[1] = prev_row.inner
#         new_outer[1] = prev_row.outer
#         new_p_vector[1]=prev_row.p_vector
#         j=2
#     end
#     for row in eachrow(df[2:end,:])
#         # if distances_array[j]
#         # 1. get vector
#         vector = TrackHelpers.getVector(prev_row.cline, row.cline,true)

#         # 2. get perpendicular vector to interpolation point
#         pvector = TrackHelpers.getRotatedVector(0.5*pi, vector)

#         # 3. get vectors of corresponding inner and outer interval points
#         outer_vector = TrackHelpers.getVector(prev_row.outer, row.outer, true)
#         inner_vector = TrackHelpers.getVector(prev_row.inner, row.inner, true)
        
#         while row.lap_distance > distances_array[j] > prev_row.lap_distance
#             d = distances_array[j] - prev_row.lap_distance

#             cpoint = prev_row.cline+d.*vector

#             #centre line point
#             new_cline[j] = cpoint
            
#             # 4. get new inner and outer points
#             new_outer[j] = TrackHelpers.getVectorLineIntersection(outer_vector, row.outer,pvector, cpoint)
#             new_inner[j] = TrackHelpers.getVectorLineIntersection(inner_vector, row.inner,-pvector, cpoint)
#             new_p_vector[j] = pvector
#             j+=1
#         end
#         if isapprox(row.lap_distance, distances_array[j])
#             # if interpolating at original data point do nothing
#             new_cline[j] = row.cline
#             new_inner[j] = row.inner
#             new_outer[j] = row.outer
#             new_p_vector[j]=row.p_vector
#             j+=1
#         end

#         prev_row = row
#     end

#     interpolated_trackdf = DataFrame(
#         outer = new_outer,
#         inner = new_inner,
#         cline = new_cline,
#         p_vector = new_p_vector,
#         lap_distance = distances_array
#     )

#     return interpolated_trackdf
# end

def createBracket(inner_boundary:np.array, outer_boundary:np.array, p_vector, n_nodes):
    # get distance
    distance = TrackHelpers.getDistance(inner_boundary, outer_boundary)
    spacing = distance / n_nodes

    # list of node coordinates
    new_points = [0]*(n_nodes+2)
    # include boundaries
    new_points[0] = inner_boundary
    new_points[-1] = outer_boundary

    # loop through number of nodes to create
    for i in range(n_nodes):
        new_points[i+1] = inner_boundary + ((i+1)*spacing) * p_vector

    return new_points

def getBrackets(df:pd.DataFrame, n_nodes):
    # attributes
    constant_velocity = 0
    
    # number of boundary points
    num_boundary_points = len(df.inner)

    # vectors
    brackets = [0]*num_boundary_points

    # loop through each boundary point
    for i in range(num_boundary_points):
        # boundary points
        inner_boundary = df.inner[i]
        outer_boundary = df.outer[i]
        perp_vector = df.p_vector[i]
        node_list = [0]*(n_nodes+2)

        # get new points and assign a node 
        bracket_points = createBracket(inner_boundary, outer_boundary, perp_vector, n_nodes)
        
        for j, P in enumerate(bracket_points):
            bracketId = i
            xy = P
            velocity = constant_velocity
            innerDistance = TrackHelpers.getDistance(P, bracket_points[0])
            outerDistance = TrackHelpers.getDistance(P, bracket_points[-1])
            nextNode = np.nan
            cost = np.inf
            # temp_node = Node(bracketId, xy, velocity, innerDistance, outerDistance, nextNode, cost)
            # append to node list
            node_list[j] = Node(bracketId, xy, velocity, innerDistance, outerDistance, nextNode, cost)

        # bracket module
        Id = i
        innerNode = node_list[0]
        outerNode = node_list[-1]
        width = TrackHelpers.getDistance(node_list[0]._xy, node_list[-1]._xy)
        # NodeList = node_list
        # temp_bracket = Bracket(Id, innerNode, outerNode, width, NodeList)
        # this code below doesn't work all brackets have the same node list!!
        brackets[i] = Bracket(Id, innerNode, outerNode, width, node_list)

        del node_list

    # plotting
    p = plt.figure()

    # inner boundary
    TrackHelpers.Plot(False, df.inner, "inner boundary")
    # outer boundary
    TrackHelpers.Plot(False, df.outer, "outer boundary")
    # nodes
    for B in brackets:
        # all_x = [P._xy[0] for P in B._nodeList]
        # all_y = [P._xy[1] for P in B._nodeList]
        # plt.plot(all_x,all_y,"ok")
        all_nodes = B._nodeList
        TrackHelpers.Plot(True, all_nodes, "nodes")

    p.savefig(f"{os.path.dirname(__file__)}/Track images/nodes.png", dpi=600) 
    plt.show()

    return brackets

# function shortestPath(track_name, df, start_point::Vector, end_point::Vector, brackets)
#     # vector of points for optimal line
#     shortest_path_line = Vector{}(undef,length(brackets)+2) # need to change to matrix with num_nodes
#     # go through each bracket NodeList
#     current_point = start_point
#     shortest_path_line[1] = current_point
#     best_point = current_point
#     for i in eachindex(brackets)
#         min_time = 999999999.0
#         # distance between current point and each node in the node list
#         for j in eachindex(brackets[i].NodeList)
#             current_node = brackets[i].NodeList[j]
#             if current_node.bracketId == brackets[i].Id
#                 distance = TrackHelpers.getDistance(current_point, current_node.xy)
#                 objective_function = (distance / current_node.velocity)
#                 if (objective_function < min_time)
#                     best_point = current_node.xy
#                     min_time = objective_function
#                 end
#             end
#         end
#         # append best point
#         shortest_path_line[i+1] = best_point
#         current_point = best_point
#     end
#     # append last point
#     shortest_path_line[end] = start_point

#     # plotting        
#     p = plot()

#     # inner boundary
#     TrackHelpers.Plot(p, false, df.inner, "inner boundary")
#     # outer boundary
#     TrackHelpers.Plot(p, false, df.outer, "outer boundary")

#     # shortest race line
#     TrackHelpers.Plot(p, false, shortest_path_line, "race line")

#     savefig("Race lines/$track_name.png")

#     return shortest_path_line
# end

def belman_ford_path(track_name, df, velocity_range, brackets, start_node):
    # Initialise first set of paths from first bracket
    for node in brackets[-1]._nodeList:
        node._cost = 0 
    
    # loop through every bracket (backwards - think as if you are doing forward but reversed)
    for i in range(len(brackets)-1,1,-1):
        # print("Bracket: $i \n")
        # the second bracket best node must only come from the starting node
        if i == 1:
            current_node_list = [start_node]
        else:
            current_node_list = brackets[i-1]._nodeList

        # loop through  every node in current bracket starting at second to last
        for current_node in current_node_list:

            # the minimum cost is the current cost at the node
            min_cost = np.inf
            best_velocity = current_node._velocity
            go_to_node = current_node._nextNode

            # node ahead (e.g. last bracket if current is second to last bracket)
            for next_node in brackets[i]._nodeList:
                distance_between_nodes = TrackHelpers.getDistance(next_node._xy, current_node._xy)

                # try every velocity range 
                for velocity in velocity_range:
                    cost = (2*distance_between_nodes) / (velocity + next_node._velocity) + next_node._cost
                    # if lower cost to travel 
                    if cost < min_cost:
                        min_cost = cost 
                        best_velocity = velocity
                        if go_to_node is not np.nan:
                            del go_to_node
                        go_to_node = next_node

                # next node cost is cumulative unless its the first next bracket
                current_node._cost = min_cost
                current_node._velocity = best_velocity
                current_node._nextNode = go_to_node

    # plotting        
    p = plt.figure()

    # inner boundary
    TrackHelpers.Plot(False, df.inner, "inner boundary")
    # outer boundary
    TrackHelpers.Plot(False, df.outer, "outer boundary")

    # optimal race line
    current_node = start_node
    optimal_nodes = []
    velocities = []
    while current_node is not np.nan:
        velocities.append(current_node._velocity)
        optimal_nodes.append(current_node)
        # update current node
        current_node = current_node._nextNode

    TrackHelpers.Plot(True, optimal_nodes, "optimal race line")

    p.savefig(f"{os.path.dirname(__file__)}/Race lines/{track_name}.png", dpi=1200) 
    plt.show()

    return start_node

