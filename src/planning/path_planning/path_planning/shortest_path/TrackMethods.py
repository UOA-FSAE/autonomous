# imports
import pandas as pd
import os

import path_planning.shortest_path.TrackHelpers as TrackHelpers
import path_planning.shortest_path.PathHelpers as PathHelpers

from path_planning.shortest_path.TrackHelpers import plt, np
from path_planning.shortest_path.CoreModels import Bracket, Node, State

from rclpy.node import Node as rclpyNode

def importTrack(track_info:pd.DataFrame=None, trackname:str=None, plot:bool=False):
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

    if trackname is None: 
        assert track_info is not None
        assert all(track_info.columns == ["x_m","y_m","w_tr_left_m","w_tr_right_m"])
        df = track_info
    else:
        # import track data
        df = pd.read_csv(f"{os.path.dirname(__file__)}/Tracks/racetrack-database/tracks/{trackname}.csv")
        df.columns = ["x_m","y_m","w_tr_right_m","w_tr_left_m"]

    # get number of rows
    num_rows = len(df.x_m)

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
    outer[0], inner[0], p_vector[0] = TrackHelpers.getBoundaries(center_points[0], track_widths[0], vector)

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
    outer[-1], inner[-1], p_vector[-1] = TrackHelpers.getBoundaries(center_points[-1], track_widths[-1], vector)

    # Note there is a discontinuity between the last and first points as the distance of the first point is 0 and the last is the sum of subsequent points
    # track_distance = TrackHelpers.getDistance(center_points[1], center_points[end])+lap_distance[end]


    # final dataframe
    trackdf = pd.DataFrame({
        "outer": outer,
        "inner": inner,
        "cline": center_points.tolist(),
        "p_vector": p_vector,
        "lap_distance": lap_distance,
        })

    if plot:
        # plotting
        p = plt.figure()

        # center line
        TrackHelpers.Plot(False, trackdf.cline, "center line")
        # inner boundary
        TrackHelpers.Plot(False, trackdf.inner, "inner boundary")
        # outer boundary
        TrackHelpers.Plot(False, trackdf.outer, "outer boundary")

        if trackname is None: trackname = "imported track"
        p.savefig(f"{os.path.dirname(__file__)}/Track images/{trackname}.png", dpi=600) 
        plt.legend()
        plt.show()
        # plt.close()

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

def createBracket(inner_boundary:np.array, outer_boundary:np.array, n_nodes, p_vector=None):
    # get distance
    distance = TrackHelpers.getDistance(inner_boundary, outer_boundary)
    spacing = distance / (n_nodes+1)
    p_vector = TrackHelpers.getVector(outer_boundary, inner_boundary, True)
    # angle = 90 * np.pi / 180
    # p_vector = TrackHelpers.getRotatedVector(angle, p_vector)

    # list of node coordinates
    new_points = [0]*(n_nodes+2)
    # include boundaries
    new_points[0] = outer_boundary
    new_points[-1] = inner_boundary

    # loop through number of nodes to create
    for i in range(n_nodes):
        new_points[i+1] = outer_boundary + ((i+1)*spacing) * p_vector

    return new_points

def getBrackets(df:pd.DataFrame, n_nodes, plot:bool=False):
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
        # perp_vector = df.p_vector[i]
        node_list = [0]*(n_nodes+2)

        # get new points and assign a node 
        bracket_points = createBracket(inner_boundary, outer_boundary, n_nodes)
        
        for j, P in enumerate(bracket_points):
            bracketId = i
            xy = P
            innerDistance = TrackHelpers.getDistance(P, bracket_points[-1])
            outerDistance = TrackHelpers.getDistance(P, bracket_points[0])
            # temp_node = Node(bracketId, xy, velocity, innerDistance, outerDistance, nextNode, cost)
            # append to node list
            node_list[j] = Node(bracketId, xy, innerDistance, outerDistance)

        # bracket module
        Id = i
        innerNode = node_list[-1]
        outerNode = node_list[0]
        width = TrackHelpers.getDistance(node_list[0]._xy, node_list[-1]._xy)
        # NodeList = node_list
        # temp_bracket = Bracket(Id, innerNode, outerNode, width, NodeList)
        # this code below doesn't work all brackets have the same node list!!
        brackets[i] = Bracket(Id, innerNode, outerNode, width, node_list)

        del node_list

    if plot:
        # plotting
        p = plt.figure()

        # inner boundary
        TrackHelpers.Plot(False, df.inner, "inner boundary")
        # outer boundary
        TrackHelpers.Plot(False, df.outer, "outer boundary")
        # nodes
        for i,B in enumerate(brackets):
            # all_x = [P._xy[0] for P in B._nodeList]
            # all_y = [P._xy[1] for P in B._nodeList]
            # plt.plot(all_x,all_y,"ok")
            all_nodes = B._nodeList
            if i == len(brackets)-1:
                col = "green"
            else:
                col = "black"
            TrackHelpers.Plot(True, all_nodes, "nodes", col)

        p.savefig(f"{os.path.dirname(__file__)}/Track images/nodes.png", dpi=600) 
        plt.show()
        # plt.close()

    return brackets

def getSectionofTrack(df, brackets, d_start, d_end):
    df_range = np.where(df.lap_distance[df.lap_distance <= d_end] >= d_start)[0]
    
    df = pd.DataFrame({
        "outer": df.outer[df_range],
        "inner": df.inner[df_range],
        "cline": df.cline[df_range],
        "p_vector": df.p_vector[df_range],
        "lap_distance": df.lap_distance[df_range],
        })
    brackets = brackets[df_range]

    return df, brackets

# def belman_ford_path(df, velocity_range, brackets, start_node, track_name=None, plot:bool=False):
#     # Initialise first set of paths from first bracket
#     for node in brackets[-1]._nodeList:
#         node._cost = 0 
    
#     # loop through every bracket (backwards - think as if you are doing forward but reversed)
#     for i in range(len(brackets)-1,0,-1):
#         # print("Bracket: $i \n")
#         # the second bracket best node must only come from the starting node
#         if i == 1:
#             current_node_list = [start_node]
#         else:
#             current_node_list = brackets[i-1]._nodeList

#         # loop through  every node in current bracket starting at second to last
#         for current_node in current_node_list:

#             # the minimum cost is the current cost at the node
#             min_cost = np.inf
#             best_velocity = current_node._velocity
#             go_to_node = current_node._nextNode

#             # node ahead (e.g. last bracket if current is second to last bracket)
#             for next_node in brackets[i]._nodeList:
#                 distance_between_nodes = TrackHelpers.getDistance(next_node._xy, current_node._xy)

#                 # try every velocity range 
#                 for velocity in velocity_range:
#                     cost = (2*distance_between_nodes) / (velocity + next_node._velocity) + next_node._cost
#                     # if lower cost to travel 
#                     if cost < min_cost:
#                         min_cost = cost 
#                         best_velocity = velocity
#                         if go_to_node is not np.nan:
#                             del go_to_node
#                         go_to_node = next_node

#                 # next node cost is cumulative unless its the first next bracket
#                 current_node._cost = min_cost
#                 current_node._velocity = best_velocity
#                 current_node._nextNode = go_to_node

#     if plot:
#         # plotting        
#         p = plt.figure()

#         # inner boundary
#         TrackHelpers.Plot(False, df.inner, "inner boundary")
#         # outer boundary
#         TrackHelpers.Plot(False, df.outer, "outer boundary")

#         # optimal race line
#         current_node = start_node
#         optimal_nodes = []
#         velocities = []
#         while current_node is not np.nan:
#             velocities.append(current_node._velocity)
#             optimal_nodes.append(current_node)
#             # update current node
#             current_node = current_node._nextNode

#         TrackHelpers.Plot(True, optimal_nodes, "optimal race line")

#         if track_name is None: track_name = "optimalPath"
#         p.savefig(f"{os.path.dirname(__file__)}/Race lines/{track_name}.png", dpi=600) 
#         plt.legend()
#         plt.show()
#         # plt.close()

#     return start_node

def optimal_path(track_name:str, df:pd.DataFrame, start_node:Node, brackets:np.array, n_vel, CAR:dict, plot:bool):
        """keeps ALL state from each pair of node state combination"""
        attributes = ["mass", "μ", "α", "α_d", "max steer angle", "max velocity", "tire width", "wheelbase"]
        mass, μ, α, α_d, max_steer_angle, max_velocity, tire_width, wheelbase = [CAR[key] for key in attributes]

        traction_force = PathHelpers.getMaxTractionForce(μ, mass)
        velocity_range = np.linspace(0, max_velocity, n_vel)
        min_steer_rad = wheelbase/np.sin(np.deg2rad(max_steer_angle)) + 0.5*tire_width
        
        for node in brackets[-1]._nodeList:
            for velocity in velocity_range:
                for previous_node in brackets[-2]._nodeList:
                    entry_vector = TrackHelpers.getVector(previous_node._xy, node._xy, True)
                    state = State(node, entry_vector, velocity, 0.0, previous_node)
                    node._stateList.append(state)
        
        start_node._stateList[0]._previousNode = Node(0,start_node._xy - 2*wheelbase*start_node._stateList[0]._entryVector, 0, 0)

        for i in range(len(brackets)-1,0,-1):
            # tmp = rclpyNode("tmp")
            # tmp.get_logger().info(f"Bracket: {i}\n")
            print(f"Bracket: {i}\n")
            
            if i == 2: 
                previous_node_list = [start_node];
                current_node_list = brackets[i-1]._nodeList
            elif i == 1:
                current_node_list = [start_node]
                previous_node_list = [];
            else:
                current_node_list = brackets[i-1]._nodeList
                previous_node_list = brackets[i-2]._nodeList; 

            def process_current_node(current_node):
            # for current_node in current_node_list:
                # current_node._stateList=[]
                # Initialising statelist for the current node
                for previous_node in previous_node_list:
                    entry_vector = TrackHelpers.getVector(previous_node._xy, current_node._xy, True)
                    current_node._stateList.append(State(current_node, entry_vector, 0.0, np.inf, previous_node, False, True))
                    for velocity in velocity_range:
                        state = State(current_node, entry_vector, velocity, np.inf, previous_node)
                        current_node._stateList.append(state)
                
                for next_node in brackets[i]._nodeList:
                    distance_between_nodes = TrackHelpers.getDistance(current_node._xy, next_node._xy)  
                    for current_state in current_node._stateList:                      
                        # prev_xy = current_state._previousNode._xy if i != 1 else current_state._xy-np.array(current_state._entryVector)*distance_between_nodes 
                        traction_velocity, radius = PathHelpers.getTractionVelocity3p(current_state._previousNode._xy, current_node._xy, next_node._xy, traction_force, mass)
                        # print(traction_velocity, radius)
                        if radius < min_steer_rad:
                            continue    # skip current current state

                        for next_node_state in next_node._stateList:
                            if next_node_state._previousNode == current_node:
                                min_va, max_va = PathHelpers.minmaxAccelerationVelocity(next_node_state._velocity, distance_between_nodes, α, α_d)
                                
                                if traction_velocity < min_va:
                                    continue    # skip next state
                                
                                # if current_state._min:
                                #     # accelerating, optimal minimum speed state
                                #     ideal_velocity = min_va
                                #     traverse_time = ((2*distance_between_nodes)/(ideal_velocity+next_node_state._velocity)) + next_node_state._cost
                                #     if traverse_time < current_state._cost:
                                #         current_state._velocity = ideal_velocity
                                #         current_state._cost = traverse_time
                                #         current_state._nextState = next_node_state
                                if current_state._max:
                                    # braking, optimum maximum speed state
                                    ideal_velocity = min(traction_velocity, max_va, max_velocity)
                                    tmp = current_state._velocity 
                                    current_state._velocity = ideal_velocity
                                    traverse_time = PathHelpers.getTraverseTime(distance_between_nodes, current_state, next_node_state)
                                    if traverse_time < current_state._cost:
                                        current_state._velocity = ideal_velocity
                                        current_state._cost = traverse_time
                                        current_state._nextState = next_node_state
                                    else:
                                        current_state._velocity = tmp

                                # non ideal state
                                elif min_va <= current_state._velocity < min(traction_velocity, max_va, max_velocity):
                                    traverse_time = PathHelpers.getTraverseTime(distance_between_nodes, current_state, next_node_state)
                                    if traverse_time < current_state._cost:
                                        # print('called')
                                        current_state._cost = traverse_time
                                        current_state._nextState = next_node_state
            
                # for state in current_node._stateList
                #     if state._cost == Inf; deleteState!(state); end
            # executor = concurrent.futures.ProcessPoolExecutor(8)
            # futures = [executor.submit(process_current_node, current_node) for current_node in current_node_list]
            # concurrent.futures.wait(futures)
            _ = list(map(process_current_node, current_node_list))
            # _ = [process_current_node(n) for n in current_node_list]

        # get optimal/best path by iterating through EVERY SINGLE state LOL
        print("getting best path")
        best_xy, best_velocities, cost = getBestStates(start_node)
        print(best_xy, "\n\n", best_velocities,"\n")

        if plot:
            # plotting        
            p = plt.figure()

            # inner boundary
            TrackHelpers.Plot(False, df.inner, "inner boundary")
            # outer boundary
            TrackHelpers.Plot(False, df.outer, "outer boundary")
            # nodes
            for i,B in enumerate(brackets):
                all_nodes = B._nodeList
                if i == len(brackets)-1:
                    col = "green"
                else:
                    col = "black"
                TrackHelpers.Plot(True, all_nodes, "nodes", col)

            # optimal path
            im = TrackHelpers.plotOptimal(best_xy, best_velocities, "optimal race line")
            plt.colorbar(im, label='velocity (m/s)')
            plt.title("Optimal Trajectory Colored by Velocity")

            p.savefig(f"{os.path.dirname(__file__)}/Race lines/{track_name}.png", dpi=600)
            plt.show()

        return start_node, brackets, cost/60

def getBestStates(start_node:Node):
    cost = start_node._stateList[0]._cost
    best_xy= []
    velocities = []
    for state in start_node._stateList:
        current_state = state
        while current_state:
            best_xy.append(current_state._xy)
            velocities.append(current_state._velocity)
            current_state = current_state._nextState

    return best_xy, velocities, cost
