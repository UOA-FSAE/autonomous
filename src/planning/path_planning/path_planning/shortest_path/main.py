# main file
def main():
    # imports 
    # import CoreModels
    # import PathHelpers
    from path_planning.shortest_path.CoreModels import Node, State
    import path_planning.shortest_path.TrackMethods as TrackMethods
    import path_planning.shortest_path.TrackHelpers as TrackHelpers
    import path_planning.shortest_path.PathHelpers as PathHelpers

    import numpy as np

    CAR = {
        "mass": 84.5,  # kg
        "μ": 0.6, # static friction coefficient - dimensionless
        "α": PathHelpers.noughtTo60(3.0), 
        "α_d": 15.0,    # max decel in m/s^2
        "max steer angle": 25.0, # degrees
        "max velocity": 10.0,  # m/s
        "tire width": 0.11, # in m 
        "wheelbase": 1.5,    # wheelbase length (in m)
    }

    # choose track
    track_name = "Silverstone"

    # read chosen track 
    print("IMPORTING TRACK")
    df = TrackMethods.importTrack(trackname=track_name)

    # print("INTERPOLATING")
    # df = TrackMethods.interpolateTrack(df, interval = 5) 

    # create brackets
    print("CREATING BRACKETS")
    brackets = TrackMethods.getBrackets(df, 8, False)
    d_start = 300
    d_end = 400
    section_of_track = True
    if section_of_track:
        df, brackets = TrackMethods.getSectionofTrack(df, np.array(brackets), d_start, d_end)

    print("COMPUTING OPTIMAL PATH")
    # mass = 50.0
    # μ = 0.7
    start_node = brackets[0]._nodeList[4]
    print(-1/df.p_vector.tolist()[0])
    start_node._stateList.append(State(start_node, -1/df.p_vector.tolist()[0], 0.0, np.Inf))
    print("starting inner distance: ",start_node._innerDistance)
    print("starting outer distance: ", start_node._outerDistance)
    # start_node = TrackMethods.belman_ford_path(df, velocity_range, brackets, start_node, plot=self._plot)
    n_vel = 10
    start_node, brackets, optimal_cost = TrackMethods.optimal_path(
        "$track_name optimal", 
        df, 
        start_node, 
        brackets, 
        n_vel,
        CAR,
        True,
    )
    # start_node = TrackMethods.optimise_path(track_name, df, brackets, start_node, mass, μ)
    print("\nOPTIMAL PATH COMPUTED")


if __name__ == "__main__":
    main()