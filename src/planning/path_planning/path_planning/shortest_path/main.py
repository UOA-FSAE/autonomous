# main file
def main():
    # imports 
    # import CoreModels
    # import PathHelpers
    import planning.path_planning.path_planning.shortest_path.TrackMethods as TrackMethods

    # choose track
    track_name = "Silverstone"

    # read chosen track 
    print("IMPORTING TRACK")
    df = TrackMethods.importTrack(trackname=track_name)

    # print("INTERPOLATING")
    # df = TrackMethods.interpolateTrack(df, interval = 5) 

    # create brackets
    print("CREATING BRACKETS")
    brackets = TrackMethods.getBrackets(df, 10)

    velocity_range = [0.01, 8, 16, 24, 32, 40 ]# velocities in meters per second

    print("COMPUTING OPTIMAL PATH")
    start_node = brackets[0]._nodeList[5]
    start_node._cost = 0
    print("starting inner distance: ",start_node._innerDistance)
    print("starting outer distance: ", start_node._outerDistance)
    # mass = 50.0
    # μ = 0.7
    start_node = TrackMethods.belman_ford_path(df, velocity_range, brackets, start_node, track_name)
    # start_node = TrackMethods.optimise_path(track_name, df, brackets, start_node, mass, μ)
    print("\nOPTIMAL PATH COMPUTED")


if __name__ == "__main__":
    main()