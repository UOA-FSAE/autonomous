# main file
def main():
    # imports 
    # import CoreModels
    # import PathHelpers
    import TrackMethods

    # choose track
    track_name = "Austin"

    # read chosen track 
    print("IMPORTING TRACK")
    df = TrackMethods.importTrack(track_name, True)

    # println("INTERPOLATING")
    # df = TrackMethods.interpolateTrack(df, interval = 5) 

    # # create brackets
    # println("CREATING BRACKETS")
    # brackets = TrackMethods.getBrackets(df, 20)

    # velocity_range = [0.01, 8, 16, 24, 32, 40 ]# velocities in meters per second

    # println("COMPUTING OPTIMAL PATH")
    # start_node = brackets[1].NodeList[10]
    # start_node.cost = 0
    # println("starting inner distance: ",start_node.innerDistance)
    # println("starting outer distance: ", start_node.outerDistance)
    # mass = 50.0
    # μ = 0.7
    # # start_node = TrackMethods.belman_ford_path(track_name, df, velocity_range, brackets, start_node)
    # start_node = TrackMethods.optimise_path(track_name, df, brackets, start_node, mass, μ)
    # println("\nOPTIMAL PATH COMPUTED")


if __name__ == "__main__":
    main()