
import rclpy
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# Purely straight based approach

# NEXT STEPS:
# 1. Join broken straights into one corner. <<<<<<<
# 2. Classify corners based on their curvature direction, then the type of corner it is.



straights_rad_threshold = 17 *math.pi/180 #10 degrees
minimum_straight_length = 3 #number of centre_points that a straight should have as a minimum

min_rank_fraction = 0.1
min_straight_length = 75 #in metres based on VERY approximate calculation outlined in my workbook


class Straight():
    def __init__(self, length=0, index=0):
        self.length = length
        self.index = index
    

#path planning

def tectonic_track(centre_points):
    # Breaks up the track into what can be considered a straight based on the straights threshold. This effectively
    # breaks up corners a lot, while actual straights remain one piece. This is kinda like tectonic plates so I used
    # it as an excuse to give this function a cooler name :)

    #simple approach for now please improve using Siva's curvature equation or some other way:

    displacement = centre_points[1] - centre_points[0]
    prev_angle_rad = np.arctan2(displacement[1], displacement[0])

    no_of_centre_points = len(centre_points)

    straights = [[centre_points[0], centre_points[1]]]
    current_straight_and_corner = 0


    for i in range(2, len(centre_points)):

        displacement = centre_points[i] - centre_points[i-1]
        angle_rad = np.arctan2(displacement[1], displacement[0])

        if abs(abs(angle_rad) - abs(prev_angle_rad)) <= straights_rad_threshold:
            #angular displacement is small enough to be considered part of the current straight
            #--------------------------------------------you don't actually need to add EVERY centre point as its literally a straight. Can optimise further to just have two points to represent the straight
            straights[current_straight_and_corner].append(centre_points[i])
        else:
            while (i < (no_of_centre_points - 1)) and abs(abs(angle_rad) - abs(prev_angle_rad)) > straights_rad_threshold:
                current_straight_and_corner += 1
                straights.append([])
                straights[current_straight_and_corner].append(centre_points[i])

                i += 1
                prev_angle_rad = angle_rad
                displacement = centre_points[i] - centre_points[i-1]
                angle_rad = np.arctan2(displacement[1], displacement[0])
                
    return straights

    #angle_deg = np.degrees(angle_rad)
    
def rank_straights(straights):
    # Ranks straights based on their lengths
    # COMBINE BOTH LOOPS FOR BETTER PERFORMNACE 

    length = len(straights)

    straightened_straights = []
    sorted_straightened_straights = []
    for i in range(length):
        if len(straights[i]) > 1:
            straightened_straights.append(Straight(np.linalg.norm(straights[i][-1] - straights[i][0]), i))
            if straightened_straights[-1].length >= min_straight_length:
                sorted_straightened_straights.append(straightened_straights[-1])
        else:
            straightened_straights.append(Straight(0, i))

    sorted_straightened_straights = sorted(sorted_straightened_straights, key=lambda straight: straight.length, reverse=True)
    ranked_straights = [straights[straightened_straight.index] for straightened_straight in sorted_straightened_straights]
    
    #-------------------------------------------------------------------------------------------------
    corners = []
    i = 0
    while i < length:
        if straightened_straights[i].length < min_straight_length:
            corners.append(straights[i].copy())
            i += 1
            while (i < length) and (straightened_straights[i].length < min_straight_length):
                corners[-1].extend(straights[i])
                i += 1
        else:
            i += 1
            


    return ranked_straights, corners


def classify_corners(ranked_corners):
    #TODO?
    pass

#plots for visualisation
def example_plot(ranked_straights, corners, centre_points):
    plt.figure(figsize=(10, 6))

    # Color gradients for heatmap-like ranking
    from matplotlib.cm import get_cmap
    corner_cmap = get_cmap("cool")   # blue to red for corners
    straight_cmap = get_cmap("RdYlGn")  # red to green for straights

    # Add all points for reference
    plt.scatter(centre_points[:, 0], centre_points[:, 1], color='black', zorder=5, s=1)

    # Plot straights
    for i, straight in enumerate(ranked_straights):
        color = straight_cmap(i / max(len(ranked_straights)-1, 1))
        pts = np.array(straight)
        if (len(straight) > 1):
            plt.plot(pts[:, 0], pts[:, 1], '-', c=color, linewidth=10, label=f"Straight {i}" if i == 0 else "")
            # plt.title("Ranked Straights (Red→Green) and Corners (Blue→Red)")
            # plt.axis("equal")
            # plt.grid(True)
            # plt.legend()
            # plt.show()

    for i, corner in enumerate(corners):
        color = corner_cmap(i / max(len(corners)-1, 1))
        pts = np.array(corner)
        if (len(corner) > 1):
            plt.plot(pts[:, 0], pts[:, 1], '-', c=color, linewidth=10, linestyle='-', label=f"Corner {i}" if i == 0 else "")
            # plt.title("Ranked Straights (Red→Green) and Corners (Blue→Red)")
            # plt.axis("equal")
            # plt.grid(True)
            # plt.legend()
            # plt.show()


    plt.title("Ranked Straights (Red→Green) and Corners (Blue→Red)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()

#test generation
def generate_large_track():
    points = []

    # Straight segment
    for i in range(200):
        points.append([i * 0.5, 0])

    # Smooth right turn (quarter circle)
    radius = 50
    for theta in np.linspace(0, np.pi/2, 150):
        x = 100 + radius * np.sin(theta)
        y = radius * (1 - np.cos(theta))
        points.append([x, y])

    # Second straight
    for i in range(200):
        points.append([200 + i * 0.5, radius])

    # Smooth left turn (quarter circle)
    for theta in np.linspace(0, -np.pi/2, 150):
        x = 300 + radius * np.sin(theta)
        y = radius + radius * (1 - np.cos(theta))
        points.append([x, y])

    # Third straight
    for i in range(200):
        points.append([400 + i * 0.5, 2 * radius])

    # Final curve back to start
    for theta in np.linspace(np.pi/2, 2 * np.pi, 150):
        x = 500 + radius * np.cos(theta)
        y = 2 * radius + radius * np.sin(theta)
        points.append([x, y])

    return np.array(points)

def generate_oval_track(num_points=1000, straight_length=200, radius=50):
    # Half the points for straights, half for curves
    num_curve_points = num_points // 2
    num_straight_points = num_points - num_curve_points

    # First straight (bottom)
    straight1_x = np.linspace(-straight_length / 2, straight_length / 2, num_straight_points // 2)
    straight1_y = np.full_like(straight1_x, -radius)

    # Right curve (bottom to top, right side)
    theta1 = np.linspace(-np.pi/2, np.pi/2, num_curve_points // 2)
    curve1_x = radius * np.cos(theta1) + straight_length / 2
    curve1_y = radius * np.sin(theta1)

    # Second straight (top)
    straight2_x = np.linspace(straight_length / 2, -straight_length / 2, num_straight_points // 2)
    straight2_y = np.full_like(straight2_x, radius)

    # Left curve (top to bottom, left side)
    theta2 = np.linspace(np.pi/2, 3*np.pi/2, num_curve_points // 2)
    curve2_x = radius * np.cos(theta2) - straight_length / 2
    curve2_y = radius * np.sin(theta2)

    # Combine all parts
    x = np.concatenate([straight1_x, curve1_x, straight2_x, curve2_x])
    y = np.concatenate([straight1_y, curve1_y, straight2_y, curve2_y])

    return np.stack((x, y), axis=1)

def generate_treyarch_track(num_points=1000, seed=42):
    np.random.seed(seed)

    # Base polar track
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

    # Generate radial distance with wavy randomness
    base_radius = 100
    radius_variation = (
        20 * np.sin(3 * angles) +
        10 * np.sin(7 * angles + np.pi / 4) +
        5 * np.random.randn(num_points)
    )
    radii = base_radius + radius_variation

    # Convert polar to cartesian
    x = radii * np.cos(angles)
    y = radii * np.sin(angles)

    return np.stack((x, y), axis=1)

def generate_smooth_closed_track(num_points=1000, num_control_points=10, radius=200, seed=42):
    np.random.seed(seed)

    # Create random control points around a circle
    angles = np.linspace(0, 2 * np.pi, num_control_points, endpoint=False)
    x = radius * np.cos(angles) + np.random.uniform(-20, 20, num_control_points)
    y = radius * np.sin(angles) + np.random.uniform(-20, 20, num_control_points)

    # Close the loop
    x = np.append(x, x[0])
    y = np.append(y, y[0])

    # Interpolate with a B-spline
    tck, u = splprep([x, y], s=0, per=True)

    # Generate evenly spaced points along the spline
    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine = splev(u_fine, tck)

    centre_points = np.stack((x_fine, y_fine), axis=1)
    return centre_points

def generate_long_straight_track(num_points=1000, seed=42):
    np.random.seed(seed)

    # Define control points for long straights and smooth corners (scaled)
    track_outline = []

    # All distances scaled to 100m+ per straight segment
    segments = [
        ((0, 0), (200, 0)),       # 200m straight
        ((200, 0), (250, 100)),   # ~111m corner segment
        ((250, 100), (100, 250)), # ~212m straight diagonal
        ((100, 250), (-20, 300)), # ~134m curve
        ((-20, 300), (-200, 150)),# ~250m straight
        ((-200, 150), (-220, 50)),# ~104m curve
        ((-220, 50), (0, 0))      # Closing back (about 250m)
    ]

    for start, end in segments:
        # Add intermediate points between start and end
        p0 = np.array(start)
        p1 = np.array(end)
        for i in range(5):
            t = i / 4
            point = (1 - t) * p0 + t * p1
            point += np.random.normal(scale=2, size=2)  # light noise
            track_outline.append(point)

    track_outline = np.array(track_outline)

    # Close the loop
    track_outline = np.vstack([track_outline, track_outline[0]])

    # B-spline interpolation for smooth loop
    tck, _ = splprep(track_outline.T, s=0, per=True)
    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine = splev(u_fine, tck)

    centre_points = np.stack((x_fine, y_fine), axis=1)
    return centre_points

def main(args=None):
    # centre_points = generate_oval_track()
    # centre_points = generate_smooth_closed_track(seed=512) # change seed to try a different circle-ish track
    centre_points = generate_long_straight_track(seed=41214) # change seed to try a different circle-ish track

    straights = tectonic_track(centre_points)
    # ranked_straights, corners = rank_straights(straights)
    ranked_straights, corners = rank_straights(straights)
    # corners = []

    example_plot(ranked_straights, corners, centre_points)




main()

