import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

# Generators:
def generate_oval_track(num_points=1000, straight_length=200, radius=50):
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

def generate_weirdo_oval_track():

    # Parameters
    straight_length = 30
    curve_radius = 10
    num_curve_points = 20  # per half circle
    num_straight_points = 20  # per straight

    # Step 1: Create top straight
    top_straight = np.linspace(-straight_length/2, straight_length/2, num_straight_points)
    top_points = np.array([top_straight, np.full_like(top_straight, curve_radius)]).T

    # Step 2: Create right half circle (top to bottom)
    theta_right = np.linspace(np.pi/2, -np.pi/2, num_curve_points)
    right_curve = np.array([
        straight_length/2 + curve_radius * np.cos(theta_right),
        curve_radius * np.sin(theta_right)
    ]).T

    # Step 3: Create bottom straight
    bottom_straight = np.linspace(straight_length/2, -straight_length/2, num_straight_points)
    bottom_points = np.array([bottom_straight, np.full_like(bottom_straight, -curve_radius)]).T

    # Step 4: Create left half circle (bottom to top)
    theta_left = np.linspace(np.pi/2, -np.pi/2, num_curve_points)  # Reverse the angles here
    left_curve = np.array([
        -straight_length/2 + curve_radius * np.cos(theta_left),  # Negative x to flip
        -curve_radius * np.sin(theta_left)  # Flip the curve direction (inverted sin)
    ]).T

    # Step 5: Combine into full centre line
    centre_points = np.vstack((top_points, right_curve, bottom_points, left_curve))

    return centre_points

def tester_oval(no_of_points=1000, extra_bs=False):
    # Generate oval track (use the `generate_oval_track` or `generate_weirdo_oval_track` function)
    centre_points = generate_oval_track(no_of_points)

    # Step 6: Generate cones
    cones = []
    track_width = 2

    for i in range(len(centre_points)):
        p = centre_points[i]
        p_next = centre_points[(i + 1) % len(centre_points)]  # wrap around

        # Direction vector
        direction = p_next - p

        # Skip identical points (to avoid zero direction vector)
        if np.all(direction == 0):
            continue

        # Normalize direction vector
        direction = direction / np.linalg.norm(direction)

        # Get left and right perpendicular vectors
        left = np.array([-direction[1], direction[0]])
        right = -left

        # Assign color based on direction
        cones.append(((p + track_width * left).tolist(), 'y'))  # yellow
        cones.append(((p + track_width * right).tolist(), 'b'))  # blue

    cone_coords = np.array([c[0] for c in cones])


    if extra_bs:
        # Step 8: Delaunay
        tri = Delaunay(cone_coords)

        # Step 9: Plotting
        plt.figure(figsize=(12, 7))
        plt.triplot(cone_coords[:, 0], cone_coords[:, 1], tri.simplices.copy(), color='gray')
        plt.scatter([c[0][0] for c in cones if c[1] == 'y'], [c[0][1] for c in cones if c[1] == 'y'], color='yellow', label='Left Cones')
        plt.scatter([c[0][0] for c in cones if c[1] == 'b'], [c[0][1] for c in cones if c[1] == 'b'], color='blue', label='Right Cones')
        plt.plot(centre_points[:, 0], centre_points[:, 1], 'r--', label='Centre Line')
        plt.axis('equal')
        plt.legend()
        plt.title('Pill-shaped Track with Centre Line and Cones')
        plt.show()

        # Print out cone coordinates and colors
        for cone in cones:
            print(cone)
    
    return cones
