import numpy as np
import matplotlib.pyplot as plt
import time
from typing import List, Tuple, Dict, Any, Optional
import heapq
from collections import deque
from scipy.spatial import KDTree


# Globals-----------------------------------------------------------------------------------------------------------------------------------------------------------
# car parameters-----------------------------------------------------------------------------------------------------------------------------------------------------------
max_lateral_accel_ms2=45.0
max_accel_ms2=13.0
max_brake_ms2=18.0
top_speed_ms=95.0  # ≈ 342 km/h
mass = 225 # kg
F_max = 1.5 * max_accel_ms2 * mass # Newtons
k_a= 0.02 # air drag coefficient, make 0 if not known

# thresholds-----------------------------------------------------------------------------------------------------------------------------------------------------------
max_direction_delta_deg = 90
cone_distance_from_centre_points = 5
margin = 0.85
angle_close_enough_threshold = max_direction_delta_deg//2 # degrees

beam_width = 1000

# state sample parameters-----------------------------------------------------------------------------------------------------------------------------------------------------------
state_speed_sample = 20
state_direction_sample = 20
lateral_slices = 10
# longitudinal_slices = 100 # approximate

corner_longitudinal_slices = 10 # approximate
straight_longitudinal_slices = 10 # approximate

# Tofu's Planning-----------------------------------------------------------------------------------------------------------------------------------------------------------

def setup_paramaters():
    state_speeds = np.linspace(0, top_speed_ms, state_speed_sample).tolist()
    state_relative_angles_deg = np.linspace(-max_direction_delta_deg, max_direction_delta_deg, state_direction_sample).tolist()

    return state_speeds, state_relative_angles_deg

def optimize_track(centre, left, right, segs):
    left_optimized = []
    right_optimized = []
    centre_optimized = []

    for seg in segs:
        start = seg['idx_start']
        end = seg['idx_end']
        slice_factor = corner_longitudinal_slices if seg['type'] == 'corner' else straight_longitudinal_slices
        sample = max((end - start) // slice_factor, 1)
        left_optimized.extend(left[start:end:sample])
        right_optimized.extend(right[start:end:sample])
        centre_optimized.extend(centre[start:end:sample])

    longitudinal_slices = len(centre_optimized)

    return np.array(centre_optimized), np.array(left_optimized), np.array(right_optimized), longitudinal_slices

class State:
    def __init__(self, position, speed, direction_deg, costs=None):
        self.position = position
        self.speed = speed
        self.direction = direction_deg
        self.costs = [] if costs is None else costs

class BeamState:
    def __init__(self, index, position, speed, direction, cost=0.0, parent=None):
        self.index = index
        self.position = position
        self.speed = speed
        self.direction = direction
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def spawn_states(centre, left, right, speeds, angles, longitudinal_slices):
    sample = max(len(left) // longitudinal_slices, 1)
    left = left[::sample]
    right = right[::sample]
    centre = centre[::sample]

    states = {}  # (i, j, k, m) -> State
    global angle_vectors
    angle_vectors = {angle: np.array([np.cos(np.radians(angle)), np.sin(np.radians(angle))]) for angle in angles}

    lateral_slices = 9  # Define or pass this as an argument if needed

    for i in range(longitudinal_slices):
        displacement = right[i] - left[i]
        gradient = displacement / (lateral_slices - 1)

        if i < (longitudinal_slices - 1):
            tangent_vec = centre[i + 1] - centre[i]
        else:
            tangent_vec = centre[i] - centre[i - 1]
        tangent = np.degrees(np.arctan2(tangent_vec[1], tangent_vec[0]))

        for j in range(lateral_slices):
            position = left[i] + gradient * j
            for k, speed in enumerate(speeds):
                for m, angle in enumerate(angles):
                    direction = tangent + angle
                    states[(i, j, k, m)] = State(position, speed, direction)
    return states, lateral_slices


def beam_search(centerline, lookahead_distance=5.0, beam_width=10, max_steps=100):
    class Node:
        def __init__(self, idx, cost, path):
            self.idx = idx
            self.cost = cost
            self.path = path  # list of centerline indices

    centerline = np.array(centerline)
    kdtree = KDTree(centerline)
    visited = set()

    # Start from first point
    start = Node(idx=0, cost=0.0, path=[0])
    beam = [start]

    for _ in range(max_steps):
        candidates = []
        for node in beam:
            current_point = centerline[node.idx]
            # Find candidates within lookahead radius
            nearby_indices = kdtree.query_ball_point(current_point, lookahead_distance)

            for nbr_idx in nearby_indices:
                if nbr_idx <= node.idx or nbr_idx in node.path:
                    continue  # avoid going backward or looping

                # Calculate angle deviation (penalize sharp turns)
                prev_idx = node.path[-1]
                if len(node.path) >= 2:
                    prev_point = centerline[node.path[-2]]
                    curr_point = centerline[prev_idx]
                    next_point = centerline[nbr_idx]

                    vec1 = curr_point - prev_point
                    vec2 = next_point - curr_point
                    angle = np.arccos(np.clip(np.dot(vec1, vec2) /
                                              (np.linalg.norm(vec1) * np.linalg.norm(vec2) + 1e-6), -1.0, 1.0))
                    angle_penalty = angle ** 2  # penalize higher angles quadratically
                else:
                    angle_penalty = 0

                distance = np.linalg.norm(centerline[nbr_idx] - centerline[prev_idx])
                new_cost = node.cost + distance + angle_penalty

                candidates.append(Node(idx=nbr_idx, cost=new_cost, path=node.path + [nbr_idx]))

        if not candidates:
            break

        # Sort by cost and keep top candidates
        candidates.sort(key=lambda n: n.cost)
        beam = candidates[:beam_width]

        # Early stopping if near the end
        if any(n.idx >= len(centerline) - 2 for n in beam):
            break

    # Choose best path
    best_node = min(beam, key=lambda n: n.cost)
    return [centerline[idx] for idx in best_node.path]
# Track Generation-----------------------------------------------------------------------------------------------------------------------------------------------------------
def generate_real_track(file_name):
    x = []
    y = []

    # Open and read the file
    with open(file_name, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            if len(parts) > 1:
                x.append(float(parts[0]))
                y.append(float(parts[1]))

    return np.array(np.column_stack((x, y)))

def generate_cones(centre_points, offset=1.5):
    """
    Generates blue (left) and yellow (right) cones offset perpendicularly from the centreline.
    
    Parameters:
    - centre_points: list of np.array([x, y])
    - offset: float, distance from centerline to each cone (in meters)
    
    Returns:
    - blue_cones: list of np.array([x, y]) to the left of the centreline
    - yellow_cones: list of np.array([x, y]) to the right of the centreline
    """
    blue_cones = []
    yellow_cones = []

    for i in range(1, len(centre_points)-1):
        prev_pt = centre_points[i-1]
        next_pt = centre_points[i+1]
        direction = next_pt - prev_pt
        direction /= np.linalg.norm(direction)  # Normalize
        perp = np.array([-direction[1], direction[0]])  # 90° rotation for left

        center = centre_points[i]
        blue_cones.append(center + offset * perp)     # Left (blue)
        yellow_cones.append(center - offset * perp)   # Right (yellow)

    return np.array(blue_cones), np.array(yellow_cones)


# Lap Time Testing----------------------------------------------------------------------------------------------------------------------------------------------------------
def estimate_lap_time_realistic(
    path_points,
    max_lateral_accel_ms2=45.0,
    max_accel_ms2=13.0,
    max_brake_ms2=18.0,
    top_speed_ms=95.0  # ≈ 342 km/h
):
    n = len(path_points)

    if n == 0:
        return 0

    distances = np.zeros(n - 1)
    curvatures = np.zeros(n - 2)
    speeds = np.zeros(n)

    # --- 1. Calculate distances between each point ---
    for i in range(n - 1):
        distances[i] = np.linalg.norm(path_points[i + 1] - path_points[i])

    # --- 2. Estimate curvature from angle between vectors ---
    for i in range(1, n - 1):
        vec1 = path_points[i] - path_points[i - 1]
        vec2 = path_points[i + 1] - path_points[i]
        angle = np.arccos(np.clip(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)), -1.0, 1.0))
        radius = np.linalg.norm(vec1) / angle if angle != 0 else np.inf
        curvatures[i - 1] = 1.0 / radius if radius != np.inf else 0.0

    # --- 3. Forward pass (acceleration-limited) ---
    speeds[0] = 0.0  # start from rest
    for i in range(n - 2):
        # Max speed from curvature
        corner_speed = min(top_speed_ms, np.sqrt(max_lateral_accel_ms2 / curvatures[i]) if curvatures[i] > 0 else top_speed_ms)
        # Accelerate toward that corner
        v_possible = np.sqrt(speeds[i]**2 + 2 * max_accel_ms2 * distances[i])
        speeds[i + 1] = min(v_possible, corner_speed)

    # --- 4. Backward pass (braking-limited) ---
    speeds[-1] = 0.0  # assume car stops at end
    for i in reversed(range(1, n - 1)):
        v_possible = np.sqrt(speeds[i + 1]**2 + 2 * max_brake_ms2 * distances[i])
        speeds[i] = min(speeds[i], v_possible)

    # --- 5. Compute time per segment ---
    total_time = 0.0
    for i in range(n - 1):
        if speeds[i] == 0 and speeds[i + 1] == 0:
            continue  # avoid div by zero
        avg_speed = (speeds[i] + speeds[i + 1]) / 2
        total_time += distances[i] / avg_speed

    return total_time

# Visualization----------------------------------------------------------------------------------------------------------------------------------------------------------
def visualize(path, centre, left, right, show_segs=False, show_debug=False, centre_optimized=[], left_optimized=[], right_optimized=[]):
    plt.figure(figsize=(12, 7))

    if show_debug:
        plt.plot(left_optimized[:, 0], left_optimized[:, 1], color='blue', label='Optimized Blue Cones', s=2)
        plt.plot(right_optimized[:, 0], right_optimized[:, 1], color='yellow', label='Optimized Yellow Cones', s=2)
        plt.plot(centre_optimized[:, 0], centre_optimized[:, 1], color='gray', label='Optimized Centreline', s=2)

    plt.scatter(left[:, 0], left[:, 1], color='blue', label='Blue Cones', s=2)
    plt.scatter(right[:, 0], right[:, 1], color='yellow', label='Yellow Cones', s=2)
    plt.plot(path[:, 0], path[:, 1], color='red', label='Stitched Path', linewidth=3)
    



    if show_segs:
        for seg in segs:
            idx_start = seg['idx_start']
            idx_end = seg['idx_end']
            seg_points = centre[idx_start:idx_end+1]  # +1 because slicing is exclusive
            
            if seg['type'] == 'corner':
                if seg['turn_dir'] == 'R':
                    color = 'red'
                elif seg['turn_dir'] == 'L':
                    color = 'cyan'
                else:
                    color = 'gray'
            else:
                color = 'gray'
            
            plt.plot(seg_points[:, 0], seg_points[:, 1], color=color, linewidth=2)


    plt.axis('equal')
    plt.legend()
    plt.title('Dynamic Programming Optimal Path')
    plt.show()


# Main----------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    start_time = time.time()

    # Generation----------------------------------------------------------------------------------------------------------------------------------------------------------
    centre = generate_real_track('berlin_2018.txt')
    left, right = generate_cones(centre, offset=cone_distance_from_centre_points)
    left_margin, right_margin = generate_cones(centre, offset=cone_distance_from_centre_points-margin) # car is approx 1.7m in width, so half of that

    speeds, angles = setup_paramaters()
    # segs = identify_segs(centre, window_length=20, max_R_for_corner=61)

    # centre_optimized, left_optimized, right_optimized, longitudinal_slices = optimize_track(centre, left_margin, right_margin, segs)
    # states = spawn_states(centre_optimized, left_optimized, right_optimized, state_speeds, state_relative_angles_deg, longitudinal_slices)
    longitudinal_slices = 100
    states, lateral_slices = spawn_states(centre, left, right, speeds, angles, longitudinal_slices)
    optimal_path = beam_search(centre)
    
    # Visualization & Testing----------------------------------------------------------------------------------------------------------------------------------------------------------
    
    end_time = time.time()
    optimal_path_time = estimate_lap_time_realistic(optimal_path)
    centreline_time = estimate_lap_time_realistic(centre)
    reduction = (1-(optimal_path_time/centreline_time))*100
    print(f"Execution time: {end_time - start_time:.3g}s")
    print(f"Optimal Path Lap Time: {optimal_path_time}s")
    print(f"Centreline Lap Time: {centreline_time}s")
    print(f"Lap time optimized by {reduction:.3g}%.")
    visualize(optimal_path, centre, left, right, show_segs=False, show_debug=False)#, centre_optimized=centre_optimized, left_optimized=left_optimized, right_optimized=right_optimized)
