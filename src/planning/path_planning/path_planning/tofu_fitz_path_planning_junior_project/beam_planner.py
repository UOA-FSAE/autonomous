import numpy as np
import heapq  # For efficiently selecting the best beam
import numpy as np
import matplotlib.pyplot as plt
import time
# Constants
track_radius = 10  # Example track radius (for curvature force calculation)
beam_width = 3  # Beam width for beam search
longitudinal_slices = 4  # Number of slices (time steps)
slice_factor = 2  # Factor for slicing the track

# Globals-----------------------------------------------------------------------------------------------------------------------------------------------------------
# car parameters-----------------------------------------------------------------------------------------------------------------------------------------------------------
max_lateral_accel_ms2=45.0
max_accel_ms2=13.0
max_brake_ms2=18.0
top_speed_ms=95.0  # ≈ 342 km/h
mass = 225 # kg
F_max = max_accel_ms2 * mass # Newtons
k_a= 0.1 # air drag coefficient, make 0 if not known

# thresholds-----------------------------------------------------------------------------------------------------------------------------------------------------------
max_direction_delta_deg = 70
cone_distance_from_centre_points = 5
margin = 0.85
angle_close_enough_threshold = max_direction_delta_deg//2 # degrees

# state sample parameters-----------------------------------------------------------------------------------------------------------------------------------------------------------
state_speed_sample = 6
state_direction_sample = 9
lateral_slices = 6
# longitudinal_slices = 100 # approximate

corner_longitudinal_slices = 10 # approximate
straight_longitudinal_slices = 10 # approximate

def setup_paramaters():
    state_speeds = np.linspace(0, top_speed_ms, state_speed_sample).tolist()
    state_relative_angles_deg = np.linspace(-max_direction_delta_deg, max_direction_delta_deg, state_direction_sample).tolist()

    return state_speeds, state_relative_angles_deg


class State:
    def __init__(self, position, speed, direction):
        self.position = np.array(position)
        self.speed = speed
        self.direction = direction
        self.cost = float('inf')  # total accumulated cost
        self.previous_state = None

def generate_initial_states(centre, left, right, speeds, angles, slice_factor):
    initial_states = []
    sample = max(len(left) // slice_factor, 1)
    left = left[::sample]
    right = right[::sample]
    centre = centre[::sample]

    for i in range(len(centre)):
        for speed in speeds:
            for angle in angles:
                direction = np.degrees(np.arctan2(centre[i][1] - centre[i-1][1] if i > 0 else 0,
                                                  centre[i][0] - centre[i-1][0] if i > 0 else 1))
                state = State(position=centre[i], speed=speed, direction=direction)
                state.cost = 0  # Starting cost is 0
                initial_states.append(state)
    return initial_states

def find_closest_point_index(array, point):
    """Find index of point in array closest to given point."""
    dists = np.linalg.norm(array - point, axis=1)
    return np.argmin(dists)

def calculate_cost(state, next_state, left, right):
    # Direction change cost
    direction_change = abs(next_state.direction - state.direction)

    # Speed difference cost
    speed_diff = abs(next_state.speed - state.speed)

    # Track boundary proximity cost
    idx = find_closest_point_index((left + right) / 2, next_state.position)
    left_dist = np.linalg.norm(next_state.position - left[idx])
    right_dist = np.linalg.norm(next_state.position - right[idx])
    boundary_penalty = max(0, 5.0 - min(left_dist, right_dist))  # penalize <5m from edge

    # Centripetal force cost (simplified curvature)
    curvature_force = 0
    if next_state.speed > 0:
        radius = 20  # estimated average turn radius
        curvature_force = (next_state.speed ** 2) / radius

    # Total cost: Weighted sum
    return (0.5 * direction_change + 
            0.5 * speed_diff + 
            2.0 * boundary_penalty + 
            0.05 * curvature_force)

def expand_state(state, speeds, angles, left, right):
    next_states = []
    for speed in speeds:
        for angle in angles:
            new_direction = state.direction + angle
            direction_rad = np.radians(new_direction)
            delta = np.array([np.cos(direction_rad), np.sin(direction_rad)]) * speed * 0.1  # 0.1s step
            new_position = state.position + delta

            next_state = State(position=new_position, speed=speed, direction=new_direction)
            incremental_cost = calculate_cost(state, next_state, left, right)
            next_state.cost = state.cost + incremental_cost
            next_state.previous_state = state

            next_states.append(next_state)
    return next_states

def beam_search_path(centre, left, right, speeds, angles, beam_width, longitudinal_slices, slice_factor):
    beam = generate_initial_states(centre, left, right, speeds, angles, slice_factor)

    for _ in range(1, longitudinal_slices):
        next_beam = []
        for state in beam:
            next_states = expand_state(state, speeds, angles, left, right)
            next_beam.extend(next_states)

        # Prune beam to keep only best-scoring states
        beam = heapq.nsmallest(beam_width, next_beam, key=lambda x: x.cost)

    # Reconstruct the best path
    best_state = min(beam, key=lambda x: x.cost)
    path = []
    while best_state:
        path.append(best_state.position)
        best_state = best_state.previous_state
    return np.array(path[::-1])




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

# Visualization----------------------------------------------------------------------------------------------------------------------------------------------------------
def visualize(path, left, right):
    plt.figure(figsize=(12, 7))
    plt.scatter(left[:, 0], left[:, 1], color='blue', label='Blue Cones', s=2)
    plt.scatter(right[:, 0], right[:, 1], color='yellow', label='Yellow Cones', s=2)
    plt.plot(path[:, 0], path[:, 1], color='red', label='Stitched Path', linewidth=3)
    plt.axis('equal')
    plt.legend()
    plt.title('Dynamic Programming Optimal Path')
    plt.show()

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



if __name__ == "__main__":
    start_time = time.time()
    centre = generate_real_track('berlin_2018.txt')
    left, right = generate_cones(centre, offset=cone_distance_from_centre_points)
    left_margin, right_margin = generate_cones(centre, offset=cone_distance_from_centre_points-margin) # car is approx 1.7m in width, so half of that

    # Generation----------------------------------------------------------------------------------------------------------------------------------------------------------
    speeds, angles = setup_paramaters()
    optimal_path = beam_search_path(centre, left, right, speeds, angles, beam_width, longitudinal_slices, slice_factor)

    # Visualization & Testing----------------------------------------------------------------------------------------------------------------------------------------------------------
    
    end_time = time.time()
    optimal_path_time = estimate_lap_time_realistic(optimal_path)
    centreline_time = estimate_lap_time_realistic(centre)
    reduction = (1-(optimal_path_time/centreline_time))*100
    print(f"Execution time: {end_time - start_time:.3g}s")
    print(f"Optimal Path Lap Time: {optimal_path_time}s")
    print(f"Centreline Lap Time: {centreline_time}s")
    print(f"Lap time optimized by {reduction:.3g}%.")
    visualize(optimal_path, left, right)