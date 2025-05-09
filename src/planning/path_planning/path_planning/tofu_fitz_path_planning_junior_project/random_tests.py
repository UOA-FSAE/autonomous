import numpy as np
import matplotlib.pyplot as plt
from scipy.special import fresnel
from pyclothoids import Clothoid
from scipy.optimize import minimize_scalar

def first_one():
    
    # Total arc length
    L = 10

    # Number of points
    n_points = 1000

    # s: arc length parameter
    s = np.linspace(0, L, n_points)

    # Scale factor (a) for curvature κ(s) = s / a²
    a = 1.0
    b = 2.0

    # Compute scaled Fresnel integrals
    t = s / (np.sqrt(2 * np.pi) * a)
    S, C = fresnel(t)

    # Coordinates of the spiral
    x = a * np.sqrt(2 * np.pi) * C
    y = a * np.sqrt(2 * np.pi) * S

    # Plot the Euler spiral
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, label='Euler Spiral A')
    # Compute scaled Fresnel integrals
    t = s / (np.sqrt(2 * np.pi) * b)
    S, C = fresnel(t)

    # Coordinates of the spiral
    x = b * np.sqrt(2 * np.pi) * C
    y = b * np.sqrt(2 * np.pi) * S
    plt.plot(x, y, label='Euler Spiral B')




    plt.axis('equal')
    plt.grid(True)
    plt.title('Euler (Clothoid) Spiral')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()

def second_one():
    def euler_spiral(L=10, n_points=1000, a=1.0, theta_deg=0):
        # s: arc length
        s = np.linspace(0, L, n_points)
        
        # Fresnel parameter
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        # Base coordinates of Euler spiral
        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S

        # Convert angle to radians
        theta = np.radians(theta_deg)

        # Apply rotation
        x_rot = x * np.cos(theta) - y * np.sin(theta)
        y_rot = x * np.sin(theta) + y * np.cos(theta)

        return x_rot, y_rot

    # Change this angle to control starting direction (0 = right, 90 = up, etc.)

    # Get spiral
    x, y = euler_spiral(theta_deg=180)
    x2, y2 = euler_spiral(theta_deg=45)
    x3, y3 = euler_spiral(a=0.5, theta_deg=45)

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, label=f'Euler Spiral @ {180}°')
    plt.plot(x2, y2, label=f'Euler Spiral @ {45}°')
    plt.plot(x3, y3, label=f'Euler Spiral @ {45}°')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Euler (Clothoid) Spiral with Custom Starting Direction')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()

def third_one():


    def euler_spiral(L=10, n_points=1000, a=1.0, theta_deg=0, flip=False):
        # s: arc length
        s = np.linspace(0, L, n_points)

        # Fresnel parameter
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        # Base coordinates
        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S

        # Flip the spiral across the tangent (horizontal axis)
        if flip:
            y = -y  # This reverses curvature (anticlockwise <-> clockwise)

        # Convert angle to radians
        theta = np.radians(theta_deg)

        # Apply rotation
        x_rot = x * np.cos(theta) - y * np.sin(theta)
        y_rot = x * np.sin(theta) + y * np.cos(theta)

        return x_rot, y_rot

    # === Parameters ===
    theta_deg = 90   # Start direction: 90° = up
    flip = True      # Flip: False = anticlockwise, True = clockwise

    # Generate spiral
    x, y = euler_spiral(theta_deg=theta_deg, flip=flip)

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, label=f'Euler Spiral @ {theta_deg}° {"(clockwise)" if flip else "(anticlockwise)"}')
    plt.axis('equal')
    plt.grid(True)
    plt.title('Euler (Clothoid) Spiral with Flip and Heading Control')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()


def fourth_one():

    def euler_spiral(a=1.0, theta_start_deg=0, delta_theta_deg=60, n_points=1000, flip=False):
        # Convert to radians
        theta_start = np.radians(theta_start_deg)
        delta_theta = np.radians(delta_theta_deg)

        # Flip direction if needed (negate angle change)
        if flip:
            delta_theta = -delta_theta

        # Compute arc length required for this delta_theta
        s_stop = np.sqrt(4 * a**2 * abs(delta_theta))
        s = np.linspace(0, s_stop, n_points)

        # Normalized Fresnel parameter
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        # Base spiral (unrotated)
        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S

        # If flipped, mirror across x-axis
        if flip:
            y = -y

        # Rotate spiral to desired starting heading
        x_rot = x * np.cos(theta_start) - y * np.sin(theta_start)
        y_rot = x * np.sin(theta_start) + y * np.cos(theta_start)

        return x_rot, y_rot

    # === Example Test ===
    theta_start_deg = 90          # Start pointing up (relative to positive x axis)
    delta_theta_deg = 90          # Turn left 90 deg (so end pointing left) reltaive to theta_start deg
    flip = True                  # Counterclockwise turn

    x, y = euler_spiral(theta_start_deg=theta_start_deg,
                        delta_theta_deg=delta_theta_deg,
                        flip=flip)

    # Plot
    plt.figure(figsize=(8, 6))
    plt.plot(x, y, label=f"Start: {theta_start_deg}°, Turn: {delta_theta_deg}° {'CW' if flip else 'CCW'}")
    plt.axis('equal')
    plt.grid(True)
    plt.title('Euler Spiral (Relative Heading Change)')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.show()

def fifth_one():

    from scipy.optimize import minimize_scalar

    def generate_unit_euler(delta_theta_rad, n_points=500):
        """Generate a unit Euler spiral for a given delta_theta (relative)."""
        a = 1.0  # unit scale
        s_stop = np.sqrt(4 * a**2 * abs(delta_theta_rad))
        s = np.linspace(0, s_stop, n_points)
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S
        if delta_theta_rad < 0:
            y = -y  # flip for clockwise
        return x, y

    def transform_spiral(x, y, x_start, y_start, theta_start_rad, scale):
        """Scale, rotate, and translate the spiral to start from (x_start, y_start) with heading theta_start."""
        x_scaled = x * scale
        y_scaled = y * scale
        x_rot = x_scaled * np.cos(theta_start_rad) - y_scaled * np.sin(theta_start_rad)
        y_rot = x_scaled * np.sin(theta_start_rad) + y_scaled * np.cos(theta_start_rad)
        return x_rot + x_start, y_rot + y_start

    def connect_points_euler_relative(A, B, theta_start_deg, delta_theta_deg, n_points=500, tol=1e-3):
        # Convert to radians
        theta_start = np.radians(theta_start_deg)
        delta_theta = np.radians(delta_theta_deg)

        # Generate unit Euler spiral with relative angle change
        x_base, y_base = generate_unit_euler(delta_theta, n_points)

        # Optimize scale
        def objective(scale):
            x_trans, y_trans = transform_spiral(x_base, y_base, A[0], A[1], theta_start, scale)
            dx = x_trans[-1] - B[0]
            dy = y_trans[-1] - B[1]
            return dx**2 + dy**2

        result = minimize_scalar(objective, bounds=(0.01, 100), method='bounded')
        best_scale = result.x

        # Reconstruct final spiral
        x_final, y_final = transform_spiral(x_base, y_base, A[0], A[1], theta_start, best_scale)

        return x_final, y_final, best_scale, result.fun < tol

    # === Example ===
    A = (0, 0)
    B = (25, 35)
    C = (45, 87)
    theta_start_deg = 0           # Start facing up
    delta_theta_deg = 70          # Curve right (CW) to face left
    final_theta_deg = 20
    # x, y, scale_used, success = connect_points_euler_relative(A, B, theta_start_deg, delta_theta_deg)
    # BB = (x[-1], y[-1])
    x2, y2, scale_used2, success = connect_points_euler_relative(A, C, theta_start_deg, delta_theta_deg + final_theta_deg)


    # === Plot ===
    plt.figure(figsize=(8, 6))
    # plt.plot(x, y, label=f'Euler Spiral (Scale: {scale_used:.2f})')
    plt.plot(x2, y2, label=f'Euler Spiral (Scale: {scale_used2:.2f})')
    plt.plot(*A, 'go', label='Start A')
    plt.plot(*B, 'ro', label='Mid B')
    plt.plot(*C, 'bo', label='End C')
    plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
            scale=5, color='green', label='Heading A')
    theta_end_deg = theta_start_deg + delta_theta_deg
    plt.quiver(*B, np.cos(np.radians(theta_end_deg)), np.sin(np.radians(theta_end_deg)),
            scale=5, color='red', label='Heading B')
    final_theta_deg += theta_end_deg
    plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
            scale=5, color='blue', label='Heading C')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title('Euler Spiral Connecting Two Points with Relative Angle')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()

    print("Success:", success)

def sixth_one():

    from pyclothoids import Clothoid
    def connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg, n_points=500):
        """
        Connects two points A and B with specified start and end headings using a clothoid.

        Parameters:
        - A: Tuple (x0, y0) representing the start point.
        - B: Tuple (x1, y1) representing the end point.
        - theta_start_deg: Start heading in degrees.
        - theta_end_deg: End heading in degrees.
        - n_points: Number of points to sample along the clothoid.

        Returns:
        - x_vals: X coordinates of the sampled points.
        - y_vals: Y coordinates of the sampled points.
        - clothoid: The Clothoid object representing the curve.
        """
        # Convert angles from degrees to radians
        theta_start_rad = np.radians(theta_start_deg)
        theta_end_rad = np.radians(theta_end_deg)

        # Create the clothoid that connects the two points with specified headings
        clothoid = Clothoid.G1Hermite(A[0], A[1], theta_start_rad, B[0], B[1], theta_end_rad)

        # Sample points along the clothoid
        x_vals, y_vals = clothoid.SampleXY(n_points)

        return x_vals, y_vals, clothoid

    from scipy.optimize import minimize_scalar

    def generate_unit_euler(delta_theta_rad, n_points=500):
        """Generate a unit Euler spiral for a given delta_theta (relative)."""
        a = 1.0  # unit scale
        s_stop = np.sqrt(4 * a**2 * abs(delta_theta_rad))
        s = np.linspace(0, s_stop, n_points)
        t = s / (np.sqrt(2 * np.pi) * a)
        S, C = fresnel(t)

        scale = a * np.sqrt(2 * np.pi)
        x = scale * C
        y = scale * S
        if delta_theta_rad < 0:
            y = -y  # flip for clockwise
        return x, y

    def transform_spiral(x, y, x_start, y_start, theta_start_rad, scale):
        """Scale, rotate, and translate the spiral to start from (x_start, y_start) with heading theta_start."""
        x_scaled = x * scale
        y_scaled = y * scale
        x_rot = x_scaled * np.cos(theta_start_rad) - y_scaled * np.sin(theta_start_rad)
        y_rot = x_scaled * np.sin(theta_start_rad) + y_scaled * np.cos(theta_start_rad)
        return x_rot + x_start, y_rot + y_start

    def check_if_close(spiral, apex_points, threshold_distance):
        length = len(spiral)
        mid_index = length //2
        for i in range(mid_index):
            upper = min(mid_index + i, length-1)
            lower = max(mid_index - i - 1, 0)
            for apoint in apex_points:
                if (np.linalg.norm(spiral[upper] - apoint) <= threshold_distance) or \
                    (np.linalg.norm(spiral[lower] - apoint) <= threshold_distance):
                    return True
        
        return False

    def find_precut_direct(start_point, end_point, apex_points, start_heading_deg, desired_end_heading_deg, threshold_distance=5, end_heading_step_deg=10):
        x, y, cloth = connect_points_with_clothoid(start_point, end_point, start_heading_deg, desired_end_heading_deg, n_points=30)


    def find_useful_part(x, y, desired_end_heading_deg, threshold_deg=5):
        lower_limit = ((desired_end_heading_deg - threshold_deg) + 360) % 360
        upper_limit = ((desired_end_heading_deg + threshold_deg) + 360) % 360
        points = np.column_stack((x, y))
        for i in range(len(points) - 1, 0, -1):
            vec = points[i] - points[i-1]
            heading = np.degrees(np.arctan2(vec[1], vec[0]))

            if heading >= lower_limit and heading <= upper_limit:
                return points[:i+1]
        
        return None


    # Example usage
    if __name__ == "__main__":


        # Define start and end points with headings
        A = (0, 0)             # Start point
        B = (25, 35)           # End point
        C = (28, 87)
        D = (60, 87)
        theta_start_deg = 0   # Start heading in degrees
        theta_end_deg = 70    # End heading in degrees
        final_theta_deg = 90
        optimal_final_theta_deg = -20

        proportion = 0.5
        
        if True:
            pass
            # AVERAGE_BC = ((B[0]+C[0])/2,(B[1]+C[1])/2)
            # BBC = (25, 87) 
            
            # average_theta_deg = (final_theta_deg + theta_end_deg) / 2

            # # Generate the clothoid, first half, second half style
            # x_vals, y_vals, clothoid = connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg)
            # BB = (x_vals[-1], y_vals[-1])
            # x2, y2, clothoid2 = connect_points_with_clothoid(C, B, final_theta_deg+180, theta_end_deg + 180)

            # # generatre clothoid directly
            # x3, y3, clothoid3 = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg, n_points=1000)

            # #average clothoid of the two:
            # x_halves = x_vals + x2
            # y_halves = y_vals + y2

            # average_x = [(x_halves[i]+x3[i])/2 for i in range(len(x_halves))]
            # average_y = [(y_halves[i]+y3[i])/2 for i in range(len(y_halves))]

            # #AIM AT AVERAGE:
            # # averx, avery, clothoid3 = connect_points_with_clothoid(A, AVERAGE_BC, theta_start_deg, average_theta_deg)
            # # divergex, divergey, clothoid4 = connect_points_with_clothoid(A, C, theta_start_deg, theta_end_deg)
            # # huggx, huggy, clothoid5 = connect_points_with_clothoid(A, BBC, theta_start_deg, final_theta_deg)

            # CBB = (30, 35) 
            # apexendx, apexendy, clothoid5 = connect_points_with_clothoid(A, CBB, theta_start_deg, theta_end_deg)
            # apexendx2, apexendy2, clothoid5 = connect_points_with_clothoid(C, CBB, (final_theta_deg + 180) % 360, (theta_end_deg + 180) % 360)
            

            # # axex, axey, scale_used, success = connect_points_euler_relative(A, B, theta_start_deg, theta_end_deg - theta_start_deg)
            # # axex2, axey2, scale_used, success = connect_points_euler_relative(C, B, 270, -20)
            # axex2, axey2, scale_used, success = connect_points_euler_relative(CBB, C, theta_end_deg, final_theta_deg-theta_end_deg)

            # # generatre clothoid directly, but changfes ending heading so it touches tangent
            # x4, y4, clothoid3 = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg-70)
            # testx, testy, clothoid3 = connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg)
            # test2x, test2y, clothoid3 = connect_points_with_clothoid(A, B, theta_start_deg, 0)
        
        xfh, yfh, clothoidfh = connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg)
        xsh, ysh, clothoidsh = connect_points_with_clothoid(B, C, theta_end_deg, final_theta_deg)
        x_direct, y_direct, clothoid_direct = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg, n_points=1000)

        x_full = xfh + xsh
        y_full = yfh + ysh

        x_complete = [(x_full[i] * proportion + x_direct[i] * (1-proportion)) for i in range(len(x_direct))]
        y_complete = [(y_full[i] * proportion + y_direct[i] * (1-proportion)) for i in range(len(x_direct))]

        x_direct2, y_direct2, clothoid_direct = connect_points_with_clothoid(A, D, theta_start_deg, optimal_final_theta_deg, n_points=20)
        optimal_path = find_useful_part(x_direct2, y_direct2, final_theta_deg)



        # Plot the clothoid
        plt.figure(figsize=(12, 9))
        plt.plot(x_full, y_full, label='Partitioned')
        plt.plot(x_direct, y_direct, label='Direct')
        plt.plot(x_complete, y_complete, label='Proportioned')
        plt.plot(x_direct2, y_direct2, label='Precut Optimal')
        plt.plot(optimal_path[:, 0], optimal_path[:, 1], color='purple', label='Optimal Path', linewidth=3)
        plt.plot(*A, 'go', label='Start Point A')
        plt.plot(*B, 'ro', label='Apex Point B')
        plt.plot(*C, 'bo', label='End Point C')

        # Plot start and end headings
        plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
                scale=5, color='green', label='Start Heading')
        plt.quiver(*B, np.cos(np.radians(theta_end_deg)), np.sin(np.radians(theta_end_deg)),
                scale=5, color='red', label='Apex Heading')
        plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
                scale=5, color='blue', label='End Heading')

        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title('Clothoid Connecting Two Points with Specified Headings')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

        # Print clothoid parameters
        # print("Clothoid Parameters:")
        # print(f"  Length (L): {clothoid.L}")
        # print(f"  Start Curvature (kappa0): {clothoid.KappaStart}")
        # print(f"  End Curvature (kappa1): {clothoid.KappaEnd}")
        # print(f"  Curvature Rate (dk): {clothoid.dk}")


    # fifth_one()

def seventh_one():

    from pyclothoids import Clothoid
    def connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg, n_points=500):
        """
        Connects two points A and B with specified start and end headings using a clothoid.

        Parameters:
        - A: Tuple (x0, y0) representing the start point.
        - B: Tuple (x1, y1) representing the end point.
        - theta_start_deg: Start heading in degrees.
        - theta_end_deg: End heading in degrees.
        - n_points: Number of points to sample along the clothoid.

        Returns:
        - x_vals: X coordinates of the sampled points.
        - y_vals: Y coordinates of the sampled points.
        - clothoid: The Clothoid object representing the curve.
        """
        # Convert angles from degrees to radians
        theta_start_rad = np.radians(theta_start_deg)
        theta_end_rad = np.radians(theta_end_deg)

        # Create the clothoid that connects the two points with specified headings
        clothoid = Clothoid.G1Hermite(A[0], A[1], theta_start_rad, B[0], B[1], theta_end_rad)

        # Sample points along the clothoid
        x_vals, y_vals = clothoid.SampleXY(n_points)
        points = np.column_stack((x_vals, y_vals))  # Combine into shape (n_points, 2)

        return points
    
    def check_if_close(spiral, apex_points, threshold_distance):
        length = len(spiral)
        mid_index = length //2
        for i in range(mid_index):
            upper = min(mid_index + i, length-1)
            lower = max(mid_index - i - 1, 0)
            for apoint in apex_points:
                if (np.linalg.norm(spiral[upper] - apoint) <= threshold_distance) or \
                    (np.linalg.norm(spiral[lower] - apoint) <= threshold_distance):
                    return True
        
        return False

    def find_precut_direct_angle(start_point, end_point, apex_points, start_heading_deg, desired_end_heading_deg, threshold_distance=5, end_heading_step_deg=10, sample=20):
        # plt.figure(figsize=(12, 9))
        # plt.plot(*A, 'go', label='Start Point A')
        # plt.plot(*B, 'ro', label='Apex Point B')
        # plt.plot(*C, 'bo', label='End Point C')
        # plt.plot(*D, 'bo', label='End Point C')
        # plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
        #         scale=5, color='green', label='Start Heading')
        # plt.quiver(*B, np.cos(np.radians(theta_end_deg)), np.sin(np.radians(theta_end_deg)),
        #         scale=5, color='red', label='Apex Heading')
        # plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
        #         scale=5, color='blue', label='End Heading')
        # plt.quiver(*D, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
        #         scale=5, color='blue', label='End Heading')
        # plt.axis('equal')


        step_sign = np.sign(start_heading_deg - desired_end_heading_deg)
        spiral = connect_points_with_clothoid(start_point, end_point, start_heading_deg, desired_end_heading_deg, n_points=sample)
        
        while not check_if_close(spiral, apex_points, threshold_distance):
            desired_end_heading_deg += step_sign * end_heading_step_deg
            spiral = connect_points_with_clothoid(start_point, end_point, start_heading_deg, desired_end_heading_deg, n_points=sample)
            # plt.plot(spiral[:, 0], spiral[:, 1], label='Spiral', linewidth=2)

        return spiral

    
    def find_precut_direct(start_point, outside_end_point, inside_end_point, apex_points, start_heading_deg, desired_end_heading_deg, slices=20, threshold_distance=5, clothoid_sample_size=20):
         # plt.figure(figsize=(12, 9))
        # plt.plot(*A, 'go', label='Start Point A')
        # plt.plot(*B, 'ro', label='Apex Point B')
        # plt.plot(*C, 'bo', label='End Point C')
        # plt.plot(*D, 'bo', label='End Point C')
        # plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
        #         scale=5, color='green', label='Start Heading')
        # plt.quiver(*B, np.cos(np.radians(theta_end_deg)), np.sin(np.radians(theta_end_deg)),
        #         scale=5, color='red', label='Apex Heading')
        # plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
        #         scale=5, color='blue', label='End Heading')
        # plt.quiver(*D, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
        #         scale=5, color='blue', label='End Heading')
        # plt.axis('equal')


        step_vec = (np.array(inside_end_point) - np.array(outside_end_point)) / slices
        spiral = connect_points_with_clothoid(start_point, outside_end_point, start_heading_deg, desired_end_heading_deg, n_points=clothoid_sample_size)
        
        while not check_if_close(spiral, apex_points, threshold_distance):
            outside_end_point += step_vec
            spiral = connect_points_with_clothoid(start_point, outside_end_point, start_heading_deg, desired_end_heading_deg, n_points=clothoid_sample_size)
            # plt.plot(spiral[:, 0], spiral[:, 1], label='Spiral', linewidth=2)

        return spiral

    def normalize_angle(angle_deg):
        return (angle_deg + 360) % 360

    def find_useful_part(spiral, desired_end_heading_deg, threshold_deg=5):
        lower_limit = desired_end_heading_deg - threshold_deg
        upper_limit = desired_end_heading_deg + threshold_deg

        length = len(spiral)
        mid_index = length //2
        for i in range(mid_index):
            upper = min(mid_index + i, length-1)
            lower = max(mid_index - i - 1, 0)

            upper_vec = spiral[upper+1] - spiral[upper]
            lower_vec = spiral[lower+1] - spiral[lower]
            upper_heading = np.degrees(np.arctan2(upper_vec[1], upper_vec[0]))
            lower_heading = np.degrees(np.arctan2(lower_vec[1], lower_vec[0]))


            if lower_heading >= lower_limit and lower_heading <= upper_limit:
                return spiral[:lower+1]
            if upper_heading >= lower_limit and upper_heading <= upper_limit:
                return spiral[:upper+1]
        
        return spiral


    # Example usage
    if __name__ == "__main__":


        # Define start and end points with headings
        # A = (0, 0)             # Start point
        # B = (25, 35)           # End point
        # C = (28, 87)
        # D = (60, 87) #outside end
        # theta_start_deg = 0   # Start heading in degrees
        # theta_end_deg = 70    # End heading in degrees
        # final_theta_deg = 90
        # A = (0, 0)             # Start point
        # B = (30, 30)           # End point
        # C = (60, 30)
        # D = (60, 60) #outside end
        # theta_start_deg = 90   # Start heading in degrees
        # # apex_heading_deg = 45    # End heading in degrees
        # final_theta_deg = 0
        # A = (0, 0)             # Start point
        # B = (-24, -18)           # End point
        # C = (-35, -40) # inside end
        # D = (-60, -40) #outside end
        # theta_start_deg = 180   # Start heading in degrees
        # final_theta_deg = 225
        # A = (0, 0)
        # B = (-20, 20)
        # C = (-15, 25)
        # D = (-25, 15)
        # theta_start_deg = 0
        # final_theta_deg = 135
        A = (0, 0)             # Start point
        B = (25, 35)           # apex point
        C = (28, 87) # end of corner inside edge
        D = (60, 87) #      outside end
        theta_start_deg = 0   # Start heading in degrees
        final_theta_deg = 90

        proportion = 0.5
        
        if True:
            pass
            # AVERAGE_BC = ((B[0]+C[0])/2,(B[1]+C[1])/2)
            # BBC = (25, 87) 
            
            # average_theta_deg = (final_theta_deg + theta_end_deg) / 2

            # # Generate the clothoid, first half, second half style
            # x_vals, y_vals, clothoid = connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg)
            # BB = (x_vals[-1], y_vals[-1])
            # x2, y2, clothoid2 = connect_points_with_clothoid(C, B, final_theta_deg+180, theta_end_deg + 180)

            # # generatre clothoid directly
            # x3, y3, clothoid3 = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg, n_points=1000)

            # #average clothoid of the two:
            # x_halves = x_vals + x2
            # y_halves = y_vals + y2

            # average_x = [(x_halves[i]+x3[i])/2 for i in range(len(x_halves))]
            # average_y = [(y_halves[i]+y3[i])/2 for i in range(len(y_halves))]

            # #AIM AT AVERAGE:
            # # averx, avery, clothoid3 = connect_points_with_clothoid(A, AVERAGE_BC, theta_start_deg, average_theta_deg)
            # # divergex, divergey, clothoid4 = connect_points_with_clothoid(A, C, theta_start_deg, theta_end_deg)
            # # huggx, huggy, clothoid5 = connect_points_with_clothoid(A, BBC, theta_start_deg, final_theta_deg)

            # CBB = (30, 35) 
            # apexendx, apexendy, clothoid5 = connect_points_with_clothoid(A, CBB, theta_start_deg, theta_end_deg)
            # apexendx2, apexendy2, clothoid5 = connect_points_with_clothoid(C, CBB, (final_theta_deg + 180) % 360, (theta_end_deg + 180) % 360)
            

            # # axex, axey, scale_used, success = connect_points_euler_relative(A, B, theta_start_deg, theta_end_deg - theta_start_deg)
            # # axex2, axey2, scale_used, success = connect_points_euler_relative(C, B, 270, -20)
            # axex2, axey2, scale_used, success = connect_points_euler_relative(CBB, C, theta_end_deg, final_theta_deg-theta_end_deg)

            # # generatre clothoid directly, but changfes ending heading so it touches tangent
            # x4, y4, clothoid3 = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg-70)
            # testx, testy, clothoid3 = connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg)
            # test2x, test2y, clothoid3 = connect_points_with_clothoid(A, B, theta_start_deg, 0)
        
        # fh = connect_points_with_clothoid(A, B, theta_start_deg, apex_heading_deg)
        # sh = connect_points_with_clothoid(B, C, apex_heading_deg, final_theta_deg)
        direct = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg, n_points=1000)

        # full = fh + sh

        # proportioned = [(full[i] * proportion + direct[i] * (1-proportion)) for i in range(len(direct))]

        # optimal_direct_using_angle = find_precut_direct_angle(A, D, [B], 
        #                                     theta_start_deg, final_theta_deg, 
        #                                     threshold_distance=2, end_heading_step_deg=5, sample=30)
        
        # optimal_path_using_angle = find_useful_part(optimal_direct_using_angle, final_theta_deg, threshold_deg=30)

        optimal_direct = find_precut_direct(A, D, C, [B], 
                                             theta_start_deg, final_theta_deg, 
                                             slices=20, threshold_distance=2, clothoid_sample_size=30)
        optimal_path = find_useful_part(optimal_direct, final_theta_deg, threshold_deg=3)
        # optimal_path = optimal_direct



        # Plot the clothoid
        plt.figure(figsize=(12, 9))
        # plt.plot(full[:, 0], full[:, 1], label='Partitioned', linewidth=3)
        plt.plot(direct[:, 0], direct[:, 1], label='To Inside', linewidth=3)
        # plt.plot(proportioned[:, 0], proportioned[:, 1], label='Proportioned', linewidth=3)
        plt.plot(optimal_direct[:, 0], optimal_direct[:, 1], label='Optimal Direct', linewidth=2)
        plt.plot(optimal_path[:, 0], optimal_path[:, 1], label='Optimal Path', linewidth=5)
        # plt.plot(optimal_direct_using_angle[:, 0], optimal_direct_using_angle[:, 1], label='Optimal Direct Using Angle', linewidth=2)
        # plt.plot(optimal_path_using_angle[:, 0], optimal_path_using_angle[:, 1], label='Optimal Path Angle', linewidth=5)
        plt.plot(*A, 'go', label='Start Point A')
        plt.plot(*B, 'ro', label='Apex Point B')
        plt.plot(*C, 'bo', label='End Point C Inside Edge')
        plt.plot(*D, 'bo', label='End Point D Outside Edge')

        # Plot start and end headings
        plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
                scale=5, color='green', label='Start Heading')
        # plt.quiver(*B, np.cos(np.radians(apex_heading_deg)), np.sin(np.radians(apex_heading_deg)),
        #         scale=5, color='red', label='Apex Heading')
        plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
                scale=5, color='blue', label='End Heading')
        plt.quiver(*D, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
                scale=5, color='blue', label='End Heading')

        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title('Clothoid Connecting Two Points with Specified Headings')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

        # Print clothoid parameters
        # print("Clothoid Parameters:")
        # print(f"  Length (L): {clothoid.L}")
        # print(f"  Start Curvature (kappa0): {clothoid.KappaStart}")
        # print(f"  End Curvature (kappa1): {clothoid.KappaEnd}")
        # print(f"  Curvature Rate (dk): {clothoid.dk}")


    # fifth_one()

def eighth_one():

    from pyclothoids import Clothoid
    def connect_points_with_clothoid(A, B, theta_start_deg, theta_end_deg, n_points=500):
        """
        Connects two points A and B with specified start and end headings using a clothoid.

        Parameters:
        - A: Tuple (x0, y0) representing the start point.
        - B: Tuple (x1, y1) representing the end point.
        - theta_start_deg: Start heading in degrees.
        - theta_end_deg: End heading in degrees.
        - n_points: Number of points to sample along the clothoid.

        Returns:
        - x_vals: X coordinates of the sampled points.
        - y_vals: Y coordinates of the sampled points.
        - clothoid: The Clothoid object representing the curve.
        """
        # Convert angles from degrees to radians
        theta_start_rad = np.radians(theta_start_deg)
        theta_end_rad = np.radians(theta_end_deg)

        # Create the clothoid that connects the two points with specified headings
        clothoid = Clothoid.G1Hermite(A[0], A[1], theta_start_rad, B[0], B[1], theta_end_rad)

        # Sample points along the clothoid
        x_vals, y_vals = clothoid.SampleXY(n_points)
        points = np.column_stack((x_vals, y_vals))  # Combine into shape (n_points, 2)

        return points
    
    def check_if_close(spiral, apex_points, threshold_distance):
        length = len(spiral)
        mid_index = length //2
        for i in range(mid_index):
            upper = min(mid_index + i, length-1)
            lower = max(mid_index - i - 1, 0)
            for apoint in apex_points:
                if (np.linalg.norm(spiral[upper] - apoint) <= threshold_distance) or \
                    (np.linalg.norm(spiral[lower] - apoint) <= threshold_distance):
                    return True
        
        return False

    def find_precut_direct(start_point, outside_end_point, inside_end_point, apex_points, start_heading_deg, desired_end_heading_deg, slices=20, threshold_distance=5, clothoid_sample_size=20):
         # plt.figure(figsize=(12, 9))
        # plt.plot(*A, 'go', label='Start Point A')
        # plt.plot(*B, 'ro', label='Apex Point B')
        # plt.plot(*C, 'bo', label='End Point C')
        # plt.plot(*D, 'bo', label='End Point C')
        # plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
        #         scale=5, color='green', label='Start Heading')
        # plt.quiver(*B, np.cos(np.radians(theta_end_deg)), np.sin(np.radians(theta_end_deg)),
        #         scale=5, color='red', label='Apex Heading')
        # plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
        #         scale=5, color='blue', label='End Heading')
        # plt.quiver(*D, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
        #         scale=5, color='blue', label='End Heading')
        # plt.axis('equal')

        iterations = 0
        step_vec = (np.array(inside_end_point) - np.array(outside_end_point)) / slices
        spiral = connect_points_with_clothoid(start_point, outside_end_point, start_heading_deg, desired_end_heading_deg, n_points=clothoid_sample_size)
        
        while not check_if_close(spiral, apex_points, threshold_distance):
            outside_end_point += step_vec
            spiral = connect_points_with_clothoid(start_point, outside_end_point, start_heading_deg, desired_end_heading_deg, n_points=clothoid_sample_size)
            # plt.plot(spiral[:, 0], spiral[:, 1], label='Spiral', linewidth=2)
            iterations += 1
            if iterations > slices:
                break

        return spiral

    def find_useful_part(spiral, desired_end_heading_deg, threshold_deg=5):
        lower_limit = desired_end_heading_deg - threshold_deg
        upper_limit = desired_end_heading_deg + threshold_deg

        length = len(spiral)
        mid_index = length //2
        for i in range(mid_index - 1):
            upper = min(mid_index + i, length-1)
            lower = max(mid_index - i - 1, 0)

            upper_vec = spiral[upper+1] - spiral[upper]
            lower_vec = spiral[lower+1] - spiral[lower]
            upper_heading = np.degrees(np.arctan2(upper_vec[1], upper_vec[0]))
            lower_heading = np.degrees(np.arctan2(lower_vec[1], lower_vec[0]))


            if lower_heading >= lower_limit and lower_heading <= upper_limit:
                return spiral[:lower+1]
            if upper_heading >= lower_limit and upper_heading <= upper_limit:
                return spiral[:upper+1]
        
        return spiral

    # Example usage
    if __name__ == "__main__":
        # Tests
        if True:
            # A = (0, 0)             # Start point
            # B = (-24, -18)           # End point
            # C = (-35, -40) # inside end
            # D = (-60, -40) #outside end
            # theta_start_deg = 180   # Start heading in degrees
            # final_theta_deg = 225
            
            # A = (0, 0)
            # B = (-20, 20)
            # C = (-15, 25)
            # D = (-25, 15)
            # theta_start_deg = 0
            # final_theta_deg = 135
            
            # A = (0, 0)             # Start point
            # B = (25, 35)           # apex point
            # C = (28, 87) # end of corner inside edge
            # D = (60, 87) #      outside end
            # theta_start_deg = 0   # Start heading in degrees
            # final_theta_deg = 90
            corner_test_cases = [
                # Left 90-degree turn (like your original)
                ((0, 0), (25, 35), (28, 87), (60, 87), 0, 90),

                # Right 45-degree turn
                ((0, 0), (20, 10), (45, 45), (45, 15), 0, -45),

                # Left 120-degree turn
                ((100, 100), (125, 130), (130, 160), (100, 167), 0, 120),

                # Right 135-degree turn
                ((50, 50), (55, 70), (65, 90), (40, 70), 90, 225),

                # Left 60-degree turn
                ((0, 0), (30, 10), (50, 25), (35, 47), 0, 60),

                # Right 90-degree turn
                ((0, 0), (30, -35), (28, -87), (-5, -87), 0, -90),

                # Left 135-degree turn
                ((80, 80), (105, 100), (115, 130), (90, 150), 0, 135),

                # Right 120-degree turn
                ((60, 60), (80, 55), (100, 40), (125, 48), 180, 60),

                # Left 30-degree turn
                ((0, 0), (15, 5), (40, 10), (35, 25), 0, 30),

                # Right 150-degree turn
                ((200, 200), (215, 180), (225, 155), (250, 135), 270, 120),
            ]
        A, B, D, C, theta_start_deg, final_theta_deg = corner_test_cases[9]

        direct = connect_points_with_clothoid(A, C, theta_start_deg, final_theta_deg, n_points=1000)
        optimal_direct = find_precut_direct(A, D, C, [B], 
                                             theta_start_deg, final_theta_deg, 
                                             slices=20, threshold_distance=2, clothoid_sample_size=30)
        optimal_path = find_useful_part(optimal_direct, final_theta_deg, threshold_deg=3)

        # Plot the clothoid
        if True:
            plt.figure(figsize=(12, 9))
            plt.plot(direct[:, 0], direct[:, 1], label='To Inside', linewidth=3)
            plt.plot(optimal_direct[:, 0], optimal_direct[:, 1], label='Optimal Direct', linewidth=2)
            plt.plot(optimal_path[:, 0], optimal_path[:, 1], label='Optimal Path', linewidth=5)
            plt.plot(*A, 'go', label='Start Point A')
            plt.plot(*B, 'ro', label='Apex Point B')
            plt.plot(*C, 'bo', label='End Point C Inside Edge')
            plt.plot(*D, 'ro', label='End Point D Outside Edge')

            # Plot start and end headings
            plt.quiver(*A, np.cos(np.radians(theta_start_deg)), np.sin(np.radians(theta_start_deg)),
                    scale=5, color='green', label='Start Heading')
            plt.quiver(*C, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
                    scale=5, color='blue', label='End Heading')
            plt.quiver(*D, np.cos(np.radians(final_theta_deg)), np.sin(np.radians(final_theta_deg)),
                    scale=5, color='blue', label='End Heading')

            plt.axis('equal')
            plt.grid(True)
            plt.legend()
            plt.title('Clothoid Connecting Two Points with Specified Headings')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.show()

def is_point_right_or_left(ref_point, ref_heading_vec, compare_point):
    # Vector from A to the other point
    to_point = compare_point - ref_point

    # Compute the 2D cross product (scalar)
    cross = ref_heading_vec[0] * to_point[1] - ref_heading_vec[1] * to_point[0]

    if cross > 0:
        return "left"
    elif cross < 0:
        return "right"
    else:
        return "colinear"  # or "straight ahead"


apex_point = np.array([0, 0])
heading = np.array([1, 1])  # pointing along +x
P1 = np.array([0, 1])       # should be left
P2 = np.array([1, -1])      # should be right

print(is_point_right_or_left(apex_point, heading, P1))  # left
print(is_point_right_or_left(apex_point, heading, P2))  # right


# eighth_one()