import numpy as np
import cvxpy as cp
from scipy.ndimage import binary_dilation
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
import csv 
import skimage.morphology as morphology
import skimage.graph as graph
import scipy.ndimage as ndimage
import cv2
import numpy as np
import skimage.morphology as morph
from scipy.optimize import minimize


# Configuration dictionary
config = {
    'HORIZON': 50,        # Number of trajectory points
    'MAX_VELOCITY': 2.0,  # Maximum velocity (m/s)
    'MAX_ACCELERATION': 1.5,  # Maximum acceleration (m/s²)
    'TRACK_WIDTH': 1.0,   # Track width (m)
    'VEHICLE_LENGTH': 0.5,  # Vehicle length (m)
    'VEHICLE_WIDTH': 0.3    # Vehicle width (m)
}



def load_occupancy():
    with open("occupancy_grid.csv", "r") as csvfile:
        reader = csv.reader(csvfile)
        occupancy_grid = np.array([list(map(float, row)) for row in reader])
    return occupancy_grid



def preprocess_occupancy_grid(occupancy_grid):
    """
    Preprocess occupancy grid for trajectory optimization
    
    Args:
    - occupancy_grid: 2D NumPy array (0 = free space, 1 = occupied)
    
    Returns:
    - Processed safe space grid
    """    
    safe_space = binary_dilation(occupancy_grid, iterations=2)
    safe_space = safe_space.astype(float)
    return safe_space




def extract_centerline(occupancy_grid):
    # Convert grid to binary image
    binary = (occupancy_grid == 0).astype(np.uint8)
    
    # Skeletonize the map
    skeleton = morph.skeletonize(binary)
    
    # Find centerline points
    centerline_points = np.column_stack(np.where(skeleton))
    
    # Calculate distance to nearest wall
    distances = cv2.distanceTransform(1 - binary, cv2.DIST_L2, 5)
    
    # Get wall distances for centerline points
    wall_distances = distances[centerline_points[:, 0], centerline_points[:, 1]]
    

    plt.imshow(distances, cmap="viridis", origin="upper")
    plt.colorbar(label="Occupancy Probability")
    plt.title("Occupancy Grid Visualization")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.show()

    
    return centerline_points, wall_distances


def calculate_boundary_distances(occupancy_grid, center_line):
    """
    Calculate distances from center line points to nearest boundaries.
    
    Parameters:
    -----------
    occupancy_grid : numpy.ndarray
        2D binary grid where 1 represents occupied space and 0 represents free space.
    center_line : numpy.ndarray
        Coordinates of the center line path
    
    Returns:
    --------
    distances : numpy.ndarray
        Distances from each center line point to the nearest boundary
    """
    # Compute distance transform
    distance_transform = ndimage.distance_transform_edt(1 - occupancy_grid)
    
    # Extract distances for center line points
    distances = distance_transform[center_line[:, 0], center_line[:, 1]]
    
    return distances



def optimize_trajectory(occupancy_grid, start_point, end_point, config):
    """
    Perform minimum time trajectory optimization
    
    Args:
    - occupancy_grid: 2D NumPy array of track
    - start_point: [x, y] starting coordinates
    - end_point: [x, y] ending coordinates
    - config: Dictionary of configuration parameters
    
    Returns:
    - Optimized trajectory
    - Velocity profile
    - Total time
    """
    safe_space = preprocess_occupancy_grid(occupancy_grid)
    
    # Configuration shortcuts
    N = config['HORIZON']
    v_max = config['MAX_VELOCITY']
    a_max = config['MAX_ACCELERATION']
    
    # Optimization Variables
    x = cp.Variable((N, 2))    # Position trajectory
    v = cp.Variable(N)          # Velocity profile
    t = cp.Variable()            # Total time
    
    # Initial trajectory guess
    initial_trajectory = compute_initial_trajectory_guess(start_point, end_point, N)
    
    # Objective: Minimize total time
    objective = cp.Minimize(t)
    
    # Constraints
    constraints = [
        t >= 0,
        v >= 0,
        v <= v_max,
        x[0] == start_point,
        x[-1] == end_point,
        v[0] == 0,
        v[-1] == 0
    ]
    
    for k in range(N - 1):
        constraints += [
            x[k + 1, 0] == x[k, 0] + v[k] * (t / N),
            x[k + 1, 1] == x[k, 1] + v[k] * (t / N),
            cp.abs(v[k + 1] - v[k]) / (t / N) <= a_max
        ]
        
        grid_x = int(x[k, 0])
        grid_y = int(x[k, 1])
        
        if 0 <= grid_x < safe_space.shape[0] and 0 <= grid_y < safe_space.shape[1]:
            constraints.append(safe_space[grid_x, grid_y] == 1)
    
    # Solve the problem
    prob = cp.Problem(objective, constraints)
    
    try:
        prob.solve(solver=cp.ECOS)
        return {
            'trajectory': x.value,
            'velocity_profile': v.value,
            'total_time': t.value
        }
    except Exception as e:
        print(f"Optimization failed: {e}")
        return None




def visualize_trajectory(occupancy_grid, trajectory):
    """
    Visualize trajectory on occupancy grid
    """
    plt.figure(figsize=(10, 8))
    plt.imshow(occupancy_grid, cmap='binary')
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=2)
    plt.title('Optimized Trajectory')
    plt.show()



def visualization(visual):

    visual = visual.astype(float)
# Visualize the occupancy grid
    plt.imshow(visual, cmap="viridis", origin="upper")
    plt.colorbar(label="Occupancy Probability")
    plt.title("Occupancy Grid Visualization")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.show()


def visualize_occupancy_grid(occupancy_grid, center_line=None, boundary_distance=None):
    """
    Visualize the occupancy grid with optional center line and boundary distances.
    
    Parameters:
    -----------
    occupancy_grid : numpy.ndarray
        2D binary grid where 1 represents occupied space and 0 represents free space.
    center_line : numpy.ndarray, optional
        Coordinates of the center line path
    boundary_distance : numpy.ndarray, optional
        Distances from each center line point to the nearest boundary
    """
    plt.figure(figsize=(12, 6))
    
    # Original Occupancy Grid
    plt.subplot(121)
    plt.imshow(occupancy_grid, cmap='binary', interpolation='nearest')
    plt.title('Occupancy Grid')
    plt.xlabel('X')
    plt.ylabel('Y')
    
    # Occupancy Grid with Center Line
    plt.subplot(122)
    plt.imshow(occupancy_grid, cmap='binary', interpolation='nearest')
    
    if center_line is not None and center_line.ndim == 2 and center_line.shape[1] == 2:
        plt.plot(center_line[:, 1], center_line[:, 0], 'r.', label='Center Line')
        if boundary_distance is not None:
            plt.scatter(center_line[:, 1], center_line[:, 0], c=boundary_distance, cmap='viridis')
            plt.colorbar(label='Boundary Distance')
        plt.legend()
    else:
        plt.title('No Center Line Found')
    
    plt.title('Center Line and Boundary Distances')
    plt.xlabel('X')
    plt.ylabel('Y')
    
    plt.tight_layout()
    plt.show()
    ()

def main():
    occupancy_grid = load_occupancy()
    heatmap = preprocess_occupancy_grid(occupancy_grid)
    center_line = extract_centerline(heatmap)
    boundry_distance = calculate_boundary_distances(heatmap, center_line)
    visualize_occupancy_grid(heatmap, boundry_distance)
    

if __name__ == "__main__":
    main()