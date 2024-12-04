import numpy as np
import cvxpy as cp
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
import csv 

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
    return safe_space

def compute_initial_trajectory_guess(start_point, end_point, horizon):
    """
    Generate initial trajectory waypoints
    
    Args:
    - start_point: [x, y] start coordinates
    - end_point: [x, y] end coordinates
    - horizon: Number of points in the trajectory
    
    Returns:
    - Initial trajectory waypoints
    """
    x_waypoints = np.linspace(start_point[0], end_point[0], horizon)
    y_waypoints = np.linspace(start_point[1], end_point[1], horizon)
    return np.column_stack([x_waypoints, y_waypoints])

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

# Example Usage
def main():

    # Load Occupancy grid 
    occpancy_grid = load_occupancy()

    proccesed_occupancy  =   preprocess_occupancy_grid(occpancy_grid)


    visualization(proccesed_occupancy)

    """
    # Define start and end points
    occupancy_grid = 0.0
    start_point = 0.0
    end_point = 0.0

    # Optimize trajectory
    result = optimize_trajectory(occupancy_grid, start_point, end_point, config)
    
    if result:
        visualize_trajectory(occupancy_grid, result['trajectory'])
        print(f"Total Time: {result['total_time']:.2f}s")


    """
if __name__ == '__main__':
    main()
