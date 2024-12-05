import numpy as np
import matplotlib.pyplot as plt

# Vehicle and trajectory parameters
a_max = 8.83  # Maximum acceleration (m/s^2)
a_lat_max = 3.0  # Maximum lateral acceleration (m/s^2)
v_max = 22.88  # Maximum velocity (m/s)



def compute_speed_profile(trajectory, a_max, a_lat_max, v_max):
    """
    Generate the optimal speed profile for the trajectory.
    
    Args:
        trajectory (numpy.ndarray): Nx2 array of [x, y] waypoints.
        a_max (float): Maximum forward/backward acceleration (m/s^2).
        a_lat_max (float): Maximum lateral acceleration (m/s^2).
        v_max (float): Maximum velocity (m/s).
    
    Returns:
        numpy.ndarray: N array of speeds (m/s).
    """
    curvature = compute_curvature(trajectory)#############################################taken from moataz
    v_curvature = np.sqrt(a_lat_max / (curvature + 1e-6))  # Avoid division by zero
    v_curvature = np.clip(v_curvature, 0, v_max)
    
    N = len(trajectory)
    speed = np.zeros(N)
    speed[0] = 0  # Start from rest

    # Forward pass
    for i in range(1, N):
        d = np.linalg.norm(trajectory[i] - trajectory[i - 1])  # Distance
        speed[i] = min(v_curvature[i], np.sqrt(speed[i - 1]**2 + 2 * a_max * d))
    
    # Backward pass
    for i in range(N - 2, -1, -1):
        d = np.linalg.norm(trajectory[i + 1] - trajectory[i])  # Distance
        speed[i] = min(speed[i], np.sqrt(speed[i + 1]**2 + 2 * a_max * d))
    
    return speed

def plot_speed_profile(trajectory, speed_profile):
    """
    Plot the trajectory and speed profile.
    
    Args:
        trajectory (numpy.ndarray): Nx2 array of [x, y] waypoints.
        speed_profile (numpy.ndarray): N array of speeds (m/s).
    """
    plt.figure(figsize=(12, 6))
    
    # Trajectory plot
    plt.subplot(1, 2, 1)
    plt.plot(trajectory[:, 0], trajectory[:, 1], label="Trajectory")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Trajectory")
    plt.axis("equal")
    plt.legend()
    
    # Speed profile plot
    plt.subplot(1, 2, 2)
    plt.plot(range(len(speed_profile)), speed_profile, label="Speed Profile", color='red')
    plt.xlabel("Waypoint Index")
    plt.ylabel("Speed (m/s)")
    plt.title("Speed Profile")
    plt.legend()
    
    plt.tight_layout()
    plt.savefig("speed_profile.png")


if __name__ == "__main__":
    # Example trajectory (circle)
    theta = np.linspace(0, 2 * np.pi, 100)
    trajectory = np.array([[0, 0], [1, 1], [2, 0], [3, 1], [4, 0]])


    
    # Compute speed profile
    speed_profile = compute_speed_profile(trajectory, a_max, a_lat_max, v_max)
    
    # Plot results
    plot_speed_profile(trajectory, speed_profile)
