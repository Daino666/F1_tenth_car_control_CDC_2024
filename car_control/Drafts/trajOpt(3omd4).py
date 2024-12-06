import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Define vehicle parameters
L = 0.3240  # Wheelbase (m)
m = 3.906  # Mass of the vehicle (kg)
drag_coeff = 0.3  # Drag coefficient (N/(m/s)^2)

# Define a path (circle for simplicity)
def circular_path(theta, radius=10):
    return np.array([radius * np.cos(theta), radius * np.sin(theta)])

# Derivative of the path

def circular_path_derivative(theta, radius=10):
    return np.array([-radius * np.sin(theta), radius * np.cos(theta)])

# Vehicle dynamics
def vehicle_dynamics(t, state, u):
    """
    state: [x, y, theta, v] - Position, orientation, and velocity
    u: [F, delta] - Control inputs: throttle force and steering angle
    """
    x, y, theta, v = state
    F, delta = u

    # Drag force
    F_drag = drag_coeff * v**2 * np.sign(v)

    # Equations of motion
    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    dtheta = (v / L) * np.tan(delta)
    dv = (F - F_drag) / m

    return [dx, dy, dtheta, dv]

# Pure pursuit controller
def pure_pursuit(state, path_func, lookahead_distance):
    """
    Pure pursuit controller to compute steering angle.
    state: [x, y, theta, v]
    path_func: Function defining the path.
    lookahead_distance: Distance to look ahead on the path.
    """
    x, y, theta, v = state

    # Find lookahead point
    theta_lookahead = np.linspace(0, 2 * np.pi, 500)
    path_points = path_func(theta_lookahead)
    distances = np.sqrt((path_points[0] - x)**2 + (path_points[1] - y)**2)
    lookahead_idx = np.argmin(np.abs(distances - lookahead_distance))
    lookahead_point = path_points[:, lookahead_idx]

    # Compute steering angle
    alpha = np.arctan2(lookahead_point[1] - y, lookahead_point[0] - x) - theta
    delta = np.arctan2(2 * L * np.sin(alpha), lookahead_distance)

    return delta

# Simulation parameters
dt = 0.01  # Time step (s)
total_time = 20  # Total simulation time (s)
num_steps = int(total_time / dt)

# Initial state
state = np.array([10, 0, np.pi / 2, 0])  # [x, y, theta, v]

# Path parameters
lookahead_distance = 5
path_func = lambda theta: circular_path(theta, radius=10)

# Logging
trajectory = [state]
time_log = [0]

# Simulation loop
for step in range(num_steps):
    t = step * dt

    # Control inputs
    delta = pure_pursuit(state, path_func, lookahead_distance)
    F = 1000  # Constant throttle force

    # Solve dynamics
    sol = solve_ivp(
        vehicle_dynamics,
        [t, t + dt],
        state,
        args=([F, delta],),
        method='RK45',
    )

    # Update state
    state = sol.y[:, -1]
    trajectory.append(state)
    time_log.append(t + dt)

trajectory = np.array(trajectory)

# Plot results
path_points = path_func(np.linspace(0, 2 * np.pi, 500))
plt.figure(figsize=(8, 8))
plt.plot(path_points[0], path_points[1], label="Path", linestyle="--")
plt.plot(trajectory[:, 0], trajectory[:, 1], label="Trajectory")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("Vehicle Path Tracking")
plt.legend()
plt.grid()
plt.axis("equal")
plt.show()
