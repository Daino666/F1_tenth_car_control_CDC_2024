#!/usr/bin/env python3
import csv
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32

# Global Variables
counter = 0
wheel_base = 0.3240  # Distance between the front and rear axles of the vehicle
vehicle_position = [0.0, 0.0]  # Vehicle's current position (x, y)
vehicle_orientation = 0.0  # Vehicle's current orientation in radians
path_x_coords = []  # X-coordinates of the path
path_y_coords = []  # Y-coordinates of the path
num_path_values = 0  # Number of waypoints in the path
waypoints = []  # List of (x, y) waypoints

# Function to read data from a CSV file
def csv_reading(file_path, column_name):
    """Reads a specific column from a CSV file."""
    column_data = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            if column_name in row:
                column_data.append(float(row[column_name]))
    return column_data

# Initialize the ROS 2 node and set up publishers and subscribers
def initialize_node():
    """Initializes the ROS 2 node and loads the path."""
    global path_x_coords, path_y_coords, num_path_values, waypoints
    rclpy.init()
    node = Node('pure_pursuit')  # Create a ROS 2 node

    # Publishers for vehicle data
    publishers = {
        #"front_camera": node.create_publisher(Float32, '/autodrive/f1tenth_1/front_camera', 0),
        "imu": node.create_publisher(Float32, '/autodrive/f1tenth_1/imu', 0),
        #"left_encoder": node.create_publisher(Float32, '/autodrive/f1tenth_1/left_encoder', 0),
        #"lidar": node.create_publisher(Float32, '/autodrive/f1tenth_1/lidar', 0),
        #"right_encoder": node.create_publisher(Float32, '/autodrive/f1tenth_1/right_encoder', 0),
        "steering": node.create_publisher(Float32, '/autodrive/f1tenth_1/steering', 0),
        "throttle": node.create_publisher(Float32, '/autodrive/f1tenth_1/throttle', 0)
    }

    # Subscribers for commands
    steering_subscription = node.create_subscription(
        Float32, '/autodrive/f1tenth_1/steering_command', process_steering_command, 0
    )
    throttle_subscription = node.create_subscription(
        Float32, '/autodrive/f1tenth_1/throttle_command', process_throttle_command, 0
    )

    # Load path from a CSV file
    file_path = '/home/autodrive_devkit/src/car_control/car_control/Test.csv'
    column_x = 'positions_x_odom'
    column_y = 'positions_y_odom'
    path_x_coords = csv_reading(file_path, column_x)
    path_y_coords = csv_reading(file_path, column_y)
    num_path_values = len(path_x_coords)
    waypoints = list(zip(path_x_coords, path_y_coords))  # Combine x and y coordinates into a path

    node.get_logger().info("Pure Pursuit Node Initialized")
    return node, publishers

# Process incoming steering command
def process_steering_command(msg):
    """Processes and logs the received steering command."""
    global vehicle_orientation
    vehicle_orientation = msg.data

# Process incoming throttle command
def process_throttle_command(msg):
    """Processes and logs the received throttle command."""
    global counter
    counter = msg.data

# Update the current vehicle state
def update_vehicle_state(pose, orientation):
    """Updates the current vehicle state, including position and orientation."""
    global vehicle_position, vehicle_orientation
    vehicle_position[0] = pose[0]  # Update x-coordinate
    vehicle_position[1] = pose[1]  # Update y-coordinate
    vehicle_orientation = orientation  # Update yaw (orientation) provided by an external system
    select_target_waypoint()

# Select the next waypoint for the vehicle to follow
def select_target_waypoint():
    """Selects the next target waypoint for the vehicle to follow."""
    for i in range(num_path_values):
        # Calculate distance and angle to each waypoint
        dx = waypoints[i][0] - vehicle_position[0]
        dy = waypoints[i][1] - vehicle_position[1]
        distance = np.sqrt(dx**2 + dy**2)  # Euclidean distance to the waypoint
        waypoint_angle = np.arctan2(dy, dx)  # Angle to the waypoint relative to the vehicle
        angle_diff = abs(normalize_angle(waypoint_angle - vehicle_orientation))
        angle_diff = np.degrees(angle_diff)  # Convert angle difference to degrees

        # Check if the waypoint satisfies the distance and angle criteria
        if 3.5 <= distance <= 4 and 0 <= angle_diff <= 60:  # Tunable parameters for target selection
            compute_steering_command(waypoints[i])
            return

# Normalize an angle to the range [-pi, pi]
def normalize_angle(angle):
    """Normalizes an angle to the range [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

# Compute the steering command for the target waypoint
def compute_steering_command(point):
    """Computes the steering command required to reach the target point."""
    dx = point[0] - vehicle_position[0]  # X difference between the vehicle and the target point
    dy = point[1] - vehicle_position[1]  # Y difference between the vehicle and the target point

    # Transform the target point to the vehicle's local coordinate frame
    cos_yaw = np.cos(vehicle_orientation)
    sin_yaw = np.sin(vehicle_orientation)
    local_x = cos_yaw * dx + sin_yaw * dy  # X-coordinate in the local frame
    local_y = -sin_yaw * dx + cos_yaw * dy  # Y-coordinate in the local frame

    if abs(local_x) < 1e-6:  # Avoid division by zero when computing curvature
        local_x = 1e-6 if local_x >= 0 else -1e-6

    # Compute curvature based on local coordinates
    curvature = 2.0 * local_y / (local_x**2 + local_y**2)
    steering_angle = np.arctan2(wheel_base * curvature, 1.0)  # Steering angle in radians
    steering_angle_deg = np.degrees(steering_angle) * -1  # Convert to degrees and invert for vehicle alignment
    steering_angle_deg = np.clip(steering_angle_deg, -20.5, 19.0)  # Limit steering angle to the vehicle's range

    publish_drive_commands(steering_angle_deg)

# Publish the steering and throttle commands
def publish_drive_commands(steering_angle_deg):
    """Publishes the steering angle and velocity commands."""
    global node, publishers

    # Publish dummy data for all vehicle data topics
    for topic, pub in publishers.items():
        msg = Float32()
        msg.data = 0.0  # Placeholder for actual sensor values or calculations
        pub.publish(msg)

    # Publish the computed steering angle
    publishers["steering"].publish(Float32(data=steering_angle_deg))

    # Compute and publish the velocity command (reduce speed for sharp turns)
    velocity = 4 * (1 - abs(steering_angle_deg / 40))  # Tunable velocity adjustment
    publishers["throttle"].publish(Float32(data=velocity))

# Main entry point
def main(args=None):
    """Main entry point for the node."""
    global node, publishers
    node, publishers = initialize_node()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()  # Cleanup the node on shutdown
    rclpy.shutdown()  # Shutdown the ROS 2 system

if __name__ == '__main__':
    main()
