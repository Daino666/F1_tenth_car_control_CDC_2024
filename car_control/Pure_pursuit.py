#!/usr/bin/env python3
import csv
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32

# Global variables for managing the path and vehicle state
counter = 0
wheel_base = 0.3240  # Distance between the front and rear axles of the vehicle (in meters)
vehicle_position = [0.0, 0.0]  # Current position of the vehicle [x, y]
vehicle_orientation = 0.0  # Current orientation (yaw) of the vehicle in radians
path_x_coords = []  # X-coordinates of the predefined path
path_y_coords = []  # Y-coordinates of the predefined path
num_waypoints = 0  # Total number of waypoints in the path
waypoints = []  # List of (x, y) waypoints in the path


def csv_reading(file_path, column_name):
    """
    Reads a specific column from a CSV file.

    Args:
        file_path (str): Path to the CSV file.
        column_name (str): Name of the column to read.

    Returns:
        list: List of float values from the specified column.
    """
    column_data = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            if column_name in row:
                column_data.append(float(row[column_name]))
    return column_data


def initialize_node():
    """
    Initializes the ROS 2 node and sets up publishers for control commands.

    Returns:
        tuple: ROS node, throttle publisher, and steering publisher.
    """
    global path_x_coords, path_y_coords, num_waypoints, waypoints
    rclpy.init()
    node = Node('pure_pursuit')  # Initialize the ROS 2 node

    # Publishers for throttle and steering commands
    throttle_pub = node.create_publisher(Float32, '/autodrive/f1tenth_1/throttle', 10)
    steering_pub = node.create_publisher(Float32, '/autodrive/f1tenth_1/steering', 10)

    # Load path waypoints from a CSV file
    file_path = '/home/daino/workspace/src/real_time/scripts/wp_file.csv'
    column_x = 'positions_x_odom'
    column_y = 'positions_y_odom'
    path_x_coords = csv_reading(file_path, column_x)
    path_y_coords = csv_reading(file_path, column_y)
    num_waypoints = len(path_x_coords)
    waypoints = list(zip(path_x_coords, path_y_coords))  # Combine x and y into tuples

    node.get_logger().info("Pure Pursuit Node Initialized")
    return node, throttle_pub, steering_pub


def update_vehicle_state(position, orientation):
    """
    Updates the vehicle's current position and orientation.

    Args:
        position (list): [x, y] coordinates of the vehicle.
        orientation (float): Orientation (yaw) of the vehicle in radians.
    """
    global vehicle_position, vehicle_orientation
    vehicle_position = position  # Update the current position
    vehicle_orientation = orientation  # Update the yaw (orientation)
    select_target_waypoint()  # Determine the next waypoint


def select_target_waypoint():
    """
    Selects the next waypoint for the vehicle to target based on distance and angle.
    """
    for i in range(num_waypoints):
        # Compute the distance and angle to each waypoint
        dx = waypoints[i][0] - vehicle_position[0]
        dy = waypoints[i][1] - vehicle_position[1]
        distance = np.sqrt(dx**2 + dy**2)  # Euclidean distance
        waypoint_angle = np.arctan2(dy, dx)  # Angle to the waypoint relative to the vehicle
        angle_diff = abs(normalize_angle(waypoint_angle - vehicle_orientation))
        angle_diff = np.degrees(angle_diff)  # Convert to degrees for easier interpretation

        # Check if the waypoint satisfies the selection criteria
        if 3.5 <= distance <= 4 and 0 <= angle_diff <= 60:  # Tunable parameters
            compute_steering_command(waypoints[i])
            return


def normalize_angle(angle):
    """
    Normalizes an angle to the range [-π, π].

    Args:
        angle (float): Angle in radians.

    Returns:
        float: Normalized angle in radians.
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def compute_steering_command(target_point):
    """
    Computes the steering angle and publishes it.

    Args:
        target_point (tuple): Target waypoint as (x, y).
    """
    dx = target_point[0] - vehicle_position[0]
    dy = target_point[1] - vehicle_position[1]

    # Transform the target point to the vehicle's local coordinate frame
    cos_yaw = np.cos(vehicle_orientation)
    sin_yaw = np.sin(vehicle_orientation)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy

    # Avoid division by zero for curvature calculation
    if abs(local_x) < 1e-6:
        local_x = 1e-6 if local_x >= 0 else -1e-6

    # Compute the curvature and steering angle
    curvature = 2.0 * local_y / (local_x**2 + local_y**2)
    steering_angle = np.arctan2(wheel_base * curvature, 1.0)
    steering_angle_deg = np.degrees(steering_angle) * -1  # Convert to degrees and invert
    steering_angle_deg = np.clip(steering_angle_deg, -20.5, 19.0)  # Clamp within limits

    publish_drive_commands(steering_angle_deg)


def publish_drive_commands(steering_angle_deg):
    """
    Publishes the steering and throttle commands.

    Args:
        steering_angle_deg (float): Steering angle in degrees.
    """
    # Publish steering angle
    steering_msg = Float32()
    steering_msg.data = steering_angle_deg
    steering_pub.publish(steering_msg)

    # Compute and publish throttle (reduce speed for sharp turns)
    throttle_msg = Float32()
    throttle_msg.data = 4 * (1 - abs(steering_angle_deg / 40))  # Tunable velocity
    throttle_pub.publish(throttle_msg)


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    global steering_pub, throttle_pub
    node, throttle_pub, steering_pub = initialize_node()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shutdown ROS 2 system


if __name__ == '__main__':
    main()
