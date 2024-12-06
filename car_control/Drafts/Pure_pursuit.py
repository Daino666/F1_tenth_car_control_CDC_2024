#!/usr/bin/env python3
import csv
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32

# Global Variables
counter = 0
wheel_base = 0.3240  # The distance between the front and rear axles of the vehicle
current_pose = [0.0, 0.0]  # Vehicle's current position (x, y)
yaw = 0.0  # Vehicle's current orientation in radians
x_values = []  # X-coordinates of the path
y_values = []  # Y-coordinates of the path
num_path_values = 0  # Number of waypoints in the path
path = []  # List of (x, y) waypoints

# CSV Reading Function
def csv_reading(file_path, column_name):
    """Reads a specific column from a CSV file."""
    column_data = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            if column_name in row:
                column_data.append(float(row[column_name]))
    return column_data

# ROS 2 Initialization
def initialize_node():
    """Initializes the ROS 2 node and loads the path."""
    global x_values, y_values, num_path_values, path
    rclpy.init()
    node = Node('pure_pursuit')  # Create a ROS 2 node

    # Publishers for throttle and steering commands
    cmd_pub = node.create_publisher(Float32, '/autodrive/f1tenth_1/throttle', 0)
    steering_pub = node.create_publisher(Float32, '/autodrive/f1tenth_1/steering', 0)

    # Load Path from CSV file
    file_path = '/home/daino/workspace/src/real_time/scripts/wp_file.csv'
    column_x = 'positions_x_odom'
    column_y = 'positions_y_odom'
    x_values = csv_reading(file_path, column_x)
    y_values = csv_reading(file_path, column_y)
    num_path_values = len(x_values)
    path = list(zip(x_values, y_values))  # Combine x and y coordinates into a path

    node.get_logger().info("Pure Pursuit Node Initialized")
    return node, cmd_pub, steering_pub

# Updates the current pose and orientation
def update_pose(pose, orientation):
    """Updates the current pose and orientation."""
    global current_pose, yaw
    current_pose[0] = pose[0]  # Update x-coordinate
    current_pose[1] = pose[1]  # Update y-coordinate
    yaw = orientation  # Update yaw (orientation) provided by an external system
    find_target_waypoint()

# Finds the next target waypoint for the vehicle to follow
def find_target_waypoint():
    """Finds the next target waypoint for the vehicle to follow."""
    for i in range(num_path_values):
        # Compute distance and angle to each waypoint
        dx = path[i][0] - current_pose[0]
        dy = path[i][1] - current_pose[1]
        distance = np.sqrt(dx**2 + dy**2)  # Euclidean distance to the waypoint
        waypoint_angle = np.arctan2(dy, dx)  # Angle to the waypoint relative to the vehicle
        angle_diff = abs(normalize_angle(waypoint_angle - yaw))
        angle_diff = np.degrees(angle_diff)  # Convert angle difference to degrees

        # Check if the waypoint satisfies the distance and angle criteria
        if 3.5 <= distance <= 4 and 0 <= angle_diff <= 60:  # Tunable parameters for target selection
            calculate_curvature(path[i])
            return

# Normalizes an angle to the range [-pi, pi]
def normalize_angle(angle):
    """Normalizes an angle to the range [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

# Calculates the curvature required to reach the target point
def calculate_curvature(point):
    """Calculates the curvature required to reach the target point."""
    dx = point[0] - current_pose[0]  # X difference between the vehicle and the target point
    dy = point[1] - current_pose[1]  # Y difference between the vehicle and the target point

    # Transform the target point to the vehicle's local coordinate frame
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    local_x = cos_yaw * dx + sin_yaw * dy  # X-coordinate in local frame
    local_y = -sin_yaw * dx + cos_yaw * dy  # Y-coordinate in local frame

    if abs(local_x) < 1e-6:  # Avoid division by zero when computing curvature
        local_x = 1e-6 if local_x >= 0 else -1e-6

    # Compute curvature based on local coordinates
    curvature = 2.0 * local_y / (local_x**2 + local_y**2)
    steering_angle = np.arctan2(wheel_base * curvature, 1.0)  # Steering angle in radians
    steering_angle_deg = np.degrees(steering_angle) * -1  # Convert to degrees and invert for vehicle alignment
    steering_angle_deg = np.clip(steering_angle_deg, -20.5, 19.0)  # Limit steering angle to the vehicle's range

    publish_controls(steering_angle_deg)

# Publishes the steering angle and velocity commands
def publish_controls(steering_angle_deg):
    """Publishes the steering angle and velocity commands."""
    # Publish the computed steering angle
    steering_msg = Float32()
    steering_msg.data = steering_angle_deg
    steering_pub.publish(steering_msg)

    # Compute and publish the velocity command (reduce speed for sharp turns)
    velocity = Float32()
    velocity.data = 4 * (1 - abs(steering_angle_deg / 40))  # Tunable velocity adjustment
    cmd_pub.publish(velocity)

# Main Function
def main(args=None):
    """Main entry point for the node."""
    global steering_pub, cmd_pub
    node, cmd_pub, steering_pub = initialize_node()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()  # Cleanup the node on shutdown
    rclpy.shutdown()  # Shutdown the ROS 2 system

if __name__ == '__main__':
    main()
