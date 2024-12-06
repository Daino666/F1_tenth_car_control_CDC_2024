import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import csv
import math

# Global variables
path_x_coords = []
path_y_coords = []
num_path_values = 0
waypoints = []
vehicle_position = (0.0, 0.0)
vehicle_orientation = 0.0
publishers = {}


def csv_reading(file_path, column_name):
    """Reads a specific column from a CSV file and returns it as a list of floats."""
    data = []
    try:
        with open(file_path, 'r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            for row in csv_reader:
                data.append(float(row[column_name]))
    except Exception as e:
        print(f"Error reading CSV file: {e}")
    return data


def initialize_node():
    """Initializes the ROS 2 node and sets up publishers and subscribers."""
    global path_x_coords, path_y_coords, num_path_values, waypoints, publishers
    rclpy.init()
    node = Node('pure_pursuit')

    # Publishers
    publishers = {
        "steering": node.create_publisher(Float32, '/autodrive/f1tenth_1/steering', 10),
        "throttle": node.create_publisher(Float32, '/autodrive/f1tenth_1/throttle', 10),
    }

    # Subscribers
    node.create_subscription(Float32, '/autodrive/f1tenth_1/steering_command', process_steering_command, 10)
    node.create_subscription(Float32, '/autodrive/f1tenth_1/throttle_command', process_throttle_command, 10)

    # Load Path
    file_path = '/home/autodrive_devkit/src/car_control/car_control/Test.csv'
    path_x_coords = csv_reading(file_path, 'positions_x_odom')
    path_y_coords = csv_reading(file_path, 'positions_y_odom')
    num_path_values = len(path_x_coords)
    waypoints = list(zip(path_x_coords, path_y_coords))

    node.get_logger().info("Pure Pursuit Node Initialized")
    return node


def process_steering_command(msg):
    """Processes steering command data (if required)."""
    pass


def process_throttle_command(msg):
    """Processes throttle command data (if required)."""
    pass


def compute_distance_and_angle(target, position, orientation):
    """Computes the distance and angle to a target waypoint."""
    dx = target[0] - position[0]
    dy = target[1] - position[1]
    distance = math.sqrt(dx ** 2 + dy ** 2)
    angle_to_target = math.degrees(math.atan2(dy, dx))
    angle_diff = (angle_to_target - math.degrees(orientation) + 360) % 360
    if angle_diff > 180:
        angle_diff -= 360
    return distance, angle_diff


def select_target_waypoint():
    """Selects the next waypoint based on the current vehicle position and orientation."""
    for waypoint in waypoints:
        distance, angle_diff = compute_distance_and_angle(waypoint, vehicle_position, vehicle_orientation)
        if 3.5 <= distance <= 4.0 and abs(angle_diff) <= 60:
            return waypoint
    return None


def compute_steering_command(target_waypoint):
    """Computes the steering command for the vehicle to follow the target waypoint."""
    if not target_waypoint:
        return 0.0
    local_x = target_waypoint[0] - vehicle_position[0]
    local_y = target_waypoint[1] - vehicle_position[1]
    if local_x == 0:
        local_x = 0.001  # Prevent division by zero
    curvature = 2 * local_y / (local_x ** 2 + local_y ** 2)
    steering_angle = math.atan(curvature)
    return steering_angle


def publish_drive_commands():
    """Publishes the drive commands (steering and throttle)."""
    target_waypoint = select_target_waypoint()
    steering_angle = compute_steering_command(target_waypoint)

    # Throttle calculation based on steering angle
    steering_angle_deg = math.degrees(steering_angle)
    velocity = 4 * (1 - abs(steering_angle_deg / 40))
    velocity = max(0.5, min(velocity, 4))

    # Publish commands
    publishers["steering"].publish(Float32(data=steering_angle))
    publishers["throttle"].publish(Float32(data=velocity))


def main():
    """Main entry point for the pure pursuit node."""
    node = initialize_node()
    rate = node.create_rate(10)  # 10 Hz

    try:
        while rclpy.ok():
            publish_drive_commands()
            rclpy.spin_once(node, timeout_sec=0.1)
            rate.sleep()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Pure Pursuit Node.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
