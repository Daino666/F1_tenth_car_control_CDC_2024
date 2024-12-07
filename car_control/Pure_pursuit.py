import csv
import rclpy
import numpy as np
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
import quaternion  

def csv_reading(file_path, column_name):
    column_data = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            if column_name in row:
                column_data.append(float(row[column_name]))
    return column_data

def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

def euler_from_quaternion(quaternion):
    """
    Convert a geometry_msgs.msg.Quaternion to Euler angles using numpy-quaternion.
    """
    # Convert quaternion to numpy-quaternion type
    q = np.quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z)

    # Compute Euler angles from quaternion
    roll = np.arctan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
    pitch = np.arcsin(2 * (q.w * q.y - q.z * q.x))
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    return roll, pitch, yaw




def convert_to_steer_Command(steering):
    command = Float32()
    command = np.clip(steering, -30, 30)
    command/=30
    return command

def get_yaw(Imu):
    global yaw
    orientation_q = Imu.orientation
    z_orien = Imu.orientation.z
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    


def get_point(Point):
    global C_pose, yaw, flag, counter, path

    C_pose = [0.0,0.0]
    C_pose[0] = Point.x
    C_pose[1] = Point.y

    for i in range (path.shape[0]):
        dx = path[i][0] - C_pose[0]
        dy = path[i][1] - C_pose[1]
        distance = np.sqrt(dx**2 + dy**2)
        #waypoint_angle = np.arctan2(dy, dx)
        #angle_diff = abs(normalize_angle(waypoint_angle - yaw))
        #angle_diff = np.degrees(angle_diff)

        if distance >= 2 and  distance <= 2.5:# and angle_diff>=0  and  angle_diff<=60: #tunable
            point =  path[i]
            calculate_curv(point)
            return



def calculate_curv(point, wheel_base =0.3240):
    global cmd_pub, steering_pub
   # Calculate relative position 
    dx = point[0] - C_pose[0]
    dy = point[1] - C_pose[1]

    # Transform to local coordinates
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    local_x = cos_yaw * dx +sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    
    # Avoid division by zero or very small numbers
    if abs(local_x) < 1e-6:
        local_x = 1e-6 if local_x >= 0 else -1e-6
    
    # Calculate curvature
    curvature = 2.0 * local_y  / (local_x**2 + local_y**2)
    
    steering_angle = np.arctan2(wheel_base * curvature, 1.0)

    # Convert to degrees and limit the steering angle
    steering_angle_deg = np.degrees(steering_angle)

    rclpy.loginfo(steering_angle_deg)

    steering_pub.publish(steering_angle_deg)
    
    cmd_pub.publish(0.01) 


def main(arg = None):
    global path, wheel_base, node, cmd_pub, steering_pub

    # Paramaeters 
    wheel_base = 0.3240
    file_path = '/home/autodrive_devkit/src/car_control/car_control/Test.csv'
    column_x = 'positions_X'
    column_y = 'positions_y'
    x_values = csv_reading(file_path, column_x)  
    y_values = csv_reading(file_path, column_y)   
    path = list(zip(x_values, y_values))
    path = np.array(path)

    #Initialize ROS2
    rclpy.init(args = arg)

    node=rclpy.create_node('PID_wall_following')

    cmd_pub = node.create_publisher(Float32, "/autodrive/f1tenth_1/throttle_command", queue_size=0) 
    steering_pub = node.create_publisher(Float32, "/autodrive/f1tenth_1/steering_command", queue_size= 0)

    node.create_subscription( Point,"/autodrive/f1tenth_1/ips", get_point)
    node.create_subscription( Imu ,"/autodrive/f1tenth_1/imu", get_yaw )



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass