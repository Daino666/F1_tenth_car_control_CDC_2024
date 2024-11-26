import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32

rclpy.init()
node=rclpy.create_node('PID_wall_following')

steering_pub= node.create_publisher(Float32, 'autodrive/f1tenth_1/steering_command', 10)
throttle_pub= node.create_publisher(Float32, 'autodrive/f1tenth_1/throttle_command', 10)

def get_angle_index(scan, angle):#to get the index of the angle in the scanning range
    index=angle*len(scan.ranges)/((-1 *scan.angle_min + scan.angle_max)*180/np.pi )
    return(int(index))

def lidar_callback(scan):
    global steering_pub
    global throttle_pub
    e_of_t_1=0
    i_e=0
    Kp=2.7
    Kd=1.4
    Ki=0.001
    prev_rd=0.0
    prev_ld=0.0

    throttle = Float32()
    steering_angle = Float32()

    #getting distance to the right wall
    try:
        theta= 30
        ang_Neg90_distance=scan.ranges[get_angle_index(scan, -90)]
        ang_Neg60_distance=scan.ranges[get_angle_index(scan,-90+theta)]

        ang_90_distance=scan.ranges[get_angle_index(scan, 90)]
        ang_60_distance=scan.ranges[get_angle_index(scan,90-theta)]
        
        angle_to_Rwall= -1* math.atan((ang_90_distance*math.cos(theta)-ang_60_distance)/(ang_90_distance*math.sin(theta)))+0.5291789535531515
        angle_to_Lwall= -1* math.atan((ang_Neg90_distance*math.cos(theta)-ang_Neg60_distance)/(ang_Neg90_distance*math.sin(theta)))+0.5291789535531515
        D_L=ang_Neg60_distance*math.cos(math.radians(angle_to_Lwall))
        D_R=ang_60_distance*math.cos(math.radians(angle_to_Rwall))

###     
        if math.isinf(ang_Neg90_distance) or math.isinf(ang_Neg60_distance):
            if(prev_ld==0):
                prev_ld=0.6
            steering_angle.data=prev_ld-D_R
            if abs(angle_to_Rwall)>0.1:
                steering_angle.data-=5*angle_to_Rwall
        elif math.isinf(ang_90_distance) or math.isinf(ang_60_distance):
            if(prev_rd==0):
                prev_rd=0.6
            steering_angle.data=D_L-prev_rd
            if abs(angle_to_Lwall)>0.1:
                steering_angle.data+=5*angle_to_Lwall
            
        else:
            steering_angle.data=(D_L-D_R)/2
            if abs(angle_to_Lwall)>0.1:
                steering_angle.data+=2.5*angle_to_Lwall
            if abs(angle_to_Rwall)>0.1:
                steering_angle.data-=2.5*angle_to_Rwall

        steering_angle.data=max(-0.5, min(0.5, steering_angle.data))
        steering_pub.publish(steering_angle)

        prev_ld=D_L
        prev_rd=D_R

        #throttle.data=0.02
        throttle.data=min(0.02,max(1/abs(steering_angle.data*150), 0.02))
        #####sure right

        
    except Exception as e:
        throttle.data = 0.00
        node.get_logger().warn(f"Error in LiDAR callback: {e}")
        if e=="ang_Neg90_distance is inf." or e=="ang_Neg60_distance is inf.":
            steering_angle.data=0.6-D_R
            ang_Neg90_distance=1
            ang_Neg60_distance=1
        elif e=="ang_90_distance is inf." or e=="ang_60_distance is inf.":
            steering_angle.data=D_R-0.6
            ang_90_distance=1
            ang_90_distance=1

    throttle_pub.publish(throttle)

    def steering_callback(msg):
        print(f"steering_angle={steering_angle.data:.2f}, act_steer={msg.data:.2f}")

    steer_sub=node.create_subscription(Float32, 'autodrive/f1tenth_1/steering_command', steering_callback, 0)


lidar_sub=node.create_subscription(LaserScan, 'autodrive/f1tenth_1/lidar', lidar_callback, 0)

#orientation=node.create_subscription(Imu, '/autodrive/f1tenth_1/imu', imu_callback, 1)


rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()

