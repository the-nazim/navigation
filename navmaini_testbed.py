<<<<<<< HEAD
#::::::::::::::::::::::::::::::::::::  LIDAR NAVIGATION CODE FOR MAINI 2 OnTEST new change#::::::::::::::::::::::::::::::::::::#
=======
#::::::::::::::::::::::::::::::::::::  LIDAR NAVIGATION CODE FOR MAINI 2  ON TEST ::::::::::::::::::::::::::::::::::::#
>>>>>>> a7e74d867cb4f7b21b74c06909bb9633229ecafc
#!/usr/bin/env python3
import math 
import numpy as np   
import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, UInt16, Float32, Float64, Float32MultiArray


current_x = 0
current_y = 0
heading = 0
wp = 0
vehicle_velocity = 0
velocity_value = 0
control_effort = 0
current_steering_angle = 0
steering_output = 0
obstacle_distance = 0
previous_velocity=0

def extract_waypoints_from_file(file_path):
    
    try:
        with open(file_path, 'r') as file:
            file_contents = file.read()
    except FileNotFoundError:
        print("File not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
    waypoints = []
    waypoints_data = file_contents.strip().split('\n')
    for line in waypoints_data:
        x, y = map(float, line.strip("[],").split(','))
        waypoints.append([x, y])
    return(waypoints)


def callback_ndt_pose(data):
   
   global current_x
   global current_y
   current_x = data.pose.position.x
   current_y = data.pose.position.y

   print("Current co-ordinates : ", current_x, ", ", current_y)

   
def callback_cur_steer(data):
   
   global current_steering_angle
   current_steering_angle=data.data


def callback_vehicle_speed(data):

    global vehicle_velocity
    vehicle_velocity = data.data


def callback_min_distance(data):
   
   global obstacle_distance
   obstacle_distance = data.data


def callback_eular_angle(data):
   
   global heading
   heading = math.degrees(data.data[2]) 


def listener():
   
   rospy.init_node("topic_subscriber")
   rospy.Subscriber("/ndt_pose", PoseStamped, callback_ndt_pose)
   rospy.Subscriber("/eular_angle", Float32MultiArray, callback_eular_angle)
   rospy.Subscriber("/currentSteerAngle", Float64, callback_cur_steer)
   rospy.Subscriber("/lidar_min_distance_topic", Float32, callback_min_distance)
   rospy.Subscriber("/vehicle_speed", Float64, callback_vehicle_speed)


listener()
rospy.wait_for_message('/ndt_pose', PoseStamped)


def calc_steer_output(required_bearing, Ld_steer):

    global steering_output

    current_bearing = float(heading)
    bearing_diff  = required_bearing - current_bearing

    if (bearing_diff < -180):
        bearing_diff = bearing_diff + 360

    if (bearing_diff > 180):
        bearing_diff = bearing_diff - 360

    #print("Current bearing : ", current_bearing)
    #print("Required bearing : ", required_bearing)
    #print("Bearing difference : ", bearing_diff)

    steering_output = 45 * np.arctan( -1 * 2 * 3.5 * np.sin(np.pi * bearing_diff / 180) / Ld_steer )


def calc_velocity():

    scale = 0.80

    # velocity_steer=(100 - (60 * math.exp(-0.10 * (30 - abs(steering_output)))))        # decreasing
    # velocity_steer=(40 / math.exp(-0.03 * (30 - abs(steering_output))))                # increasing
    velocity_steer = 55 + 45 * (30 - abs(steering_output)) / 30                          # linear

    velocity_brake=max(0,100 - 85 * math.exp(-2.5 * (obstacle_distance - 2.0)/(15)))

    velocity_steer = velocity_steer * scale
    velocity_brake = velocity_brake * scale
    
    return(min(velocity_steer, velocity_brake))


def control_velocity(previous_velocity, velocity_value):
    if (velocity_value>previous_velocity):
        velocity_value=previous_velocity+1
    return velocity_value

def calc_control_effort():
    
    global control_effort
    control_effort = 1200 + 75*abs(steering_output)

def find_nearest_waypoint_index(waypoints):

    distance_threshold = 3
    min_distance = float("inf")
    next_index = -1

    for i in range(len(waypoints) - 1):
        waypoint = waypoints[i]
        waypoint_x, waypoint_y = waypoint
        distance = ((waypoint_x - current_x) ** 2 + (waypoint_y - current_y) ** 2) ** 0.5

        if distance < min_distance and distance < distance_threshold: 
            min_distance = distance
            next_index = i
    
    print("The next nearest waypoint index is : ", next_index)
    return(next_index)



waypoints = extract_waypoints_from_file("/home/tihan/testbed_demo/waypoints_unf_dis.txt")

wp = find_nearest_waypoint_index(waypoints)



pub_cart_control = rospy.Publisher('/cart_control', Int8, queue_size=10)
cart_control = Int8()

pub_sa = rospy.Publisher('/mpc/steer_angle', Float64, queue_size=10)
steer_op = Float64()

pub_ce = rospy.Publisher('/control_effort', Float64, queue_size=10)
c_effort = Float64()

pub_v = rospy.Publisher('/mpc/velocity_value', UInt16, queue_size=10)
velocity_v = UInt16()



def publish_cart_control(cart_control_data):

    cart_control.data = cart_control_data
    pub_cart_control.publish(cart_control)


def publish_streer_angle(steer_op_data):

    steer_op.data = steer_op_data
    pub_sa.publish(steer_op)
    print("Steering output : ", steer_op_data)


def publish_control_effort(c_effort_data):

    c_effort.data = c_effort_data
    pub_ce.publish(c_effort)
    print("Control effort : ", c_effort_data)


def publish_velocity(velocity_value_data):

    velocity_v.data = int(velocity_value_data) 
    pub_v.publish(velocity_v)
    print("Velocity : ", velocity_value_data)




while True:

    distance = math.sqrt((current_x - waypoints[wp][0]) ** 2 + (current_y - waypoints[wp][1]) ** 2)

    print("Current_x : ", current_x)
    print("Current_y : ", current_y)
    print("Distance between current position and next waypoint is : ", distance)
    #print("Current heading is : ", heading)

    if (wp == len(waypoints)-1):
        publish_velocity(0)
        publish_control_effort(0)
        publish_streer_angle(0)
        break

    if (wp < len(waypoints)-1):

        print("Obstacle distance : ", obstacle_distance)

        off_x = waypoints[wp][0] - current_x
        off_y = waypoints[wp][1] - current_y

        bearing_ppc=math.degrees(math.atan2(off_y, off_x))

        if bearing_ppc < 0 :
            bearing_ppc += 360

        Ld_steer = 8
        calc_steer_output(bearing_ppc, Ld_steer)
        publish_streer_angle(steering_output)

        calc_control_effort()
        velocity_value = calc_velocity()
        
        velocity_value=control_velocity(previous_velocity, velocity_value)
        previous_velocity=velocity_value
        
        # Start slow
        if (wp<15):  
            velocity_value=min(50,velocity_value)
        

        # Stop slow
        if (wp>170):  
            velocity_value=min(50,velocity_value)   

        publish_control_effort(control_effort)
        publish_velocity(velocity_value)
        

        Ld = 3.75

        if (distance < Ld) and (wp < len(waypoints)) :
            wp = wp + 1
    
    print("Waypoint Index : ", wp, "..........................................................")
    
    time.sleep(0.1)

#:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#




'''
Updated by Rakshith on 18-1-24
note:
1. paste pat of waypoint file in line 149
2. Start from intermediate waypoints - Working
'''
