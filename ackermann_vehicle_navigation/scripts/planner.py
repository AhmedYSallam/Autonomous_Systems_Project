#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose,Twist
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import time

#this is added to give time for other programes to start before starting this node
time.sleep(5)
rospy.init_node('planner_node') 
rate = rospy.Rate(30) 

def quaternion_to_euler(x, y, z, w):
   t0 = +2.0 * (w * x + y * z)
   t1 = +1.0 - 2.0 * (x * x + y * y)
   roll = math.atan2(t0, t1)
   t2 = +2.0 * (w * y - z * x)
   t2 = +1.0 if t2 > +1.0 else t2
   t2 = -1.0 if t2 < -1.0 else t2
   pitch = math.asin(t2)
   t3 = +2.0 * (w * z + x * y)
   t4 = +1.0 - 2.0 * (y * y + z * z)
   yaw = math.atan2(t3, t4)
   return [yaw, pitch, roll]

def pose_callback(data):
   global real_position,real_speed	
   vehicle_index = data.name.index("ackermann_vehicle")
   real_position = data.pose[vehicle_index]
   real_speed = data.twist[vehicle_index]

#PUBLISHERS AND SUBSCRIBERS
pub1 = rospy.Publisher('/planner_speed_topic', Float64, queue_size=10)
pub2 = rospy.Publisher('/planner_steering_topic', Float64, queue_size=10)
sub1 = rospy.Subscriber('/gazebo/model_states', ModelStates, pose_callback)

#How far do you want to the car to go in the x direction
x_pos_des = rospy.get_param("~x_desired")
#Add as much lanes and different speed as you desire below 
lane_1 = rospy.get_param("~lane_1_desired") 
lane_2 = rospy.get_param("~lane_2_desired") 
min_speed = rospy.get_param("~min_speed") 
max_speed = rospy.get_param("~min_speed") 

real_speed = Twist()
real_position = Pose()
desired_v = 0 #m/s
desired_lane = 0.0 #m

while not rospy.is_shutdown():
   desired_v = 0.0
   desired_lane = 0.0
   orientation = quaternion_to_euler(real_position.orientation.x, real_position.orientation.y, real_position.orientation.z, real_position.orientation.w)
   yaw = orientation[0] 
   actual_pose = [real_position.position.x, real_position.position.y,yaw]
   x_pos=actual_pose[0]
   while x_pos < x_pos_des:
         actual_pose = [real_position.position.x, real_position.position.y,yaw]
         x_pos=actual_pose[0]
         if x_pos < 3.0:
            desired_v = max_speed
            desired_lane = lane_1
         elif x_pos > 8:
            desired_v = min_speed
            desired_lane = lane_1
         else:
            desired_v = max_speed
            desired_lane = lane_2
         pub1.publish(desired_v)    
         pub2.publish(desired_lane)    
         rate.sleep()        
   pub1.publish(desired_v)    
   pub2.publish(desired_lane)    
   rate.sleep()        





