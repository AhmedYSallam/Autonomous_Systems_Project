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
rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_21_node')
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

def Speed_Control(desired_v,actual_velocity,t):
  kp = 5
  u = kp*(desired_v - actual_velocity)
  v = actual_velocity + t*u
  return v

def pose_callback(data):
  global real_position,real_speed		
  vehicle_index = data.name.index("ackermann_vehicle")
  real_position = data.pose[vehicle_index]
  real_speed = data.twist[vehicle_index]

def desired_velocity_callback(data):
  global desired_v	
  desired_v = data.data
  print('desired_v = '+str(desired_v))
  

#PUBLISHERS AND SUBSCRIBERS
pub1 = rospy.Publisher('/speed_topic', Float64, queue_size=10)
sub1 = rospy.Subscriber('/gazebo/model_states', ModelStates, pose_callback)
sub2 = rospy.Subscriber('/planner_speed_topic', Float64, desired_velocity_callback)

#Inputs to controller
t = 0.0
v = 0

real_position = Pose()
real_speed = Twist()
desired_v = 0

while not rospy.is_shutdown():
  start_t = time.time()
  orientation = quaternion_to_euler(real_position.orientation.x, real_position.orientation.y, real_position.orientation.z, real_position.orientation.w)
  yaw = orientation[0] 
  actual_pose = [real_position.position.x, real_position.position.y,yaw]
  actual_velocity = real_speed.linear.x
  v = Speed_Control(desired_v,actual_velocity,t)
  print('actual_velocity = '+str(actual_velocity))
  print('actual_pose = '+str(actual_pose))
  pub1.publish(v)	
  rate.sleep()		
  end_t = time.time()
  t  = end_t - start_t





