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
rospy.init_node('Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_21_node')
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

def stanley_controller(desired_lane,actual_pose,actual_velocity):
  kv = 3
  global wheel_base
  lane = actual_pose[0] + np.sign(actual_velocity)
  Lookahead_pnt = [lane,desired_lane]
  dx = Lookahead_pnt[0]-(actual_pose[0]+wheel_base*np.cos(actual_pose[2]))
  dy = Lookahead_pnt[1]-(actual_pose[1]+wheel_base*np.sin(actual_pose[2]))
  thetaP = 0 - actual_pose[2] 
  try:
    steering = thetaP + np.arctan(((kv*dy)/(actual_velocity)))
  except:
    steering = 0
  if(abs(steering) >= np.radians(30)):
    steering = np.sign(steering)*np.radians(30)
  return steering

def pose_callback(data):
  global real_position,real_speed		
  vehicle_index = data.name.index("ackermann_vehicle")
  real_position = data.pose[vehicle_index]
  real_speed = data.twist[vehicle_index]

def desired_steering_callback(data):
  global desired_lane	
  desired_lane = data.data
  print('desired_lane = ' + str(desired_lane))

#PUBLISHERS AND SUBSCRIBERS
pub1 = rospy.Publisher('/steering_topic', Float64, queue_size=10)
sub1 = rospy.Subscriber('/gazebo/model_states', ModelStates, pose_callback)
sub2 = rospy.Subscriber('/planner_steering_topic', Float64, desired_steering_callback)

wheel_base = rospy.get_param("~Wheel_base")
steering = 0
desired_lane = 0
real_speed = Twist()
real_position = Pose()

while not rospy.is_shutdown():

  orientation = quaternion_to_euler(real_position.orientation.x, real_position.orientation.y, real_position.orientation.z, real_position.orientation.w)
  yaw = orientation[0]
  actual_velocity = real_speed.linear.x
  actual_pose = [real_position.position.x, real_position.position.y,yaw]
  steering = stanley_controller(desired_lane,actual_pose,actual_velocity)
  pub1.publish(steering)	
  rate.sleep()		
 



