#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
import numpy as np
import math
import time

#this is added to give time for other programes to start before starting this node
time.sleep(5)
rospy.init_node('ackermann_cmd_data_fuser_node')
rate = rospy.Rate(30) 

def steering_data_callback(data):
  global steering		
  steering = data.data

def speed_data_callback(data):
  global speed	
  speed = data.data

#PUBLISHERS AND SUBSCRIBERS
pub1 = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)
sub1 = rospy.Subscriber('/steering_topic', Float64, steering_data_callback)
sub2 = rospy.Subscriber('/speed_topic', Float64, speed_data_callback)

cmd_msg = AckermannDrive()
steering = 0
speed = 0

while not rospy.is_shutdown():
  
  cmd_msg.speed = speed
  cmd_msg.steering_angle = steering
  pub1.publish(cmd_msg)	
  rate.sleep()
