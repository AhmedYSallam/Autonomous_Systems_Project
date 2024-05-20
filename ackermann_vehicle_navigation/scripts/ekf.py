#!/usr/bin/env python
import numpy as np
import rospy
import math
import tf
import time
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from std_msgs.msg import Float64

np.set_printoptions(precision=3, suppress=True)

#########################################################################################

time.sleep(5)  # Sleep till everything runs smoothly
rospy.init_node("EKF")  # Initializing the node
rate = rospy.Rate(30)  # Rate of 30Hz
dk = 1 / 30  # Time interval between each operation (Seconds)

#########################################################################################

# Variables
sensor_readings = [0, 0, 0]
cur_pos_x = 0
cur_pos_y = 0
cur_heading = 0
cur_vel = 0
cur_ang_vel = 0
steer = 0

## Global Variables
H_mat = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])  # Measurement Matrix

v_matrix = np.array([0.03, 0.05, 0.005])  # Noise applied to the kinematic model

Q_cov = np.array([[0.01, 0, 0],
                  [0, 0.01, 0],
                  [0, 0, 0.01]])  # State Model noise Covariance

R_mat = np.array([[0.01, 0, 0],
                  [0, 0.01, 0],
                  [0, 0, 0.01]])

sensor_noise = np.array([0,
                         0,
                         0])

noise = np.array([np.random.normal(0, 1),
                  np.random.normal(0, 1),
                  np.random.normal(0, 1)])

sensor_noise = sensor_noise + noise

#########################################################################################

def pose_callback(msg):
    global cur_pos_x, cur_pos_y, cur_vel, cur_heading, cur_ang_vel
    vehicle_index = msg.name.index("ackermann_vehicle")
    cur_pos_x = msg.pose[vehicle_index].position.x
    cur_pos_y = msg.pose[vehicle_index].position.y
    cur_vel = msg.twist[vehicle_index].linear.x
    cur_ang_vel = msg.twist[vehicle_index].angular.z

    # Robot Heading
    quaternion = (msg.pose[vehicle_index].orientation.x,
                  msg.pose[vehicle_index].orientation.y,
                  msg.pose[vehicle_index].orientation.z,
                  msg.pose[vehicle_index].orientation.w)  # Getting all info regarding the orientation

    euler = tf.transformations.euler_from_quaternion(quaternion)  # Transform Quaternion to Euler

    cur_heading = euler[2]  # Car Heading

def steer_callback(msg):
    global steer
    steer = msg.data

#########################################################################################

# Generate B Matrix
def getB(yaw, dk, steer):
    """
    Calculates B matrix
    :param yaw: Yaw angle of the Robot
    :param dk: Time step
    """

    L = 0.25
    B = np.array([[np.cos(yaw) * dk, 0],
                  [np.sin(yaw) * dk, 0],
                  [np.tan(steer) / L, 0]])

    return B

# Generate A Matrix
def getA():
    A = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])
    return A

#########################################################################################

# EKF
def EKF(state_estimate_km1, dk, control_input_km1, P_cov_km1, Z_mat, steer):
    # State Space
    states_k = getA() @ state_estimate_km1 + getB(state_estimate_km1[1], dk, steer) @ control_input_km1 + v_matrix
    ## Same as X = AX + BU + V

    print(f'\nStates before EKF = {states_k}')

    # State Covariance
    P_cov_k = getA() @ P_cov_km1 @ getA().T + Q_cov

    # Diff between sensor and prediction at time K
    diff = Z_mat - (H_mat @ states_k) + sensor_noise

    print(f'Sensor={Z_mat}')

    # Residual Covariance
    S_mat = H_mat @ P_cov_k @ H_mat.T + R_mat

    # Near optimal Kalman Gain
    Kalman_Gain = P_cov_k @ H_mat.T @ np.linalg.inv(S_mat)

    # Update state Estimate
    state_estimate_k = states_k + (Kalman_Gain @ diff)

    # Update Covariance
    P_cov_k = P_cov_k - (Kalman_Gain @ H_mat @ P_cov_k)

    print(f'State Estimate After EKF={state_estimate_k}')
    print(f'Speed = {cur_vel}')

    return state_estimate_k, P_cov_k

#########################################################################################

# Subs and Pubs
state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, pose_callback)
odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)

# Main Function
K = 0  # Starting At time step 1

Z_mat = np.array([0, 0, 0])

state_estimate_km1 = np.array([0.0,
                               0.0,
                               0.0])

## control matrix is [V, W] ##
control_input_km1 = np.array([0,
                              0])

P_cov_km1 = np.array([[0.1, 0, 0],
                      [0, 0.1, 0],
                      [0, 0, 0.1]])

#########################################################################################

while not rospy.is_shutdown():
    print(f'Time step = {K}')

    optimal_state_estimate_k, P_k = EKF(
        state_estimate_km1,
        dk,
        control_input_km1,
        P_cov_km1,
        Z_mat,
        steer
    )
    Z_mat = np.array([cur_pos_x, cur_pos_y, cur_heading])
    control_input_km1 = np.array([cur_vel, cur_ang_vel])
    state_estimate_km1 = optimal_state_estimate_k
    P_cov_km1 = P_k
    K += dk

    # Create and publish the Odometry message
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    # Set the position
    odom_msg.pose.pose.position.x = optimal_state_estimate_k[0]
    odom_msg.pose.pose.position.y = optimal_state_estimate_k[1]
    odom_msg.pose.pose.position.z = 0.0

    # Convert heading to quaternion
    quaternion = tf.transformations.quaternion_from_euler(0, 0, optimal_state_estimate_k[2])
    odom_msg.pose.pose.orientation = Quaternion(*quaternion)

    # Set the velocity
    odom_msg.twist.twist.linear.x = cur_vel
    odom_msg.twist.twist.angular.z = cur_ang_vel

    odom_pub.publish(odom_msg)
    rate.sleep()
