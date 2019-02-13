#! /usr/bin/env python

import rospy
import numpy as np
import math
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# initial values
x = np.array([0, 0, 0]) # state vector
z = np.array([0, 0, 0]) # measurement vector
P = np.random.normal(0, 10, (3,3)) # Covariance matrix, initial uncertainty
f = np.zeros((3,2)) # velocity transition matrix
F = np.zeros((3,3)) # Jacobian matrix i.e F = Jacobian(f)
h = np.zeros((3,2)) # Measurement Function
H = np.zeros((3,3)) # Jacobian matrix i.e H = Jacobian(h)
R = np.zeros((3,3)) ; np.fill_diagonal(R, 3.0)  # Measurement Noise/Uncertainty.
Q = np.zeros((3,3)) ; np.fill_diagonal(Q, 3.0) # Motion Noise


def ExtendedKalmanFilter(z, vx, vy, angular_vel, dt):

    # prediction/motion, velocity model
    theta = z[2] # angle
    f = np.array([[dt * math.cos(theta), 0],[dt * math.sin(theta), 0],[0, dt]])
    linear_vel = math.sqrt((vx)**2 + (vy)**2)
    vel = np.array([linear_vel, angular_vel ])
    # x' from motion/ prediction
    old_x = x
    x = x + np.dot(f, vel)
    # use F = Jacobian(f) to update covariance
    F = np.array([[1, 0, -1*vel*dt*math.sin(theta)],[0, 1, vel*dt*math.cos(theta)],[0, 0, 1]])
    P = np.dot(F,P).dot(F.T) + Q

    # update
    # innovation
    dth = angular_vel * dt

    h = np.array([[math.cos(theta + dth/2), 0],[math.sin(theta + dth/2), 0],[0, 1]])
    pred_z = old_x + np.dot(h,)
    y = z - pred_z

    return x, P


def callback(msg):
    # get current measurements
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.x
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    angular_vel = msg.twist.twist.angular.z
    quaternion = msg.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion(quaternion)
    th = euler[2]
    z = np.array([x, y, th]) #actual measurement
    # call extended kalman filter function
    filtered = Odometry()
    dt = rospy.Time.now().to_sec() - last_time # delta time
    filtered = ExtendedKalmanFilter(z, vx, vy, angular_vel, dt)

    # publish filtered message
    pub.publish(filtered)

if __name__ == "__main__":
    try:
        rospy.init_node('ekfnode', anonymous=True)
        last_time = rospy.Time.now().to_sec()

        pub = rospy.Publisher('filtered_odom', Odometry, queue_size=10)
        sub = rospy.Subscriber('odom',Odometry,callback)

        rospy.spin()
    except rospy.ROSInterruptexception:
        pass
