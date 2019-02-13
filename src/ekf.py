#! /usr/bin/env python

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# initial values
x = np.array([0, 0, 0]) # state vector
z = np.array([0, 0, 0]) # measurement vector
P = np.random.normal(0, 10, (3,3)) # Covariance matrix, initial uncertainty
f = np.zeros((3,2)) # velocity transition matrix
F = np.zeros((3,3)) # Jacobian matrix i.e F = Jacobian(f)
H = np.zeros((3,3)) # Measurement Function
R = np.zeros((3,3)) ; np.fill_diagonal(R, 3.0)  # Measurement Noise/Uncertainty.
Q = np.zeros((3,3)) ; np.fill_diagonal(Q, 3.0) # Motion Noise


def ExtendedKalmanFilter(vx, vy, th, angular_vel, ast_time):
    dt = rospy.Time.now().to_sec() - t0 # delta time

    # prediction/motion, velocity model
    f = np.array([[dt * math.cos(th), 0],[dt * math.sin(th), 0],[0, th]])
    linear_vel = math.sqrt((vx)**2 + (vy)**2)
    vel = np.array([linear_vel, angular_vel ])
    # x' from motion/ prediction
    x = x + np.dot(f, vel)
    # use F = Jacobian(f) to update covariance
    F = np.array([[1, 0, -1*vel*dt*math.sin(th)],[0, 1, vel*dt*math.cos(th)],[0, 0, 1]])
    P = np.dot(F,P).dot(F.T) + Q

    # update




    return x, P
def callback():

    filtered = ekf.update()
    pub.publish(filtered)

if __name__ == "__main__":
    try:
        rospy.init_node('ekfnode', anonymous=True)
        t0 = rospy.Time.now().to_sec()
        pub = rospy.Publisher('filtered_odom', Odometry, queue_size=10)

        sub = rospy.Subscriber('odom',Odometry,callback)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptexception:
        pass
