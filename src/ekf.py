#! /usr/bin/env python

import rospy
import numpy as np
import math
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped


class ExtendedKalmanFilter(object):

    def __init__(self):
        # initial values
        self.x = np.array([0, 0, 0]) # state vector
        # self.z = np.array([0, 0, 0]) # measurement vector
        self.P = np.zeros((3,3)) ; np.fill_diagonal(self.P, 3.0) # Covariance matrix, initial uncertainty
        self.f = np.zeros((3,2)) # velocity transition matrix
        self.F = np.zeros((3,3)) # Jacobian matrix i.e F = Jacobian(f)
        self.h = np.zeros((3,2)) # Measurement Function
        self.H = np.zeros((3,3)) # Jacobian matrix i.e H = Jacobian(h)
        self.R = np.zeros((3,3)) ; np.fill_diagonal(self.R, 3.0)  # Measurement Noise/Uncertainty.
        self.Q = np.zeros((3,3)) ; np.fill_diagonal(self.Q, 3.0) # Motion Noise

    def estimate(self, z, vx, vy, angular_vel, dt):
        # prediction/motion, velocity model
        theta = z[2] # angle
        self.f = np.array([[dt * math.cos(theta), 0],[dt * math.sin(theta), 0],[0, dt]])
        linear_vel = math.sqrt((vx)**2 + (vy)**2)
        vel = np.array([linear_vel, angular_vel ])
        # x' from motion/ prediction
        old_x = self.x
        old_theta = self.x[2]
        x_prime = self.x + np.dot(self.f, vel)
        # use F = Jacobian(f) to update covariance
        self.F = np.array([[1, 0, -1* linear_vel * dt* math.sin(theta)],[0, 1, linear_vel * dt * math.cos(theta)],[0, 0, 1]])
        P_prime = np.dot(self.F,self.P).dot(self.F.T) + self.Q

        # update
        dth = angular_vel * dt
        dtrans = math.sqrt((x_prime[0] - old_x[0])**2 + (x_prime[1] - old_x[1])**2)
        self.h = np.array([[math.cos(old_theta + dth/2), 0],[math.sin(old_theta + dth/2), 0],[0, 1]])
        disp = np.array([dtrans, dth]) # displacement
        pred_z = old_x + np.dot(self.h, disp)
        # innovation
        innovation = z - pred_z
        self.H = np.array([[1, 0, -1* dtrans * math.sin(old_theta + dth/2)],[0, 1, dtrans * math.cos(old_theta + dth/2)],[0, 0, 1]])
        innovation_covariance = np.dot(self.H,P_prime).dot(self.H.T) + self.R
        #  compute Kalman gain
        kalman_gain = np.dot(P_prime, self.H.T).dot(np.linalg.inv(innovation_covariance))

        self.x = x_prime + np.dot(kalman_gain, innovation)
        self.P = P_prime - np.dot(kalman_gain,self.H).dot(P_prime)
        return self.x, self.P


def callback(msg):
    # get current measurements
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.x
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    angular_vel = msg.twist.twist.angular.z
    quaternion =   (msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    th = euler[2]
    z = np.array([x_pos, y_pos, th]) #actual measurement
    # call extended kalman filter function
    dt = rospy.Time.now().to_sec() - last_time # delta time
    new_state, new_cov = ekf.estimate(z, vx, vy, angular_vel, dt)

    p_cov = np.array([0.0]*36).reshape(6,6)

    # position covariance
    p_cov[0:2,0:2] = new_cov[0:2,0:2]
    # orientation covariance for Yaw
    # x and Yaw
    p_cov[5,0] = p_cov[0,5] = new_cov[2,0]
    # y and Yaw
    p_cov[5,1] = p_cov[1,5] = new_cov[2,1]
    # Yaw and Yaw
    p_cov[5,5] = new_cov[2,2]

    euler = list(euler)
    euler[2] = new_state[2]

    filtered_msg = PoseWithCovarianceStamped()
    filtered_msg.header.stamp = rospy.Time.now()
    filtered_msg.header.frame_id = "odom_combined"
    filtered_msg.pose.covariance = tuple(p_cov.ravel().tolist())
    filtered_msg.pose.pose.position.x = new_state[0]
    filtered_msg.pose.pose.position.y = new_state[1]
    filtered_msg.pose.pose.position.z = 0.0
    filtered_msg.pose.pose.orientation = Quaternion(*(tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])))
    # publish filtered message
    pub.publish(filtered_msg)



if __name__ == "__main__":
    try:
        rospy.init_node('ekfnode')
        last_time = rospy.Time.now().to_sec()

        tf_br = tf.TransformBroadcaster()
        ekf = ExtendedKalmanFilter()
        pub = rospy.Publisher('odom_combined', PoseWithCovarianceStamped, queue_size=10)
        sub = rospy.Subscriber('odom',Odometry,callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
