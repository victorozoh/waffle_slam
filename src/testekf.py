#! /usr/bin/env python

import rospy
import numpy as np
import math
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3, PoseWithCovarianceStamped

last_time = 0.0

class ExtendedKalmanFilter(object):

    def __init__(self):
        # initial values
        self.x = np.array([0, 0, 0]) # state vector
        # self.z = np.array([0, 0, 0]) # measurement vector
        self.P = np.zeros((3,3)) ; np.fill_diagonal(self.P, 1.0) # Covariance matrix, initial uncertainty
        self.f = np.zeros((3,2)) # velocity transition matrix
        self.F = np.zeros((3,3)) # Jacobian matrix i.e F = Jacobian(f)
        self.h = np.zeros((3,2)) # Measurement Function
        self.H = np.zeros((3,3)) # Jacobian matrix i.e H = Jacobian(h)
        self.R = np.zeros((3,3)) ; np.fill_diagonal(self.R, 1.0)  # Measurement Noise/Uncertainty.
        self.Q = np.zeros((3,3)) ; np.fill_diagonal(self.Q, 1.0) # Process/Motion Noise
        self.velocity = None

    def twistCallback(self, msg):
        self.velocity = msg


    def odomCallback(self, msg):
        # get current measurements
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.x

        # vx = msg.twist.twist.linear.x
        # vy = msg.twist.twist.linear.y
        # angular_vel = msg.twist.twist.angular.z
        if self.velocity is not None:
            vx = self.velocity.linear.x
            vy = self.velocity.linear.y
            angular_vel = self.velocity.angular.z

            quaternion =   (msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            theta = euler[2]
            z = np.array([x_pos, y_pos, theta]) #actual measurement

            dt = rospy.Time.now().to_sec() - last_time # delta time
            global last_time
            last_time = rospy.Time.now().to_sec()

            # call extended kalman filter function
            new_state, new_cov = self.estimate(z, vx, vy, angular_vel, dt)

            # covariance update in odometry message form
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

            filtered_msg = Odometry()
            filtered_msg.header.stamp = rospy.Time.now()
            filtered_msg.header.frame_id = "odom"
            filtered_msg.child_frame_id = 'base_footprint'
            filtered_msg.pose.pose.position = Point(new_state[0], new_state[1], 0.0)
            filtered_msg.pose.pose.orientation = Quaternion(*(tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])))
            # filtered_msg.twist.twist.linear = Vector3(vx, vy, 0.0)
            # filtered_msg.twist.twist.angular = Vector3(0.0, 0.0, angular_vel)
            filtered_msg.pose.covariance = tuple(p_cov.ravel().tolist())

            new_ori =  (filtered_msg.pose.pose.orientation.x,
                        filtered_msg.pose.pose.orientation.y,
                        filtered_msg.pose.pose.orientation.z,
                        filtered_msg.pose.pose.orientation.w)

            # publish filtered message
            pub.publish(filtered_msg)
            # Also publish tf
            #tf_pub.sendTransform((new_state[0], new_state[1], 0.0), new_ori, filtered_msg.header.stamp, base_frame, odom_frame)
        else:
            pass

    def estimate(self, z, vx, vy, angular_vel, dt):
        # prediction/motion, velocity model
        theta = z[2] # angle
        self.f = np.array([[dt * math.cos(theta), 0],
                        [dt * math.sin(theta), 0], [0, dt]])
        linear_vel = math.sqrt((vx)**2 + (vy)**2)
        vel = np.array([linear_vel, angular_vel])
        # x' from motion/ prediction
        old_x = self.x
        old_theta = self.x[2]
        x_prime = self.x + np.dot(self.f, vel)
        # use F = Jacobian(f) to update covariance
        self.F = np.array([[1, 0, -1* linear_vel * dt* math.sin(theta)],
                            [0, 1, linear_vel * dt * math.cos(theta)], [0, 0, 1]])
        P_prime = np.dot(self.F,self.P).dot(self.F.T) + self.Q

        # measurement update
        # get h(x) and H(x) measurement models

        # compute innovation

        # compute kalman gain

        # update state
        self.x = x_prime #+ np.dot(kalman_gain, innovation)
        self.P = P_prime #- np.dot(kalman_gain,self.H).dot(P_prime)
        return self.x, self.P





if __name__ == "__main__":
    try:
        rospy.init_node('ekfnode')
        odom_frame = rospy.get_param("~output_frame",'odom')
        base_frame = rospy.get_param("~base_footprint_frame",'base_footprint')
        tf_pub = tf.TransformBroadcaster()

        ekf = ExtendedKalmanFilter()
        pub = rospy.Publisher('odom_combined', Odometry, queue_size=10)

        global last_time
        last_time = rospy.Time.now().to_sec()
        twist_sub = rospy.Subscriber('cmd_vel', Twist, ekf.twistCallback)
        odom_sub = rospy.Subscriber('odom', Odometry, ekf.odomCallback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
