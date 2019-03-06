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

    def __init__(self, dim_z, dim_x = 3):
        # dim_z corresponds to the dimensions of the measurements
        # dim_x corresponds to the dimensions of the state. In this case, [x, y, theta]
        self.dim_x = dim_x
        self.dim_z = dim_z
        # initial values
        self.noise_value = 0.5
        self.x = np.zeros((dim_x,1)) # state vector
        #self.z = np.array([0, 0, 0]) # measurement vector-holds time t-1 measurement
        self.P = np.eye(dim_x) * self.noise_value # Covariance matrix, initial uncertainty
        self.Q = np.eye(dim_x) * self.noise_value # Process/Motion Noise
        self.f = np.eye(dim_x) # x = f(x) + Bu
        self.F = np.eye(dim_x) # Jacobian matrix of motion model for updating covariance
        self.B = None#np.zeros((dim_x,2)) # controls matrix
        self.h = np.eye(3,3) # Measurement Function for odometry
        self.H = np.zeros((dim_z,dim_x)) # Jacobian matrix i.e H = Jacobian(h)
        self.R = np.eye(dim_z) * self.noise_value  # Measurement Noise/Uncertainty.

        self.control_velocity = None # current velocity commands from twist message on cmd_vel topic
        self.odom_data = None # holds current odometry velocities
        self.imu_data = None # current data from /imu topic
        self.old_control_vel = np.array([0.0]*3)

    def twistCallback(self, msg):
        self.control_velocity = msg

    def imuCallback(self, msg):
        self.imu_data = msg

    def odomCallback(self, msg):
            self.odom_data = msg

            # update and publish with all the measurements gathered
            self.update_and_publish()
            # Also publish tf
            #tf_pub.sendTransform((new_state[0], new_state[1], 0.0), new_ori, filtered_msg.header.stamp, base_frame, odom_frame)

    def update_and_publish(self):
        if not None in [self.control_velocity, self.imu_data, self.odom_data]:
            # controls
            vel_controls = (self.control_velocity.linear.x,
                            self.control_velocity.linear.y,
                            self.control_velocity.angular.z)

            # current Imu measurements
            quat_imu = (self.imu_data.orientation.x,
                        self.imu_data.orientation.y,
                        self.imu_data.orientation.z,
                        self.imu_data.orientation.w)
            euler_imu = tf.transformations.euler_from_quaternion(quat_imu)
            imu_meas = (self.imu_data.linear_acceleration.x,
                        self.imu_data.linear_acceleration.y,
                        euler_imu[2])

            # current odom measurements
            quaternion =   (self.odom_data.pose.pose.orientation.x,
                            self.odom_data.pose.pose.orientation.y,
                            self.odom_data.pose.pose.orientation.z,
                            self.odom_data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            odom_meas = (self.odom_data.pose.pose.position.x,
                         self.odom_data.pose.pose.position.y,
                         euler[2])

            dt = rospy.Time.now().to_sec() - last_time # delta time
            global last_time
            last_time = rospy.Time.now().to_sec()

            # call extended kalman filter function
            new_state, new_cov = self.estimate(vel_controls, imu_meas, odom_meas, dt)

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
            filtered_msg.pose.covariance = tuple(p_cov.ravel().tolist())

            # publish filtered message
            pub.publish(filtered_msg)
        else:
            pass


    def estimate(self, vel_controls, imu_meas, odom_meas, dt):
        # prediction/motion, velocity model
        theta = odom_meas[2] # angle
        self.B = np.array([[dt * math.cos(theta), 0],
                        [dt * math.sin(theta), 0],
                        [0, dt]])
        linear_vel = math.sqrt((vel_controls[0])**2 + (vel_controls[1])**2)
        vel = np.array([linear_vel, vel_controls[2]])
        vel = vel.reshape((2,1))
        # x' from motion/ prediction
        old_x = self.x
        x_prime = np.dot(self.f, self.x) + np.dot(self.B, vel)
        # use F = Jacobian(f) to update covariance
        self.F = np.array([[1, 0, -1 * linear_vel * dt* math.sin(theta)],
                            [0, 1, linear_vel * dt * math.cos(theta)],
                            [0, 0, 1]])
        #self.F = np.eye(3,3)
        P_prime = np.dot(self.F, self.P).dot(self.F.T) + self.Q

        # measurement update->odometry model. compute residual/innovation
        # get h(x) and H(x) measurement models
        predicted_z = np.dot(self.h, x_prime) # using odometry measurement
        innovation = np.asarray(odom_meas).reshape((3,1)) - predicted_z # innovation using odometry measurements
        self.H = np.eye(3)

        #predicted_z = x_prime[2] # using only Imu orientation
        # innovation = imu_meas[2] - predicted_z # now a numpy array
        # self.H = np.array([0 ,0 , 1]) # because of one measurement
        #rospy.loginfo('The shape of innovation is %s', innovation.shape)

        # ensure shapes match for gain calculations
        innovation = innovation.reshape((self.dim_z, 1))
        self.H = self.H.reshape((self.dim_z, self.dim_x))

        innovation_covariance = np.dot(self.H, P_prime).dot(self.H.T) + self.R
        # compute kalman gain
        kalman_gain = np.dot(P_prime, self.H.T).dot(np.linalg.inv(innovation_covariance))

        # update state
        #rospy.loginfo('The error is %s', np.dot(kalman_gain, innovation))
        self.x = x_prime + np.dot(kalman_gain, innovation)
        self.P = P_prime - np.dot(kalman_gain,self.H).dot(P_prime)

        self.old_control_vel = np.array(vel_controls)
        # return updated state
        return self.x, self.P




if __name__ == "__main__":
    try:
        rospy.init_node('ekfnode')
        # odom_frame = rospy.get_param("~output_frame",'odom')
        # base_frame = rospy.get_param("~base_footprint_frame",'base_footprint')
        # tf_pub = tf.TransformBroadcaster()

        ekf = ExtendedKalmanFilter(dim_z=3, dim_x=3)
        pub = rospy.Publisher('odom_combined', Odometry, queue_size=10)

        global last_time
        last_time = rospy.Time.now().to_sec()
        twist_sub = rospy.Subscriber('cmd_vel', Twist, ekf.twistCallback)
        imu_sub = rospy.Subscriber('imu', Imu, ekf.imuCallback)
        odom_sub = rospy.Subscriber('odom', Odometry, ekf.odomCallback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
