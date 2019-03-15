#! /usr/bin/env python

import rospy
import numpy as np
import math
import tf
from std_msgs.msg import Float32
from waffle_pick_place.msg import Spot, SpotArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3, PoseWithCovarianceStamped

last_time = 0.0

class ExtendedKalmanFilter(object):

    def __init__(self, dim_z=2, dim_x = 3):
        # dim_z corresponds to the dimensions of the measurements
        # dim_x corresponds to the dimensions of the state. In this case, [x, y, theta, x1, y1, x2, y2, ......xn, yn]
        self.dim_x = dim_x
        self.dim_z = dim_z
        # initial values
        self.noise_value = 0.01
        # state vector; Robot pose is self.x[:3]
        self.x = np.zeros((self.dim_x,1))
        # There are 5 landmarks(AR tags)
        self.landmarks = None
        self.map = [0]*5 # robot = map[0], while landmarks make up indices 1:5
        # State vector Covariance matrix, initial uncertainty
        self.P = np.zeros((self.dim_x,self.dim_x)) # start with zero uncertainty
        # Process/Motion Noise
        self.Q = np.eye(self.dim_x) * self.noise_value
        # Measurement Noise/Uncertainty.
        self.R = np.eye(self.dim_z) * self.noise_value
        self.f = np.eye(self.dim_x) # x = f(x) + Bu
        self.F = np.eye(self.dim_x) # Jacobian matrix of motion model for updating covariance
        self.B = None#np.zeros((dim_x,2)) # controls matrix
        self.H = np.zeros((2,self.dim_x)) # Jacobian matrix i.e H = Jacobian(h)
        self.control_velocity = None # current velocity commands from twist message on cmd_vel topic
        self.odom_data = None # holds current odometry velocities
        self.imu_data = None # current data from /imu topic
        self.laser_data = None # holds measurements from the ar_pose_marker topic

    def twistCallback(self, msg):
        self.control_velocity = msg

    def imuCallback(self, msg):
        self.imu_data = msg

    def odomCallback(self, msg):
        self.odom_data = msg

    def laserCallback(self, msg):
        self.laser_data = msg
        # update and publish with all the measurements gathered
        self.update_and_publish()


    def getHmatrix(self, i):
        Hmatrix = self.H
        Hmatrix[0][0] = Hmatrix[1][1] = 1
        Hmatrix[0][(2*i) + 3] = Hmatrix[1][(2*i) + 4] = 1
        return Hmatrix

    def update_and_publish(self):
        if None in [self.laser_data, self.odom_data]:
            pass
        else:
            # controls
            vel_controls = (self.control_velocity.linear.x,
                            self.control_velocity.linear.y,
                            self.control_velocity.angular.z)

            # current odom measurements
            quaternion =   (self.odom_data.pose.pose.orientation.x,
                            self.odom_data.pose.pose.orientation.y,
                            self.odom_data.pose.pose.orientation.z,
                            self.odom_data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            odom_meas = (self.odom_data.pose.pose.position.x,
                         self.odom_data.pose.pose.position.y,
                         euler[2])

            # get current laser scan measurements
            laser_angles = (self.laser_data.angle_min, self.laser_data.angle_max, self.laser_data.angle_increment)
            # get indexes and range values for objects detected
            laser =  [(i, val) for i, val in enumerate(self.laser_data.ranges) if val != float('inf')]
            L = len(laser)
            templist = []
            biglist =[]
            for i in range(1,L):
                if laser[i-1][0] == laser[i][0] -1:
                    templist.append(laser[i-1])
                else:
                    templist.append(laser[i-1])
                    biglist.append(templist)
                    templist =[]
                if i == (L -1) and (laser[i-1][0] == laser[i][0]-1):
                    templist.append(laser[i])
                    biglist.append(templist)

            #rospy.loginfo('Detected ranges %s', biglist)
            biglist = biglist[:-1]
            # loop through list of lists to get range measurement
            for val in biglist:
                # Each list holds tuples of (index, range) for a landmark
                # compute bearing and range for each landmark
                distlist = [r[1] for r in val] # get list of distances to an object.
                dist = sum(distlist)/len(distlist)
            # change self.dim_x to match the number of features detected
            # measurements will be (x_i, y_i); TODO: (range, bearing)
            #self.update_state_dimensions(ar_meas)

            dt = rospy.Time.now().to_sec() - last_time # delta time
            global last_time
            last_time = rospy.Time.now().to_sec()

            # call extended kalman filter function
            new_state, new_cov = self.estimate(vel_controls, laser_meas, odom_meas, dt)

            # publish the positions of landmarks for plotting
            map_state = SpotArray()
            temp_spot = Spot()
            temp_spot.id = 'robot'
            temp_spot.position = list(new_state[:3]) # robot's pose
            map_state.data.append(temp_spot)
            for feature in ar_meas:
                i = feature[2]
                temp_spot = Spot()
                temp_spot.id = 'marker_' + str(i)
                temp_spot.position = list(new_state[(2*i) + 3 : (2*i) + 5])
                map_state.data.append(temp_spot)
            pub.publish(map_state)

    def estimate(self, vel_controls, ar_meas, odom_meas, dt):
        # prediction/motion, velocity model
        theta = odom_meas[2] # angle
        self.B = np.array([[dt * math.cos(theta), 0],
                        [dt * math.sin(theta), 0],
                        [0, dt]])
        linear_vel = math.sqrt((vel_controls[0])**2 + (vel_controls[1])**2)
        vel = np.array([linear_vel, vel_controls[2]]).reshape((2,1)) # combine lineer and angular velocity into one vector
        # add some process noise with mean = 0 and std = 0.1
        noise = np.random.normal(0, 0.05, 3)
        vel = vel + noise[:2].reshape((2,1))

        # x' from motion/ prediction
        # x(k+1) = x(k) + B*x(k) where f(x(k)) = x(k) + B*x(k)
        x_prime = self.x
        x_prime[:3] =  self.x[:3] + np.dot(self.B, vel)
        # use F = Jacobian(f) to update covariance. Dimension of
        self.F = np.array([[1, 0, -1 * linear_vel * dt* math.sin(theta)],
                            [0, 1, linear_vel * dt * math.cos(theta)],
                            [0, 0, 1]])
        # update state covariance after motion
        Pr_prime = np.dot(self.F, self.P[:3, :3]).dot(self.F.T) + (np.eye(3) * noise[0])
        self.P[:3, :3] = Pr_prime
        self.P[:3, 3:] = self.F.dot(self.P[:3, 3:])
        self.P[3:, :3] = self.P[:3, 3:].T

        #----------------- LANDMARK CORRECTION--------------------#

        #1 Known landmarks
        # if len(laser_meas) > 0:
        #     # loop through all observed features(x_i, y_i, id)
        #     for feature in ar_meas:
        #         index = feature[2]
        #         # measurement update->odometry model. compute residual/innovation
        #         # get h(x) and H(x) measurement models
        #         measured_z = np.array([feature[0] + odom_meas[0], feature[1] + odom_meas[1]])
        #         rospy.loginfo('feature[0] is %s', feature[0])
        #         predicted_z = np.array([feature[0] + x_prime[0], feature[1] + x_prime[1]])
        #         innovation = measured_z.reshape((2,1)) - predicted_z.reshape((2,1))
        #
        #         # ensure shapes match for gain calculations
        #         # Get the proper dimensions and values for H and P
        #         Hmat = self.getHmatrix(index)
        #         # compute innovation covariance
        #         innovation_covariance = np.dot(Hmat, self.P).dot(Hmat.T) + self.R
        #         # compute kalman gain
        #         kalman_gain = np.dot(self.P, Hmat.T).dot(np.linalg.inv(innovation_covariance))
        #
        #         # update state
        #         #rospy.loginfo('The error is %s', np.dot(kalman_gain, innovation))
        #         self.x = x_prime + np.dot(kalman_gain, innovation)
        #         #rospy.loginfo('The new state is %s ', self.x)
        #         self.P = self.P - np.dot(kalman_gain,Hmat).dot(self.P)

        # return updated state
        return self.x, self.P


if __name__ == "__main__":
    try:
        rospy.init_node('ekf_slam')

        # 5(range, bearing) markers + robot pose(x, y, theta), means dim_x = 13
        ekf = ExtendedKalmanFilter(dim_z=2, dim_x=9)
        pub = rospy.Publisher('landmark', SpotArray, queue_size=10)

        global last_time
        last_time = rospy.Time.now().to_sec()
        twist_sub = rospy.Subscriber('cmd_vel', Twist, ekf.twistCallback)
        odom_sub = rospy.Subscriber('odom', Odometry, ekf.odomCallback)

        # Subscribe to ar_pose_marker topic to get measuerements
        laser_sub = rospy.Subscriber('scan', LaserScan, ekf.laserCallback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
