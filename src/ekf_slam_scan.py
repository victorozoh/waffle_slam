#! /usr/bin/env python

import rospy
import numpy as np
from numpy.random import randn
import math
import tf
from std_msgs.msg import Float32
from waffle_pick_place.msg import Spot, SpotArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3, PoseWithCovarianceStamped

last_time = 0.0

class ExtendedKalmanFilter(object):

    def __init__(self, dim_z=3, dim_x = 3):
        # dim_z corresponds to the dimensions of the measurements
        # dim_x corresponds to the dimensions of the state. In this case, [x, y, theta, x1, y1, s1, x2, y2, s2......xn, yn, sn]
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.num_landmarks = int((self.dim_x - 3)/3)
        # initial noise values for range and bearing
        self.std_range = 0.1
        self.std_bearing = 0.1
        # state vector; Robot pose is self.x[:3]
        self.x = np.zeros((self.dim_x,1))
        # There are 2 landmarks(coke cans)
        self.map = [0]*5 # robot = map[0], while landmarks make up indices 1:5
        # State vector Covariance matrix, initial uncertainty
        self.P = np.zeros((self.dim_x,self.dim_x)) # start with zero uncertainty
        # Process/Motion Noise
        self.R = None
        # Measurement Noise/Uncertainty.
        self.Q = None
        self.f = np.eye(self.dim_x) # x = f(x) + Bu
        self.F = np.hstack((np.eye(3), np.zeros(3, self.dim_x - 3))) # Jacobian matrix of motion model for updating covariance
        self.g = None # using notation in probabilistic robotics
        self.G = None # I + FgF
        self.B = None #np.zeros((dim_x,2)) # controls matrix
        self.h = None
        self.H = np.zeros((3,self.dim_x)) # Jacobian matrix i.e H = Jacobian(h)
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
            biglist = biglist[:-1] # ignore the last list because the reading is the same as the first list
            laser_meas = []
            # loop through list of lists to get range measurement
            for i, val in enumerate(biglist):
                # Each list holds tuples of (index, range) for a landmark. val is a list
                # compute bearing and range for each landmark
                distlist = [r[1] for r in val] # get list of distances to an object.
                dist = sum(distlist)/len(distlist) # use average of range readings
                bearing = laser_angles[0] + val[0][0] * laser_angles[2] # same as angle_min + i*angle_increment
                laser_meas.append((dist, bearing, i))
            #rospy.loginfo('laser readings are %s', laser_meas)

            dt = rospy.Time.now().to_sec() - last_time # delta time
            global last_time
            last_time = rospy.Time.now().to_sec()

            # call extended kalman filter function
            new_state, new_cov = self.estimate(vel_controls, laser_meas, odom_meas, dt)

            # publish the positions of landmarks for plotting
            map_state = SpotArray()
            temp_spot = Spot()
            # temp_spot.id = 'robot'
            # temp_spot.position = list(new_state[:3]) # robot's pose
            # map_state.data.append(temp_spot)
            # for feature in ar_meas:
            #     i = feature[2]
            #     temp_spot = Spot()
            #     temp_spot.id = 'marker_' + str(i)
            #     temp_spot.position = list(new_state[(2*i) + 3 : (2*i) + 5])
            #     map_state.data.append(temp_spot)

            pub.publish(map_state)

    def estimate(self, vel_controls, ar_meas, odom_meas, dt):
        #----------------- MOTION UPDATE--------------------#

        rx = odom_meas[0] # robot's x position
        ry = odom_meas[1] # robot's y position
        theta = odom_meas[2] # angle

        self.B = np.array([[dt * math.cos(theta), 0],
                        [dt * math.sin(theta), 0],
                        [0, dt]]) # 3x2 matrix
        linear_vel = math.sqrt((vel_controls[0])**2 + (vel_controls[1])**2)
        vel = np.array([linear_vel, vel_controls[2]]).reshape((2,1)) # combine lineer and angular velocity into one vector

        # add some process noise with mean = 0 and std = 0.1
        noise = np.random.normal(0, 0.05, 3)
        vel = vel + noise[:2].reshape((2,1))

        # x_prime[:3] =  self.x[:3] + np.dot(self.B, vel)
        x_prime = self.x + np.dot(self.F.T, np.dot(self.B, vel))
        self.g = np.array([[0, 0, -1 * linear_vel * dt* math.sin(theta)],
                            [0, 0, linear_vel * dt * math.cos(theta)],
                            [0, 0, 0]])
        self.G = np.eye(self.dim_x) + np.dot(self.F.T, self.g).dot(self.F) # (3N + 3) x (3N + 3) matrix

        # update state covariance after motion
        self.R = np.diag([randn()*0.1**2, randn()*0.1**2, randn()*0.1**2]) # motion noise
        P_prime = np.dot(self.G, self.P).dot(self.G.T) + np.dot(self.F.T, self.R).dot(self.F)

        #----------------- LANDMARK CORRECTION--------------------#

        self.Q = np.diag([randn() * self.std_range**2, randn() * self.std_bearing**2, 1]) # process noise

        if len(laser_meas) > 0:
            # loop through all observed features => (range, bearing)
            for feature in laser_meas:
                # get measurement
                measured_z = np.array(feature).reshape((3,1))
                # initial landmark estimate => mu_prime
                mu_prime = np.array([feature[0]*math.cos(feature[1] + theta),
                                     feature[0]*math.sin(feature[1] + theta),
                                     feature[2]]).reshape((3,1))
                mu_prime = np.vstack((x_prime[:2],np.zeros((1,1)))) + mu_prime #keep shape as (3x1)

                phi_klist = [] # list of mahalanobis distances
                # second loop to get data association
                for j in range(1, self.num_landmarks + 1):
                    m_jx = self.x[(3*j]
                    m_jy = self.x[(3*j + 1]
                    m_js = self.x[(3*j + 2]

                    delta_x = m_jx - rx
                    delta_y = m_jy - ry
                    delta_q = np.linalg.norm([delta_x, delta_y])

                    predicted_z = np.array([delta_q, math.atan2(delta_y,delta_x) - theta, m_js]).reshape((3,1))

                    self.h = np.array([[-delta_q * delta_x , -delta_q * delta_y, 0, delta_q * delta_x, delta_q * delta_y, 0],
                                       [delta_y, -delta_x, -1, -delta_y, delta_x, 0],
                                       [0, 0, 0, 0, 0, 1]])
                    tempmat = np.hstack((np.zeros((3,3)), np.zeros((3, 3*j -3)), np.eye(3), np.zeros((3, 3 * self.num_landmarks - 3*j))))
                    F_j = np.vstack((self.F, tempmat))

                    self.H = (1/delta_q**2) *  np.dot(self.h, F_j)
                    innovation_covariance = np.dot(self.H, self.P).dot(self.H.T) + self.Q

                    # mahalanobis distance
                    innovation = measured_z - predicted_z
                    phi_k = np.dot(innovation.T, np.linalg.inv(innovation_covariance)).dot(innovation)
                    phi_klist.append(phi_k)


                # compute kalman gain
                kalman_gain = np.dot(self.P, self.H.T).dot(np.linalg.inv(innovation_covariance))
                # update state
                self.x = x_prime + np.dot(kalman_gain, innovation)
                #rospy.loginfo('The new state is %s ', self.x)
                self.P = self.P - np.dot(kalman_gain,Hmat).dot(self.P)

        # return updated state
        return self.x, self.P


if __name__ == "__main__":
    try:
        # initialise the ekf_slam node
        rospy.init_node('ekf_slam')

        # 3(range, bearing, signature) landmarks + robot pose(x, y, theta), means dim_x = 9
        # 3N + 3 = number of dimensions of the state. For N=2, 3N + 3 = 9
        ekf = ExtendedKalmanFilter(dim_z=2, dim_x=9)
        pub = rospy.Publisher('landmark', SpotArray, queue_size=10)

        global last_time
        last_time = rospy.Time.now().to_sec()
        twist_sub = rospy.Subscriber('cmd_vel', Twist, ekf.twistCallback)
        odom_sub = rospy.Subscriber('odom', Odometry, ekf.odomCallback)

        # Subscribe to lser scan topic to get measuerements
        laser_sub = rospy.Subscriber('scan', LaserScan, ekf.laserCallback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
