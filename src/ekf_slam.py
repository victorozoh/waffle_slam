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
        self.num_landmarks = 0 # int((self.dim_x - 3)/3)
        self.alpha = 0.3 # mahalanobis distance threshold
        # initial noise values for range and bearing
        self.std_range = 0.6
        self.std_bearing = 0.6
        # state vector; Robot pose is self.x[:3]
        self.x = np.zeros((self.dim_x,1))
        # State vector Covariance matrix, initial uncertainty
        self.P = np.zeros((self.dim_x,self.dim_x)) # start with zero uncertainty
        # Process/Motion Noise
        self.R = None
        # Measurement Noise/Uncertainty.
        self.Q = None
        self.f = np.eye(self.dim_x) # x = f(x) + Bu
        self.F = None# Jacobian matrix of motion model for updating covariance
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

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def getInno_InnoCov_Hmat(self, j, theta, measured_z, rx, ry):
        m_jx = float(self.x[2*j + 1])
        m_jy = float(self.x[2*j + 2])
        # m_js = float(self.x[3*j + 2])

        delta_x = m_jx - rx
        delta_y = m_jy - ry
        delta_q = np.linalg.norm([delta_x, delta_y])
        lm_angle = math.atan2(delta_y, delta_x) - theta

        predicted_z = np.array([delta_q, lm_angle], dtype=np.float64).reshape((2,1))

        self.h = (1/delta_q**2) * np.array([[-delta_q * delta_x , -delta_q * delta_y, 0, delta_q * delta_x, delta_q * delta_y],
                                            [delta_y, -delta_x, -delta_q**2, -delta_y, delta_x]])

        tempmat = np.hstack((np.zeros((2,3)), np.zeros((2, 2*j -2)), np.eye(2), np.zeros((2, 2 * self.num_landmarks - 2*j))))
        F_j = np.vstack((self.F, tempmat))

        Hmat =  np.dot(self.h, F_j)
        innovation_covariance = np.dot(Hmat, self.P).dot(Hmat.T) + self.Q
        innovation_covariance = innovation_covariance.astype(np.float64) # in order to compute inverse
        # mahalanobis distance
        innovation = measured_z - predicted_z
        innovation[1] = self.pi_2_pi(innovation[1])

        return innovation, innovation_covariance, Hmat

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

            biglist = biglist[:-1] # ignore the last list because the reading is the same as the first list
            laser_meas = []
            # loop through list of lists to get range measurement
            for i, val in enumerate(biglist):
                # Each list holds tuples of (index, range) for a landmark. val is a list
                # compute bearing and range for each landmark
                distlist = [r[1] for r in val] # get list of distances to an object.
                dist = sum(distlist)/len(distlist) # use average of range readings
                bearing = laser_angles[0] + val[0][0] * laser_angles[2] # same as angle_min + i*angle_increment
                laser_meas.append((dist, bearing))

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
            for i, feature in enumerate(laser_meas):
                temp_spot = Spot()
                temp_spot.id = 'landmark_' + str(i)
                temp_spot.position = list(new_state[(2*i + 3): (2*i) + 5])
                map_state.data.append(temp_spot)

            pub.publish(map_state)

    def estimate(self, vel_controls, laser_meas, odom_meas, dt):
        #----------------- MOTION UPDATE--------------------#

        rx = odom_meas[0] # robot's x position
        ry = odom_meas[1] # robot's y position
        theta = odom_meas[2] # angle

        self.B = np.array([[dt * math.cos(theta), 0],
                        [dt * math.sin(theta), 0],
                        [0, dt]]) # 3x2 matrix
        linear_vel = math.sqrt((vel_controls[0])**2 + (vel_controls[1])**2)
        vel = np.array([linear_vel, vel_controls[2]]).reshape((2,1)) # combine lineer and angular velocity into one vector

        # x_prime[:3] =  self.x[:3] + np.dot(self.B, vel)
        if self.num_landmarks > 0:
            self.F = np.hstack((np.eye(3), np.zeros((3, 2*self.num_landmarks)) ))
        else:
            self.F = np.eye(3)

        # x_prime = self.x + np.dot(self.F.T, np.dot(self.B, vel))
        self.x = self.x + np.dot(self.F.T, np.dot(self.B, vel))
        self.g = np.array([[0, 0, -1 * linear_vel * dt* math.sin(theta)],
                            [0, 0, linear_vel * dt * math.cos(theta)],
                            [0, 0, 0]])
        self.G = np.eye(self.dim_x) + np.dot(self.F.T, self.g).dot(self.F) # (3N + 3) x (3N + 3) matrix

        # update state covariance after motion
        self.R = np.diag([0.1**2, 0.1**2, 0.1**2]) # motion noise
        self.P = np.dot(self.G, self.P).dot(self.G.T) + np.dot(self.F.T, self.R).dot(self.F)

        #----------------- LANDMARK CORRECTION--------------------#

        self.Q = np.diag([self.std_range**2, self.std_bearing**2]) # sensor noise

        if len(laser_meas) > 0:
            # loop through all observed features => (range, bearing)
            for feature in laser_meas:
                # get measurement
                measured_z = np.array(feature).reshape((2,1))
                # initial landmark estimate => mu_prime
                mu_prime = np.array([feature[0]*math.cos(feature[1] + theta),
                                     feature[0]*math.sin(feature[1] + theta)]).reshape((2,1))
                # mu_prime = np.vstack((self.x[:2],np.zeros((1,1)))) + mu_prime #keep shape as (3x1)
                mu_prime = self.x[:2] + mu_prime


                phi_klist = [] # list of mahalanobis distances
                # calculate distance to current landmark

                # second loop to get data association. calculate distance to previous landmarks
                for j in range(1, self.num_landmarks + 1):
                    innovation, innovation_covariance, Hmat = self.getInno_InnoCov_Hmat(j, theta, measured_z, rx, ry)
                    phi_k = float(np.dot(innovation.T, np.linalg.inv(innovation_covariance)).dot(innovation))
                    phi_klist.append(phi_k)

                phi_klist.append(self.alpha)
                ml_index = phi_klist.index(min(phi_klist)) # max likelihood for data association
                # if distance to all existing landmarks exceeds threshold, a new landmark is created
                if ml_index == len(phi_klist) - 1:
                    # new landmark
                    self.num_landmarks += 1 #increase landmark count
                    self.dim_x += 2 #increase the dimension of the state vector
                    # enlarge the state and covariance matrix to account for new landmark
                    state_update = np.vstack((self.x, mu_prime))
                    # rospy.loginfo('The shape of P is %s ', self.P.shape)
                    # rospy.loginfo('The shape of state vector is %s ', self.x.shape)

                    cov_update = np.vstack((np.hstack((self.P, np.zeros((len(self.x), 2)))),
                                            np.hstack((np.zeros((2, len(self.x))), np.eye(2)))))

                    self.F = np.hstack((np.eye(3), np.zeros((3, 2*self.num_landmarks)) ))

                    self.x = state_update
                    self.P = cov_update

                # get the appropriate H matrix and innovation_covariance
                innovation, innovation_covariance, Hmat = self.getInno_InnoCov_Hmat(ml_index+1, theta, measured_z, rx, ry)
                # compute kalman gain
                kalman_gain = np.dot(self.P, Hmat.T).dot(np.linalg.inv(innovation_covariance))
                # update state
                self.x = self.x + np.dot(kalman_gain, innovation)
                #rospy.loginfo('The new state is %s ', self.x)
                self.P = self.P - np.dot(kalman_gain,Hmat).dot(self.P)

        # return updated state
        return self.x, self.P


if __name__ == "__main__":
    try:
        # initialise the ekf_slam node
        rospy.init_node('ekf_slam')

        # 2(range, bearing) landmarks + robot pose(x, y, theta), means dim_x = 7
        # 2N + 3 = number of dimensions of the state. For N=2, 2N + 3 = 7
        ekf = ExtendedKalmanFilter(dim_z=3, dim_x=3)
        pub = rospy.Publisher('landmark', SpotArray, queue_size=10)

        global last_time
        last_time = rospy.Time.now().to_sec()
        twist_sub = rospy.Subscriber('cmd_vel', Twist, ekf.twistCallback)
        odom_sub = rospy.Subscriber('odom', Odometry, ekf.odomCallback)

        # Subscribe to laser scan topic to get measuerements
        laser_sub = rospy.Subscriber('scan', LaserScan, ekf.laserCallback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
