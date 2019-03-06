#! /usr/bin/env python

import rospy
import tf
from numpy.random import uniform
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import helper



class ParticleFilter(object):

    def __init__(self, dim_x =3, N=100):
        self.N = N # N is number of particles
        self.particles = np.zeros((dim_x, N)) # particles representing belief
        self.weights = np.ones(N) / N
        self.odom_data = None # odom controls
        self.laser_data = None # holds LaserScan measurements
        self.range_method = None
        self.B = None
        self.control_velocity = None # holds the twist from the cmd_vel topic
        # noise parameters
        self.mu = 0
        self.std = 0.1

        # get the map
        self.get_map()

    def twist_callback(self, msg):
        self.control_velocity = msg

    def odom_callback(self, msg):
            self.odom_data = msg

    def laser_callback(self, msg):
        self.laser_data = msg

        # when laser scan data comes in call the update_and_publish method
        self.update_and_publish()

    # start with a set of particles S = {(xi,wi).......(xn,wn)}
    # each particle has (x, y, theta)
    def create_particles(self, x_range, y_range, theta_range, N):
        self.particles[0, :] = uniform(x_range[0], x_range[1], size=N)
        self.particles[1, :] = uniform(y_range[0], y_range[1], size=N)
        self.particles[2, :] = uniform(theta_range[0], theta_range[1], size=N)
        return self.particles


    # prediction step or sampling to get new set of particles
    # with added noise
    def prediction(self, vel_controls, dt):
        # add some noise to the linear and angular velocities
        noise = self.mu + self.std * np.random.randn()
        #self.Q = np.random.random_sample((dim_x, 1))

        linear_vel = np.linalg.norm([vel_controls[0], vel_controls[1]]) + noise # noise added to linear velocity
        angular_vel = vel_controls[2] + noise # noise added to angular velocity
        self.B = np.array([[linear_vel * dt * np.cos(theta), 0, 0],
                            [0, linear_vel * dt * np.sin(theta), 0],
                            [0, 0, angular_vel * dt]])

        self.particles += np.dot(self.B, self.particles)

    # update with measurement(z) <=> importance
    def measurement(self):
        pass

        # Normalize

    # Resampling


    def update_and_publish(self):
        # controls
        vel_controls = (self.control_velocity.linear.x,
                        self.control_velocity.linear.y,
                        self.control_velocity.angular.z)

        dt = rospy.Time.now().to_sec() - last_time # delta time
        global last_time
        last_time = rospy.Time.now().to_sec()

        self.prediction(vel_controls, dt) # perform motion


    def display_particles(self, particles):
        # publish particles as a PoseArray object
        pa = PoseArray()
        pa.header = helper.make_header("map")
        pa.poses = helper.particles_to_poses(self.particles)
        self.particle_pub.publish(pa)


    def get_map(self):
        '''
        Fetch the occupancy grid map from the map_server instance, and initialize the correct
        RangeLibc method. Also stores a matrix which indicates the permissible region of the map
        '''
        # this way you could give it a different map server as a parameter
        map_service_name = rospy.get_param("~static_map", "static_map")
        print("getting map from service: ", map_service_name)
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

        self.map_info = map_msg.info
        oMap = range_libc.PyOMap(map_msg)
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        # Set range method


if __name__ == "__main__":
    try:
        rospy.init_node('particlefilter')

        pFilter = ParticleFilter() # instantiate particle filter
        global last_time
        last_time = rospy.Time.now().to_sec()

        twist_sub = rospy.Subscriber('cmd_vel', Twist, pFilter.twist_callback)
        laser_sub = rospy.Subscriber('scan', LaserScan, pFilter.laser_callback)
        #odom_sub = rospy.Subscriber('odom', Odometry, pFilter.odomCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
