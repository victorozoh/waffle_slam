#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
import tf.transformations
import tf

# convert euler angle to quaternion
def euler_to_quaternion(angle):
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

# convert quaternion to euler angle
def quaternion_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


#  returns an array of Pose objects
def particles_to_poses(particles):
    pt = Point()
    return [ Pose(pt(particle[0], particle[1], 0.0), euler_to_quaternion(particle[2]))
            for particle in particles.T]

def make_header(frame_id, stamp=None):
    ''' Creates a Header object for stamped ROS objects '''
    if stamp == None:
        stamp = rospy.Time.now()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header
