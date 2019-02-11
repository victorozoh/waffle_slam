#! /usr/bin/env python

import rospy
import numpy as np

# define extended Kalman filter function
def extendedKalman():



def main():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('filter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    # call extended kalman filter function

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptexception:
        pass
