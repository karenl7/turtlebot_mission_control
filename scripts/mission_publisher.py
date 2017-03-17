#!/usr/bin/env python

# This is the script that will be used to publish the mission specification during testing.

import rospy
from std_msgs.msg import Int32MultiArray

if __name__ == '__main__':

    mission = Int32MultiArray()
    mission.data = [6, 5, 1]    # feel free to change this "mission specification"
    pub = rospy.Publisher('/mission', Int32MultiArray, queue_size=10)
    rospy.init_node('mission_publisher', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(mission)
        rate.sleep()
