#!/usr/bin/env python3
from email import header
import rospy
import sys
import numpy as np
from utils import *
from math import sin, cos, pi, atan2, asin, sqrt
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, PoseArray 
from venv_node.msg import RangeBearing, RBarray


class NoFilters():
    def __init__(self, finds_pub) -> None:
        #To save ROS messages from subscribers
        self.buoys_msg = None
        self.pose_msg = None
        self.buoys = []
        self.finds_pub = finds_pub
        self.prev_seq = -1
        self.seq = 0

    def buoys_callback(self, buoys_msg):
        self.buoys_msg = buoys_msg

    def pose_callback(self, pose_msg):
        self.pose_msg = pose_msg

    def process(self):
        buoys_msg = self.buoys_msg
        if buoys_msg == None or self.pose_msg == None:
            return
        if buoys_msg.header.seq == self.prev_seq:
            return
        self.prev_seq = buoys_msg.header.seq
        
        boat_pose = rosPose2mypose(self.pose_msg.pose)
        buoys = []
        for z in self.buoys_msg.measurements:
            x = boat_pose[0] + z.range*cos(boat_pose[2] + z.bearing)
            y = boat_pose[1] + z.range*sin(boat_pose[2] + z.bearing) 
            buoy_pose = mypose2rosPose([x, y, 0])
            buoy_pose.position.z = z.id
            buoys.append(buoy_pose)
        header = Header(self.seq, rospy.Time.now(), "fosenkaia_NED")
        self.finds_pub.publish(PoseArray(header, buoys))
        self.seq +=1


def main(args):
    rospy.init_node('no_filter_converter', anonymous=True)
    rate = rospy.Rate(1)

    #Publishers for the buoys 
    finds_pub = rospy.Publisher('/not_filtered', PoseArray, queue_size=5)
    nofilters = NoFilters(finds_pub)

    rospy.Subscriber('detections', RBarray, nofilters.buoys_callback)
    rospy.Subscriber('nav/pose', PoseStamped, nofilters.pose_callback)


    while not rospy.is_shutdown():
        nofilters.process()
        rate.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass