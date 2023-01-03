#!/usr/bin/env python3



import rospy
import sys
import numpy as np
from math import sin, cos, pi, atan2, asin
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped 

   

class Planner():
    
    def __init__(self, ocg_pub) -> None:
        self.ocg_pub = ocg_pub

        #To save ROS messages from subscribers
        self.pose_msg = None
        self.scan_msg = None
        self.buoys_msg = None

        self.resolution = 0.5   # [m/cell]     
        self.width = 200        # nº of cells
        self.height = 200
        self.grids = None        # grids initialized by "gen_ini_grids" function

    def laser_callback(self, laser_msg):
        self.scan = laser_msg
    
    def buoys_callback(self, buoys_msg):
        self.buoys = buoys_msg

    def pose_callback(self, pose_msg):
        self.pose = pose_msg
              
    def process(self):
        pass

def main(args):

    rospy.init_node('local_map_node', anonymous=True)
    rate = rospy.Rate(1)

    #Publishers for the next waypoint
    ref_pub = rospy.Publisher('/waypt', PoseStamped, queue_size=5)

    planner = Planner()

    # Subscriber to the (x, y, teta) coordinates in the inertial frame
    rospy.Subscriber('nav/pose', PoseStamped, mapper.pose_callback)
    #Subscriber to the laser scan if available:
    rospy.Subscriber('scan', LaserScan, mapper.laser_callback)
    #TODO: Add buoys subscriber

    mapper.gen_ini_grids()

    while not rospy.is_shutdown():
        mapper.process()
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        #fp.close()
        pass