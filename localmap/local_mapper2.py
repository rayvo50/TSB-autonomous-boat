#!/usr/bin/env python3

# OccupancyGrid publisher
# ============================================
# subscribed topics:
#	/car/scan - contains LaserScan messages
# published topics:
#   /car/map - contains OccupancyGrid messages
# listened tf:
#   /car_base_link to /world
#	/sonar_link to /car_base_link
#   /map to /world
# ============================================

import rospy
import sys
import numpy as np
from math import sin, cos, pi, atan2, asin
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped 

#TODO: to decrease the number of multiplications store side of grid in metres


#TODO: move the following to a utils.py module

def pi2pi(angle):
    return (angle + pi) % (2 * pi) - pi

def quaternion_from_euler(roll, pitch, yaw):
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x,y,z,w]

def yaw_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)
     
    return yaw_z

def mypose2rosPose(pose):
    res = Pose()
    res.position.x = pose[0]
    res.position.y = pose[1]
    q = quaternion_from_euler(0, 0, pose[2])
    res.orientation.x = q[0]
    res.orientation.y = q[1]
    res.orientation.z = q[2]
    res.orientation.w = q[3]
    return res

def rosPose2mypose(Pose):
    q = Pose.orientation
    yaw = pi2pi(yaw_from_quaternion([q.x, q.y, q.z, q.w]))
    return [Pose.position.x, Pose.position.y, yaw]    

class Mapper():
    
    def __init__(self, ocg_pub) -> None:
        self.ocg_pub = ocg_pub

        #To save ROS messages from subscribers
        self.pose_msg = None
        self.scan_msg = None
        self.buoys_msg = None

        self.resolution = 0.5   # [m/cell]     
        self.width = 500        # nº of cells
        self.height = 500
        self.grids = None        # grids initialized by "gen_ini_grids" function
        self.grid_translations = None

        # self.map_msg = OccupancyGrid()
        # self.map_msg.header.frame_id = "map"
        # #This may change in the future
        # self.map_msg.info.resolution = 0.5
        # self.map_msg.info.width = 500
        # self.map_msg.info.height = 500
        # self.map_msg.info.origin.position.x = 10
        # self.map_msg.info.origin.position.y = 10
        # self.grid = np.ndarray((self.map_msg.info.width, self.map_msg.info.height), dtype=np.int, buffer=np.zeros((self.map_msg.info.width, self.map_msg.info.height), dtype=np.int))
        # self.grid.fill(int(0))
        # self.grid[0:10,0:10].fill(int(100))

    def laser_callback(self, laser_msg):
        self.scan = laser_msg
    
    def buoys_callback(self, buoys_msg):
        self.buoys = buoys_msg

    def pose_callback(self, pose_msg):
        self.pose = pose_msg

    def GenOccupancyGrid(self, pose):
        grid = OccupancyGrid()
        grid.info.origin = mypose2rosPose([pose[0], pose[1], 0])
        grid.info.resolution = self.resolution
        grid.info.height = self.height
        grid.info.width = self.width
        data = np.ndarray((self.map_msg.info.width, self.map_msg.info.height), dtype=np.int, buffer=np.zeros((self.map_msg.info.width, self.map_msg.info.height), dtype=np.int)) 
        data.fill(-1)
        grid.data = data
        return grid

    def gen_ini_grids(self):
        pose = rosPose2mypose(self.pose)
        x0 = round(pose[0] - self.width * self.resolution / 2)
        y0 = round(pose[1] - self.height * self.resolution / 2)
        self.grids = []
        if -pi <= pose[2] < -pi/2:
            x1 = x0 - self.width * self.resolution
            y1 = y0 - self.height * self.resolution
            self.grids.append(self.GenOccupancyGrid([x1, y0]))
            self.grids.append(self.GenOccupancyGrid([x0, y0]))
            self.grids.append(self.GenOccupancyGrid([x1, y1]))
            self.grids.append(self.GenOccupancyGrid([x0, y1]))
        elif -pi/2 <= pose[2] < 0:
            x1 = x0 - self.width * self.resolution
            y1 = y0 + self.height * self.resolution
            self.grids.append(self.GenOccupancyGrid([x1, y1]))
            self.grids.append(self.GenOccupancyGrid([x0, y1]))
            self.grids.append(self.GenOccupancyGrid([x1, y0]))
            self.grids.append(self.GenOccupancyGrid([x0, y0]))
        elif 0 <= pose[2] < pi/2:
            x1 = x0 + self.width * self.resolution
            y1 = y0 + self.height * self.resolution
            self.grids.append(self.GenOccupancyGrid([x0, y1]))
            self.grids.append(self.GenOccupancyGrid([x1, y1]))
            self.grids.append(self.GenOccupancyGrid([x0, y0]))
            self.grids.append(self.GenOccupancyGrid([x1, y0]))
        elif pi/2 <= pose[2] < pi:
            x1 = x0 + self.width * self.resolution
            y1 = y0 - self.height * self.resolution
            self.grids.append(self.GenOccupancyGrid([x0, y0]))
            self.grids.append(self.GenOccupancyGrid([x1, y0]))
            self.grids.append(self.GenOccupancyGrid([x0, y1]))
            self.grids.append(self.GenOccupancyGrid([x1, y1]))
        else:
            print("Error: invalid yaw angle obtained")
            
    def where_is(self, x_obj, y_obj):
        g = self.grids[2]
        start = rosPose2mypose(g.info.origin)
        if start[0] < x_obj < start[0] + self.width * self.resolution:
            if start[1] < y_obj < start[1] + self.height * self.resolution:
                origin = rosPose2mypose(self.grids[2].info.origin)
                #encontrar os indices com a formula
            if start[1] + self.height * self.resolution < y_obj < start[1] + 2 * self.height * self.resolution:
                return 0
        elif start[0] + self.width * self.resolution < x_obj < start[0] + 2 * self.width * self.resolution:
            if start[1] < y_obj < start[1] + self.height * self.resolution:
                return 3
            if start[1] + self.height * self.resolution < y_obj < start[1] + 2 * self.height * self.resolution:
                return 1
        else:
            return -1
    
    def find_indices():
        pass

    def process(self):
        print(". ")

        # first thing here there must be a function for managing the maps like:
        # manage_maps()

        # copy into local variables
        #q = self.pose.quaternion
        #rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        #pose = [self.pose.position.x, self.pose.position.y, rpy[2]]
        scan = self.scan

        #if self.scan != None:
        #min = self.scan.angle_min
        #max = self.scan.angle_max
        #increment = self.scan.angle_increment

        # loop through the ranges in laserscan:
        angle = min
        i = 0
        # while angle <= max:
        #     obs = (pose[0] + scan[i]*sin(pi2pi(angle+pose[2])), pose[1] + scan[i]*cos(pi2pi(angle+pose[2])))     
            
        #     angle += increment

        # grid = np.ndarray((self.map_msg.info.width, self.map_msg.info.height), buffer=np.zeros((self.map_msg.info.width, self.map_msg.info.height)))
        # grid.fill(int(-1))

        self.map_msg.header.stamp = rospy.Time.now()
        self.map_msg.data = self.grid.flatten()
        self.ocg_pub.publish(self.map_msg)

def main(args):

    rospy.init_node('local_map_node', anonymous=True)
    rate = rospy.Rate(1)

    #Publishers for the maps
    map0_pub = rospy.Publisher('/local_map0', OccupancyGrid, queue_size=5)
    map1_pub = rospy.Publisher('/local_map1', OccupancyGrid, queue_size=5)
    map2_pub = rospy.Publisher('/local_map2', OccupancyGrid, queue_size=5)
    map3_pub = rospy.Publisher('/local_map3', OccupancyGrid, queue_size=5)

    mapper = Mapper()

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