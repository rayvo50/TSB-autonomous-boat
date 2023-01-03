#!/usr/bin/env python3
import re
import rospy
import sys
import numpy as np
from math import sin, cos, tan, pi, atan2, asin
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, PoseArray 


def pi2pi(angle):
    return (angle + pi) % (2 * pi) - pi

# def quaternion_from_euler(roll, pitch, yaw):
#     cy = cos(yaw * 0.5)
#     sy = sin(yaw * 0.5)
#     cp = cos(pitch * 0.5)
#     sp = sin(pitch * 0.5)
#     cr = cos(roll * 0.5)
#     sr = sin(roll * 0.5)

#     w = cr * cp * cy + sr * sp * sy
#     x = sr * cp * cy - cr * sp * sy
#     y = cr * sp * cy + sr * cp * sy
#     z = cr * cp * sy - sr * sp * cy

#     return [x,y,z,w]

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
    yaw = pi2pi(yaw_from_quaternion(q.x, q.y, q.z, q.w))
    return [Pose.position.x, Pose.position.y, pi2pi(yaw)]    



class Mapper():
    
    def __init__(self, ocg_pub) -> None:
        self.ocg_pub = ocg_pub

        #To save ROS messages from subscribers
        self.pose_msg = None
        self.scan_msg = None
        self.buoys_msg = None

        self.resolution = 0.5   # [m/cell]     
        self.width = 200        # nº of cells
        self.height = 200
        self.seq = 0

    def pose_callback(self, pose_msg):
        self.pose_msg = pose_msg
    
    def buoys_callback(self, buoys_msg):
        self.buoys_msg = buoys_msg
    
    def laser_callback(self, laser_msg):
        self.scan_msg = laser_msg

    def process(self):
        if self.pose_msg == None:
            return
        pose = rosPose2mypose(self.pose_msg.pose)
        print(pose)
        #origin = [pose[0] - self.width/2*self.resolution, pose[1] - self.height/2*self.resolution, 0]
        origin = [pose[0], pose[1], pose[2]]
        grid = np.zeros((self.width, self.height))

        # if self.buoys_msg != None:
        #     for buoy in self.buoys_msg.poses:
        #         x0, y0, type = buoy.position.x, buoy.position.y, buoy.position.z
        #         x0g, y0g = round((x0 - origin[0])/self.resolution), round((y0 - origin[1])/self.resolution)
                
        #         if type == 1: # green buoy
        #             #line goes to the left because boat goes to the right
        #             if -pi/4 < pose[2] < pi/4:
        #                 x1g, y1g = self.width, y0g
        #             if pi/4 < pose[2] < 3*pi/4:
        #                 x1g, y1g = x0g, 0
        #             if -3*pi/4 < pose[2] < -pi/4:
        #                 x1g, y1g = x0g, self.height
        #             else:
        #                 x1g, y1g = 0, y0g   
        #             cv2.line(grid, (x0g, y0g), (x1g, y1g), 100, 10)
                
        #         if type == 2: # red buoy
        #             #line goes to the right because boat goes to the left
        #             if -pi/4 < pose[2] < pi/4:
        #                 x1g, y1g = 0, y0g
        #             if pi/4 < pose[2] < 3*pi/4:
        #                 x1g, y1g = x0g, self.height
        #             if -3*pi/4 < pose[2] < -pi/4:
        #                 x1g, y1g = x0g, 0
        #             else:
        #                 x1g, y1g = self.width, y0g
        #             cv2.line(grid, (x0g, y0g), (x1g, y1g), 100, 10)
                
        #         if type == 3: # yellow buoy
        #             #line goes to the right because boat goes to the left
        #             cv2.circle(grid, (x0g, y0g), 30, 100, 2)
            
        grid[:10,:10].fill(100)

        grid = grid.astype(int)
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.seq = self.seq
        map_msg.header.frame_id = "fosenkaia_NED"
        map_msg.info.resolution = self.resolution
        map_msg.info.height = self.height
        map_msg.info.width = self.width
        map_msg.info.origin = mypose2rosPose(origin)
        map_msg.data = grid.flatten()
        self.ocg_pub.publish(map_msg)
        self.seq += 1

# def write_to_file(grid):
#     with open("grid.txt", 'w') as fh:
#         for row in grid:
#             fh.write(str(row))

def main(args):

    rospy.init_node('local_map_node', anonymous=True)
    rospy.loginfo(f"Starting local mapper node - using Python{sys.version}")
    rate = rospy.Rate(1)

    #Publishers for the maps
    map_pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size=5)

    mapper = Mapper(map_pub)

    # Subscriber to the (x, y, teta) coordinates in the inertial frame
    rospy.Subscriber('nav/pose', PoseStamped, mapper.pose_callback)
    #Subscriber to the detected buoys:
    rospy.Subscriber('/not_filtered', PoseArray, mapper.buoys_callback)
    #Subscriber to the laser scan if available:
    rospy.Subscriber('scan', LaserScan, mapper.laser_callback)
    #TODO: Add buoys subscriber

    while not rospy.is_shutdown():
        mapper.process()
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        #fp.close()
        pass