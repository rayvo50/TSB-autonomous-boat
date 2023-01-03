#!/usr/bin/env python3
import sys
import numpy as np
from math import sqrt, atan, tan, pi

#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages/')
#sys.path.__add__('/opt/ros/melodic/lib/python2.7/dist-packages/')

import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField

def pub_pointcloud(pontos, publisher):
    """
    pub_pointcloud takes in a list of XYZ points and publishes them as a ros Pointcloud2 msg 
    :param pontos: set of points to be published
    :param publisher: ros publisher to use
    """ 

    # create test point cloud to visualize on rviz
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]
    
    header = Header()
    header.frame_id = 'velodyne'
    header.stamp = rospy.Time.now()

    points = np.array([pontos[:,0], pontos[:,1], pontos[:,2], pontos[:,2]])
    points = points.reshape(4,-1).T

    pointcloud = pc2.create_cloud(header, fields, points)
    publisher.publish(pointcloud)



def gen_points():
    pontos = []
    x = -50
    while x <= 50:
        y = -10
        while y <= 100:
            z = -5
            while z <= 15:
                pontos.append([x,y,z])
                z+=1
            y+=1
        x+=1        
    pontos = np.reshape(pontos, (len(pontos), 3))
    return pontos

def main(args):
    rospy.init_node('CloudPub', anonymous=True)
    cloud_pub = rospy.Publisher('cloud', PointCloud2, queue_size=2)
    rate = rospy.Rate(2)
    pontos = gen_points()
    print(pontos)
    while not rospy.is_shutdown():
        pub_pointcloud(pontos, cloud_pub)
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass