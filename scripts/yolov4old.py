#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys
from scripts.detector import main

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg


def main():

    rospy.init_node('lidar')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = sys.argv[1]

    static_transformStamped.transform.translation.x = float(sys.argv[2])
    static_transformStamped.transform.translation.y = float(sys.argv[3])
    static_transformStamped.transform.translation.z = float(sys.argv[4])

    quat = tf.transformations.quaternion_from_euler(
                float(sys.argv[5]),float(sys.argv[6]),float(sys.argv[7]))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()



if __name__ == '__main__':
    main()
