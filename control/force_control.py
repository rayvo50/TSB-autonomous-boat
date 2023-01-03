#!/usr/bin/env python3

from cmath import exp
import sys
import numpy as np
from math import sqrt, pi, cos, sin, atan2, asin
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from ros_clients.msg import GeneralizedForce
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Point, TwistStamped

def euler_2_quat(roll, pitch, yaw):
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

def euler_from_quaternion(x, y, z, w):
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



def yaw_error_calc (yaw_error,x_error,y_error):
    dist= sqrt(x_error**2 + y_error**2)
    if dist<0.75:
        ref=0
        return ref
    
    if yaw_error<180 and yaw_error>-180:
        ref=yaw_error
        return ref

    if yaw_error>180:
        ref=360-yaw_error
        return ref

    if yaw_error<-180:
        ref=yaw_error+360

    ref=yaw_error
    return ref


def speed_ref_calc (yaw_error,x_error,y_error):
    error=abs(yaw_error)
    dist_min=5
    if(error>30):
        speed_ref=0
        return speed_ref
    dist= sqrt(x_error**2 + y_error**2)
    print(dist)
    if(dist<1):
        speed_ref=0
        return speed_ref
    if(dist>dist_min):
        speed_ref=2.57
    else:
        a=1.2
        b=dist_min
        speed_ref=2.57*exp(a*(dist-b))
    return speed_ref

class Controller():
    def __init__(self,force_pub):
        self.goal = []
        self.position = []
        self.lidar = []

        self.previous_error=0
        self.integral=0
        self.time=time.time()
        self.dt=0
        self.force_pub=force_pub
        self.brake_counter = 0
        self.danger = 0
        
    def callback_pose(self,pose):
        self.position = pose.pose.position
        self.yaw=np.rad2deg(euler_from_quaternion(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w))

    def callback_goal(self,goal):
        self.goal = goal
       

    def callback_twist(self,twist):
        self.linear=twist.twist.linear
        self.angular=twist.twist.angular

    def callback_lidar(self,lidar):
        self.lidar=lidar

    def get_lidar_pts(self):
        if self.lidar == []:
            return []
        assert isinstance(self.lidar, PointCloud2)
        gen = pc2.read_points(self.lidar, field_names=('x','y','z'), skip_nans=True)
        points_xyz = []
        for p in gen:
            if not(p[0]>-10.0 and p[0]<10.0 and p[1]>-10.0 and p[1]<10.0 and p[2]>-10.0 and p[2]<10.0):  # imediatly remove points too close to the boat (2 metres)
                points_xyz.append(p)
        points_xyz = np.reshape(points_xyz, (len(points_xyz), 3))
        return points_xyz

    def process(self):
        print("GOAL:")
        print(self.goal)
        print("CURRENT:")
        print(self.position)
        print("--------------")
        if self.goal == [] or self.position == []:
            return

        if self.danger and self.brake_counter < 8:
            gf=GeneralizedForce(-100.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.force_pub.publish(gf)
            self.brake_counter += 1
            return
        if self.danger: 
            gf=GeneralizedForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.force_pub.publish(gf)
            return

        points = self.get_lidar_pts()
        for p in points:
            if sqrt(p[0]**2 + p[1]**2) < 20:
                print("Object detected, waiting...")
                self.danger = 1
                return

        self.x_error = self.goal.x - self.position.x
        self.y_error=self.goal.y - self.position.y
        #self.yaw_ref=np.rad2deg(atan2(self.goal.y,self.goal.x))
        self.yaw_ref=np.rad2deg(atan2(self.y_error,self.x_error))
        print(self.yaw_ref)
        self.yaw_error=self.yaw_ref-self.yaw
        yaw_error=yaw_error_calc(self.yaw_error, self.x_error,self.y_error)
        print(yaw_error)
        speed_ref=speed_ref_calc(self.yaw_error, self.x_error,self.y_error) 
        print(speed_ref)
        speed_error=speed_ref-abs(self.linear.x)
        fx=self.PID(80,0,0,speed_error)
        mz=self.PID(10,0,0,yaw_error)
        gf=GeneralizedForce(fx,0,0,0,0,mz)
        self.force_pub.publish(gf)


    def PID(self,Kp,Kd,Ki,error):
        self.dt=time.time()-self.time
        self.time=time.time()
        self.integral=self.integral+error*self.dt 
        derivative=(error-self.previous_error)/self.dt 
        out=Kp*error+Ki*self.integral+Kd*derivative
        self.previous_error=error
        return out 



def main(args):

    rospy.init_node('force_control_node', anonymous=True)
    force_pub = rospy.Publisher('/force_control', GeneralizedForce, queue_size=1)

    ctr = Controller(force_pub)

    rospy.Subscriber('nav/pose', PoseStamped, ctr.callback_pose)
    rospy.Subscriber('/scenario/target/local', Point, ctr.callback_goal)
    rospy.Subscriber('nav/twist', TwistStamped, ctr.callback_twist)
    rospy.Subscriber('lidar', PointCloud2, ctr.callback_lidar)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        ctr.process()
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        #fp.close()
        pass
