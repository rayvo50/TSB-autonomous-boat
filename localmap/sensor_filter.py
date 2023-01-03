#!/usr/bin/env python3
import rospy
import sys
import numpy as np
from utils import *
from math import sin, cos, pi, atan2, asin, sqrt
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, PoseArray 
from venv_node.msg import RangeBearing, RBarray

QT = np.diag([0.2, np.deg2rad(20)])         # sensor model covariance


def jacobian(particle, last_mean):
    Ht = np.array( 
            [[(last_mean[0,0]- particle.x)/sqrt((last_mean[0,0]- particle.x)**2 + (last_mean[1,0]- particle.y)**2), 
              (last_mean[1,0]- particle.y)/sqrt((last_mean[0,0]- particle.x)**2 + (last_mean[1,0]- particle.y)**2)],
            [ -(last_mean[1,0]- particle.y)/((last_mean[0,0]- particle.x)**2 + (last_mean[1,0]- particle.y)**2),
            (last_mean[0,0]- particle.x)/((last_mean[0,0]- particle.x)**2 + (last_mean[1,0]- particle.y)**2)]])
    return Ht

def new_ldmrk(particle, z):
    mean_t = np.array([particle.x + z[0]*cos(pi2pi(particle.teta + z[1])), particle.y + z[0]*sin(pi2pi(particle.teta + z[1]))]).reshape(2,1)
    H = jacobian(particle, mean_t)
    H_inv = np.linalg.inv(H)
    sigma = H_inv @ QT @ H_inv.T
    landmark = LandmarkEKF(mean_t, sigma, z[2])
    return landmark

def predict_measurement(particle, mean):
    d = sqrt( (mean[0,0] - particle.x)**2 + (mean[1,0] - particle.y)**2 )
    teta = pi2pi(atan2(mean[1,0] - particle.y, mean[0,0] - particle.x) - particle.teta)
    return np.array([d, teta]).reshape(2,1)

class LandmarkEKF():
    def __init__(self, mean, sigma, id, type):
        self.mean = np.array(np.reshape(mean, (2,1)))       # [mean_x, mean_y]
        self.sigma = np.array(np.reshape(sigma, (2,2)))     # covariance matrix
        self.id = id
        self.type = type

    def comp_w8_gains(self, particle, z):

        # measurement prediction
        z_pred = predict_measurement(particle, self.mean)
        z = np.array([z[0], z[1]]).reshape(2,1)

        # compute jacobian of sensor model 
        H = jacobian(particle, self.mean)

        # measurement covariance
        Q = H @ self.sigma @ H.T + QT
        Q_inv = np.linalg.inv(Q)

        # compute kalman gain
        K = self.sigma @ H.T @ Q_inv
        c = (z - z_pred)
        c[1,0] = pi2pi(c[1,0])
        
        # Compute weight
        e = c.T @ Q_inv @ c
        det = abs(np.linalg.det(Q))
        weight = (1/(2*pi*sqrt(det)))*np.exp(-0.5*e[0,0])

        # save information for updating EKF later
        self.K = K
        self.c = c
        self.H = H
        
        return weight   

    def update(self):
        
        K = self.K
        c = self.c
        H = self.H

        # update mean: µ(t) = µ(t-1) + K (z - ẑ)
        self.mean = self.mean + K @ c
        # update covariance: Σ(t) = (I - K H) Σ(t-1) 
        self.sigma = (np.identity(2) - K @ H) @ self.sigma



    # most likely data association 
    # TODO: change sigma matrix to be one always big enough to acomodate bad measurements
    # if len(particle.ldmrks) == 0:
    #     return (-1, -1)
    # x = particle.x + z[0]*cos(particle.teta + z[1])
    # y = particle.y + z[0]*sin(particle.teta + z[1])
    # max, max_i = (0,0)
    # for i, lm in enumerate(particle.ldmrks):
    #     temp = np.array([[x-lm.mean[0,0]], [y-lm.mean[1,0]]])
    #     temp = temp.T @ np.linalg.inv(lm.sigma) @ temp
    #     det = np.linalg.det(lm.sigma)
    #     p = (1/(2*pi*sqrt(abs(det)))) * np.exp(-0.5*temp[0,0])
    #     if p > max:
    #         max = p
    #         max_i = i
    # return (max_i, max)    

class Filters():
    def __init__(self, ocg_pub) -> None:
        self.ocg_pub = ocg_pub

        #To save ROS messages from subscribers
        self.buoys_msg = None

        self.filters = []

    def buoys_callback(self, buoys_msg):
        self.buoys = buoys_msg

    def pose_callback(self, pose_msg):
        self.pose = pose_msg

    def data_association(self, z):
        #known data association
        if len(self.filters) == 0:
            return 0
        i = 0
        for buoy in self.filters:
            #check
            pass
        return i

    def process(self):

        if self.buoys_msg == None:
            return
        
        # TODO: check if the message is the same as previous:

        for z in self.buoys_msg.measurements:
            pass


def main(args):

    rospy.init_node('local_map_node', anonymous=True)
    rate = rospy.Rate(1)

    #Publishers for the maps
    map0_pub = rospy.Publisher('/filtered_stuff', PoseArray, queue_size=5)
    filters = Filters()

    rospy.Subscriber('detections', RBarray, filters.pose_callback)

    while not rospy.is_shutdown():
        filters.process()
        rate.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass