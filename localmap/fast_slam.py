#!/usr/bin/env python3

from array import array
from email import header
import time
import sys
import numpy as np
from math import sqrt, pi, cos, sin, atan2, floor
#import matplotlib.pyplot as plt

import rospy
#from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion
from venv_node.msg import RangeBearing, RBarray
# from fiducial_msgs.msg import FiducialTransformArray
#from tf.transformations import quaternion_from_euler, euler_from_quaternion


M_PARTICLES = 100       # number os particles 

QT = np.diag([0.2, np.deg2rad(20)])         # sensor model covariance
#R = np.diag([0.25, np.deg2rad(15)])        # motion model covariance

# for micro simulation
ROOM_SIZE = 5
MAP = np.array([
    [2,2],
    [3,-4],
    [-3, 2],
    [1,-1],
    [-1,-2],
    [2,3],
    [3,-1],
    [-1,0],
    [0,-2],
    [-1,1]
    ])
CAM_FOV = 90
COLORS = [(0,0,255/255),(0,128/255,255/255),(0,255/255,255/255),(128/255,128/255,128/255),(0,255/255,0),(255/255,255/255,0),(255/255,128/255,0),(255/255,0,0),(255/255,0,128/255),(255/255,0,255/255)]


# Defines the shape of the particles
#TODO: add garbage collection?
class Particle():
    def __init__(self, x, y, teta, ldmrks):
        self.x = x
        self.y = y
        self.teta = teta
        self.ldmrks = ldmrks
        self.trajectory = []

    def copy(self):
        new = Particle(self.x, self.y, self.teta, self.ldmrks)
        return new


class LandmarkEKF():
    def __init__(self, mean, sigma, id):
        self.mean = np.array(np.reshape(mean, (2,1)))       # [mean_x, mean_y]
        self.sigma = np.array(np.reshape(sigma, (2,2)))     # covariance matrix
        self.id = id

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
        c[1,0] = pi_2_pi(c[1,0])
        
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
    
    #TODO: check if this is even being usec
    def copy(self):
        new = LandmarkEKF(self.mean, self.sigma, self.id)
        return new


def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi

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

    
def jacobian(particle, last_mean):
    Ht = np.array( 
            [[(last_mean[0,0]- particle.x)/sqrt((last_mean[0,0]- particle.x)**2 + (last_mean[1,0]- particle.y)**2), 
              (last_mean[1,0]- particle.y)/sqrt((last_mean[0,0]- particle.x)**2 + (last_mean[1,0]- particle.y)**2)],
            [ -(last_mean[1,0]- particle.y)/((last_mean[0,0]- particle.x)**2 + (last_mean[1,0]- particle.y)**2),
            (last_mean[0,0]- particle.x)/((last_mean[0,0]- particle.x)**2 + (last_mean[1,0]- particle.y)**2)]])
    return Ht

def new_ldmrk(particle, z):
    mean_t = np.array([particle.x + z[0]*cos(pi_2_pi(particle.teta + z[1])), particle.y + z[0]*sin(pi_2_pi(particle.teta + z[1]))]).reshape(2,1)
    H = jacobian(particle, mean_t)
    H_inv = np.linalg.inv(H)
    sigma = H_inv @ QT @ H_inv.T
    landmark = LandmarkEKF(mean_t, sigma, z[2])
    return landmark

def predict_measurement(particle, mean):
    d = sqrt( (mean[0,0] - particle.x)**2 + (mean[1,0] - particle.y)**2 )
    teta = pi_2_pi(atan2(mean[1,0] - particle.y, mean[0,0] - particle.x) - particle.teta)
    return np.array([d, teta]).reshape(2,1)
    
# particle.ldmrks is an EFK, new_lm is a [d, teta] pair
def data_association(particle, z):
    #known data association
    for i, lm in enumerate(particle.ldmrks):
        if lm.id == z[2]:
            return (i, 100)
    return (-1, -1)

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


# Main class for implementing ROS stuff
class ParticleFilter():
    def __init__(self, info_pub, map_pub, pose_pub):
        self.pub = info_pub             # for debug purposes
        self.pose_pub = pose_pub        # for display purposes
        self.map_pub = map_pub
        self.sample_counter = 0
        self.seq = 0
        self.counter = 0

        # variables for saving latest ROS msgs
        self.odom_data = [-1,0,0,0]                # latest odometry msgs
        self.prev_odom_data = [-1,0,0,0]   
        self.sensor_data = []                   # last sensor input

   
    def pub_map_pose(self):
        p = self.Xt[np.argmax(self.w)]
        array = []
        for lm in p.ldmrks:
            point = Point(lm.mean[0,0], lm.mean[1,0], 0)
            quat = Quaternion(0,0,0,1)
            lm_pose = Pose(point, quat)
            array.append(lm_pose)
        header = Header(self.seq, rospy.Time.now(), "fosenkaia_NED")
        if len(array) != 0:
            msg = PoseArray(header, array)
            self.map_pub.publish(msg)
        point = Point(p.x, p.y, 0)
        q = euler_2_quat(0,0,p.teta)
        quat = Quaternion(q[0], q[1], q[2], q[3])
        pose = Pose(point, quat)
        msg = PoseStamped(header, pose)
        self.pose_pub.publish(msg)
        

            
    # For micro simulation
    def sense(self, map):
        detections = []
        fov = CAM_FOV/2*pi/180
        #lims = [add_angle(self.teta, fov), add_angle(self.teta, -fov)]
        for id, lm in enumerate(map):
            d = sqrt((lm[0]-self.x)**2 + (lm[1]-self.y)**2)
            teta_d = atan2((lm[1]-self.y), (lm[0]-self.x)) - self.teta
            if d <= 2: # sense only if its close
                # add some noise
                d += np.random.normal(0, abs(0.1*d))
                teta_d += np.random.normal(0, abs(0.1*teta_d))
                detections.append([d, pi_2_pi(teta_d), id])
        detections = np.array(detections)
        return detections


    def check_map_quality(self):
        max = np.argmax(self.w)
        av = np.zeros((2,2))
        for lm in self.Xt[max].ldmrks:
            av = av + lm.sigma
        av = av / len(self.Xt[max].ldmrks)
        print(av)


    def normalize_weights(self):        # O(M)
        sum = np.sum(self.w)
        if np.isinf(sum):
            self.w = np.ones(M_PARTICLES) / M_PARTICLES
        else:    
            self.w = np.array(self.w) / np.sum(self.w)

    
    def low_variance_resample(self):    # O(M*log(M))
        
        n_eff = 1/(sum(self.w ** 2))
        if n_eff > M_PARTICLES/2:
            return self.Xt
        Xt = []
        r = np.random.uniform(0, 1/M_PARTICLES)
        c = self.w[0]
        i = 0
        for m in range(len(self.Xt)):
            U = r + m/M_PARTICLES
            while U >= c:
                i+=1
                c = c + self.w[i]
            particle = self.Xt[i].copy()
            Xt.append(particle)

        return Xt


    # save information from ROS msgs into class variables
    def callback(self, odom, sensor):
        #rpy = euler_from_quaternion([odom.pose.orientation.x, odom.pose.orientation.y, odom.pose.orientation.z, odom.pose.orientation.w])
        q0 = odom.pose.orientation.w 
        q1 = odom.pose.orientation.x
        q2 = odom.pose.orientation.y
        q3 = odom.pose.orientation.z
        yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
        self.odom_data = np.array([odom.pose.position.x, odom.pose.position.y, odom.pose.position.z, yaw])
        
        sensor_data = []
        for rb in sensor.measurements:
            sensor_data.append([rb.range, rb.bearing, rb.id])
        self.sensor_data = np.array(sensor_data)

        #print(self.sensor_data)

    def process(self):
        # copy msgs info into local variables 
        odom_data = np.copy(self.odom_data)
        sensor_data = np.copy(self.sensor_data)      


        if self.prev_odom_data[0] == -1:             # first message must be ignored in order to compute ΔT
            self.prev_odom_data = odom_data
            # create particles
            self.Xt = []
            for i in range(M_PARTICLES):
                self.Xt.append(Particle(odom_data[0], odom_data[1], pi_2_pi(odom_data[3]), []))
            self.w = np.ones(M_PARTICLES) / M_PARTICLES
            return

        #if self.prev_odom_data[0] == self.odom_data[0]:       # dont process if there are no new msgs
        #    return

        dx = odom_data[0] - self.prev_odom_data[0]
        dy = odom_data[1] - self.prev_odom_data[1]
        dteta = pi_2_pi(odom_data[3] - self.prev_odom_data[3])
        self.prev_odom_data = odom_data

        # update particles with input:
        for i in range(len(self.Xt)):
            self.Xt[i].x += dx
            self.Xt[i].y += dy
            self.Xt[i].teta += dteta
            self.Xt[i].teta = pi_2_pi(self.Xt[i].teta)

        #print("COORDINATES: (x: " + str(self.Xt[0].x) + ", y: " + str(self.Xt[0].y) + ", θ: " + str(self.Xt[0].teta) + ")")

        # update particles based on sensor data
        if len(sensor_data) == 0:        # dont update EKFs if no landmarks were found
            return

        # SENSOR UPDATE
        weights = []
        ldmrks_to_update = []
        for i in range(len(self.Xt)):
            weight = 1
            for z in sensor_data:
                max_i, p = data_association(self.Xt[i], z)
                if p < 0.1 or max_i == -1:
                    # add new landmark
                    landmark = new_ldmrk(self.Xt[i], z)
                    self.Xt[i].ldmrks.append(landmark)
                else:
                    # update an already found landmark
                    w = self.Xt[i].ldmrks[max_i].comp_w8_gains(self.Xt[i], z)
                    # add the pointer to the landmark to a list so it can be updated later
                    ldmrks_to_update.append(self.Xt[i].ldmrks[max_i])
                    weight = weight * w

            weights.append(weight)
        self.w = np.array(weights)

        # weights are ok, update landmarks
        for lm in ldmrks_to_update:
            lm.update()

        self.pub_map_pose()
        #print("map published")
        self.normalize_weights()
        # # RESAMPLING
        self.Xt = self.low_variance_resample()



def main(args):

    rospy.init_node('FSLAML_node', anonymous=True)
    rospy.loginfo('Initializing FastSLAM1.0 node with Python version: ' + sys.version)

    odom_sub = Subscriber('nav/pose', PoseStamped)
    sensor_sub = Subscriber('detection', RBarray)
    #aruco_sub = Subscriber('fiducial_transforms', FiducialTransformArray)
    #scan_sub = Subscriber('lidar', PointCloud2)
    
    info_pub = rospy.Publisher('info', String, queue_size=2)
    map_pub = rospy.Publisher('fast_slam_map', PoseArray, queue_size=2)
    pose_pub = rospy.Publisher('fast_slam_pose', PoseStamped, queue_size=2)
    # map_pub = ... TODO: inventar um mapa

    pf = ParticleFilter(info_pub, map_pub, pose_pub)
    rate = rospy.Rate(5)
    ats = ApproximateTimeSynchronizer([odom_sub, sensor_sub], queue_size=10, slop=0.1, allow_headerless=False)
    ats.registerCallback(pf.callback)

    while not rospy.is_shutdown():
        pf.process()
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        #fp.close()
        pass
