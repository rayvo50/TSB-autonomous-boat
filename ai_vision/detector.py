#!/usr/bin/env python3
import cv2
import time
import sys
import numpy as np
from math import sqrt, atan, atan2, tan, pi

#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages/')
#sys.path.__add__('/opt/ros/melodic/lib/python2.7/dist-packages/')

import rospy
import sensor_msgs.point_cloud2 as pc2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
from venv_node.msg import RangeBearing, RBarray

#fp = open("/home/zepedrofig/catkin_ws/src/ai_vision/scripts/dados_out.txt", "w")


def calc_angle(res_h, res_w, ptx, pty, FOVx=62.11118, FOVy=37.42529, Degree=False):
    """
    calc_angle computes the vertical and horizontal angles of a certain pixel
    based on camera FOV.
    :param res_h, res_w: resolution of the camera
    :param ptx, pty: pixel coordinates
    :param FOVx, FOVy: camera FOV
    :return: a tuple containing the horizontal and vertical angles
    """

    #  convert FOV to radians
    fovx = FOVx/180*pi
    fovy = FOVy/180*pi  

    #  Horizontal angle
    dipH = (res_w/2)/tan(fovx/2) 
    x = -(res_w/2 - ptx)
    angleH = atan(x/dipH)  
    #  Vertical angle
    dipV = (res_h/2)/tan(fovy/2)
    y = (res_h/2 - pty)
    angleV = atan(y/dipV) 

    #  convert angles to degrees
    if Degree:
        angleH = angleH/pi*180
        angleV = angleV/pi*180

    return (angleH, angleV)


def get_object_pts(points, angleH, angleV, cam_id, desvio=0.04, estim_dist=-1, max_dist=20 ):
    """
    get_object_pts filters a set of XYZ points to only those that match the specified 
    angles and the specified camera.
    :param points: points from the lidar pointcloud
    :param angleH, angleV: angles where to search for points
    :param cam_id: identifier of the camera that the angle cooresponds to
    :return: the subset of points found or an empty list if no points are found
    """
    
    #TODO: this has to be changed for the physical boat but works fine for the simulator
    
    # for now assume camera_pos = lidar_pos = origin (0, 0, 0)
    lim_sup_H = angleH + desvio
    lim_inf_H = angleH - desvio
    lim_sup_V = angleV + desvio*3       # for better acomodate seamarker shape
    lim_inf_V = angleV - desvio*3
    
    # compute angles of each point and filter pointcloud to only points in the right angles (points in a kinda-cone shape)
    pontos = []
    distances = []
    for i, p in enumerate(points):
        if cam_id == 'Front':
            angH = atan(p[0]/p[1])      # atan( x / y )
            angV = atan(p[2]/p[1])      # atan( z / y )
            dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  # distance to the origin
            if (p[1] > 0) and (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :  # y > 0
                pontos.append(p)
                distances.append(dist)
        elif cam_id == 'Back':
            angH = atan(-p[0]/(-p[1]))  # atan( -x / -y )
            angV = atan(p[2]/(-p[1]))   # at 
            angH = atan(p[1]/(-p[0]))   # atan( y / -x )
            angV = atan(p[2]/(-p[0]))   # atan( z / -x )
            dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  
            if (p[0] < 0) and (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :  # x < 0
                pontos.append(p)
                distances.append(dist)
        elif cam_id == 'Starboard':
            angH = atan(-p[1]/p[0])     # atan( -y / x )
            angV = atan(p[2]/p[0])      # atan( z / x )
            dist = sqrt(pow(p[0],2) + pow(p[1],2) + pow(p[2],2) )  
            if (p[0] > 0) and (angH > lim_inf_H and angH < lim_sup_H) and (angV > lim_inf_V and angV < lim_sup_V) :  # x > 0
                pontos.append(p)
                distances.append(dist)
        else:
            pass
    # print(pontos)
    # print(len(distances))
    # print(estim_dist)
    if len(distances) != 0:         # not go if there are no points 
        if estim_dist != -1:        # if an estimated distance is provided:
            search_dist = 5         # initial value
            pontos_temp = []
            distances_temp = []      
            while (len(pontos_temp) == 0) and (search_dist <=max_dist) :    # while no points have been found
                for i, p in enumerate(pontos):
                    if (distances[i] <= estim_dist+search_dist) and ((distances[i] >= estim_dist-search_dist)):     # check if there are any points within the search region
                        pontos_temp.append(p)
                        distances_temp.append(distances[i])
                search_dist  += 2.5     # incremente search region 
            pontos = pontos_temp
            distances = distances_temp    
    
    if len(distances) == 0:
        return []
    
    # merge pontos and distances in the same numpy array
    pontos = np.reshape(pontos, (len(pontos), 3))          
    distances = np.reshape(distances, (len(pontos), 1))
    pontos = np.append(pontos, distances, axis=1)
    return pontos


def calc_distance(points):
    """
    calc_distance calculates the average of each coordinate as well as distance from a set of points
    :param points: a set of points
    :return: a vector containing the averages of each coordinate + distance
    """ 
    
    if len(points) != 0:
      return [np.mean(points[:,0]), np.mean(points[:,1]), np.mean(points[:,2]), np.mean(points[:,3])]
    else: 
      return [-1, -1]



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



class Detector():

    def __init__(self, image_pub, cloud_pub, detections_pub, CONFIDENCE_THRESHOLD=0.2, NMS_THRESHOLD=0.4):

        self.image_pub = image_pub        # to publish the image with detections
        self.cloud_pub = cloud_pub        # to publish a cloud with the detected objects
        self.detections_pub = detections_pub              
        self.CONFIDENCE_THRESHOLD = CONFIDENCE_THRESHOLD
        self.NMS_THRESHOLD = NMS_THRESHOLD
        self.bridge = CvBridge()    # to convert from img msg to cv2 format
        self.seq = 0

        # the names of the classes for the detection
        self.class_names = ['DangerMarker', 'GreenMarker', 'RedMarker', 'ShallowMarker']           

        weights = '/home/zepedrofig/catkin_ws/src/venv/ai_vision/our_net.weights'
        cfg = '/home/zepedrofig/catkin_ws/src/venv/ai_vision/our_net.cfg'

        net = cv2.dnn.readNet(weights, cfg)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

        self.model = cv2.dnn_DetectionModel(net)
        self.model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)
        
    
    def callback(self, image, cloud):
    
        COLORS = [(0, 255, 255), (0, 255, 0), (0, 0, 255), (64, 64, 64)]

        # get the frame from the image msg (camera)
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        size = frame.shape # (360, 640, 3)
        frame2 = cv2.resize(frame, (416, 416))
        xratio = size[1]/416
        yratio = size[0]/416

        # get points from pointcloud msg (lidar)
        assert isinstance(cloud, PointCloud2)
        gen = pc2.read_points(cloud, field_names=('x','y','z'), skip_nans=True)
        points_xyz = []
        for p in gen:
            if not(p[0]>-2.0 and p[0]<2.0 and p[1]>-2.0 and p[1]<2.0 and p[2]>-2.0 and p[2]<2.0):  # imediatly remove points too close to the boat (2 metres)
                points_xyz.append(p)
        points_xyz = np.reshape(points_xyz, (len(points_xyz), 3))

        # detect stuff
        start_time = time.time()
        classes, scores, boxes = self.model.detect(
            frame2, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)
        end_time = time.time()

        # draw bounding box
        # for each detected object:
        # display all detected objects in the same cloud for now and for easy visualization
        measurements = []
        points = []
        for (classid, score, box) in zip(classes, scores, boxes):
            color = COLORS[int(classid) % len(COLORS)]
            label = "%s : %f" %(self.class_names[classid], score)
            # box = [x, y, w, h]
            new_box = [int(xratio*box[0]), int(yratio*box[1]), int(xratio*box[2]), int(yratio*box[3])]
            cv2.rectangle(frame, new_box, color, 1)
            cv2.putText(frame, label, (new_box[0], new_box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            center = (int(new_box[0]+new_box[2]/2), int(new_box[1]+new_box[3]/2))
            angleH, angleV = calc_angle(size[0], size[1], center[0], center[1])
            
            # estimate distance where to search
            #estimated_dist =  3745/(new_box[3] + 5.275) # model obtained from matlab
            estimated_dist =  -3.63 + 4510/(new_box[3] + 14.1) # anothdr model obtained from matlab
            # search for points in the right angles and in the estimated distance 
            object_pts = get_object_pts(points_xyz, angleH, angleV, 'Front', estim_dist=estimated_dist) 
            
            # if an object is in fact found in the cloud
            if len(object_pts) != 0:
                points.append(object_pts)       # add object to the cloud for publishing
                distance = calc_distance(object_pts)
                if len(distance) == 4:
                    measure = RangeBearing()
                    measure.id = classid #self.class_names[classid]
                    measure.range = sqrt(distance[0]**2 + distance[1]**2) 
                    measure.bearing = atan2(distance[0], distance[1]) 
                    measurements.append(measure)
                    rospy.loginfo(self.class_names[classid] + ' at: ' + "(%.3f, %.3f, %.3f), distance : %.3f m | estimated : %.3f" %(distance[0], distance[1], distance[2], distance[3], estimated_dist))
                    #global fp 
                    #fp.write("%.3f %.3f %.3f %.3f %d %d %d \n"%(distance[0], distance[1], distance[2], distance[3], new_box[3], center[0], center[1]))
            #else:
                #rospy.loginfo('Warning: Object not found in Pointcloud')
        if len(measurements) != 0:
            self.seq +=1
            header = Header(self.seq, rospy.Time.now(), "velodyne")
            rbarray = RBarray(header, measurements)
            self.detections_pub.publish(rbarray)
        

        # # PRINT POINTCLOUD
        # # if any object was found 
        # if len(points) != 0:
        #     # merge all points to the same cloud
        #     cloud = points[0]
        #     if len(points) > 1:
        #         for i in range(len(points)-1):
        #             cloud = np.concatenate((cloud, points[i+1]))
        #     pub_pointcloud(cloud, self.cloud_pub)

        # display fps (doesn't actually represent fps but is a way to check cnn perfomance)
        fps_label = "FPS: %.2f"%(1/(end_time - start_time))
        cv2.putText(frame, fps_label, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        Image_to_pub = self.bridge.cv2_to_imgmsg(frame)
        self.image_pub.publish(Image_to_pub) #image with detections



def main(args):
    rospy.init_node('Detector', anonymous=True)
    rospy.loginfo('Python version: ' + sys.version)
    camera_sub = Subscriber('/optical/Front/image_raw', Image)
    lidar_sub = Subscriber('lidar', PointCloud2)
    image_pub = rospy.Publisher('detected_image', Image, queue_size=2)
    cloud_pub = rospy.Publisher('detected_cloud', PointCloud2, queue_size=2)
    detections_pub = rospy.Publisher('detections', RBarray, queue_size=2)

    detector = Detector(image_pub, cloud_pub, detections_pub)

    ats = ApproximateTimeSynchronizer([camera_sub, lidar_sub], queue_size=10, slop=0.3, allow_headerless=False)
    ats.registerCallback(detector.callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        #fp.close()
        pass


