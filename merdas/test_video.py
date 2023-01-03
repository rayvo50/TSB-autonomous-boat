#!/usr/bin/env python3
import rospy
import cv2
import time
import sys
#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages/')
#sys.path.__add__('/opt/ros/melodic/lib/python2.7/dist-packages/sensor_msgs')

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#from std_msgs.msg import String


class Detector():
    def __init__(self, pub, CONFIDENCE_THRESHOLD=0.2, NMS_THRESHOLD=0.4):
        self.pub = pub              # to publish the image with detections
        self.CONFIDENCE_THRESHOLD = CONFIDENCE_THRESHOLD
        self.NMS_THRESHOLD = NMS_THRESHOLD
        self.bridge = CvBridge()    # to convert from img msg to cv2 format

        # the names of the classes for the detection
        self.class_names = ['DangerMarker', 'GreenMarker', 'RedMarker', 'ShallowMarker']           

        weights = '/home/zepedrofig/catkin_ws/src/ai_vision/scripts/our_net.weights'
        cfg = '/home/zepedrofig/catkin_ws/src/ai_vision/scripts/our_net.cfg'

        net = cv2.dnn.readNet(weights, cfg)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

        self.model = cv2.dnn_DetectionModel(net)
        self.model.setInputParams(size=(416, 416), scale=1/255, swapRB=True)
        
    
    def callback_detect(self, Image):
    
        start_time = time.time()
        COLORS = [(0, 255, 255), (0, 255, 0), (0, 0, 255), (64, 64, 64)]

        #get the frame from the image message
        frame_og = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        size = frame_og.shape # (360, 640, 3)
        frame = cv2.resize(frame_og, (416, 416))
        xratio = size[1]/416
        yratio = size[0]/416

        # detect stuff
        classes, scores, boxes = self.model.detect(
            frame, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)
        end_time = time.time()

        # draw bounding box
        for (classid, score, box) in zip(classes, scores, boxes):
            color = COLORS[int(classid) % len(COLORS)]
            label = "%s : %f" %(self.class_names[classid], score)
            # box = [x, y, w, h]
            box_og = [int(xratio*box[0]), int(yratio*box[1]), int(xratio*box[2]), int(yratio*box[3])]
            cv2.rectangle(frame_og, box_og, color, 1)
            cv2.putText(frame_og, label, (box_og[0], box_og[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            center = (int(box_og[0]+box_og[2]/2), int(box_og[1]+box_og[3]/2))
            #cv2.circle(frame_og, center, 2, (255,255,0), thickness=cv2.FILLED)
            #cv2.rectangle(frame, box, color, 1)
            #cv2.putText(frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
        fps_label = "FPS: %.2f"%(1/(end_time - start_time))
        cv2.putText(frame_og, fps_label, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        
        frame = cv2.resize(frame, (size[1], size[0]))
        Image_to_pub = self.bridge.cv2_to_imgmsg(frame_og)
        self.pub.publish(Image_to_pub) #image with detections



def main():
    rospy.init_node('Detector', anonymous=True)
    #rospy.loginfo('Python version: ' + sys.version)
    detection_pub = rospy.Publisher('detections', Image, queue_size=2)
    detector = Detector(detection_pub)
    rospy.Subscriber('/optical/Front/image_raw', Image, detector.callback_detect)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass