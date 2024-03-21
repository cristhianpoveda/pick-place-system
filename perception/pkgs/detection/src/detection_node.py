#!/usr/bin/env python3

import numpy as np
import rospy
import rospkg
import time
import math
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
from std_msgs import String
from sensor_msgs import Image

class ObjectDetector():

    def __init__(self, node_name):

        # params
        rospack = rospkg.RosPack()
        rospack.get_path('detection')
        model_path = rospack.get_path('detection') + '/model/best.pt'
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.pub_projeciton = rospy.Publisher('/projection', String, queue_size=0)

        self.pub_result = rospy.Publisher('/result', String, queue_size=0)

        self.pub_image = rospy.Publisher('/detection', Image, queue_size=0)

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.camera, queue_size=1)

    def get_center(self, keypoints):
        max_l = 0
        center = ''
        for b in keypoints:
            base_x = int(b[0][0])
            base_y = int(b[0][1])
            thread_x = int(b[1][0])
            thread_y = int(b[1][1])
            print(base_x, base_y, thread_x, thread_y)
            if not ((base_x == 0 and base_y == 0) or (thread_x == 0 and thread_y ==0)):
                lenght = np.sqrt(np.power(abs(thread_x - base_x),2) + np.power(abs(thread_y - base_y),2))
                if lenght > max_l:
                    max_l = lenght
                    center = str(abs(thread_x + base_x) / 2) + ',' + str(abs(thread_y + base_y) / 2)
        
        return center
    
    def get_pose(self, center):
        pose = center # get from aligned data
        return pose

    def camera(self, image):
        image_np = np.zeros((640, 480, 3), dtype = "uint8")
        try:
            image_np = self.bridge.imgmsg_to_cv2(image, "rgb8")
            detections = self.model(image_np, conf=0.7)
            keypoints = detections[0].keypoints.xy
            if len(keypoints[0]) != 0:
                center = self.get_center(keypoints)
                if len(center) != 0:
                    pose = self.get_pose(center)
                    self.pub_projeciton.publish(pose)

                    if self.pub_image.anybody_listening():

                        cv2.circle(image_np, (int(center[0]), int(center[2])), 3, (0, 255, 0), 2)

                        label = 'pos: ' + center
                        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                        cv2.rectangle(image_np, (int(center[0]), int(center[1])+labelSize[1]+10), (int(center[0])+labelSize[0], int(center[1])-baseLine+10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                        cv2.putText(image_np, label, (int(center[0]), int(center[1])+labelSize[1]+7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_np, "rgb8"))

                    result = 'detection: success!'
                else:
                    result = 'no accurate detections'

            else:
                result = 'no bottle detections'

            if self.pub_result.anybody_listening():
                self.pub_result.publish(result)
                
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # create the node
    node = ObjectDetector(node_name='detection_node')

    # keep spinning
    rospy.spin()