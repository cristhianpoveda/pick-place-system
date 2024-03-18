#!/usr/bin/env python3

import numpy as np
import rospy
import rospkg
import time
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs import String
from sensor_msgs import Image

class ObjectDetector():

    def __init__(self, node_name):

        # params
        rospack = rospkg.RosPack()
        rospack.get_path('detection')
        self.model_path = rospack.get_path('detection') + '/model/best.pt'

        self.bridge = CvBridge()

        self.pub_time_ann = rospy.Publisher('/projection', String, queue_size=0)

        # subscriber to camera_node/image/compressed
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.camera, queue_size=1)
    
    def camera(self, image):
        image_np = np.zeros((640, 480, 3), dtype = "uint8")
        try:
            image_np = self.bridge.imgmsg_to_cv2(image, "rgb8")
            detections = self.model_path(image_np, conf=0.7)
            keypoints = detections[0].keypoints.xy
            for bottle in keypoints:
                base_x = int(bottle[0][0][0])
                base_y = int(bottle[0][0][1])
                thread_x = int(bottle[0][1][0])
                thread_y = int(bottle[0][1][1])
                
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # create the node
    node = ObjectDetector(node_name='detection_node')

    # keep spinning
    rospy.spin()