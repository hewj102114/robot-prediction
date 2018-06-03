#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy 
import roslib
import time
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import cv2
import sys
import os
import glob
import tf
import tf2_ros

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from robo_perception.msg import ObjectList
from robo_perception.msg import Object


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



count = 0

SAVE_NUM = 10000
SAVE_PATH = '/media/ubuntu/p600/infrared'
ENABLE_VISULIAZE = False


def callback_image(image):
    global count, sess, model, mc, video, frame_rate_list, frame_rate_idx, frame_rate
    print ('I here image !',count)
    count = count + 1

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="8UC1")
        print(cv_image.shape)
        #cv_image_depth = bridge.imgmsg_to_cv2(depth, desired_encoding="16UC1")
    except CvBridgeError as e:
        print(e)


    #visualize
    #cv2.imshow("Image window", cv_image_depth)
    #cv2.waitKey(3)

    image_count = count
    image_name = image_count % SAVE_NUM
    file_name = str(image_name) + '.jpg'
    out_file_name = os.path.join(SAVE_PATH, 'out_'+file_name)
    #im = im.astype('uint8')
    if ENABLE_VISULIAZE:
        cv2.imshow('demo', im)
        cv2.waitKey(3)  
    cv2.imwrite(out_file_name, cv_image)
        
        
    print ('Image detection output saved to {}'.format(out_file_name))



rospy.init_node('rgb_detection')

image_sub = rospy.Subscriber('camera/infra1/image_rect_raw', Image, callback_image)



rospy.spin()

