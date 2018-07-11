#!/usr/bin/python2.7
# coding=utf-8
# !/usr/bin/python2.7
# !/home/ubuntu/anaconda2/bin/python2.7
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

from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Twist
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry


class PIDCtrl:
    def __init__(self, Kp, Ki, Kd, max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.preErr = 0.0
        self.sumErr = 0.0
        self.dErr = 0.0
        self.output = 0.0
        self.outMax = max
        self.stop = False

    def reset(self):
        self.preErr = 0.0
        self.sumErr = 0.0
        self.dErr = 0.0

    def calc(self, curErr):
        self.sumErr = self.sumErr + curErr
        self.dErr = curErr - self.preErr
        self.preErr = curErr
        Kisum = self.Ki * self.sumErr
        if Kisum > 0.8:
            Kisum = 0.8
        if Kisum < -0.8:
            Kisum = -0.8
        self.output = self.Kp * curErr + Kisum + self.Kd * self.dErr
        if abs(self.output) > self.outMax:
            self.output = self.outMax * self.output / abs(self.output)

        if self.stop == True:
            self.output = self.output / 5
        return self.output

def callback_odom(msg):
    global goal_pose, current_pose
    current_pose = msg.pose.pose

def callback_calculate_vel(msg):
    global goal_pose, current_pose, pidctrl
    goal_pose = msg
    
    dx = np.power((current_pose.position.x - goal_pose.position.x), 2)
    dy = np.power((current_pose.position.y - goal_pose.position.y), 2)
    
    vel_x = pidctrl.calc(dx)
    vel_y = pidctrl.calc(dy)

    msg_vel = Twist()
    msg_vel.linear.x = vel_x
    msg_vel.linear.y = vel_y
    # msg_vel.angular.z = 0

    pub_vel.publish(msg_vel)

if __name__ == '__main__':
    pidctrl = PIDCtrl(1.5, 0.1, 0, 1.0)
    rospy.init_node("robo_navigation")

    sub_goal = rospy.Subscriber("base/goal", Pose, callback_calculate_vel)
    sub_odom = rospy.Subscriber("odom", Odometry, callback_odom)

    pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    
    rospy.spin()
