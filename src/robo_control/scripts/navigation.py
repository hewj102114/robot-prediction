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

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Twist, Point
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
    global current_pose
    current_pose = msg.pose.pose
    # print(current_pose)

def callback_calculate_vel(msg):
    global goal_pose, current_pose, pidctrl, point1, point2
    # goal_pose = msg

    qn_ukf = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
    (current_roll, current_pitch, current_yaw) = euler_from_quaternion(qn_ukf)

    goal_pose = generate_goal(point1, point2, 0)
    print("current_x: {}, current_y: {}, current_yaw: {}".format(current_pose.position.x, current_pose.position.y, current_yaw))
    print("goal_x: {}, goal_y: {}, goal_yaw: {}".format(goal_pose.position.x, goal_pose.position.y, goal_yaw))
    
    dx = current_pose.position.x - goal_pose.position.x
    dy = current_pose.position.y - goal_pose.position.y
    dyaw = current_yaw - goal_yaw

    print("dx: {}, dy: {}, dyaw: {}".format(dx, dy, dyaw))
    if abs(dyaw) < 10*PI/180:
        vel_yaw = 0
        if abs(dx) < 0.1:
            vel_x = 0.01
        else:
            vel_x = pidctrl_x.calc(dx)

        if abs(dy) < 0.1:
            vel_y = 0.01
        else:
            vel_y = pidctrl_y.calc(dy)
    else:
        vel_yaw = pidctrl_yaw.calc(dyaw)
        vel_x = 0
        vel_y = 0

    msg_vel = Twist()
    msg_vel.linear.x = -vel_x
    msg_vel.linear.y = -vel_y
    msg_vel.angular.z = -vel_yaw
    # msg_vel.angular.z = 0

    print("vel_x: {}, vel_y: {}, vel_yaw: {}".format(vel_x, vel_y, vel_yaw))
    pub_vel.publish(msg_vel)

def generate_goal(point1, point2, mode):
    """
    point1: (x1, y1)
    point2: (x2, y2)
    model: mode=1 -> vy=0, mode=2 -> vx=0
    """
    global current_pose, last_goal_pose, flag1, flag2
    goal_pose = Pose()
    x1, y1, x2, y2 = point1[0], point1[1], point2[0], point2[1]
    if mode == 1:
        y1 = current_pose.position.y
        y2 = current_pose.position.y
    if mode == 2:
        x1 = current_pose.position.x
        x2 = current_pose.position.x
        
    current_x, current_y = current_pose.position.x, current_pose.position.y
    distance1 = np.power((np.power((current_x - x1), 2) + np.power((current_y - y1), 2)), 0.5)
    distance2 = np.power((np.power((current_x - x2), 2) + np.power((current_y - y2), 2)), 0.5)
    print("distance1: {}, distance2: {}".format(distance1, distance2))

    if distance1 < 0.1:
        flag1, flag2 = True, False
        goal_pose.position.x = x2
        goal_pose.position.y = y2
        last_goal_pose = goal_pose

    if distance2 < 0.1:
        flag1, flag2 = False, True
        goal_pose.position.x = x1
        goal_pose.position.y = y1
        last_goal_pose = goal_pose
    if mode == 1:
        last_goal_pose.position.y = current_pose.position.y
    if mode == 2:
        last_goal_pose.position.x = current_pose.position.x
        
    return last_goal_pose
    

if __name__ == '__main__':
    PI = 3.1415926
    current_pose = Pose()  
    flag1, flag2 = False, False
    point1 = (1, 1.5)
    point2 = (1, 2.5)
    goal_yaw = 0.0*PI/180.0
    last_goal_pose = Pose()
    last_goal_pose.position.x = point1[0]
    last_goal_pose.position.y = point1[1]
    
    pidctrl_x = PIDCtrl(2, 0, 0, 1)
    pidctrl_y = PIDCtrl(2, 0, 0, 1)
    pidctrl_yaw = PIDCtrl(1, 0, 0, 0.5)
    rospy.init_node("robo_navigation")

    sub_odom = rospy.Subscriber("odom", Odometry, callback_odom)
    sub_goal = rospy.Subscriber("base/goal", Pose, callback_calculate_vel)
    
    pub_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    
    rospy.spin()
