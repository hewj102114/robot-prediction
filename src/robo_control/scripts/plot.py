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
import matplotlib.pyplot as plt
import cPickle as pickle

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Twist, Point
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from robo_control.msg import TeamInfo


def callback_odom(msg):
    pass

def callback_team_info(msg):
    # global min_time_team_info
    global team_info_dict

    time_team_info_secs = msg.header.stamp.secs
    time_team_info_nsecs = msg.header.stamp.nsecs
    time_team_info = float(rospy.Time(time_team_info_secs, time_team_info_nsecs).to_sec() - min_time_team_info)

    vel_x_team_info_enemy = float(msg.twist.linear.x)
    vel_y_team_info_enemy = float(msg.twist.linear.y)
    # min_time_team_info = time_team_info if time_team_info < min_time_team_info else min_time_team_info
    # print("min time time info: {}".format(min_time_team_info))
    
    
    print("time info time: {}, time info vel_x: {}, time info vel_y: {}".format(time_team_info, vel_x_team_info_enemy, vel_y_team_info_enemy))
    team_info_dict[time_team_info] = [vel_x_team_info_enemy, vel_y_team_info_enemy]
    with open('./team_info.txt','wb') as f:
        pickle.dump(team_info_dict, f)
    # plt.scatter(time_team_info, vel_x_team_info_enemy, c='g')
    # plt.scatter(time_team_info, vel_y_team_info_enemy, c='m')
    # plt.draw()
    # plt.pause(0.000000000001)

def callback_ukf_enemy(msg):
    # global min_time
    global ukf_dict

    time_ukf_secs = msg.header.stamp.secs
    time_ukf_nsecs = msg.header.stamp.nsecs
    time_ukf = float(rospy.Time(time_ukf_secs, time_ukf_nsecs).to_sec() - min_time_ukf)
    vel_x_ukf_enemy = float(msg.twist.twist.linear.x)
    vel_y_ukf_enemy = float(msg.twist.twist.linear.y)
    # min_time = time_ukf if time_ukf < min_time else min_time
    # print("min time: {}".format(min_time))
    print("time: {}, vel_x: {}, vel_y: {}".format(time_ukf, vel_x_ukf_enemy, vel_y_ukf_enemy))
    ukf_dict[time_ukf] = [vel_x_ukf_enemy, vel_y_ukf_enemy]
    with open('./ukf.txt','wb') as f:
        pickle.dump(ukf_dict, f)
    # plt.scatter(time_ukf, vel_x_ukf_enemy, c='r')
    # plt.scatter(time_ukf, vel_y_ukf_enemy, c='b')
    
    # plt.draw()
    # plt.pause(0.000000000001)

if __name__ == '__main__':
    rospy.init_node("robo_plot")
    min_time_ukf = 1533699673.23
    min_time_team_info = 1533699672.26

    team_info_dict = {}
    ukf_dict = {}

    sub_odom = rospy.Subscriber("/odom", Odometry, callback_odom)
    sub_goal = rospy.Subscriber("/team/info", TeamInfo, callback_team_info)
    sub_goal = rospy.Subscriber("/ukf/enemy", Odometry, callback_ukf_enemy)

    # plt.ion()
    # plt.show()
    rospy.spin()
