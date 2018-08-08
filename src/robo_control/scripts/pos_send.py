#!/usr/bin/python2.7
import rospy
import roslib
from nav_msgs.msg import Odometry
from robo_vision.msg import ArmorInfo
from robo_control.msg import GameInfo
from robo_perception.msg import ObjectList
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import socket
import time
import numpy as np


pos_x = pos_y = vel_x = vel_y = pos_yaw = speed_x = speed_y = 0
remainingHP = bulletCount = 0
target_global_x = target_global_y = target_rel_x = target_rel_y = 0
enemy_num = 0
time_sec = time_nsec = 0

def callback_odom(msg):
    global pos_x, pos_y, speed_x, speed_y, pos_yaw, vel_x, vel_y,time_sec, time_nsec

    time_sec = msg.header.stamp.secs
    time_nsec = msg.header.stamp.nsecs

    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    qua = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    speed_x = msg.twist.twist.linear.x
    speed_y = msg.twist.twist.linear.y
    (roll, pitch, yaw) = euler_from_quaternion(qua)
    pos_yaw = yaw


def callback_gameinfo(game):
    global remainingHP, bulletCount
    remainingHP = game.remainingHP
    bulletCount = game.bulletCount


def callback_target(target):
    global target_global_x, target_global_y, target_rel_x, target_rel_y, enemy_num
    target_global_x = target.object[0].globalpose.position.x
    target_global_y = target.object[0].globalpose.position.y

    target_rel_x = target.object[0].pose.position.x
    target_rel_y = target.object[0].pose.position.y

    enemy_num = np.shape(target.object)[0]



rospy.init_node('pos_socket_send')
subodom = rospy.Subscriber('odom', Odometry, callback_odom)
subtarget = rospy.Subscriber('enemy/target', ObjectList, callback_target)
subgameinfo = rospy.Subscriber('base/game_info', GameInfo, callback_gameinfo)


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = rospy.get_param('~ip_addr_send', '192.168.1.148')
print ip
addr = (ip, 10001)

rate = rospy.Rate(60)
while not rospy.is_shutdown():
    
    data_send = "%f %f %f %f %f %f %f %d %d %d %f %f %d %d" % (
    pos_x, pos_y, pos_yaw, target_global_x, target_global_y, target_rel_x, target_rel_y, remainingHP, bulletCount, enemy_num, vel_x, vel_y,
    time_sec, time_nsec)

    s.sendto(data_send.encode(encoding="utf-8"), addr)
    rate.sleep()
s.close()
