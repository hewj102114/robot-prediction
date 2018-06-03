#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

###############UKF参数选取和系统方程测定方法##############
#参数选取原则  
#首先按照动力学方程建立F方程 速度等于加速度乘时间。 v（i+1） = v（i） + a*dt
#F要和H相乘，H=【vx,vx',vy,vy'】，所以写为下面的形式，两个yaw分别是imu和uwb的观测
#用H来选择有几个输入参数，这里我有三个参数，分别是yaw观测1，yaw观测2，dyaw。  
#dim_x = 4， 表示输入方程有四个输入  
#dim_z = 4 表示观测方程z有4个观测。两个轴，xy，速度和加速度
#v_std_x, a_std_x, 表示速度和加速度的测量误差，这里根据测量结果填写
#Q噪声项，var这里我发现用0.2的结果比较好，目前还不知道为什么  
#P里头分别填上前面两轴速度加速度能达到的最大值。  【vx,vx',vy,vy'】
#ukf.x = np.array([0., 0., 0., 0.])表示起始位置和起始速度都是0
# MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=-1.0) 4表示输入参数有三个yaw，yaw，dyaw  
#######################################################
#数据融合方法说明：
#when in short distance mode using pnp as the main prediction source, if pnp lost target, use realsense to predict.
#when in long distance mode, using the realsense to predict, and use a state machine to dicede all possible situation.


#注意事项：开车前保持车辆静止
#################比赛场地坐标系定义###########################
#                  X+
#                  1
#                  0
#                  0
#                  0
#                  0
#                  0
#                  0
# Y+  <-000000000000

# Check list
# KALMAN_GAIN是不是对应上去了

import rospy 
import roslib
import pickle
import math
import tf
import tf2_ros
import time
from numpy.random import randn
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from robo_perception.msg import ObjectList
from robo_perception.msg import Object
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

T_DELAY = 0.1
PREDICT_INIT = True
PNP_INIT = True
LONG_RANGE = False
ENABLE_PREDICT = True
LAST_AVAILABLE = False
DATA_AVAILABLE = False
DEMO = True
BULLET_SPEED = 17

ukf_result = []
robo_vel_x = robo_vel_y = 0
pnp_vel_x = pnp_vel_y = pnp_pos_x = pnp_pos_y = 0
ukf_yaw = ukf_pos_x = ukf_pos_y = 0
last_pnp_pos_x = 0
last_pnp_pos_y = 0
# S（i+1） = S（i） + V*dt
def f_cv(x, dt):
    """ state transition function for a 
    constant velocity aircraft"""
    
    F = np.array([[1, dt, 0,  0],
                  [0,  1, 0,  0],
                  [0,  0, 1, dt],
                  [0,  0, 0,  1]], dtype=float)
    return np.dot(F, x)

def h_cv(x):
    return np.array([x[0], x[1], x[2], x[3]])


def UKFinit(in_dt, init_x):
    global ukf

    p_std_x, p_std_y = 0.03, 0.03
    v_std_x, v_std_y = 0.03, 0.03
    dt = in_dt #50HZ


    sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=-1.0)
    ukf = UKF(dim_x=4, dim_z=4, fx=f_cv, hx=h_cv, dt=dt, points=sigmas)
    ukf.x = init_x
    ukf.R = np.diag([p_std_x, v_std_x, p_std_y, v_std_y]) 
    ukf.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf.P = np.diag([8, 2.0 ,5, 2.0])


def callback_enemy(enemy):
    global last_enemy_num, PREDICT_INIT, robo_vel_x, robo_vel_y, enemy_num, team_num, tfBuffer, enemy_object_trans, team_object_trans
    global ukf_result, ukf, max_move_distance, aim_target_x, aim_target_y, found_aim, aim_lost, enemy_last_time
    global enemy_0_x, enemy_0_y, enemy_1_x, enemy_1_x
    if PREDICT_INIT == True:
        print "realsense callback init Finished!"
        enemy_last_time = enemy.header.stamp.secs + enemy.header.stamp.nsecs * 10**-9
        last_enemy_num = 0
        max_move_distance = 0.5
        lost_time = []
        enemy_num = 0
        team_object_trans = []
        enemy_object_trans = []
        PREDICT_INIT = False
        found_aim = False
        aim_lost = False
    else:
        team_object_trans = []
        enemy_object_trans = []
        enemy_num = 0
        team_num = 0
        enemy_time = enemy.header.stamp.secs + enemy.header.stamp.nsecs * 10**-9
        for i in range(int(enemy.num)):
            object_name =  enemy.object[i].team.data
            if object_name == 'red0' or object_name == 'red1':
                enemy_num = enemy_num + 1
                try:
                    enemy_object_trans.append(tfBuffer.lookup_transform('odom', object_name, rospy.Time(0)))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print('ENEMY OBJ TRANS FAIL! set enemy number 0')
                    enemy_num = 0
                #print enemy_object_trans
            elif object_name == 'blue0':
                team_num = team_num + 1
                try:
                    team_object_trans.append(tfBuffer.lookup_transform('odom', object_name, rospy.Time(0)))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print('TEAM OBJ TRANS FAIL! set team number 0')
                    team_num = 0

        #object judgement
        if enemy_num == 1:
            enemy_0_x = enemy_object_trans[0].transform.translation.x
            enemy_0_y = enemy_object_trans[0].transform.translation.y
            enemy_1_x = 99
            enemy_1_y = 99
        elif enemy_num == 2:
            enemy_0_x = enemy_object_trans[0].transform.translation.x
            enemy_0_y = enemy_object_trans[0].transform.translation.y
            enemy_1_x = enemy_object_trans[1].transform.translation.x
            enemy_1_y = enemy_object_trans[1].transform.translation.y
        else:
            enemy_0_x = 0
            enemy_0_y = 0
            enemy_1_x = 0
            enemy_1_y = 0
        
        #temperate set the aim target to the first detected obj.
        aim_target_x = enemy_0_x
        aim_target_y = enemy_0_y


        dt = enemy_time - enemy_last_time  
        if LONG_RANGE == True:
            if enemy_num == 0 and last_enemy_num == 0:
                found_aim = False
                if aim_lost == True or lost_counter < 15:
                    print 'Lost OBJ'
                    lost_counter = lost_counter + 1 
                    lost_time.append[dt_0]
                else:
                    print 'NO ENEMY!'
                    lost_counter = 0
            elif enemy_num == 1 and last_enemy_num == 0:
                print 'Found enemy!!'
                euclid_distance_0 = np.sqrt((enemy_0_x - aim_target_x) ** 2 + (enemy_0_y - aim_target_y) ** 2)
                if aim_lost == True:
                    if euclid_distance_0 > 0.5:
                        print 'Find an enemy, but not the targted one.'
                        found_aim = False
                        aim_lost = True
                    else:
                        found_aim = True
                        aim_lost = False
                        dt = np.sum(lost_time)
                        enemy_vel_x = (enemy_0_x - last_enemy_0_x) / dt
                        enemy_vel_y = (enemy_0_y - last_enemy_0_y) / dt
                        ukf_input = [enemy_0_x, enemy_0_vel_x, enemy_0_y, enemy_0_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)
                else:
                    if euclid_distance_0 > 0.5:
                        print 'Find an enemy, but not the targted one.'
                        found_aim = False
                        aim_lost = False
                    else:
                        found_aim = True
                        aim_lost = False
                        ukf_input = [enemy_0_x, 0, enemy_0_y, 0]
                        ukf.predict()
                        ukf.update(ukf_input)
                    print 'not detect the targeted one'
            elif enemy_num == 2 and last_enemy_num == 0:
                print 'Found enemy!!'
                euclid_distance_0 = np.sqrt((enemy_0_x - aim_target_x) ** 2 + (enemy_0_y - aim_target_y) ** 2)
                euclid_distance_1 = np.sqrt((enemy_1_x - aim_target_x) ** 2 + (enemy_1_y - aim_target_y) ** 2)
                if euclid_distance_0 > 0.5 and euclid_distance_1 > 0.5:
                    print 'Diveration too large, somthing wrong, check all, unable to predict!!!!'
                    found_aim = False
                else:
                    found_aim = True
                    if euclid_distance_1 < euclid_distance_0:
                        dt_0 = np.sum(lost_time_0)
                        enemy_vel_x = (enemy_0_x - last_enemy_0_x) / dt
                        enemy_vel_y = (enemy_0_y - last_enemy_0_y) / dt
                        ukf_input = [enemy_0_x, enemy_0_vel_x, enemy_0_y, enemy_0_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)
                    else:
                        enemy_vel_x = (last_enemy_1_x - enemy_1_x) / dt
                        enemy_vel_y = (last_enemy_1_y - enemy_1_y) / dt
                        ukf_input = [enemy_1_x, enemy_vel_x, enemy_1_y, enemy_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)
            elif enemy_num == 0 and last_enemy_num != 0:
                print 'Found enemy!!'
                if found_aim == True:
                    found_aim = False
                    aim_lost = True
                    lost_counter = lost_counter + 1 
                    lost_time.append[dt]
            elif enemy_num == 1 and last_enemy_num == 2:
                print 'Found enemy!!'
                euclid_distance_0 = np.sqrt((enemy_0_x - aim_target_x) ** 2 + (enemy_0_y - aim_target_y) ** 2)
                if euclid_distance_0 > 0.5:
                    print 'Diveration too large, aming object may not appear in realsense camera, lost object'
                    found_aim = False
                    aim_lost = True
                    lost_counter = lost_counter + 1 
                    lost_time.append[dt]
                    ukf.predict()                
                else:
                    found_aim = True
                    dt_0 = np.sum(lost_time_0)
                    enemy_vel_x = (enemy_0_x - last_enemy_0_x) / dt
                    enemy_vel_y = (enemy_0_y - last_enemy_0_y) / dt
                    ukf_input = [enemy_0_x, enemy_0_vel_x, enemy_0_y, enemy_0_vel_y]
                    ukf.predict()
                    ukf.update(ukf_input)
            elif enemy_num == 2 and last_enemy_num == 2:
                print 'Found enemy!!'
                euclid_distance_0 = np.sqrt((enemy_0_x - aim_target_x) ** 2 + (enemy_0_y - aim_target_y) ** 2)
                euclid_distance_1 = np.sqrt((enemy_1_x - aim_target_x) ** 2 + (enemy_1_y - aim_target_y) ** 2)
                found_aim = True
                if euclid_distance_1 < euclid_distance_0:
                    dt_0 = np.sum(lost_time_0)
                    enemy_vel_x = (enemy_0_x - last_enemy_0_x) / dt
                    enemy_vel_y = (enemy_0_y - last_enemy_0_y) / dt
                    ukf_input = [enemy_0_x, enemy_0_vel_x, enemy_0_y, enemy_0_vel_y]
                    ukf.predict()
                    ukf.update(ukf_input)
                else:
                    enemy_vel_x = (last_enemy_1_x - enemy_1_x) / dt
                    enemy_vel_y = (last_enemy_1_y - enemy_1_y) / dt
                    ukf_input = [enemy_1_x, enemy_vel_x, enemy_1_y, enemy_vel_y]
                    ukf.predict()
                    ukf.update(ukf_input)
            elif enemy_num == 1 and last_enemy_num == 1:
                euclid_distance_0 = np.sqrt((enemy_0_x - aim_target_x) ** 2 + (enemy_0_y - aim_target_y) ** 2)
                if found_aim == True:
                    if euclid_distance_0 > 0.5:
                        found_aim = False
                        aim_lost = True
                        lost_counter = lost_counter + 1 
                        lost_time.append[dt]
                        ukf.predict()   
                    else:
                        found_aim = True
                        enemy_vel_x = (enemy_0_x - last_enemy_0_x) / dt
                        enemy_vel_y = (enemy_0_y - last_enemy_0_y) / dt
                        ukf_input = [enemy_0_x, enemy_0_vel_x, enemy_0_y, enemy_0_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)
                else:
                    if euclid_distance_0 > 0.5:
                        found_aim = False
                        aim_lost = True
                        lost_counter = lost_counter + 1 
                        lost_time.append[dt]
                        ukf.predict()   
                    else:
                        found_aim = True
                        aim_lost = False
                        dt = np.sum(lost_time)
                        enemy_vel_x = (enemy_0_x - last_enemy_0_x) / dt
                        enemy_vel_y = (enemy_0_y - last_enemy_0_y) / dt
                        ukf_input = [enemy_0_x, enemy_0_vel_x, enemy_0_y, enemy_0_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)

            elif enemy_num == 2 and last_enemy_num == 1:
                euclid_distance_0 = np.sqrt((enemy_0_x - aim_target_x) ** 2 + (enemy_0_y - aim_target_y) ** 2)
                euclid_distance_1 = np.sqrt((enemy_1_x - aim_target_x) ** 2 + (enemy_1_y - aim_target_y) ** 2)
                if found_aim == True:
                    found_aim = True
                    aim_lost = False 
                    if euclid_distance_1 < euclid_distance_0:
                        enemy_vel_x = (enemy_0_x - last_enemy_0_x) / dt
                        enemy_vel_y = (enemy_0_y - last_enemy_0_y) / dt
                        ukf_input = [enemy_0_x, enemy_0_vel_x, enemy_0_y, enemy_0_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)
                    else:
                        enemy_vel_x = (last_enemy_1_x - enemy_1_x) / dt
                        enemy_vel_y = (last_enemy_1_y - enemy_1_y) / dt
                        ukf_input = [enemy_1_x, enemy_vel_x, enemy_1_y, enemy_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)
                else:
                    euclid_distance_0 = np.sqrt((enemy_0_x - aim_target_x) ** 2 + (enemy_0_y - aim_target_y) ** 2)
                    euclid_distance_1 = np.sqrt((enemy_1_x - aim_target_x) ** 2 + (enemy_1_y - aim_target_y) ** 2)
                    found_aim = True
                    aim_lost = False
                    if euclid_distance_1 < euclid_distance_0:
                        dt = np.sum(lost_time)
                        enemy_vel_x = (enemy_0_x - last_enemy_0_x) / dt
                        enemy_vel_y = (enemy_0_y - last_enemy_0_y) / dt
                        ukf_input = [enemy_0_x, enemy_0_vel_x, enemy_0_y, enemy_0_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)
                    else:
                        dt = np.sum(lost_time)
                        enemy_vel_x = (last_enemy_1_x - enemy_1_x) / dt
                        enemy_vel_y = (last_enemy_1_y - enemy_1_y) / dt
                        ukf_input = [enemy_1_x, enemy_vel_x, enemy_1_y, enemy_vel_y]
                        ukf.predict()
                        ukf.update(ukf_input)
                        
            ukf_out_pos_x = ukf.x[0]
            ukf_out_vel_y = ukf.y[1]
            ukf_out_pos_x = ukf.x[2]
            ukf_out_vel_y = ukf.y[3]

            ukf_vel = Odometry()
            ukf_vel.header.frame_id = "ukf_vel"
            ukf_vel.header.stamp.secs = imu.header.stamp.secs
            ukf_vel.header.stamp.nsecs = imu.header.stamp.nsecs
            ukf_vel.twist.twist.linear.x = ukf_out_vel_x
            ukf_vel.twist.twist.linear.y = ukf_out_vel_y
            pub_ukf_vel.publish(ukf_vel)
            
        last_enemy_num = enemy_num
        enemy_last_time = enemy_time




def callback_pnp(pnp):
    global pnp_pos_x, pnp_pos_y, aim_target_x, aim_target_y, enemy_num, pnp_lost_counter, last_pnp_pos_x, last_pnp_pos_y, pnp_vel_x, pnp_vel_y
    global pnp_last_time, enemy_0_x, enemy_0_y, enemy_1_x, enemy_1_y, LAST_AVAILABLE, pnp_lost_time, tfBuffer, PNP_INIT, pnp_trans
    global DATA_AVAILABLE
    if PNP_INIT == True:
        print "pnp callback init Finished!"
        pnp_lost_counter = 0
        pnp_last_time = 0
        pnp_lost_time = []
        pnp_trans = TransformStamped()
        PNP_INIT = False   
    else:
        FIND_PNP = False
        pnp_time = pnp.header.stamp.secs + pnp.header.stamp.nsecs * 10**-9
        dt = pnp_time - pnp_last_time
      
        if pnp.pose.position.x != 0 and pnp.pose.position.y != 0:
            FIND_PNP = True
            try:
                pnp_trans = tfBuffer.lookup_transform('odom', 'enemy_pnp_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('PNP TF TRANS TRANS FAIL! check your code')
            #print pnp_trans
            pnp_pos_x = pnp_trans.transform.translation.x
            pnp_pos_y = pnp_trans.transform.translation.y
        else:
            FIND_PNP = False

        if  last_pnp_pos_x - pnp_pos_x == 0:
            LAST_AVAILABLE = False
        
        if FIND_PNP == True:
            #print 'USE PNP PREDICTING'
            pnp_lost_time = []
            pnp_lost_counter = 0
            if LAST_AVAILABLE:
                
                pnp_vel_x = (pnp_pos_x - last_pnp_pos_x) / dt
                pnp_vel_y = (pnp_pos_y - last_pnp_pos_y) / dt
                DATA_AVAILABLE = True
            
            LAST_AVAILABLE = True
            last_pnp_pos_x = pnp_pos_x
            last_pnp_pos_y = pnp_pos_y
         
        elif enemy_num == 2:
            euclid_distance_0 = np.sqrt((enemy_0_x - last_pnp_pos_x) ** 2 + (enemy_0_y - last_pnp_pos_y) ** 2)
            euclid_distance_1 = np.sqrt((enemy_1_x - last_pnp_pos_x) ** 2 + (enemy_1_y - last_pnp_pos_x) ** 2)
            if euclid_distance_0 < euclid_distance_1 and euclid_distance_0 == 99:
                #print 'PNP lost use realsense to compensate, find Two Available obj USE 0'
                pnp_lost_counter = 0
                pnp_lost_time = []
                if LAST_AVAILABLE:
                    pnp_pos_x = enemy_0_x
                    pnp_pos_y = enemy_0_y
                    pnp_vel_x = (last_pnp_pos_x - pnp_pos_x) / dt
                    pnp_vel_y = (last_pnp_pos_y - pnp_pos_y) / dt
                    DATA_AVAILABLE = True

                
                LAST_AVAILABLE = True
                last_pnp_pos_x = pnp_pos_x
                last_pnp_pos_y = pnp_pos_y
            elif euclid_distance_1 < euclid_distance_0 and euclid_distance_1 == 99:
                pnp_lost_counter = 0
                pnp_lost_time = []
                #print 'PNP lost use realsense to compensate, find Two Available obj USE 1'
                if LAST_AVAILABLE:
                    pnp_pos_x = enemy_1_x
                    pnp_pos_y = enemy_1_y
                    pnp_vel_x = (last_pnp_pos_x - pnp_pos_x) / dt
                    pnp_vel_y = (last_pnp_pos_y - pnp_pos_y) / dt
                    DATA_AVAILABLE = True
                    
                LAST_AVAILABLE = True
                last_pnp_pos_x = pnp_pos_x
                last_pnp_pos_y = pnp_pos_y
            else:
                #print 'NO Available update in both realsense and gimble camera', 'lost_counter:', pnp_lost_counter
                pnp_lost_counter = pnp_lost_counter + 1
                pnp_lost_time.append(dt)
                LAST_AVAILABLE = False
                DATA_AVAILABLE = False

                
        elif enemy_num == 1:
            pnp_lost_counter = 0
            euclid_distance_0 = np.sqrt((enemy_0_x - last_pnp_pos_x) ** 2 + (enemy_0_y - last_pnp_pos_y) ** 2)
            #print 'realsense',enemy_0_x,enemy_0_y
            #print 'last',last_pnp_pos_x,last_pnp_pos_y
            #print euclid_distance_0
            if euclid_distance_0 == 0:
                pnp_lost_counter = 0
                pnp_lost_time = []
                print 'PNP lost, use realsense to compensate, find one Available obj'
                if LAST_AVAILABLE:
                    pnp_pos_x = enemy_0_x
                    pnp_pos_y = enemy_0_y
                    pnp_vel_x = (last_pnp_pos_x - pnp_pos_x) / dt
                    pnp_vel_y = (last_pnp_pos_y - pnp_pos_y) / dt
                    DATA_AVAILABLE = True
                
                LAST_AVAILABLE = True
                last_pnp_pos_x = pnp_pos_x
                last_pnp_pos_y = pnp_pos_y
            else:
                #print 'NO Available update in both realsense and gimble camera', 'lost_counter:', pnp_lost_counter
                pnp_lost_counter = pnp_lost_counter + 1
                pnp_lost_time.append(dt)
                LAST_AVAILABLE = False
                DATA_AVAILABLE = False

                
        elif enemy_num == 0:
            #print 'NO Available update in both realsense and gimble camera', 'lost_counter:', pnp_lost_counter
            pnp_lost_counter = pnp_lost_counter + 1
            pnp_lost_time.append(dt)
            LAST_AVAILABLE = False
            DATA_AVAILABLE = False

            

        pnp_last_time = pnp_time
    
#TODO see weather to combine self speed. or leave it to other code    
def callback_ukf(ukf):
    global ukf_yaw, ukf_pos_x, ukf_pos_y
    #only yaw are available
    ukf_pos_x = ukf.pose.pose.position.x
    ukf_pos_y = ukf.pose.pose.position.y
    qn_ukf = [ukf.pose.pose.orientation.x, ukf.pose.pose.orientation.y, ukf.pose.pose.orientation.z, ukf.pose.pose.orientation.w]
    (ukf_roll,ukf_pitch,ukf_yaw) = euler_from_quaternion(qn_ukf)

def TFinit():
    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


rospy.init_node('ukf_predict_node')
UKFinit(0.02,np.array([0., 0., 0., 0.]) )
TFinit()
subenemy = rospy.Subscriber('infrared_detection/enemy_position', ObjectList, callback_enemy)
subpnp = rospy.Subscriber('base/armor_pose', PoseStamped, callback_pnp)
subukf = rospy.Subscriber('ukf/pos', Odometry, callback_ukf)

pub_ukf_vel = rospy.Publisher('ukf/enemy', Odometry, queue_size=1)

rate = rospy.Rate(50) # 10hz
while not rospy.is_shutdown():
    global BULLET_SPEED, DEMO, pnp_pos_x, pnp_vel_x, pnp_pos_y, pnp_vel_y 
    

    if ENABLE_PREDICT and DATA_AVAILABLE and DEMO:
        np.array([pnp_pos_x, pnp_vel_x, pnp_pos_y, pnp_vel_y])
        DEMO = False

    if DATA_AVAILABLE:
        ukf_input = [pnp_pos_x, pnp_vel_x, pnp_pos_y, pnp_vel_y]
        ukf.predict()
        ukf.update(ukf_input)
            
        predict_pos = Odometry()
        predict_pos.header.frame_id = "predict"
        predict_pos.header.stamp = rospy.Time.now()
        ukf_out_pos_x = ukf.x[0]
        ukf_out_pos_y = ukf.x[2]
        ukf_out_vel_x = ukf.x[1]
        ukf_out_vel_y = ukf.x[3]
        predict_pos.pose.pose.position.x = ukf_out_pos_x
        predict_pos.pose.pose.position.y = ukf_out_pos_y
        predict_pos.twist.twist.linear.x = ukf_out_vel_x
        predict_pos.twist.twist.linear.y = ukf_out_vel_y
        
        
        #print 'PNP','X',ukf_input[0],'Y',ukf_input[2]
        print 'PNP','VX',ukf_input[1],'VY',ukf_input[3]
        print 'KALMAN', 'VX',ukf.x[1],'VY', ukf.x[3]
        #DATA_AVAILABLE = True
        predict_pos.pose.pose.orientation.x = 1
        
    else:
        ukf.predict()
        predict_pos = Odometry()
        predict_pos.header.frame_id = "predict"
        predict_pos.header.stamp = rospy.Time.now()
        ukf_out_pos_x = ukf.x[0]
        ukf_out_pos_y = ukf.x[2]
        ukf_out_vel_x = ukf.x[1]
        ukf_out_vel_y = ukf.x[3]
        predict_pos.pose.pose.position.x = 0
        predict_pos.pose.pose.position.y = 0
        predict_pos.twist.twist.linear.x = 0
        predict_pos.twist.twist.linear.y = 0
        
        #DATA_AVAILABLE = False
        predict_pos.pose.pose.orientation.x = 0
        
        
    #   #
    #           #
    #                 #
    #                        #
    #                               #
    #                                     #
    ########################################
    
    # distance_to_enemy
    # Bullet flying time = 17m/s
                
    V_verticle = ukf_out_vel_x * np.cos(ukf_yaw) + ukf_out_vel_y * np.sin(ukf_yaw)
    #print V_verticle, ukf_yaw
    #print 'cos',ukf_out_vel_x * np.cos(ukf_yaw),'sin',ukf_out_vel_y * np.sin(ukf_yaw)
    distance_to_enemy = np.sqrt((ukf_out_pos_x - ukf_pos_x)**2 +(ukf_out_pos_y - ukf_pos_y)**2)
    T_FLY = distance_to_enemy / BULLET_SPEED
    predict_angle = np.arctan(V_verticle * (T_DELAY + T_FLY) / distance_to_enemy)
    
    predict_pos.pose.pose.orientation.y = 0
    predict_pos.pose.pose.orientation.z = 0
    predict_pos.pose.pose.orientation.w = predict_angle

    pub_ukf_vel.publish(predict_pos) 

    rate.sleep()