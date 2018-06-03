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
from scipy.signal import butter, lfilter, freqz
from collections import deque

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
from robo_control.msg import GameInfo

T_PNP_DELAY = 0.06 #系统延时系数，单位秒
T_RS_DELAY = 0.06
RS_INIT = True
PNP_INIT = True

ENABLE_PREDICT = True
PNP_LAST_AVAILABLE = False
PNP_DATA_AVAILABLE = False
PNP_LAST_VEL = False
RS_LAST_AVAILABLE = False
RS_DATA_AVAILABLE = False
RS_PREDICT_INIT = True
PNP_PREDICT_INIT = True
UNABLE_PREDICT = 0
TEMPERAL_LOST = 0
RS_PREDICT_INIT = True
PNP_PREDICT_INIT = True
TARGET_RECETIVED = False
PNP_UKF_AVAILABLE = False
RS_UKF_AVAILABLE = False
MAX_PREDICT_ANGLE = 6 

BULLET_SPEED = 16.1
PNP_CLOSE_THRESH = 10 #判断pnp是否瞄准到了正确目标
RS_CLOSE_THRESH = 10 #判断rs是否瞄准到了正确目标
LOST_TRESH = 10

#LOW PASS FILTER
LPF_ORDER = 6
PNP_LPS_SAMPLING_FREQ = 100.0       # sample rate, Hz
PNP_LPF_CUTOFF = 4  # desired cutoff frequency of the filter, Hz

RS_LPS_SAMPLING_FREQ = 100.0       # sample rate, Hz
RS_LPF_CUTOFF = 8  # desired cutoff frequency of the filter, Hz

lpf_input_list = deque([0,0,0,0,0],maxlen = 6)

ukf_result = []
robo_vel_x = robo_vel_y = 0
pnp_vel_x = pnp_vel_y = pnp_pos_x = pnp_pos_y = last_pnp_pos_x = last_pnp_pos_y = last_pnp_vel_x = last_pnp_vel_y = 0
odom_yaw = odom_pos_x = odom_pos_y = odom_vel_x = odom_vel_y = 0
rs_pos_x = rs_pos_y = rs_vel_x = rs_vel_y = last_rs_pos_x = last_rs_pos_y = 0
gimbal_yaw = gimbal_dtheta = 0
aimtheta  = predict_angle = 0
ukf_out_pos_x = ukf_out_pos_y = ukf_out_vel_x = ukf_out_vel_y = 0
pnp_lost_counter = rs_lost_counter = 0
rs_lost_time = []
pnp_lost_time = []
aim_target_x = aim_target_y = 0

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# S（i+1） = S（i） + V*dt
def f_cv(x, dt):
    """ state transition function for a 
    constant velocity aircraft"""
    
    F = np.array([[1, dt, 0,  0],
                  [0,  1, 0,  0],
                  [0,  0, 1, dt],
                  [0,  0, 0,  1]], dtype=float)
    return np.dot(F, x)

def f_cv_9x9(x, dt):
    """ state transition function for a 
    constant velocity aircraft"""
    
    F = np.array([[1, dt, 0.5*dt*dt, 0,  0,          0],
                  [0,  1, dt,        0,  0,          0],
                  [0,  0, 1,         0,  0,          0],
                  [0,  0, 0,         1, dt,  0.5*dt*dt],
                  [0,  0, 0,         0,  1,         dt],
                  [0,  0, 0,         0,  0,          1]], dtype=float)
    return np.dot(F, x)

def h_cv(x):
    return np.array([x[0], x[1], x[2], x[3]])

def h_cv_6(x):
    return np.array([x[0], x[1], x[2], x[3], x[4], x[5]])


def UKFRsInit(in_dt, init_x):
    global ukf_rs

    p_std_x, p_std_y = 0.01, 0.01
    v_std_x, v_std_y = 0.01, 0.01
    dt = in_dt 


    sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=-1.0)
    ukf_rs = UKF(dim_x=4, dim_z=4, fx=f_cv, hx=h_cv, dt=dt, points=sigmas)
    ukf_rs.x = init_x
    ukf_rs.R = np.diag([p_std_x, v_std_x, p_std_y, v_std_y]) 
    ukf_rs.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf_rs.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf_rs.P = np.diag([8, 1.5 ,5, 1.5])

def UKFPnpInit(in_dt, init_x):
    global ukf_pnp

    p_std_x, p_std_y = 0.4, 0.4
    v_std_x, v_std_y = 1, 1
    a_std_x, a_std_y = 30, 30
    dt = in_dt 


    sigmas = MerweScaledSigmaPoints(6, alpha=.1, beta=2., kappa=-1.0)
    ukf_pnp = UKF(dim_x=6, dim_z=6, fx=f_cv_9x9, hx=h_cv_6, dt=dt, points=sigmas)
    ukf_pnp.x = init_x
    ukf_pnp.R = np.diag([p_std_x, v_std_x, a_std_x, p_std_y, v_std_y, a_std_y]) 
    ukf_pnp.Q[0:3, 0:3] = Q_discrete_white_noise(3, dt=dt, var=2)
    ukf_pnp.Q[3:6, 3:6] = Q_discrete_white_noise(3, dt=dt, var=2)
    ukf_pnp.P = np.diag([8, 1.5 ,0.2 ,5, 1.5, 0.2])

def TFinit():
    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

def callback_enemy(enemy):
    global enemy_num, team_num, tfBuffer, enemy_object_trans, team_object_trans, aim_target_x, aim_target_y, rs_last_time, rs_lost_time, rs_lost_counter
    global rs_pos_x, rs_pos_y, rs_vel_x, rs_vel_y, last_rs_pos_x, last_rs_pos_y, odom_yaw, odom_pos_x, odom_pos_y
    global ENABLE_PREDICT, RS_DATA_AVAILABLE, RS_LAST_AVAILABLE, RS_INIT,RS_PREDICT_INIT, ukf_rs,RS_UKF_AVAILABLE
    global ukf_rs_pos_x,ukf_rs_pos_y, ukf_rs_vel_x,ukf_rs_vel_y
    if RS_INIT == True:
        print "realsense callback init Finished!"
        rs_last_time = enemy.header.stamp.secs + enemy.header.stamp.nsecs * 10**-9
        enemy_num = 0
        team_object_trans = []
        enemy_object_trans = []
        rs_lost_time = []
        rs_lost_counter = 0
        RS_INIT = False
    
    elif ENABLE_PREDICT:
        team_object_trans = []
        enemy_object_trans = []
        enemy_num = 0
        team_num = 0
        enemy_0_x = enemy_0_y = enemy_1_x = enemy_1_y = 999
        FIND_RS = False
        rs_time = enemy.header.stamp.secs + enemy.header.stamp.nsecs * 10**-9
        dt = rs_time - rs_last_time
        if 1/dt<30:
            print 'WARNING!!!,detection frequency to low! Graph card may need COOLING operation!'
        if enemy.num == 0:
            print 'NOTHING'
        for i in range(int(enemy.num)):
            object_name =  enemy.object[i].team.data
            #decoding the enemy position
            if object_name == 'red0' or object_name == 'red1':
                enemy_num = enemy_num + 1
                rs_x = enemy.object[i].pose.position.x
                rs_y = enemy.object[i].pose.position.y
                #theta in rs axis
                theta = np.arctan2(rs_y,rs_x)
                #global = cos(theta+yaw) * target_distance + rs_relative_distance_to_base_link + base_link_global_axis
                #0.22 means rs_relative_distance_to_base_link = 22CM
                rs_global_x = np.cos(theta + odom_yaw)*np.sqrt(rs_x**2 +rs_y**2) + 0.22*np.cos(odom_yaw) + odom_pos_x
                rs_global_y = np.sin(theta + odom_yaw)*np.sqrt(rs_x**2 +rs_y**2) + 0.22*np.sin(odom_yaw) + odom_pos_y                

                #print rs_global_x,rs_global_y,'odom_yaw',odom_yaw,'theta',theta,'odom_pos_x',odom_pos_x,'odom_pos_y',odom_pos_y

                enemy_object_trans.append([rs_global_x,rs_global_y])

            elif object_name == 'blue0':
                team_num = team_num + 1
                #DO NOTHING
                
        #object judgement
        if enemy_num == 1:
            enemy_0_x = enemy_object_trans[0][0]
            enemy_0_y = enemy_object_trans[0][1]
            enemy_1_x = 999
            enemy_1_y = 999
        elif enemy_num == 2:
            enemy_0_x = enemy_object_trans[0][0]
            enemy_0_y = enemy_object_trans[0][1]
            enemy_1_x = enemy_object_trans[1][0]
            enemy_1_y = enemy_object_trans[1][1]
        else:
            enemy_0_x = 999
            enemy_0_y = 999
            enemy_1_x = 999
            enemy_1_y = 999

        if np.sqrt((aim_target_x - enemy_0_x) ** 2 + (aim_target_y - enemy_0_y) ** 2) < RS_CLOSE_THRESH:
            rs_pos_x = enemy_0_x
            rs_pos_y = enemy_0_y
            FIND_RS = True
        elif np.sqrt((aim_target_x - enemy_1_x) ** 2 + (aim_target_y - enemy_1_y) ** 2) < RS_CLOSE_THRESH:
            rs_pos_x = enemy_1_x
            rs_pos_y = enemy_1_y
            FIND_RS = True
        else:
            # no available data, then not last data.
            FIND_RS = False
            RS_LAST_AVAILABLE = False

            
        if FIND_RS == True:
            #print 'USE RS PREDICTING'
            rs_lost_time = []
            rs_lost_counter = 0
            if RS_LAST_AVAILABLE:
                #last data available, use to update the speed
                rs_vel_x = (rs_pos_x - last_rs_pos_x) / dt
                rs_vel_y = (rs_pos_y - last_rs_pos_y) / dt
                RS_DATA_AVAILABLE = True
            else:
                RS_DATA_AVAILABLE = False
                rs_lost_counter = rs_lost_counter + 1

            
            RS_LAST_AVAILABLE = True
            last_rs_pos_x = rs_pos_x
            last_rs_pos_y = rs_pos_y


        else:
            #print 'NO Available update in realsense ', 'lost_counter:', rs_lost_counter
            rs_lost_counter = rs_lost_counter + 1
            rs_lost_time.append(dt)
            RS_LAST_AVAILABLE = False
            RS_DATA_AVAILABLE = False

        rs_last_time = rs_time
        RS_UKF_AVAILABLE = False
        #rs有数据就用rs的进行更新，第一次直接初始化，第二次再更新   
        if RS_DATA_AVAILABLE and RS_PREDICT_INIT:
            UKFRsInit(0.033, np.array([rs_pos_x, rs_vel_x, rs_pos_y, rs_vel_y]))
            ukf_rs_pos_x = ukf_rs.x[0]
            ukf_rs_vel_x = ukf_rs.x[1]
            ukf_rs_pos_y = ukf_rs.x[2]
            ukf_rs_vel_y = ukf_rs.x[3]
            RS_PREDICT_INIT = False
        elif RS_DATA_AVAILABLE:
            rs_ukf_input = [rs_pos_x, rs_vel_x, rs_pos_y, rs_vel_y]
            ukf_rs.predict()
            ukf_rs.update(rs_ukf_input)
            ukf_rs_pos_x = ukf_rs.x[0]
            ukf_rs_vel_x = ukf_rs.x[1]
            ukf_rs_pos_y = ukf_rs.x[2]
            ukf_rs_vel_y = ukf_rs.x[3]
            RS_UKF_AVAILABLE = True
        #第一次直接初始化，第二次再更新
        if RS_DATA_AVAILABLE == False:
            RS_PREDICT_INIT = True 

        
        #print 'RS_LAST_AVAILABLE',RS_LAST_AVAILABLE,'RS_DATA_AVAILABLE',RS_DATA_AVAILABLE,rs_lost_counter

        

def callback_pnp(pnp):
    global pnp_pos_x, pnp_pos_y, aim_target_x, aim_target_y, pnp_lost_counter, last_pnp_pos_x, last_pnp_pos_y, pnp_vel_x, pnp_vel_y
    global pnp_last_time, pnp_lost_time, tfBuffer, pnp_trans, ukf_pnp
    global PNP_DATA_AVAILABLE, PNP_LAST_AVAILABLE, PNP_INIT, ENABLE_PREDICT, PNP_PREDICT_INIT,PNP_UKF_AVAILABLE
    global ukf_pnp_pos_x,ukf_pnp_pos_y, ukf_pnp_vel_x,ukf_pnp_vel_y,PNP_LAST_VEL,last_pnp_vel_x,last_pnp_vel_y

    if PNP_INIT == True:
        print "pnp callback init Finished!"
        pnp_lost_counter = 0
        pnp_last_time = 0
        pnp_lost_time = []
        pnp_trans = TransformStamped()
        PNP_INIT = False  
    elif ENABLE_PREDICT:
        FIND_PNP = False
        pnp_time = pnp.header.stamp.secs + pnp.header.stamp.nsecs * 10**-9
        dt = pnp_time - pnp_last_time
        #print 'dt',dt


        #print pnp.pose.position.x,pnp.pose.position.y,pnp.pose.position.z
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

        # wrong target judgement   
        if np.sqrt((aim_target_x - pnp_pos_x) ** 2 + (aim_target_y - pnp_pos_y) ** 2) > PNP_CLOSE_THRESH:
            FIND_PNP = False
            PNP_LAST_AVAILABLE = False
            PNP_LAST_VEL = False
            

        if FIND_PNP == True:
            #print 'USE PNP PREDICTING'
            pnp_lost_time = []
            pnp_lost_counter = 0
            if PNP_LAST_AVAILABLE:
                #last data available, use to update the speed
                pnp_vel_x = (pnp_pos_x - last_pnp_pos_x) / dt
                pnp_vel_y = (pnp_pos_y - last_pnp_pos_y) / dt


                if PNP_LAST_VEL:
                    pnp_acc_x = (pnp_vel_x - last_pnp_vel_x)/dt
                    pnp_acc_y = (pnp_vel_y - last_pnp_vel_y)/dt
                    #print pnp_vel_x,last_pnp_vel_x,pnp_acc_x, pnp_acc_y
                    #if pnp_acc_x>1.2 or pnp_acc_y >1.2:
                    #     PNP_DATA_AVAILABLE = False
                    # else:
                    PNP_DATA_AVAILABLE = True
                    
                last_pnp_vel_x = pnp_vel_x
                last_pnp_vel_y = pnp_vel_y
                PNP_LAST_VEL = True
                #print 'x',pnp_pos_x,last_pnp_pos_x,pnp_vel_x,'y',pnp_pos_y,last_pnp_pos_y,pnp_vel_y
            else:
                PNP_DATA_AVAILABLE = False
                pnp_lost_counter = pnp_lost_counter + 1
            PNP_LAST_AVAILABLE = True
            last_pnp_pos_x = pnp_pos_x
            last_pnp_pos_y = pnp_pos_y

        else:
            #print 'NO Available update in both realsense and gimble camera', 'lost_counter:', pnp_lost_counter
            pnp_lost_counter = pnp_lost_counter + 1
            pnp_lost_time.append(dt)
            PNP_LAST_AVAILABLE = False
            PNP_LAST_VEL = False
            PNP_DATA_AVAILABLE = False
        #print 'PNP_DATA_AVAILABLE',PNP_DATA_AVAILABLE,'PNP_LAST_AVAILABLE',PNP_LAST_AVAILABLE
        pnp_last_time = pnp_time

        PNP_UKF_AVAILABLE = False
        if PNP_PREDICT_INIT and PNP_DATA_AVAILABLE:
            UKFPnpInit(0.01, np.array([pnp_pos_x, pnp_vel_x,pnp_acc_x, pnp_pos_y, pnp_vel_y, pnp_acc_y]))
            ukf_pnp_pos_x = ukf_pnp.x[0]
            ukf_pnp_vel_x = ukf_pnp.x[1]
            ukf_pnp_pos_y = ukf_pnp.x[3]
            ukf_pnp_vel_y = ukf_pnp.x[4]  
            PNP_PREDICT_INIT = False
        elif PNP_DATA_AVAILABLE:
            pnp_ukf_input = [pnp_pos_x, pnp_vel_x, pnp_acc_x, pnp_pos_y, pnp_vel_y, pnp_acc_y]
            ukf_pnp.predict()
            ukf_pnp.update(pnp_ukf_input)
            ukf_pnp_pos_x = ukf_pnp.x[0]
            ukf_pnp_vel_x = ukf_pnp.x[1]
            ukf_pnp_pos_y = ukf_pnp.x[3]
            ukf_pnp_vel_y = ukf_pnp.x[4]

            #print 'PNP','X',pnp_pos_x,'Y',pnp_pos_y, 'VX',pnp_vel_x,'VY',pnp_vel_x,'PDA',PNP_DATA_AVAILABLE
            PNP_UKF_AVAILABLE = True

        if PNP_DATA_AVAILABLE == False:
            PNP_PREDICT_INIT = True 
        
         
        
                
#TODO see weather to combine self speed. or leave it to other code    
def callback_odom(odom):
    global odom_yaw, odom_pos_x, odom_pos_y, odom_vel_x, odom_vel_y
    #only yaw are available
    odom_pos_x = odom.pose.pose.position.x
    odom_pos_y = odom.pose.pose.position.y

    odom_vel_x = odom.twist.twist.linear.x
    odom_vel_y = odom.twist.twist.linear.y

    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)


def callback_target(target):
    global aim_target_x, aim_target_y, ENABLE_PREDICT, aimtheta,tfBuffer,gimbal_yaw,odom_yaw,gimbal_dtheta,TARGET_RECETIVED
    aim_target_x = target.pose.pose.position.x
    aim_target_y = target.pose.pose.position.y
    TARGET_RECETIVED = False
    if aim_target_x and aim_target_y != 0:
        TARGET_RECETIVED = True
        rs_trans = TransformStamped()
        try:
            rs_trans = tfBuffer.lookup_transform('odom', 'realsense_camera', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('realsense TF TRANS TRANS FAIL! check your code')
        
        #use sele as orign of axis
        relative_x = aim_target_x - rs_trans.transform.translation.x
        relative_y = aim_target_y - rs_trans.transform.translation.y
        
        # print rs_trans.transform.translation.x,rs_trans.transform.translation.y
        # print 'aim_target_x',aim_target_x,'aim_target_y',aim_target_y
        # print 'relative',relative_x,relative_y
        #target 
        aimtheta = np.arctan2(relative_y, relative_x)




        ENABLE_PREDICT = target.pose.pose.orientation.w

def callback_gimbal(gimbal):
    global gimbal_yaw, BULLET_SPEED
    gimbal_yaw = gimbal.gimbalAngleYaw*(np.pi/180)
    if gimbal.bulletSpeed > 0:
        BULLET_SPEED = gimbal.bulletSpeed

rospy.init_node('ukf_predict_node')
UKFRsInit(0.033,np.array([0., 0., 0., 0.]))
UKFPnpInit(0.01,np.array([0., 0., 0., 0.,0.,0.]))
TFinit()
subenemy = rospy.Subscriber('infrared_detection/enemy_position', ObjectList, callback_enemy)
subpnp = rospy.Subscriber('base/armor_pose', PoseStamped, callback_pnp)
subodom = rospy.Subscriber('odom', Odometry, callback_odom)
subgimbal = rospy.Subscriber('base/game_info', GameInfo, callback_gimbal)
subtarget = rospy.Subscriber('enemy/target', Odometry, callback_target)

pub_ukf_vel = rospy.Publisher('ukf/enemy', Odometry, queue_size=1)

rate = rospy.Rate(30) # 40hz
while not rospy.is_shutdown():
    global BULLET_SPEED, pnp_pos_x, pnp_pos_y, pnp_vel_y, pnp_vel_x, gimbal_yaw, odom_vel_x, odom_vel_y,odom_yaw
    global UNABLE_PREDICT, aimtheta, predict_angle, ukf_out_pos_x, ukf_out_pos_y, ukf_out_vel_x, ukf_out_vel_y
    global rs_pos_x, rs_pos_y, rs_vel_x, rs_vel_y, last_pnp_pos_x, last_pnp_pos_y, rs_lost_counter, pnp_lost_counter
    global LOST_TRESH, TEMPERAL_LOST,RS_DATA_AVAILABLE, PNP_DATA_AVAILABLE,gimbal_dtheta
    global TARGET_RECETIVED, ukf_rs_pos_x,ukf_rs_pos_y, ukf_rs_vel_x,ukf_rs_vel_y,ukf_pnp_pos_x,ukf_pnp_pos_y, ukf_pnp_vel_x,ukf_pnp_vel_y
    global PNP_UKF_AVAILABLE,RS_UKF_AVAILABLE, lpf_input_list, LPF_CUTOFF, LPS_SAMPLING_FREQ, LPF_ORDER


    # 选择传感器预测优先级
    if RS_UKF_AVAILABLE:
        ukf_out_pos_x = ukf_rs_pos_x  
        ukf_out_vel_x = ukf_rs_vel_x
        ukf_out_pos_y = ukf_rs_pos_y 
        ukf_out_vel_y = ukf_rs_vel_y
        UNABLE_PREDICT = 0
        TEMPERAL_LOST = 0
    elif PNP_UKF_AVAILABLE:  
        ukf_out_pos_x = ukf_pnp_pos_x  
        ukf_out_vel_x = ukf_pnp_vel_x
        ukf_out_pos_y = ukf_pnp_pos_y 
        ukf_out_vel_y = ukf_pnp_vel_y
        UNABLE_PREDICT = 0
        TEMPERAL_LOST = 0
        #print 'predict!'
    else:
        #print 'temperal unable to predict!','pnp_lost_counter',pnp_lost_counter,'rs_lost_counter',rs_lost_counter 
        TEMPERAL_LOST = 1

    if pnp_lost_counter > LOST_TRESH and rs_lost_counter > LOST_TRESH:
        UNABLE_PREDICT = 1
        #print 'UNABLE TO PREDICT!!!!!!!!!!!!'

    #from gimbal yaw to global gimbal_yaw
    global_gimbal_yaw = odom_yaw + gimbal_yaw

    if global_gimbal_yaw < -np.pi:
        global_gimbal_yaw = global_gimbal_yaw + 2 * np.pi
    if global_gimbal_yaw > np.pi:
        global_gimbal_yaw = global_gimbal_yaw - 2 * np.pi

    
    if PNP_UKF_AVAILABLE or RS_UKF_AVAILABLE:
        print 'RS','X',rs_pos_x,'Y',rs_pos_y, 'VX',rs_vel_x,'VY',rs_vel_y,'RDA',RS_DATA_AVAILABLE
        print 'PNP','X',pnp_pos_x,'Y',pnp_pos_y, 'VX',pnp_vel_x,'VY',pnp_vel_x,'PDA',PNP_DATA_AVAILABLE
        print 'KALMAN', 'X',ukf_out_pos_x,'Y',ukf_out_pos_y, 'VX',ukf_out_vel_x,'VY',ukf_out_vel_y

    #   #
    #           #
    #                 #
    #                        #
    #                               #
    #                                     #
    ########################################
    
    # distance_to_enemy
    # Bullet flying time = 17m/s

    if RS_UKF_AVAILABLE:
        #计算相对速度
        relative_speed_x = ukf_out_vel_x - odom_vel_x
        relative_speed_y = ukf_out_vel_y - odom_vel_y
        #print 'relative_speed_x',relative_speed_x,'relative_speed_y',relative_speed_y
        #计算水平于枪口方向的速度,trans to ploe axis then calculate verticle speed
        # target_speed = np.sqrt(relative_speed_x**2 + relative_speed_y**2)
        # target_theta = np.arctan2(relative_speed_y , relative_speed_x)
        # V_verticle = target_speed * np.sin(2 * np.pi - (global_gimbal_yaw + target_theta + 90))
        # print 'target_speed',target_speed, 'target_theta',target_theta,'V_verticle',V_verticle
        V_verticle = relative_speed_x * np.sin(global_gimbal_yaw) + relative_speed_y * np.cos(global_gimbal_yaw)
        #print 'V_verticle',V_verticle,'global_gimbal_yaw',global_gimbal_yaw,'relative_speed_x',relative_speed_x,'relative_speed_y',relative_speed_y
        #print V_verticle, odom_yaw
        #计算检测到的目标和我自身的距离
        distance_to_enemy = np.sqrt((ukf_out_pos_x - (odom_pos_x + 0.22*np.cos(odom_yaw)))**2 +(ukf_out_pos_y - (odom_pos_y+ 0.22*np.cos(odom_yaw)))**2)
        #计算子弹飞行时间
        T_FLY = distance_to_enemy / BULLET_SPEED
        #反解算出需要的预瞄角度
        predict_angle = np.arctan(V_verticle * (T_RS_DELAY + T_FLY) / distance_to_enemy)

        lpf_input_list.append(predict_angle)
        lpf_out_list = butter_lowpass_filter(lpf_input_list, RS_LPF_CUTOFF, RS_LPS_SAMPLING_FREQ, LPF_ORDER)
        predict_angle = lpf_out_list[5]

    elif PNP_UKF_AVAILABLE:
        #计算相对速度
        relative_speed_x = ukf_out_vel_x - odom_vel_x
        relative_speed_y = ukf_out_vel_y - odom_vel_y
        #print 'relative_speed_x',relative_speed_x,'relative_speed_y',relative_speed_y
        #计算水平于枪口方向的速度,trans to ploe axis then calculate verticle speed
        # target_speed = np.sqrt(relative_speed_x**2 + relative_speed_y**2)
        # target_theta = np.arctan2(relative_speed_y , relative_speed_x)
        # V_verticle = target_speed * np.sin(2 * np.pi - (global_gimbal_yaw + target_theta + 90))
        # print 'target_speed',target_speed, 'target_theta',target_theta,'V_verticle',V_verticle
        V_verticle = relative_speed_x * np.sin(global_gimbal_yaw) + relative_speed_y * np.cos(global_gimbal_yaw)
        #print 'V_verticle',V_verticle,'global_gimbal_yaw',global_gimbal_yaw,'relative_speed_x',relative_speed_x,'relative_speed_y',relative_speed_y
        #print V_verticle, odom_yaw
        #计算检测到的目标和我自身的距离
        distance_to_enemy = np.sqrt((ukf_out_pos_x - (odom_pos_x + 0.22*np.cos(odom_yaw)))**2 +(ukf_out_pos_y - (odom_pos_y+ 0.22*np.cos(odom_yaw)))**2)
        #计算子弹飞行时间
        T_FLY = distance_to_enemy / BULLET_SPEED
        #反解算出需要的预瞄角度
        predict_angle = np.arctan(V_verticle * (T_PNP_DELAY + T_FLY) / distance_to_enemy)

        lpf_input_list.append(predict_angle)
        lpf_out_list = butter_lowpass_filter(lpf_input_list, PNP_LPF_CUTOFF, PNP_LPS_SAMPLING_FREQ, LPF_ORDER)
        predict_angle = lpf_out_list[5]

    predict_pos = Odometry()
    predict_pos.header.frame_id = "predict"
    predict_pos.header.stamp = rospy.Time.now()
    predict_pos.pose.pose.position.x = ukf_out_pos_x
    predict_pos.pose.pose.position.y = ukf_out_pos_y

    predict_pos.twist.twist.linear.x = ukf_out_vel_x
    predict_pos.twist.twist.linear.y = ukf_out_vel_y

    predict_pos.twist.twist.linear.z = TEMPERAL_LOST
    
    predict_pos.pose.pose.orientation.x = UNABLE_PREDICT    
    predict_pos.pose.pose.orientation.y = aimtheta
    if TARGET_RECETIVED == True: 
        #dyaw between target and gimbal
        gimbal_dtheta = aimtheta - global_gimbal_yaw 
        # print 'gimbal_dtheta',gimbal_dtheta/np.pi*180,'aimtheta', aimtheta, 'global_gimbal_yaw',global_gimbal_yaw/np.pi*180
        predict_pos.pose.pose.orientation.z = gimbal_dtheta
    else:
        predict_pos.pose.pose.orientation.z = 999 

    #send predict when ukf avaliable
    if PNP_UKF_AVAILABLE or RS_UKF_AVAILABLE:
        predict_pos.pose.pose.orientation.w = -predict_angle/np.pi*180
    elif pnp_lost_counter > LOST_TRESH and rs_lost_counter > LOST_TRESH:
        predict_pos.pose.pose.orientation.w = 0


    #MAX PREDICT ANGLE
    if predict_pos.pose.pose.orientation.w >= MAX_PREDICT_ANGLE:
        predict_pos.pose.pose.orientation.w = MAX_PREDICT_ANGLE
    elif predict_pos.pose.pose.orientation.w <= - MAX_PREDICT_ANGLE:
        predict_pos.pose.pose.orientation.w = -MAX_PREDICT_ANGLE

    #predict_pos.pose.pose.orientation.w = 0

    pub_ukf_vel.publish(predict_pos) 

    rate.sleep()