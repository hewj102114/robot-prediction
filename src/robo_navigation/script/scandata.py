#!/usr/bin/env python
import rospy 
import roslib
import cv2
import numpy as np
import matplotlib.pyplot as plt 
from sensor_msgs.msg import LaserScan
from numpy import inf
loop=5
thr=0
#front 0.38  edge 0.42
# left-right 0.29 edge 0.33

def callback_scan(data):
    
    global loop
    global thr
    rho=data.ranges
    rho=np.array(rho)
    rho[rho<0.25]=inf
    print 'center dis=%f, edge dis=%f'%(rho[thr+180], rho[thr+150])
    theta=np.arange(0,2*np.pi,np.pi/180)
    #plt.figure()
    
    loop=loop-1
    if loop==0:
	plt.subplot(121,polar=True) 
	plt.plot(theta,rho,lw=2)
	plt.plot(theta[thr:thr+2],rho[thr:thr+2],lw=10,color='red')
	plt.plot(theta[thr+30:thr+32],rho[thr+30:thr+32],lw=10,color='red')
	plt.plot(theta[thr-30+360:thr-28+360],rho[thr-30+360:thr-28+360],lw=10,color='red')
	plt.plot(theta[thr+90:thr+92],rho[thr+90:thr+92],lw=10,color='yellow')
	plt.plot(theta[thr+180:thr+182],rho[thr+180:thr+182],lw=10,color='blue')
	plt.plot(theta[thr+270:thr+272],rho[thr+270:thr+272],lw=10,color='black')
	plt.draw()
	plt.pause(0.000000000001)
	plt.clf()
	loop=10
    



rospy.init_node('scandata')

sublidar = rospy.Subscriber('/scan', LaserScan, callback_scan)
    
  
plt.ion()
plt.show()
rospy.spin()




