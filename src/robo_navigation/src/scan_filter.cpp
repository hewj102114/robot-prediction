/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define OFFSET 20    //rplidar front offset

/*obs_point record the valid obs point in four directions
 * for rplidar (360 degree and 360 points), point steps by one degree
 *  obs_point[0][]  -----forward  [0][0]=20
 *  obs_point[1][]  -----left
 *  obs_point[2][]  -----backward
 *  obs_point[3][]  -----right
 *  
 */
double obs_point[4][3]={0}; //90,+-60

double Filter_ScanData(int index, const sensor_msgs::LaserScan::ConstPtr& sscan)
{
    int Cindex=index;
    double data=0;
    int m=0;
        while(1)
        {
            if(index+m<0) Cindex=360+index+m;
            else if(index+m>=360) Cindex=index+m-360;
            else Cindex=index+m;
            if(sscan->intensities[Cindex]==47)
            {
                data=sscan->ranges[Cindex];
                break;
            }

            if(index-m<0) Cindex=360+index-m;
            else if(index-m>=360) Cindex=index-m-360;
            else Cindex=index-m;
            if(sscan->intensities[Cindex]==47)
            {
                data=sscan->ranges[Cindex];
                break;
            }
            m++;
        }

    if(data<0.25)  data=8.0;
    else if(data>8.0)   data=8.0;

    return data;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    /*
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
    */
    for(int i=0;i<4;i++)
    {
        //right
        int index_r=OFFSET+i*90-30;
        obs_point[i][0]=Filter_ScanData(index_r,scan);
        //center
        int index=OFFSET+i*90;
        obs_point[i][1]=Filter_ScanData(index,scan);
        //left
        int index_l=OFFSET+i*90+30;
        obs_point[i][2]=Filter_ScanData(index_l,scan);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}
