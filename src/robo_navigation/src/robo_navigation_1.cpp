#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "robo_navigation/global_planner.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/LaserScan.h"
#include "robo_navigation/PID.h"
#include "robo_perception/ObjectList.h"
#include "robo_control/TeamInfo.h"
using namespace std;
#define OFFSET 0 //rplidar front offset
#define DEFFENCE 0.40
#define DEFF_CORNER 0.60

int local_adj=1; //0 for close local adjustment, 1 for open.
int update_path=0; //0 for don't update, 1 for update
int GO_CENTER_S = 1; //0 direct go center; 1: using one other point
int center_flag = 0;   //0 for have not go center, 1 have been center
int first_go=0;
int read_lidar=0;
class RoboNav
{
  public:
    ros::NodeHandle *pnh;
    ros::Publisher pub_local_goal_pose;
    Floyd floyd;
    Mat arrArcs, point_list;
    vector<int> path;
    geometry_msgs::Pose cur_pose,team_pose;
    geometry_msgs::Pose pre_goal, cur_goal;
    double fix_angle;
    PIDctrl pid_x;
    PIDctrl pid_y;
    PIDctrl pid_yaw;
    tf::TransformListener *tf_;
    std_msgs::Bool state;
    robo_perception::ObjectList enemy_information;
    /*obs_point record the valid obs point in four directions
     * for rplidar (360 degree and 360 points), point steps by one degree
     *  obs_point[0][]  -----forward  [0][0]=20
     *  obs_point[1][]  -----left
     *  obs_point[2][]  -----backward
     *  obs_point[3][]  -----right
     *  
     */
    double obs_min[4][2]; //90,+-60
    int dx_flag, dy_flag, dyaw_flag;
    int selection;
    int lidar_state;
    RoboNav(ros::NodeHandle* _pnh);
    void init();
    void cb_tar_pose(const geometry_msgs::Pose &msg);
    void cb_cur_pose(const nav_msgs::Odometry &msg);
    void cb_teaminfo(const robo_control::TeamInfo &msg);
    int findClosestPt(double x, double y);
    void path_plan(geometry_msgs::Pose &target);
    void get_vel(geometry_msgs::Twist &msg_vel);
    void lidar_nav(geometry_msgs::Twist &msg_vel);
    void setFixAngle(const geometry_msgs::Quaternion &qua);
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &scan);
    void cb_enemy_infor(const robo_perception::ObjectList &msg);
    void cb_first_point(const std_msgs::Int32 &msg);
    int go_center();
    geometry_msgs::Pose adjustlocalgoal(double yaw);
};

RoboNav::RoboNav(ros::NodeHandle* _pnh)
{
    obs_min[4][2] = {0};
    selection=999;
    lidar_state=0;
    pnh = _pnh;
    pub_local_goal_pose = pnh->advertise<geometry_msgs::PoseStamped>("nav/local_goal", 1);
    tf_ = new tf::TransformListener();
}

void RoboNav::init()
{
    FileStorage fs("/home/ubuntu/robot/src/robo_navigation/script/matrix.xml", FileStorage::READ);
    fs["Matrix"] >> arrArcs;
    fs["Point"] >> point_list;
    floyd.loadMatrix(arrArcs);
    floyd.initFloydGraph();
    path.clear();

    state.data = false;
    dx_flag = 0;
    dy_flag = 0;
    dyaw_flag=0;
    double Kp_linear, Ki_linear, Kd_linear;
    double limit_linear_max = 1.0;
    double Kp_angular, Ki_angular, Kd_angular;
    double limit_angular = 1.5;
    ros::param::param<double>("Kp_linear", Kp_linear, 1.5);
    ros::param::param<double>("Ki_linear", Ki_linear, 0);
    ros::param::param<double>("Kd_linear", Kd_linear, 0);
    ros::param::param<double>("Kp_angular", Kp_angular, 2.0);
    ros::param::param<double>("Ki_angular", Ki_angular, 0);
    ros::param::param<double>("Kd_angular", Kd_angular, 0);

    pid_x.init(Kp_linear, Ki_linear, Kd_linear, limit_linear_max);
    pid_y.init(Kp_linear, Ki_linear, Kd_linear, limit_linear_max);
    pid_yaw.init(Kp_angular, Ki_angular, Kd_angular, limit_angular);

    cur_pose.position.x = 0;
    cur_pose.position.y = 0;
    cur_pose.position.y = 0;
    cur_pose.orientation.x = 0;
    cur_pose.orientation.y = 0;
    cur_pose.orientation.z = 0;
    cur_pose.orientation.w = 1;
}

void RoboNav::path_plan(geometry_msgs::Pose &target)
{
    int start_pt = findClosestPt(cur_pose.position.y, cur_pose.position.x);
    int end_pt = findClosestPt(target.position.y, target.position.x);
    if (start_pt != end_pt)
    {
        floyd.calcPath(start_pt, end_pt);
        //ROS_INFO("start %f  %f  end   %f  %f ", cur_pose.position.x, cur_pose.position.y, target.position.x, target.position.y);
        ROS_INFO("GET GOAL start: %d  end: %d", start_pt, end_pt);
        floyd.printPath();
        path.assign(floyd.path.begin(), floyd.path.end());
    }
    else
        path.clear();

    if (path.size() > 1)
    {
        int first_index = path[0], second_index = path[1];
        double first_dis = sqrt(pow(point_list.at<double>(first_index, 0) * 1.0 / 100 - cur_pose.position.y, 2) + pow(point_list.at<double>(first_index, 1) * 1.0 / 100 - cur_pose.position.x, 2));
        double second_dis = sqrt(pow(point_list.at<double>(second_index, 0) * 1.0 / 100 - cur_pose.position.y, 2) + pow(point_list.at<double>(second_index, 1) * 1.0 / 100 - cur_pose.position.x, 2));
        double pairwise_dis = sqrt(pow(point_list.at<double>(second_index, 0) - point_list.at<double>(first_index, 0), 2) + pow(point_list.at<double>(second_index, 1) - point_list.at<double>(first_index, 1), 2)) * 1.0 / 100;
        if (first_dis + second_dis < 1.2 * pairwise_dis)
        {
            path.erase(path.begin());
            ROS_INFO("First point erased!!!!!!!!!!!!!!!!");
        }
    }
}

void RoboNav::cb_tar_pose(const geometry_msgs::Pose &msg)
{
    cur_goal = msg;
    if (cur_pose.position.y == 0 & cur_pose.position.x == 0)
        return;
    if ((abs(pre_goal.position.x - msg.position.x) > 0.1) || (abs(pre_goal.position.y - msg.position.y) > 0.1))
    {
        //init the neighor matrix 
        floyd.loadMatrix(arrArcs);
        floyd.initFloydGraph();
        path_plan(cur_goal);
        pre_goal.position.x = msg.position.x;
        pre_goal.position.y = msg.position.y;
        dyaw_flag = 0; //new goal, rotation first.
    }
    //isolated rotation control
    //setFixAngle(cur_pose.orientation);
    setFixAngle(msg.orientation); //orientation fixed all the time in every frame.
}

double calyaw(double set_yaw, double cur_yaw)
{
    double dyaw = 0;
    if (set_yaw > 0 && cur_yaw < 0 && (set_yaw - cur_yaw) > 3.14)
    {
        dyaw = -(cur_yaw + 6.28 - set_yaw);
    }
    else if (set_yaw < 0 && cur_yaw > 0 && (cur_yaw - set_yaw) > 3.14)
    {
        dyaw = set_yaw + 6.28 - cur_yaw;
    }
    else
    {
        dyaw = set_yaw - cur_yaw;
    }
    return dyaw;
}

void RoboNav::cb_cur_pose(const nav_msgs::Odometry &msg)
{
    cur_pose = msg.pose.pose;
    
    double dis = sqrt(pow(cur_pose.position.x - cur_goal.position.x, 2) + pow(cur_pose.position.y - cur_goal.position.y, 2));
    double dyaw = abs(calyaw(fix_angle, tf::getYaw(cur_pose.orientation)));  
    // if (dis < 0.5 && dyaw < 0.05)
    // {
    //     state.data = true;
    //     center_flag = 1; //first time use
    // }
    // else
    //     state.data = false;
}


void RoboNav::cb_teaminfo(const robo_control::TeamInfo &msg)
{
    team_pose = msg.pose;
    ROS_INFO("Team Pose, x: %f, y: %f",team_pose.position.x, team_pose.position.y);
    if (path.size() > 0)
    {
        double pt_x = team_pose.position.x;
        double pt_y = team_pose.position.y;
        vector<float> dis_list;
        for (int i = 0; i < point_list.rows; i++)
        {
            float dx = pt_y - point_list.at<double>(i, 0) * 1.0 / 100;
            float dy = pt_x - point_list.at<double>(i, 1) * 1.0 / 100;
            float distance = sqrt(dx * dx + dy * dy);
            dis_list.push_back(distance);
        }

        vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());
        int nest = -1, near = -1;
        nest = distance(dis_list.begin(), smallest);
        double small_dis = *smallest;
        ROS_INFO("Team robot on %d point", nest);
        //too close to a point, all the path related to this point should be invalid
        if (*smallest < 0.5)
        {
            if (nest == path.back())
            {
                ROS_INFO("Team Robot on the target goal");
                path.pop_back();
            }
            //if the occupied point is on the point, update
            else
            {
                floyd.loadMatrix(arrArcs);
                ROS_INFO("Team Robot on the path");
                for (int i = 0; i < path.size() - 1; i++)
                {
                    for (int i = 0; i < point_list.rows; i++)
                    {
                        if (i != nest)
                            floyd.updateFloydGraph(i, nest, 100);
                    }
                    floyd.initFloydGraph();
                    path_plan(cur_goal);
                    break;
                }
            }
        }
        // else //not too close to a point, find the invalid path
        // {
        //     dis_list.erase(smallest);
        //     vector<float>::iterator smallestK = min_element(dis_list.begin(), dis_list.end());
        //     near = distance(dis_list.begin(), smallestK);
        //     double pairwise_dis = sqrt(pow(point_list.at<double>(nest, 0) - point_list.at<double>(near, 0), 2) + pow(point_list.at<double>(nest, 1) - point_list.at<double>(near, 1), 2)) * 1.0 / 100;
        //     floyd.loadMatrix(arrArcs);
        //     if (small_dis + *smallestK < pairwise_dis * 1.2&&path.size()>2)
        //         //if the occupied point is on the point, update
        //         for (int i = 1; i < path.size() - 1; i++)
        //         {
        //             if (nest == path[i] && (path[i - 1] == near || path[i + 1] == near))
        //             {
                        
        //                 floyd.updateFloydGraph(near, nest, 100);
        //                 floyd.initFloydGraph();
        //                 path_plan(cur_goal);
        //                 break;
        //             }
        //         }
        // }
    }
}

// obstable avoidance
void RoboNav::cb_enemy_infor(const robo_perception::ObjectList &msg)
{
    enemy_information = msg;

    //update path
    int index = -1, i = 0, j = 0;
    while (index < enemy_information.num&&update_path)
    {
        //string::size_type idx;
        //idx = enemy_information.object[index].team.data.find("death");
        //if (idx != string::npos)
        {

            double pt_x = enemy_information.object[index].globalpose.position.x;
            double pt_y = enemy_information.object[index].globalpose.position.y;
            vector<float> dis_list;
            for (int i = 0; i < point_list.rows; i++)
            {
                float dx = pt_y - point_list.at<double>(i, 0) * 1.0 / 100;
                float dy = pt_x - point_list.at<double>(i, 1) * 1.0 / 100;
                float distance = sqrt(dx * dx + dy * dy);
                dis_list.push_back(distance);
            }

            vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());
            int n = distance(dis_list.begin(), smallest);
            double small_dis = *smallest;
            ROS_INFO("center: %f", dis_list.back());

            if (center_flag == 0 && dis_list.back() < 0.25)
            {
                ROS_INFO("Enemy on bonus zone, Attack!!!");
                if (path.size() > 0)
                    path.erase(path.begin());
                state.data = true;
            }
            //too close to a point, all the path related to this point should be invalid
            if (*smallest < 0.25)
            {

                int i = 0;
                while (i != n)
                {
                    floyd.updateFloydGraph(i, n, 100);
                    i++;
                }
            }
            else //not too close to a point, find the invalid path
            {

                dis_list.erase(smallest);
                vector<float>::iterator smallestK = min_element(dis_list.begin(), dis_list.end());
                int m = distance(dis_list.begin(), smallestK);
                double pairwise_dis = sqrt(pow(point_list.at<double>(n, 0) - point_list.at<double>(m, 0), 2) + pow(point_list.at<double>(n, 1) - point_list.at<double>(m, 1), 2)) * 1.0 / 100;
                if (small_dis + *smallestK < pairwise_dis * 1.2)
                    floyd.updateFloydGraph(i, n, 10000);
            }
        }
        index++;

         if (index == enemy_information.num - 1)
         {
             floyd.initFloydGraph();
             path_plan(cur_goal);
         }
    }
}

int RoboNav::findClosestPt(double x, double y)
{
    vector<float> dis_list;
    //cout<<point_list<<endl;
    for (int i = 0; i < point_list.rows; i++)
    {
        float dx = x - point_list.at<double>(i, 0) * 1.0 / 100;
        float dy = y - point_list.at<double>(i, 1) * 1.0 / 100;
        float distance = sqrt(dx * dx + dy * dy);
        dis_list.push_back(distance);
    }

    vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());

    int n = distance(dis_list.begin(), smallest);
    return n;
}

void RoboNav::get_vel(geometry_msgs::Twist &msg_vel)
{

    double vel_x = 0;
    double vel_y = 0;
    double vel_yaw = 0;
    double cur_yaw = tf::getYaw(cur_pose.orientation);

    //yaw control
    double dyaw = calyaw(fix_angle, cur_yaw);
    //ROS_INFO("cur_yaw: %f, fixed_yaw: %f, dyaw %f ", cur_yaw,fix_angle, dyaw);

    //if (dyaw_flag == 0)
    {
        vel_yaw = pid_yaw.calc(dyaw);
        if (abs(dyaw) < 0.05)
        {
            vel_yaw = 0;
            dyaw_flag = 1;
            dx_flag=0;
            dy_flag=0;
        }
    }
    ROS_INFO(" cur_x %f , cur_y %f", cur_pose.position.x, cur_pose.position.y);

    if (path.size() > 0 && dyaw_flag)
    {
        //obstacle
        vel_yaw=0.0;
        geometry_msgs::Pose cur_local_goal = adjustlocalgoal(cur_yaw);
        double cur_local_goal_y = cur_local_goal.position.y;
        double cur_local_goal_x = cur_local_goal.position.x;

        double dx = (cur_local_goal_x - cur_pose.position.x) * cos(cur_yaw) + (cur_local_goal_y - cur_pose.position.y) * sin(cur_yaw);
        double dy = -(cur_local_goal_x - cur_pose.position.x) * sin(cur_yaw) + (cur_local_goal_y - cur_pose.position.y) * cos(cur_yaw);

  
        if (abs(dx) < 0.10)
            dx_flag = 1;
        if (abs(dy) < 0.10)
            dy_flag = 1;
        if (dx_flag && abs(dy) < 0.10)
        {
            dy_flag = 1;
        }

        if (dy_flag && abs(dx) < 0.10)
        {
            dx_flag = 1;
        }
        ROS_INFO("dx_flag: %d  dy_flag : %d",dx_flag,dy_flag);
        if (dx_flag == 1 && dy_flag == 1)
        {
            path.erase(path.begin());
            dx_flag = 0;
            dy_flag = 0;
            vel_x=0;
            vel_y=0;    //if enter, it could stop. Test!!!!   if not, find which one is not 1...
        }
        else
        {
            vel_x = pid_x.calc(dx);
            vel_y = pid_y.calc(dy);
        }
        
        if (GO_CENTER_S == 1)
            if (center_flag == 0 && (path[0] == 34||path[0] == 13) && dx > 1.8)
            {
                vel_y = 0;
            }
        if (GO_CENTER_S == 0)
        {
            ROS_INFO("dx: %f, dy: %f", dx, dy);
            if (center_flag == 0 && path[0] == 27 && dx > 0.8)
                vel_y = 0;
            
        }
    }
    else
    {
        vel_x=0;
        vel_y=0;
    }
    ROS_INFO("vel_x: %f, vel_y: %f，vel_yaw: %f", vel_x, vel_y,vel_yaw);

    msg_vel.linear.x = vel_x;
    msg_vel.linear.y = vel_y;
    msg_vel.angular.z = vel_yaw;
}

void RoboNav::setFixAngle(const geometry_msgs::Quaternion &qua)
{

    fix_angle = tf::getYaw(qua);
}

double Filter_ScanData(int index, const sensor_msgs::LaserScan::ConstPtr &sscan)
{
    int Cindex = index;
    double data = 8;
    int m = 0;
    for (int i = -15; i < 15; i++)
    {
        if (index + i < 0)
            Cindex = 360 + index + i;
        else if (index + i >= 360)
            Cindex = index + i - 360;
        else
            Cindex = index + i;
        if (sscan->ranges[Cindex] > 0.25 && sscan->ranges[Cindex] < data)
        {
            data = sscan->ranges[Cindex];
        }
    }

    return data;
}

void RoboNav::cb_scan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    for (int i = 0; i < 4; i++)
    {
        int index = OFFSET + i * 90;
        int indexC = OFFSET + i * 90 + 45;
        obs_min[i][0] = Filter_ScanData(index, scan);
        obs_min[i][1] = Filter_ScanData(indexC, scan);

    }
    read_lidar=1;

    ROS_INFO("obs_min[0][0]: %f",obs_min[0][0]);

}

void RoboNav::lidar_nav(geometry_msgs::Twist &msg_vel)
{
    double vel_x = 0;
    double vel_y = 0;
    double cur_yaw = tf::getYaw(cur_pose.orientation);
    ROS_INFO("lidar state: %d",lidar_state);
    switch(lidar_state)
    {
    case 0:

        vel_x=0.7;
        vel_y=0.0;
        if(obs_min[0][0]<DEFF_CORNER)

        {
            lidar_state=1;
        }
        break;
    case 1:
        vel_x=0.0;
        vel_y=0.7;
        if(obs_min[0][0]>1.0)

        {
            lidar_state=2;
        }
        break;
    case 2:
        vel_x=0.0;
        vel_y=0.0;

        state.data=true;
        break;
    default:
        break;
    }

    msg_vel.linear.x = vel_x;
    msg_vel.linear.y = vel_y;
    msg_vel.angular.z = 0;
}

geometry_msgs::Pose RoboNav::adjustlocalgoal(double yaw)
{
    geometry_msgs::Pose local_goal;
    int local_goal_index = path[0];
    double local_goal_y = point_list.at<double>(local_goal_index, 0) * 1.0 / 100;
    double local_goal_x = point_list.at<double>(local_goal_index, 1) * 1.0 / 100;
    local_goal.position.y = point_list.at<double>(local_goal_index, 0) * 1.0 / 100;
    local_goal.position.x = point_list.at<double>(local_goal_index, 1) * 1.0 / 100;
    if (local_adj)   //open or close local adjustment
    {
        double dis_x = abs(local_goal_x - cur_pose.position.x);
        double dis_y = abs(local_goal_y - cur_pose.position.y);

        pid_x.stop = false;
        pid_y.stop = false;

        if (obs_min[0][0] < 0.34) //front
            pid_y.stop = true;
        else if (obs_min[0][0] < DEFFENCE)
        {
            local_goal.position.x = local_goal_x - 0.1 * cos(yaw);
            local_goal.position.y = local_goal_y - 0.1 * sin(yaw);
        }

        if (obs_min[0][1] < 0.45) //front-left
        {
            if (dis_x > dis_y)
                pid_x.stop = true;
            if (dis_x < dis_y)
                pid_y.stop = true;
        }
        else if (obs_min[0][1] < DEFF_CORNER)
        {
            local_goal.position.x = local_goal_x + 0.1 * (sin(yaw) - cos(yaw));
            local_goal.position.y = local_goal_y - 0.1 * (cos(yaw) + sin(yaw));
        }

        if (obs_min[1][0] < 0.28)
            pid_x.stop = true;
        else if (obs_min[1][0] < DEFFENCE) //left
        {
            local_goal.position.x = local_goal_x + 0.1 * sin(yaw);
            local_goal.position.y = local_goal_y - 0.1 * cos(yaw);
        }

        if (obs_min[1][1] < 0.45) //left-back
        {
            if (dis_x > dis_y)
                pid_x.stop = true;
            if (dis_x < dis_y)
                pid_y.stop = true;
        }
        else if (obs_min[1][1] < DEFF_CORNER)
        {
            local_goal.position.x = local_goal_x + 0.1 * (cos(yaw) + sin(yaw));
            local_goal.position.y = local_goal_y + 0.1 * (sin(yaw) - cos(yaw));
        }

        if (obs_min[2][0] < 0.34)
            pid_y.stop = true;
        else if (obs_min[2][0] < DEFFENCE) //back
        {
            local_goal.position.x = local_goal_x + 0.1 * cos(yaw);
            local_goal.position.y = local_goal_y + 0.1 * sin(yaw);
        }

        if (obs_min[2][1] < 0.45) //back-right
        {
            if (dis_x > dis_y)
                pid_x.stop = true;
            if (dis_x < dis_y)
                pid_y.stop = true;
        }
        else if (obs_min[2][1] < DEFF_CORNER)
        {
            local_goal.position.x = local_goal_x + 0.1 * (cos(yaw) - sin(yaw));
            local_goal.position.y = local_goal_y + 0.1 * (sin(yaw) + cos(yaw));
        }

        if (obs_min[3][0] < 0.28)
            pid_x.stop = true;
        else if (obs_min[3][0] < DEFFENCE)
        {
            local_goal.position.x = local_goal_x - 0.1 * sin(yaw);
            local_goal.position.y = local_goal_y + 0.1 * cos(yaw);
        }

        if (obs_min[3][1] < 0.45) //right-front
        {
            if (dis_x > dis_y)
                pid_x.stop = true;
            if (dis_x < dis_y)
                pid_y.stop = true;
        }
        else if (obs_min[3][1] < DEFF_CORNER)
        {
            local_goal.position.x = local_goal_x - 0.1 * (cos(yaw) + sin(yaw));
            local_goal.position.y = local_goal_y + 0.1 * (cos(yaw) - sin(yaw));
        }
    }

    if (center_flag == 0)
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }
    //ROS_INFO("stop x: %d, y: %d", pid_x.stop, pid_y.stop);
    return local_goal;
}

// zhongdian: 3-34; (4.0, 2.5) GO_CENTER_S=1  +34
// point 1:  3-7; (2.6, 3.15)  GO_CENTER_S=1  no
//point 3: 3-13; (4.00,3.8)   GO_CENTER_S=1   +13
//point 2(robot 2): 9-8; (2.6,2.25)  GO_CENTER_S=1
//point 4(robot 2): 9-8-28; (4.7,1.8)  GO_CENTER_S=0   +28
//cation:  get velecity, change the last control point [34], [13], 

void RoboNav::cb_first_point(const std_msgs::Int32 &msg)
{
    selection=msg.data;
}
int RoboNav::go_center()
{    
    ROS_INFO("select %d",selection);
    switch (selection)
    {
    case 0:  //7
        //192.168.1.150
        first_go=1;
        cur_goal.position.x = 2.6;
        cur_goal.position.y = 2.25;

        cur_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        GO_CENTER_S = 1;
        {
            path.push_back(9);
            path.push_back(8);
        }
        break;

    case 1:   //8

        //192.168.1.148
        first_go=1;
        cur_goal.position.x = 2.6;
        cur_goal.position.y = 3.15;
        cur_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        GO_CENTER_S = 1;
        {
            path.push_back(3);
            path.push_back(7);
            //path.push_back(28);
        }
        break;
    case 2:   //13
        first_go=1;
        cur_goal.position.x = 4.0;
        cur_goal.position.y = 3.8;
        cur_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        GO_CENTER_S = 1;
        {
            path.push_back(3);
            path.push_back(13);
            //path.push_back(28);
        }
        break;
    case 3:  //34
        first_go=1;
        cur_goal.position.x = 4.0;
        cur_goal.position.y = 2.5;
        cur_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        GO_CENTER_S = 1;
        {
            path.push_back(3);
            path.push_back(34);

        }
        break;
    case 4: //27
        first_go=1;
        cur_goal.position.x = 4.7;
        cur_goal.position.y = 0.6;
        cur_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        GO_CENTER_S = 0;
        {
            path.push_back(9);
            path.push_back(8);
            path.push_back(27);
        }
        break;
    default:
        break;
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_navigation");
    ros::NodeHandle nh;

    RoboNav robo_nav(&nh);
    robo_nav.init();

    

    ros::Subscriber cb_tar_pose = nh.subscribe("base/goal", 1, &RoboNav::cb_tar_pose, &robo_nav);
    ros::Subscriber cb_cur_pose = nh.subscribe("odom", 1, &RoboNav::cb_cur_pose, &robo_nav);
    ros::Subscriber cb__teaminfo = nh.subscribe("team/info", 1, &RoboNav::cb_teaminfo, &robo_nav);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &RoboNav::cb_scan, &robo_nav);
    ros::Subscriber sub_enemy_info = nh.subscribe("infrared_detection/enemy_position", 1, &RoboNav::cb_enemy_infor, &robo_nav);
    ros::Subscriber cb_start_selection = nh.subscribe("base/first_point", 1, &RoboNav::cb_first_point, &robo_nav);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher pub_state = nh.advertise<std_msgs::Bool>("nav_state", 1);
    ros::Publisher pub_front = nh.advertise<std_msgs::Float64>("front_dis", 1);
    ros::Rate rate(80);

    int delay=0;
    
    while (ros::ok())
    {
        delay++;
        geometry_msgs::Twist msg_vel;
        if (read_lidar)
        {
            

            if (robo_nav.lidar_state < 2) //main control cannot get the end state signal

            {
                robo_nav.lidar_nav(msg_vel);
            }


            if (robo_nav.lidar_state == 2)
            {
                //robo_nav.get_vel(msg_vel);

                robo_nav.state.data=true;
            }

            //ROS_INFO("vel x: %f y:%f", msg_vel.linear.x, msg_vel.linear.y);
            if (robo_nav.path.size() > 0)
            {
                for (int i = 0; i < robo_nav.path.size() - 1; i++)
                {
                    cout << robo_nav.path[i] << " - > ";
                }
                cout << robo_nav.path.back() << endl;
            }
        }
        pub_vel.publish(msg_vel);   
        pub_state.publish(robo_nav.state);
        std_msgs::Float64 front_distance;
        front_distance.data = robo_nav.obs_min[0][0];
        pub_front.publish(front_distance);

        ros::spinOnce();
        rate.sleep();
    }
}
