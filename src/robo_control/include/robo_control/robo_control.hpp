#pragma once
#include <string>
#include <cmath>
#include <time.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>

#include <actionlib_msgs/GoalStatusArray.h>

#include <opencv2/opencv.hpp>

#include "robo_control/serial.h"
#include "robo_vision/ArmorInfo.h"
#include "robo_vision/FishCamInfo.h"
#include "robo_control/GameInfo.h"
#include "robo_control/TeamInfo.h"
#include "robo_perception/ObjectList.h"
#include "robo_perception/Object.h"


using namespace std;
#define PI 3.1415926535898
#define KYAW 70

//Armor Info Data
struct ArmorInfo
{
    int mode;
    float global_z;
    float pitch;
    float yaw;
};

//Chassis Velocity Data
struct ChassisCMD
{
    float v_x = 0;
    float v_y = 0;
    float v_yaw = 0;
};

struct KeyPoint
{
    int idx;   // index of KeyPoint
    float x;   // x coordinate of keypoint
    float y;   // y coordinate of keypoint
    float yaw; // the direction of keypoint
};

struct GambalInfo
{
    int mode;
    float global_z;
    float pitch;
    float yaw;
};

struct VelInfo
{
    int mode;
    float v_x;
    float v_y;
    float v_yaw;
};

struct PointInfo
{
    int mode;
    float x;
    float y;
};

class RoboControl
{
  public:
    RoboControl()
    {
        pnh = new ros::NodeHandle("");

        pub_game_info = pnh->advertise<robo_control::GameInfo>("base/game_info", 1);
        pub_uwb_odom = pnh->advertise<nav_msgs::Odometry>("map/uwb/data", 1);
        pub_wheel_vel = pnh->advertise<geometry_msgs::Vector3Stamped>("robo/wheel/data", 1);
        pub_imu_data = pnh->advertise<sensor_msgs::Imu>("gimbal/imu/data_raw", 1);
        pub_nav_goal = pnh->advertise<geometry_msgs::Pose>("base/goal", 1);
        pub_first_point = pnh->advertise<std_msgs::Int32>("base/first_point", 1);
        pub_enemy_target = pnh->advertise<robo_perception::ObjectList>("enemy/target", 1);

        serial = new Serial("/dev/ttyUSBA1");
        serial->configurePort();

        pre_uwb_ready_flag = 0;
        enemy_found_flag = 0;
        armor_ready_flag = 0;
        init_flag = 0;
        nav_start_time = ros::Time::now();
    };

    //ROS CallBack Function
    void cb_armorInfo(const robo_vision::ArmorInfo &msg);
    void cb_cmd_vel(const geometry_msgs::Twist &msg);
    void cb_move_base(const move_base_msgs::MoveBaseActionFeedback &msg);
    void cb_enemy_pose(const geometry_msgs::TransformStamped &msg);
    void cb_enemy_pnp_pose(const geometry_msgs::TransformStamped &msg);
    void cb_ukf_pose(const nav_msgs::Odometry &msg);
    void cb_cur_goal(const geometry_msgs::PoseStamped &msg);
    void cb_move_base_status(const actionlib_msgs::GoalStatusArray &msg);
    void cb_another_robo_odom(const nav_msgs::Odometry &msg);
    void cb_front_dis(const std_msgs::Float64 &msg);

    void readMCUData();
    void sendNavGoal(geometry_msgs::Pose &target_pose);
    void sendMCUMsg(int _chassis_mode, int _gimbal_mode, float _velocity_x, float _velocity_y,
                    float _velocity_yaw, float _gimbal_yaw, float _gimbal_pitch,
                    float _enemy_distance);
    bool judgeKeyPointPosition(KeyPoint currentPosition,
                               KeyPoint goalKeyPoint,
                               float x_threshold, float y_threshold, float yaw_threshold);
    int getClosestKeyPoint(float pose_x, float pose_y);

    void cb_finish_navigation(const std_msgs::Bool &msg);
    void go_on_patrol(int flag, int key_point_count, float current_position, float enemy_position);
    void cb_enemy_information(const robo_perception::ObjectList &msg);
    void read_xml_file();
    int find_enemy_self_closest_point(double enemy_x, double enemy_y, double self_x, double self_y);
    robo_perception::ObjectList sendEnemyTarget(const robo_perception::ObjectList &msg, robo_perception::ObjectList &last_enemy_target_msg);
    float calculator_enemy_angle(double enemy_x, double enemy_y, double self_x, double self_y);
    void cb_ukf_enemy_information(const nav_msgs::Odometry &msg);
    GambalInfo ctl_stack_enemy(bool enable_chassis_rotate=false);
    void main_control_init();
    geometry_msgs::Point ctl_track_enemy(double enemy_x, double enemy_y);
    void cb_fishcam_info(const robo_vision::FishCamInfo &msg);
    float ctl_yaw(int mode, float goal_yaw);
    PointInfo ctl_go_to_point(int mode, float goal_x, float goal_y);
    VelInfo ctl_chassis(int xy_mode, int yaw_mode, float goal_x, float goal_y, float goal_yaw);
    void mustRunInWhile(ros::NodeHandle private_nh);
    void cb_team_info(const robo_control::TeamInfo &msg);
    void get_param(ros::NodeHandle private_nh);
    float ctl_pid(float P, float I, float D, float error, float last_error);
    void sendFirstPoint(int first_point);
    float ctl_v_yaw(float yaw, float current_yaw);
    double calyaw(double set_yaw, double cur_yaw);
    float ctl_pid_v_yaw(float P, float I, float D, float error, float last_error_v_yaw);
    

    
    
    ros::NodeHandle *pnh;

    ros::Publisher pub_game_info;
    ros::Publisher pub_uwb_odom;
    ros::Publisher pub_wheel_vel;
    ros::Publisher pub_imu_data;
    ros::Publisher pub_nav_goal;
    ros::Publisher pub_first_point;
    ros::Publisher pub_enemy_target;

    tf2_ros::TransformBroadcaster gimbal_tf;
    tf2_ros::TransformBroadcaster camera_tf;

    Serial *serial;

    int uwb_ready_flag; //current UWB flag  valid if = 2
    int pre_uwb_ready_flag;
    int pre_uwb_x,pre_uwb_y,pre_uwb_yaw;

    int enemy_found_flag;     //if enemy found in realsence -> 1
    int enemy_found_pnp_flag; //if enemy found in pnp -> 1
    int armor_ready_flag;     //armor data come ->1  not = find armor
    int init_flag;
    ArmorInfo armor_info_msg;

    ChassisCMD cmd_vel_msg;

    geometry_msgs::Pose robo_uwb_pose;
    geometry_msgs::Pose robo_ukf_pose;
    geometry_msgs::Pose enemy_odom_pose;
    geometry_msgs::Pose enemy_odom_pnp_pose;
    geometry_msgs::Pose another_robo_pose;
    geometry_msgs::Pose robo_ukf_enemy_information;
    robo_control::GameInfo game_msg;
    robo_perception::ObjectList last_enemy_target;
    /* uint8 PENDING=0
     *  uint8 ACTIVE=1
     *  uint8 PREEMPTED=2
     *  uint8 SUCCEEDED=3
     *  uint8 ABORTED=4
     *  uint8 REJECTED=5
     *  uint8 PREEMPTING=6
     *  uint8 RECALLING=7
     *  uint8 RECALLED=8
     *  uint8 LOST=9*/
    int nav_status; //status from move_base pkg

    geometry_msgs::Pose nav_current_goal;

    bool keyPointNav(int keypoint_num);
    ros::Time nav_start_time;

    std_msgs::Bool finish_navigation;

    robo_perception::ObjectList enemy_information;

    // 各种 flag
    bool finish_goto_center = false;

    int key_point_count = 1;

    cv::Mat point_list;

    bool callback_navigation_flag = false;

    bool realsense_detection_state = false; // realsense 检测状态
    int realsense_lost_counter = 0;     // realsense 丢帧数量
    bool armor_detction_state = false;      // armor 检测状态
    int armor_lost_counter = 0;             // armor 丢帧数量
    int armor_around_lost_counter = 0; 
    bool armor_lost_state = false;          // armor 丢帧状态

    bool detected_armor_flag = false;
    bool first_in_armor_flag = false;
    bool first_in_realsense_flag = true;

    float first_in_gimbal_angle = 0;
    float current_gimbal_angle = 0;
    float target_gimbal_angle = 1000;

    double front_dis = 0.0;

    GambalInfo sent_mcu_gimbal_msg;
    GambalInfo sent_mcu_gimbal_result;

    VelInfo sent_mcu_vel_msg;
    VelInfo sent_mcu_vel_result;

    geometry_msgs::Pose last_enemy_target_pose;

    robo_vision::ArmorInfo armor_info_target;

    robo_vision::FishCamInfo fishcam_msg;
    float last_yaw = 0;

    float SELF_ENEMY_TARGET_DISTANCE = 0;
    float ENEMY_REALSENSE_ANGLE = 0;
    float ENEMY_GLOBAL_ANGLE = 0;

    robo_control::TeamInfo team_info;
    bool first_in_realsense_yaw = true;
    bool first_in_fishcam_yaw = true;
    ros::Time fishCamRotateStart = ros::Time::now();
    
    float current_pitch = 0;
    float error = 0;
    float output_pitch = 0;
    float last_error = 0;
    float sum_error = 0;

    // 参数服务器

    float DEATH_AREA = 20;      // 旋转的死区

    float SWITCH_FORWARD_BACKWARD_DIS = 0.6;
    float FORWARD_DIS = 3.0;
    float BACKWARD_DIS = 0.6;

    float MIN_TRACK_ENEMY_DIS = 0.7;
    float MAX_TRACK_ENEMY_DIS = 3.0;

    int ARMOR_MAX_LOST_NUM = 120;               // armor 最大允许丢帧数量 (真正丢帧)
    int ARMOR_AROUND_MAX_LOST_NUM = 120;        // armor 最大允许丢帧数量(摇头)
    int REALSENSE_AROUND_MAX_LOST_NUM = 120;    // realsense 最大允许的丢帧数量(摇头)

    float LOW_SHOT_SPEED_DISTANCE = 4.0;        // 低速射击最小距离
    float HIGH_SHOT_SPEED_DISTANCE = 3.0;       // 高速射击最大距离

    float ARMOR_LOST_PITCH = 5.0;

    float P_pitch = 1;
    float I_pitch = 0.1;
    float D_pitch = 0;
    float MAX_SUM_ERROR = 1;

    bool enable_direct_control_yaw = true;

    float P_v_yaw = 1;
    float I_v_yaw = 0.1;
    float D_v_yaw = 0;
    float MAX_SUM_ERROR_V_YAW = 1;

    float last_error_v_yaw = 0;
    float sum_error_v_yaw = 0;

    float current_yaw = 0;
    int realsense_first_in_lose_count = 0;


    // yaw: 1 -> 2
    KeyPoint KEY_POINT[30] = {
        {0, 1.30, 0.80, 0 * PI / 180.0},
        {1, 2.60, 0.80, 90 * PI / 180.0},
        {2, 2.60, 1.50, 90 * PI / 180.0},
        {3, 2.60, 2.50, 90 * PI / 180.0},

        //{4, 2.00, 3.20, 0*PI/180.0},
        //{5, 3.00, 3.50, 0*PI/180.0},

        {6, 4.00, 2.50, -90 * PI / 180.0},
        {7, 4.00, 1.50, -90 * PI / 180.0},
        {8, 4.00, 0.50, 0 * PI / 180.0},
        {9, 5.00, 0.50, 0 * PI / 180.0},
        {10, 6.40, 0.50, 0 * PI / 180.0},
        {11, 7.50, 1.0, 90 * PI / 180.0},
        //{12, 7.10, 1.80, 90*PI/180.0},
        //{13, 6.70, 2.50, 90*PI/180.0},
        {14, 6.70, 3.50, 90 * PI / 180.0},
        {15, 6.70, 4.20, 180 * PI / 180.0},
        {16, 5.40, 4.20, -90 * PI / 180.0},
        {17, 5.40, 3.50, -90 * PI / 180.0},
        {18, 5.40, 2.50, -90 * PI / 180.0},

        //{19, 6.00, 1.80, 180*PI/180.0},
        //{20, 5.00, 1.50, 180*PI/180.0},

        {21, 4.00, 2.50, 90 * PI / 180.0},
        {22, 4.00, 3.50, 90 * PI / 180.0},
        {23, 3.80, 4.20, 180 * PI / 180.0},
        {24, 3.00, 4.50, 180 * PI / 180.0},
        {25, 1.60, 4.50, 180 * PI / 180.0},
        {26, 0.60, 4.50, -90 * PI / 180.0},
        //{27, 0.90, 3.20, -90*PI/180.0},
        {28, 1.30, 2.50, -90 * PI / 180.0},
        {29, 1.30, 1.50, -90 * PI / 180.0},
    };
};