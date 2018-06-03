/*************************************************************************
*
*  2 策略: 防守策略 1
*  假设: 敌人 1 车开局抢占中点, 2 车站在敌方区域 
*  应对策略: 
*  1. 我方 1 车占据 A(2.6, 3.1, 0度), 2 车占据 B(4.0, 4.2, 90度), 全力攻击敌方抢中点的车, 不管有没有将敌方车辆打死, 回到基地蹲点
*  2. 我方 1 车蹲守 C(1.3, 1.8, -90度), 2 车占据 D(2.6, 2.0, -90度), 进行蹲点, 如果有一辆车看到敌人, 另一辆车就过去帮忙, 直到比赛结束
*************************************************************************/

#include <iostream>
#include "robo_control/robo_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_session1");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    vector<float> pointA;
    vector<float> pointB;
    vector<float> pointC;
    vector<float> pointD;
    
    private_nh.getParam("pointA", pointA);
    private_nh.getParam("pointB", pointB);
    private_nh.getParam("pointC", pointC);
    private_nh.getParam("pointD", pointD);

    vector<float> robo_point1;
    vector<float> robo_point2;

    private_nh.getParam("robo_point1", robo_point1);
    private_nh.getParam("robo_point2", robo_point2);

    int first_point;
    private_nh.getParam("first_point", first_point);
    

    RoboControl robo_ctl;
    ros::Subscriber sub_armor_info = nh.subscribe("base/armor_info", 1, &RoboControl::cb_armorInfo, &robo_ctl);
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, &RoboControl::cb_cmd_vel, &robo_ctl);
    ros::Subscriber sub_move_base = nh.subscribe("move_base/feedback", 1, &RoboControl::cb_move_base, &robo_ctl);
    // ros::Subscriber sub_enemy_pose = nh.subscribe("enemy/odom_pose", 1, &RoboControl::cb_enemy_pose, &robo_ctl);
    ros::Subscriber sub_enemy_pnp_pose = nh.subscribe("enemy/pnp_pose", 1, &RoboControl::cb_enemy_pnp_pose, &robo_ctl);
    ros::Subscriber sub_robo_pose = nh.subscribe("odom", 1, &RoboControl::cb_ukf_pose, &robo_ctl);
    ros::Subscriber sub_nav_cur_goal = nh.subscribe("move_base/current_goal", 1, &RoboControl::cb_cur_goal, &robo_ctl);
    ros::Subscriber sub_move_base_status = nh.subscribe("move_base/status", 1, &RoboControl::cb_move_base_status, &robo_ctl);
    ros::Subscriber sub_another_robo_pose = nh.subscribe("/DeepWhuRobot1/odom", 1, &RoboControl::cb_another_robo_odom, &robo_ctl);
    ros::Subscriber sub_finish_navigation = nh.subscribe("nav_state", 1, &RoboControl::cb_finish_navigation, &robo_ctl);
    ros::Subscriber sub_enemy_information = nh.subscribe("infrared_detection/enemy_position", 1, &RoboControl::cb_enemy_information, &robo_ctl);
    ros::Subscriber sub_ukf_enemy_information = nh.subscribe("ukf/enemy", 1, &RoboControl::cb_ukf_enemy_information, &robo_ctl);
    ros::Subscriber sub_front_dis = nh.subscribe("front_dis", 1, &RoboControl::cb_front_dis, &robo_ctl);
    ros::Subscriber sub_team_info = nh.subscribe("team/info", 1, &RoboControl::cb_team_info, &robo_ctl);
    ros::Subscriber sub_fishcam_info = nh.subscribe("/base/fishcam_info", 1, &RoboControl::cb_fishcam_info, &robo_ctl);

    robo_ctl.main_control_init(); // init main contol function

    /* 重要参数: 
	gimbal 1:serach 2:shoot
	chassis 1:velcity 2:angle pose 3:init
	*/
    geometry_msgs::Pose target_pose;
    int work_state = 0; // switch case 的状态位
    ros::Time stackCenterEnemyStart;
    ros::Time waitNavigationFlagStart;

    ros::Rate loop_rate(150); // ROS 帧率
    while (ros::ok())
    {
        robo_ctl.readMCUData();
        switch (work_state)
        {
        /*************************************************************************
		*
		*  0. 抢占中点, 本策略跳过
		*
		*************************************************************************/
        // case 0:
        // {
        //     ROS_INFO("Stage 0: Go to center!");
        //     // robo_ctl.sendFirstPoint(first_point);
        //     // robo_ctl.sent_mcu_vel_msg.mode = 3;
        //     robo_ctl.sent_mcu_vel_msg.v_yaw = tf::getYaw(robo_ctl.robo_ukf_pose.orientation);
        //     if (robo_ctl.game_msg.RFID)
        //     {
        //         work_state = 1;

        //     }
        //     break;
        // }
        /*************************************************************************
		*
		*  1. 原地旋转, 跟着敌人旋转
		*
		*************************************************************************/
        case 0:
        {
            ROS_INFO("Stage 1: Rotate on a point!!!!!!");
            // xy_mode: 1 -> navigation goal, 2 -> track goal
            // yaw_mode: 1 -> 普通模式, 2 -> 永不转身, 维持当前角度
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_chassis(1, 1, 0, 0, 0);
            break;
        }
        
        default:
            break;
        }
        ROS_INFO("OK5");
        robo_ctl.last_enemy_target = robo_ctl.sendEnemyTarget(robo_ctl.enemy_information, robo_ctl.last_enemy_target); // 目标会一直发送, 通过 'Nothing' 来判断有没有敌人
        ROS_INFO("OK6");
        robo_ctl.last_enemy_target_pose.position.x = robo_ctl.last_enemy_target.object[0].globalpose.position.x;
        ROS_INFO("OK7");
        robo_ctl.last_enemy_target_pose.position.y = robo_ctl.last_enemy_target.object[0].globalpose.position.y;
        ROS_INFO("OK8");
        robo_ctl.sent_mcu_gimbal_msg = robo_ctl.ctl_stack_enemy(); // 云台一直转动, 无论干什么都是一直转动
        ROS_INFO("OK9");
        // v_yaw: -3.14 -> 逆时针 3.14 -> 顺时针
        robo_ctl.sendMCUMsg(robo_ctl.sent_mcu_vel_msg.mode, robo_ctl.sent_mcu_gimbal_msg.mode,
                            robo_ctl.sent_mcu_vel_msg.v_x, robo_ctl.sent_mcu_vel_msg.v_y, robo_ctl.sent_mcu_vel_msg.v_yaw,
                            robo_ctl.sent_mcu_gimbal_msg.yaw, robo_ctl.sent_mcu_gimbal_msg.pitch, robo_ctl.sent_mcu_gimbal_msg.global_z);
        ROS_INFO("OK_sent");
        ros::spinOnce();
        loop_rate.sleep();
    }
}