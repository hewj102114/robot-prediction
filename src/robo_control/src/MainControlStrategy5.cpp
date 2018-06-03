/*************************************************************************
*
*  5 策略: 攻击策略 1
*  假设: 我方 1 车抢中点, 2 车守家 
*  应对策略: 
*  1. 我方 1 车占据 A(2.6, 3.1, 0度), 2 车占据 B(4.0, 4.2, 90度), 全力攻击敌方抢中点的车, 不管有没有将敌方车辆打死, 回到基地蹲点
*  2. 我方 1 车蹲守 C(1.3, 1.8, -90度), 2 车占据 D(2.6, 2.0, -90度), 进行蹲点, 如果有一辆车看到敌人, 另一辆车就过去帮忙, 直到比赛结束
*************************************************************************/

#include <iostream>
#include "robo_control/robo_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_control_strategy_1");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    RoboControl robo_ctl;
    ros::Subscriber sub_armor_info = nh.subscribe("base/armor_info", 1, &RoboControl::cb_armorInfo, &robo_ctl);
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, &RoboControl::cb_cmd_vel, &robo_ctl);
    ros::Subscriber sub_move_base = nh.subscribe("move_base/feedback", 1, &RoboControl::cb_move_base, &robo_ctl);
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
    int work_state = 1; // switch case 的状态位

    ros::Rate loop_rate(150); // ROS 帧率
    while (ros::ok())
    {
        robo_ctl.readMCUData();
        switch (work_state)
        {
        /*************************************************************************
		*
		*  0. 抢占中点
		*
		*************************************************************************/
        case 0:
        {
            ROS_INFO("Stage 0: Go to center!");
            work_state = 1;
            break;
        }
        /*************************************************************************
		*
		*  1. 站位, 最简单策略, 固定一个站位点, 没有看到敌人就回到这个点
		*
		*************************************************************************/
        case 1:
        {
            ROS_INFO("Stage 1: Go to certain point!!!!!!");
            // point 1: (2.6, 3.1)
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_chassis(1, 1, 3.3, 3.2, 0);
            if (robo_ctl.last_enemy_target.red_num == 1)
            {
                work_state = 1;
            }
            break;
        }

        /*************************************************************************
		*
		*  2. 检测到敌人, 跟着打
		*
		*************************************************************************/
        case 2:
        {
            ROS_INFO("Stage 2: Tracking enemy!!!!!!");
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_chassis(2, 1, robo_ctl.last_enemy_target_pose.position.x, robo_ctl.last_enemy_target_pose.position.y, 0);
            if (robo_ctl.last_enemy_target.red_num == 0)
            {
                work_state = 1;
            }
            break;
        }
        case 3:
        {
            robo_ctl.readMCUData();
            ROS_INFO("Test Case");
            robo_ctl.sendMCUMsg(1, 1, 0, 0, 0, 0, 0, 0);
            ros::spinOnce();
            break;
        }
        default:
            break;
        }
        ROS_INFO("OK5");
        robo_ctl.get_param(private_nh);                                                                                          // 获取动态参数
        robo_ctl.last_enemy_target = robo_ctl.sendEnemyTarget(robo_ctl.enemy_information, robo_ctl.last_enemy_target); // 目标会一直发送, 通过 'Nothing' 来判断有没有敌人
        ROS_INFO("OK6");
        robo_ctl.last_enemy_target_pose.position.x = robo_ctl.last_enemy_target.object[0].globalpose.position.x;
        ROS_INFO("OK7");
        robo_ctl.last_enemy_target_pose.position.y = robo_ctl.last_enemy_target.object[0].globalpose.position.y;
        ROS_INFO("OK8");
        robo_ctl.sent_mcu_gimbal_msg = robo_ctl.ctl_stack_enemy(); // 云台一直转动, 无论干什么都是一直转动
        ROS_INFO("OK9");
        robo_ctl.sendMCUMsg(robo_ctl.sent_mcu_vel_msg.mode, robo_ctl.sent_mcu_gimbal_msg.mode,
                            robo_ctl.sent_mcu_vel_msg.v_x, robo_ctl.sent_mcu_vel_msg.v_y, robo_ctl.sent_mcu_vel_msg.v_yaw,
                            robo_ctl.sent_mcu_gimbal_msg.yaw, robo_ctl.sent_mcu_gimbal_msg.pitch, robo_ctl.sent_mcu_gimbal_msg.global_z);
        ros::spinOnce();
        loop_rate.sleep();
    }
}