/*************************************************************************
*
*  4 策略: 防守策略 3
*  假设: 敌人 1, 2 车都不抢中点, 全部过来攻击我方 
*  应对策略: 
*  1. 我方 1 车占据 E(2.6, 2.0, -90度), 2 车占据 A(2.6, 3.1, 0度), 攻击距离自己最近的车
*  2. 我方 1 车蹲守 D(2.6, 0.8, -90度), 2 车占据 C(1.3, 1.8, -90度), 进行蹲点, 如果有一辆车看到敌人, 另一辆车就过去帮忙, 直到比赛结束
*************************************************************************/

#include <iostream>
#include "robo_control/robo_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_control_strategy_3");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    vector<float> pointA;
    vector<float> pointB;
    vector<float> pointC;
    vector<float> pointD;
    vector<float> pointE;

    private_nh.getParam("pointA", pointA);
    private_nh.getParam("pointB", pointB);
    private_nh.getParam("pointC", pointC);
    private_nh.getParam("pointD", pointD);
    private_nh.getParam("pointE", pointE);

    vector<float> robo_point1;
    vector<float> robo_point2;

    private_nh.getParam("robo_point1", robo_point1);
    private_nh.getParam("robo_point2", robo_point2);

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
    ros::Subscriber sub_fishcam_info = nh.subscribe("/base/fishcam_info", 1, &RoboControl::cb_fishcam_info, &robo_ctl);
    ros::Subscriber sub_team_info = nh.subscribe("team/info", 1, &RoboControl::cb_team_info, &robo_ctl);
    

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
        case 0:
        {
            ROS_INFO("Stage 0: Go to center!");
            robo_ctl.finish_navigation.data = 0;
            work_state = 1;
            break;
        }
        /*************************************************************************
		*
		*  1. 站位, 本策略两个车占据不同的点, 点的坐标由 launch 文件给定
		*
		*************************************************************************/
        case 1:
        {
            ROS_INFO("Stage 1: Go to certain point!!!!!!");
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_chassis(1, 1, robo_point1[0], robo_point1[1], robo_point1[2]);
            if (robo_ctl.finish_navigation.data == 1) // 到达指点站位点, 跳转到下一个状态
            {
                robo_ctl.sent_mcu_vel_msg.v_x = 0;
                robo_ctl.sent_mcu_vel_msg.v_y = 0;
                robo_ctl.sent_mcu_vel_msg.v_yaw = 0;
                stackCenterEnemyStart = ros::Time::now();
                work_state = 2;
            }
            break;
        }
        /*************************************************************************
		*
		*  2. 检测有没有将敌人打死, 如果打死, 退回到基地
        *  TODO: 判断条件 (1. 中心车被打死 or 2. 计时 or 3. 看抢占中点的状态, 地方抢到立刻就撤退)
        *  TODO: 读取 buff 状态, 检测中点死车(要求稳定)
		*
		*************************************************************************/
        case 2:
        {
            ROS_INFO("Stage 2: Stacking enemy!!!!!!");
            ros::Duration timeout(10);
            if (ros::Time::now() - stackCenterEnemyStart > timeout)
            {
                robo_ctl.finish_navigation.data = 0;
                waitNavigationFlagStart = ros::Time::now();
                work_state = 3;
            }
            break;
        }
        /*************************************************************************
		*
		*  3. 返回基地
		*
		*************************************************************************/
        case 3:
        {
            ROS_INFO("Stage 3: Go back home!!!!!!");
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_chassis(1, 1, robo_point2[0], robo_point2[1], robo_point2[2]);
            ros::Duration timeout(0.1); // Timeout of 2 seconds
            while (ros::Time::now() - waitNavigationFlagStart < timeout)
            {
                robo_ctl.mustRunInWhile(private_nh);
                ros::spinOnce();
            }
            if (robo_ctl.finish_navigation.data == 1) // 到达指点站位点, 跳转到下一个状态, 必须完成
            {
                robo_ctl.sent_mcu_vel_msg.v_x = 0;
                robo_ctl.sent_mcu_vel_msg.v_y = 0;
                robo_ctl.sent_mcu_vel_msg.v_yaw = 0;
                work_state = 4;
            }
            break;
        }
        /*************************************************************************
		*
		*  4. 蹲点, 蹲守基地
		*
		*************************************************************************/
        case 4:
        {
            ROS_INFO("Stage 4: Stay at home!!!!!!");
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_chassis(1, 1, robo_point2[0], robo_point2[1], robo_point2[2]);
            if (robo_ctl.last_enemy_target.red_num == 1) // 看到敌人, 冲上去打
            {
                work_state = 5;
            }
            break;
        }
        /*************************************************************************
		*
		*  5. 检测到敌人, 跟着打
		*
		*************************************************************************/
        case 5:
        {
            ROS_INFO("Stage 5: Tracking and stacking enemy!!!!!!");
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_chassis(2, 1, robo_ctl.last_enemy_target_pose.position.x, robo_ctl.last_enemy_target_pose.position.y, 0);
            if (robo_ctl.last_enemy_target.red_num == 0)
            {
                work_state = 4;
            }
            break;
        }
        case 6:
        {
            ROS_INFO("Case for test!");
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
        robo_ctl.sendMCUMsg(1, robo_ctl.sent_mcu_gimbal_msg.mode,
                            robo_ctl.sent_mcu_vel_msg.v_x, robo_ctl.sent_mcu_vel_msg.v_y, robo_ctl.sent_mcu_vel_msg.v_yaw,
                            robo_ctl.sent_mcu_gimbal_msg.yaw, robo_ctl.sent_mcu_gimbal_msg.pitch, robo_ctl.sent_mcu_gimbal_msg.global_z);
        ros::spinOnce();
        loop_rate.sleep();
    }
}