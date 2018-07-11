#include <iostream>
#include "robo_control/robo_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_control");
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

        case 0:
        {
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_chassis(1, 1, 0, 0, 0);    // stay here
            break;
        }

        default:
            break;
        }
        robo_ctl.last_enemy_target = robo_ctl.sendEnemyTarget(robo_ctl.enemy_information, robo_ctl.last_enemy_target); // 目标会一直发送, 通过 'Nothing' 来判断有没有敌人
        robo_ctl.last_enemy_target_pose.position.x = robo_ctl.last_enemy_target.object[0].globalpose.position.x;
        robo_ctl.last_enemy_target_pose.position.y = robo_ctl.last_enemy_target.object[0].globalpose.position.y;
        robo_ctl.sent_mcu_gimbal_msg = robo_ctl.ctl_stack_enemy(); // 云台一直转动, 无论干什么都是一直转动
        // v_yaw: -3.14 -> 逆时针 3.14 -> 顺时针
        robo_ctl.sendMCUMsg(robo_ctl.sent_mcu_vel_msg.mode, robo_ctl.sent_mcu_gimbal_msg.mode,
                            robo_ctl.sent_mcu_vel_msg.v_x, robo_ctl.sent_mcu_vel_msg.v_y, robo_ctl.sent_mcu_vel_msg.v_yaw,
                            robo_ctl.sent_mcu_gimbal_msg.yaw, robo_ctl.sent_mcu_gimbal_msg.pitch, robo_ctl.sent_mcu_gimbal_msg.global_z);
        ros::spinOnce();
        loop_rate.sleep();
    }
}