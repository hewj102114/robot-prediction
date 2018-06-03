#include <iostream>
#include "robo_control/robo_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_control");
    ros::NodeHandle nh;

    RoboControl robo_ctl;
    ros::Subscriber sub_armor_info = nh.subscribe("base/armor_info", 1, &RoboControl::cb_armorInfo, &robo_ctl);
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, &RoboControl::cb_cmd_vel, &robo_ctl);
    ros::Subscriber sub_move_base = nh.subscribe("move_base/feedback", 1, &RoboControl::cb_move_base, &robo_ctl);
    ros::Subscriber sub_enemy_pose = nh.subscribe("enemy/odom_pose", 1, &RoboControl::cb_enemy_pose, &robo_ctl);
    ros::Subscriber sub_enemy_pnp_pose = nh.subscribe("enemy/pnp_pose", 1, &RoboControl::cb_enemy_pnp_pose, &robo_ctl);
    ros::Subscriber sub_robo_pose = nh.subscribe("odom", 1, &RoboControl::cb_ukf_pose, &robo_ctl);
    ros::Subscriber sub_nav_cur_goal = nh.subscribe("move_base/current_goal", 1, &RoboControl::cb_cur_goal, &robo_ctl);
    ros::Subscriber sub_move_base_status = nh.subscribe("move_base/status", 1, &RoboControl::cb_move_base_status, &robo_ctl);
    ros::Subscriber sub_another_robo_pose = nh.subscribe("/DeepWhuRobot1/odom", 1, &RoboControl::cb_another_robo_odom, &robo_ctl);

    int init_flag = 1;
    geometry_msgs::Pose nav_goal; // goal of navigation
    int armor_lost_count = 0;
    int key_point_no = 1;
    clock_t start = clock(), end;

    int flag = -10;
    ros::Rate rate(150);
    geometry_msgs::Pose target_pose;
    double tar_x, tar_y;
    nh.setParam("px", 8);
    nh.setParam("py", 0);

    while (ros::ok())
    {
        robo_ctl.readMCUData();
        nh.getParam("px", tar_x);
        nh.getParam("py", tar_y);
        target_pose.position.x = tar_x;
        target_pose.position.y = tar_y;
        target_pose.orientation = robo_ctl.robo_ukf_pose.orientation;
        robo_ctl.sendNavGoal(target_pose);

        //ROS_INFO("MODE :  Search  %f  %f  %f",  robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y,robo_ctl.cmd_vel_msg.v_yaw);
        robo_ctl.sendMCUMsg(1, 1, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
        ros::spinOnce();
        rate.sleep();
    }
}
