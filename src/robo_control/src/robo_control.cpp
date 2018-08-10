#include "robo_control/robo_control.hpp"
#include "stdio.h"
void RoboControl::cb_armorInfo(const robo_vision::ArmorInfo &msg)
{
    armor_info_msg.mode = msg.mode;
    armor_info_msg.global_z = msg.target.pose_base.x;
    armor_info_msg.pitch = msg.angle.y;
    armor_info_msg.yaw = msg.angle.x;
    armor_info_target = msg;
    armor_ready_flag = 1;
}

void RoboControl::cb_cmd_vel(const geometry_msgs::Twist &msg)
{
    cmd_vel_msg.v_x = msg.linear.x;
    cmd_vel_msg.v_y = msg.linear.y;
    cmd_vel_msg.v_yaw = msg.angular.z;
}

void RoboControl::cb_move_base(const move_base_msgs::MoveBaseActionFeedback &msg)
{
    nav_status = msg.status.status;
    if (nav_status != 1)
    {
        nav_current_goal.position.x = 0;
        nav_current_goal.position.y = 0;
        nav_current_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    }
}

void RoboControl::cb_enemy_pose(const geometry_msgs::TransformStamped &msg)
{
    enemy_odom_pose.position.x = msg.transform.translation.x;
    enemy_odom_pose.position.y = msg.transform.translation.y;
    enemy_odom_pose.orientation = robo_ukf_pose.orientation;
    enemy_found_flag = 1;
}

void RoboControl::cb_enemy_pnp_pose(const geometry_msgs::TransformStamped &msg)
{
    enemy_odom_pnp_pose.position.x = msg.transform.translation.x - 0.2;
    enemy_odom_pnp_pose.position.y = msg.transform.translation.y - 0.2;
    enemy_odom_pnp_pose.orientation = robo_ukf_pose.orientation;
    enemy_found_pnp_flag = 1;
}

void RoboControl::cb_ukf_pose(const nav_msgs::Odometry &msg)
{
    robo_ukf_pose = msg.pose.pose;
}

void RoboControl::cb_another_robo_odom(const nav_msgs::Odometry &msg)
{
    another_robo_pose = msg.pose.pose;
}

void RoboControl::cb_move_base_status(const actionlib_msgs::GoalStatusArray &msg)
{
    ROS_INFO("receive!!!!!!!!!!!!!!!!!!!");
}
void RoboControl::cb_cur_goal(const geometry_msgs::PoseStamped &msg)
{
    nav_current_goal = msg.pose;
    nav_current_goal.position.x += 1;
    nav_current_goal.position.y += 1;
}

void RoboControl::cb_front_dis(const std_msgs::Float64 &msg)
{
    front_dis = msg.data;
}

void RoboControl::cb_team_info(const robo_control::TeamInfo &msg)
{
    team_info = msg;
}

void RoboControl::main_control_init()
{
    // init sent_mcu_gimbal_msg
    sent_mcu_gimbal_msg.mode = 1;
    sent_mcu_gimbal_msg.yaw = 0;
    sent_mcu_gimbal_msg.pitch = 0;
    sent_mcu_gimbal_msg.global_z = 0;

    // init sent_mcu_vel_result
    sent_mcu_vel_result.mode = 1;
    sent_mcu_vel_result.v_x = 0;
    sent_mcu_vel_result.v_y = 0;
    sent_mcu_vel_result.v_yaw = 0;

    // init enemy_information
    enemy_information.header.frame_id = "enemy";
    enemy_information.header.stamp = ros::Time::now();
    enemy_information.num = 0;
    enemy_information.red_num = 0;
    enemy_information.death_num = 0;
    enemy_information.blue_num = 0;

    robo_perception::Object temp_object;
    temp_object.team.data = "Nothing";
    temp_object.basepose.position.x = 0;
    temp_object.basepose.position.y = 0;
    temp_object.basepose.position.z = 0;

    temp_object.globalpose.position.x = 0;
    temp_object.globalpose.position.y = 0;
    temp_object.globalpose.position.z = 0;

    enemy_information.object.push_back(temp_object);

    // init last_enemy_target_pose
    last_enemy_target_pose.position.x = 0;
    last_enemy_target_pose.position.y = 0;
    last_enemy_target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    // read xml file
    read_xml_file();

    // init armor_info_target
    armor_info_target.armor_count = 0;
    armor_info_target.header.frame_id = "base_link";
    armor_info_target.header.stamp = ros::Time::now();
    armor_info_target.mode = 1;
}

void RoboControl::readMCUData()
{
    ros::Time time = ros::Time::now();
    RobotMsgFromMCU msg_frommcu;
    cout<<"saaaaa"<<endl;
    if (!serial->ReadData(msg_frommcu))
        return;

    init_flag = msg_frommcu.init_flag;
cout<<"ssssssssss"<<endl;
    game_msg.header.stamp = time;
    game_msg.header.frame_id = "base_link";
    game_msg.remainingHP = msg_frommcu.remaining_HP;
    game_msg.attackArmorID = msg_frommcu.attack_armorID;
    game_msg.bulletCount = msg_frommcu.remaining_bullet;
    game_msg.gimbalAngleYaw = msg_frommcu.gimbal_chassis_angle * 1.0 / 100;
    game_msg.gimbalAnglePitch = -msg_frommcu.gimbal_pitch_angle * 1.0 / 100;
    game_msg.bulletSpeed = msg_frommcu.bullet_speed * 1.0 / 1000;
    game_msg.RFID = msg_frommcu.rfid_flag;
    pub_game_info.publish(game_msg);

    nav_msgs::Odometry uwb_odom_msg;
    uwb_odom_msg.header.stamp = time;
    uwb_odom_msg.header.frame_id = "odom";
    uwb_odom_msg.child_frame_id = "base_link";
    uwb_odom_msg.pose.pose.position.x = msg_frommcu.uwb_x * 1.0 / 100;
    uwb_odom_msg.pose.pose.position.y = msg_frommcu.uwb_y * 1.0 / 100;

     ROS_INFO("AAA:  %f",msg_frommcu.uwb_yaw.num);

    //  ROS_INFO("BB: %d %d %d %d",msg_frommcu.uwb_yaw.byte[0],msg_frommcu.uwb_yaw.byte[1],msg_frommcu.uwb_yaw.byte[2],msg_frommcu.uwb_yaw.byte[3]);
    uwb_odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, (msg_frommcu.uwb_yaw.num) * PI * 2 / 360);
    //     uwb_odom_msg.twist.twist.linear.x = msg_frommcu.wheel_odom_x
    //     * 1.0 / 10000; uwb_odom_msg.twist.twist.linear.y =
    //     msg_frommcu.wheel_odom_y * 1.0 / 10000;
    robo_uwb_pose = uwb_odom_msg.pose.pose;
    if (pre_uwb_ready_flag != msg_frommcu.uwb_ready_flag && pre_uwb_x!=msg_frommcu.uwb_x &&  pre_uwb_y!=msg_frommcu.uwb_y && pre_uwb_yaw!=msg_frommcu.uwb_yaw.num)
    {
        
    }
    pub_uwb_odom.publish(uwb_odom_msg);
    pre_uwb_ready_flag = msg_frommcu.uwb_ready_flag;
    pre_uwb_x=msg_frommcu.uwb_x;
    pre_uwb_y=msg_frommcu.uwb_y;
    pre_uwb_yaw=msg_frommcu.uwb_yaw.num;

    geometry_msgs::Vector3Stamped wheel_vel_msg;
    wheel_vel_msg.header.stamp = time;
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = msg_frommcu.wheel_odom_x * 1.0 / 10000;
    wheel_vel_msg.vector.y = msg_frommcu.wheel_odom_y * 1.0 / 10000;
    pub_wheel_vel.publish(wheel_vel_msg);

    geometry_msgs::TransformStamped gimbal_trans;
    gimbal_trans.header.stamp = time;
    gimbal_trans.header.frame_id = "base_link";
    gimbal_trans.child_frame_id = "gimbal_link";
    gimbal_trans.transform.translation.x = 0.15;
    gimbal_trans.transform.translation.y = 0.0;
    gimbal_trans.transform.translation.z = 0.0;
    gimbal_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(
        0, -msg_frommcu.gimbal_pitch_angle * 1.0 / 100 * PI * 2 / 360,
        msg_frommcu.gimbal_chassis_angle * 1.0 / 100 * PI * 2 / 360);
    gimbal_tf.sendTransform(gimbal_trans);

    geometry_msgs::TransformStamped camera_trans;
    camera_trans.header.stamp = time;
    camera_trans.header.frame_id = "gimbal_link";
    camera_trans.child_frame_id = "usb_camera_link";
    camera_trans.transform.translation.x = 0.15;
    camera_trans.transform.translation.y = 0;
    camera_trans.transform.translation.z = 0.0;
    camera_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(-1.57, 0, -1.57);
    camera_tf.sendTransform(camera_trans);
}

void RoboControl::sendMCUMsg(int _chassis_mode, int _gimbal_mode,
                             float _velocity_x, float _velocity_y,
                             float _velocity_yaw, float _gimbal_yaw,
                             float _gimbal_pitch, float _enemy_distance)
{
    struct RobotMsgToMCU msg_tomcu;
    msg_tomcu.chassis_mode = _chassis_mode;
    msg_tomcu.gimbal_mode = _gimbal_mode;
    msg_tomcu.velocity_x = _velocity_x * 100;
    msg_tomcu.velocity_y = _velocity_y * 100;
    msg_tomcu.velocity_yaw = _velocity_yaw * KYAW;
    msg_tomcu.gimbal_yaw = _gimbal_yaw;
    msg_tomcu.gimbal_pitch = _gimbal_pitch;
    msg_tomcu.enemy_distance = _enemy_distance;
    serial->SendData(msg_tomcu);
    armor_ready_flag = 0;
}

void RoboControl::sendNavGoal(geometry_msgs::Pose &target_pose)
{
    // target_pose.position.x = 2.6;
    // target_pose.position.y = 3.1;
    // target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1.57);
    ROS_INFO("tar: %f  %f  %f  %f", target_pose.position.x,
             target_pose.position.y, nav_current_goal.position.x,
             nav_current_goal.position.y);
    //     if (abs(target_pose.position.x - nav_current_goal.position.x) > 0.2 ||
    //         abs(target_pose.position.y - nav_current_goal.position.y) > 0.2) {
    pub_nav_goal.publish(target_pose);
    callback_navigation_flag = false;
    //     }
}

void RoboControl::sendFirstPoint(int first_point)
{
    // target_pose.position.x = 2.6;
    // target_pose.position.y = 3.1;
    // target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1.57);
    //     if (abs(target_pose.position.x - nav_current_goal.position.x) > 0.2 ||
    //         abs(target_pose.position.y - nav_current_goal.position.y) > 0.2) {
    std_msgs::Int32 msg;
    msg.data = first_point;
    pub_first_point.publish(msg);
    //     }
}

bool RoboControl::judgeKeyPointPosition(KeyPoint currentPosition,
                                        KeyPoint goalKeyPoint,
                                        float x_threshold, float y_threshold,
                                        float yaw_threshold)
{
    float x_error = abs(currentPosition.x - goalKeyPoint.x);
    float y_error = abs(currentPosition.y - goalKeyPoint.y);
    // float yaw_error = abs(currentPosition.yaw - goalKeyPoint.yaw);
    float yaw_error = 0;
    if (x_error < x_threshold && y_error < y_threshold &&
        yaw_error < yaw_threshold)
    {
        return (true);
    }
    return (false);
}

bool RoboControl::keyPointNav(int keypoint_num)
{
    KeyPoint goalKeyPoint = KEY_POINT[keypoint_num];
    KeyPoint currentPosition = KEY_POINT[keypoint_num];
    currentPosition.x = robo_ukf_pose.position.x;
    currentPosition.y = robo_ukf_pose.position.y;

    geometry_msgs::Pose nav_goal;
    nav_goal.position.x = goalKeyPoint.x;
    nav_goal.position.y = goalKeyPoint.y;

    float angle = 0.0;
    if (goalKeyPoint.yaw != 0.0 || goalKeyPoint.yaw != 180 * PI / 180.0)
    {
        angle = 0.0;
    }
    nav_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    sendNavGoal(nav_goal);

    float x_threshold = 0.5;
    float y_threshold = 0.5;
    float yaw_threshold = 0.2;

    float x_error = abs(currentPosition.x - goalKeyPoint.x);
    float y_error = abs(currentPosition.y - goalKeyPoint.y);
    float yaw_error = 0;
    // float yaw_error = abs(currentPosition.yaw - goalKeyPoint.yaw);
    ROS_INFO("Goal: %d %f %f %f, error: %f %f %f", keypoint_num, goalKeyPoint.x,
             goalKeyPoint.y, goalKeyPoint.yaw, x_error, y_error, yaw_error);

    if (judgeKeyPointPosition(currentPosition, goalKeyPoint, x_threshold,
                              y_threshold, yaw_threshold))
    {
        ROS_INFO("robo_ctl.nav_status");
        nav_start_time = ros::Time::now();
        return true;
    }
    ros::Duration dtime = ros::Time::now() - nav_start_time;
    if (dtime.toSec() > 9)
    {
        nav_start_time = ros::Time::now();
        return true;
    }
    return false;
}

int RoboControl::getClosestKeyPoint(float pose_x, float pose_y)
{
    float distance = 1000;
    int num = 0;
    for (int i = 0; i < 23; i++)
    {
        float cur_dis = (pose_x - KEY_POINT[i].x) * (pose_x - KEY_POINT[i].x) +
                        (pose_y - KEY_POINT[i].y) * (pose_y - KEY_POINT[i].y);
        if (cur_dis < distance)
        {
            num = i;
            distance = cur_dis;
        }
    }
    return num;
}

void RoboControl::cb_finish_navigation(const std_msgs::Bool &msg)
{
    finish_navigation = msg;
    callback_navigation_flag = true;
}

void RoboControl::cb_enemy_information(const robo_perception::ObjectList &msg)
{
    enemy_information = msg;
}

void RoboControl::cb_fishcam_info(const robo_vision::FishCamInfo &msg)
{
    fishcam_msg = msg;
}

void RoboControl::go_on_patrol(int flag, int key_point_count, float current_position, float enemy_position)
{
    /*************************************************************************
    *  函数名称：go_on_patrol
    *  功能说明：巡图
    *  参数说明：   flag: 1 -> 从中点开始的巡图, 2 -> 丢失敌人巡图
    *               current_position: 自身当前位置
    *               enemy_position: 敌人最后一次出现的位置
    *  函数返回：返回下次去的位置
    *************************************************************************/
    geometry_msgs::Pose target_pose;
    if (flag == 1)
    {
        // 从中点开始的巡图
        float x_go_on_patrol[4] = {1.30, 6.70, 6.70, 1.30};
        float y_go_on_patrol[4] = {0.80, 0.80, 4.20, 4.20};

        target_pose.position.x = x_go_on_patrol[key_point_count];
        target_pose.position.y = y_go_on_patrol[key_point_count];
        target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        sendNavGoal(target_pose);
    }
    if (flag == 2)
    {
        // 丢失敌人之后的巡图
    }
}

void RoboControl::read_xml_file()
{
    /*************************************************************************
    *  read_xml_file()
    *  功能说明：读取导航点文件, 并将数据存入 point_list, 在初始化的时候进行
    *  参数说明：无数如参数
    *  函数返回：无返回值
    *************************************************************************/
    cv::FileStorage fs("/home/ubuntu/robot-prediction/src/robo_navigation/script/matrix.xml", cv::FileStorage::READ);
    fs["Point"] >> point_list;
}

int RoboControl::find_enemy_self_closest_point(double enemy_x, double enemy_y, double self_x, double self_y)
{
    vector<float> dis_list;
    for (int i = 0; i < point_list.rows; i++)
    {
        ROS_INFO("find_enemy_self_closest_point: %d", i);
        float d_enemy_y = enemy_y - point_list.at<double>(i, 0) * 1.0 / 100;
        float d_enemy_x = enemy_x - point_list.at<double>(i, 1) * 1.0 / 100;

        float d_self_y = self_y - point_list.at<double>(i, 0) * 1.0 / 100;
        float d_self_x = self_x - point_list.at<double>(i, 1) * 1.0 / 100;

        float distance_enemy = sqrt(0.4 * d_enemy_x * d_enemy_x + 0.6 * d_enemy_y * d_enemy_y);
        float distance_self = sqrt(0.4 * d_self_x * d_self_x + 0.6 * d_self_y * d_self_y);

        if (distance_enemy < 0.5 || distance_enemy > 2.0)
        {
            distance_enemy = 1000.0;
        }

        dis_list.push_back(0.4 * distance_self + 0.6 * distance_enemy);
    }

    vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());

    int n = distance(dis_list.begin(), smallest);
    return n;
}

float RoboControl::calculator_enemy_angle(double enemy_x, double enemy_y, double self_x, double self_y)
{
    float dx = enemy_x - self_x;
    float dy = enemy_y - self_y;
    float angle = atan2(dy, dx);
    return angle * 180.0 / PI + 90.0;
}

void RoboControl::cb_ukf_enemy_information(const nav_msgs::Odometry &msg)
{
    robo_ukf_enemy_information = msg.pose.pose;

    {
        SELF_ENEMY_TARGET_DISTANCE = robo_ukf_enemy_information.position.z;
    }
    if (robo_ukf_enemy_information.orientation.z != 999)
    {
        ENEMY_REALSENSE_ANGLE = -robo_ukf_enemy_information.orientation.z * 180.0 / PI * 100;
    }
    if (robo_ukf_enemy_information.orientation.y != 999)
    {
        ENEMY_GLOBAL_ANGLE = robo_ukf_enemy_information.orientation.y * 180.0 / PI * 100;
    }
}

robo_perception::ObjectList RoboControl::sendEnemyTarget(const robo_perception::ObjectList &msg, robo_perception::ObjectList &last_enemy_target_msg)
{
    /*************************************************************************
    *  sendEnemyTarget()
    *  功能说明：选择打击目标并 publish, realsense 的优先级最高, armor 其次, 如果realsense能够检测到, 就以 realsense 为主
    *  参数说明：msg -> realsense 的检测信息, last_enemy_target_msg -> 上一帧的打击目标
    *  函数返回：本帧的打击目标
    *  TODO: 1. 融合 armor 检测信息, 2. 测试断错误 bug, 3. 重新单独测试此函数, 看判断的敌人是不是稳定的
    *************************************************************************/
    ROS_INFO("sent enemy target!");
    vector<int> enemy_index;
    vector<float> enemy_self_distance; // 当前敌人与自己的相对距离
    vector<float> enemy_last_distance; // 当前敌人与上一帧敌人的距离

    robo_perception::ObjectList result_enemy_target; // return
    result_enemy_target.header = msg.header;
    result_enemy_target.header.stamp = ros::Time::now();

    result_enemy_target.num = 0;
    result_enemy_target.red_num = 0;
    result_enemy_target.death_num = 0;
    result_enemy_target.blue_num = 0;
    int enemy_index1 = 0;
    int enemy_index2 = 0;

    int select_idx = 0;

    robo_perception::ObjectList enemy_odom_target_msg; // publish
    enemy_odom_target_msg.header = msg.header;
    enemy_odom_target_msg.header.stamp = ros::Time::now();

    enemy_odom_target_msg.num = 0;
    enemy_odom_target_msg.red_num = 0;
    enemy_odom_target_msg.death_num = 0;
    enemy_odom_target_msg.blue_num = 0;

    robo_perception::Object temp_object;
    ROS_INFO("enemy target red num: %f", msg.red_num);

    // 计算打击哪个车, 给定 infrared detection 的检测结果和上一帧的打击目标
    if (msg.red_num == 0)
    {
        // 丢失敌人
        ROS_INFO("red_num = 0");

        if (armor_info_target.armor_count == 0)
        {
            ROS_INFO("red_num = 0 and armor = 0");
            // 此 if 表明 realsense 和 armor 都没有检测到
            if (fishcam_msg.size == 0)
            {
                ROS_INFO("red_num = 0 and armor = 0 and fish = 0");
                // realsense, armor 和 fishcam 都没有检测到, 读取队友的 enemytarget
                if (team_info.enemyNum == 1)
                {
                    ROS_INFO("red_num = 0 and armor = 0 and fish = 0 and team = 1");
                    // 队友有发布 enemytrget, 将这个 enemytarget 作为自己的 enemytarget 发布
                    enemy_odom_target_msg.num = 1;
                    enemy_odom_target_msg.red_num = 1;
                    temp_object.team.data = "red0";
                    temp_object.basepose.position.x = team_info.targetRelative.position.x;
                    temp_object.basepose.position.y = team_info.targetRelative.position.y;
                    temp_object.basepose.position.z = 0;

                    temp_object.globalpose.position.x = team_info.targetGlobal.position.x;
                    temp_object.globalpose.position.y = team_info.targetGlobal.position.y;
                    temp_object.globalpose.position.z = 0;

                    result_enemy_target.object.push_back(temp_object);
                    enemy_odom_target_msg = result_enemy_target;
                    enemy_odom_target_msg.object[0].pose.orientation.w = 1;
                    enemy_odom_target_msg.object[0].globalpose.orientation.w = 1;
                    pub_enemy_target.publish(enemy_odom_target_msg);
                    return result_enemy_target;
                }
                else
                {
                    ROS_INFO("red_num = 0 and armor = 0 and fish = 0 and team = 0");
                    // 队友没有发布 enemytarget, 发布 Nothing
                    temp_object.team.data = "Nothing";
                    temp_object.basepose.position.x = 0;
                    temp_object.basepose.position.y = 0;
                    temp_object.basepose.position.z = 0;

                    temp_object.globalpose.position.x = 0;
                    temp_object.globalpose.position.y = 0;
                    temp_object.globalpose.position.z = 0;

                    result_enemy_target.object.push_back(temp_object);
                    enemy_odom_target_msg = result_enemy_target;
                    pub_enemy_target.publish(enemy_odom_target_msg);
                    return result_enemy_target;
                }
            }
            else
            {
                ROS_INFO("red_num = 0 and armor = 0 and fish = 1");
                // realsense, armor 没有检测到, 鱼眼检测到, 发布 Nothing
                temp_object.team.data = "Nothing";
                temp_object.basepose.position.x = 0;
                temp_object.basepose.position.y = 0;
                temp_object.basepose.position.z = 0;

                temp_object.globalpose.position.x = 0;
                temp_object.globalpose.position.y = 0;
                temp_object.globalpose.position.z = 0;

                result_enemy_target.object.push_back(temp_object);
                enemy_odom_target_msg = result_enemy_target;
                pub_enemy_target.publish(enemy_odom_target_msg);
                return result_enemy_target;
            }
        }
        else
        {
            // 此 if 表明 armor 检测到, realsense 没有检测到
            if (armor_info_target.armor_count == 1)
            {
                ROS_INFO("red_num = 0 and armor = 1");
                // 只检测到一个 armor
                enemy_odom_target_msg.num = 1;
                enemy_odom_target_msg.red_num = 1;

                temp_object.team.data = "red0";
                temp_object.basepose.position.x = armor_info_target.armor_list[0].pose_base.x;
                temp_object.basepose.position.y = armor_info_target.armor_list[0].pose_base.y;
                temp_object.basepose.position.z = 0;

                temp_object.globalpose.position.x = armor_info_target.armor_list[0].pose_odom.x;
                temp_object.globalpose.position.y = armor_info_target.armor_list[0].pose_odom.y;
                temp_object.globalpose.position.z = 0;

                enemy_odom_target_msg.object.push_back(temp_object);

                enemy_odom_target_msg.object[0].pose.orientation.w = 1;
                enemy_odom_target_msg.object[0].globalpose.orientation.w = 1;
                result_enemy_target = enemy_odom_target_msg;
                pub_enemy_target.publish(enemy_odom_target_msg);
                return result_enemy_target;
            }
            if (armor_info_target.armor_count >= 2)
            {
                ROS_INFO("red_num = 0 and armor >= 2");
                // 检测到多个 armor
                for (int i = 0; i < armor_info_target.armor_count; i++)
                {
                    enemy_index.push_back(i);
                    enemy_self_distance.push_back(armor_info_target.armor_list[0].pose_base.x);
                    enemy_last_distance.push_back(pow(armor_info_target.armor_list[0].pose_odom.x - last_enemy_target_msg.object[0].globalpose.position.x, 2) + pow(armor_info_target.armor_list[0].pose_odom.y - last_enemy_target_msg.object[0].globalpose.position.y, 2));
                }

                if (last_enemy_target_msg.num == 0)
                {
                    // 没有打击过敌人, 选择相对距离近的敌人
                    vector<float>::iterator smallest = min_element(enemy_self_distance.begin(), enemy_self_distance.end());
                    select_idx = distance(enemy_self_distance.begin(), smallest);
                }
                else
                {
                    // 之前打击过敌人, 选择离之前的选择近的敌人
                    vector<float>::iterator smallest = min_element(enemy_last_distance.begin(), enemy_last_distance.end());
                    select_idx = distance(enemy_last_distance.begin(), smallest);
                }
                enemy_odom_target_msg.num = 1;
                enemy_odom_target_msg.red_num = 1;

                temp_object.basepose.position.x = armor_info_target.armor_list[select_idx].pose_base.x;
                temp_object.basepose.position.y = armor_info_target.armor_list[select_idx].pose_base.y;
                temp_object.basepose.position.z = 0;

                temp_object.globalpose.position.x = armor_info_target.armor_list[select_idx].pose_odom.x;
                temp_object.globalpose.position.y = armor_info_target.armor_list[select_idx].pose_odom.y;
                temp_object.globalpose.position.z = 0;

                enemy_odom_target_msg.object.push_back(temp_object);
                enemy_odom_target_msg.object[0].team.data = "red0";
                enemy_odom_target_msg.object[0].basepose.orientation.w = 1;
                enemy_odom_target_msg.object[0].globalpose.orientation.w = 1;
                result_enemy_target = enemy_odom_target_msg;

                pub_enemy_target.publish(enemy_odom_target_msg);
                return result_enemy_target;
            }
        }
    }
    if (msg.red_num == 1)
    {
        ROS_INFO("red_num = 1");
        // 没有选择, 只打当前的敌人, 遍历所有检测到的目标, 注意是 msg.num
        for (int i = 0; i < msg.num; i++)
        {
            if (msg.object[i].team.data == "red0")
            {
                enemy_index1 = i;
            }
        }
        enemy_odom_target_msg.num = 1;
        enemy_odom_target_msg.red_num = 1;
        enemy_odom_target_msg.object.push_back(msg.object[enemy_index1]);
        enemy_odom_target_msg.object[0].basepose.orientation.w = 1;
        enemy_odom_target_msg.object[0].globalpose.orientation.w = 1;
        result_enemy_target = enemy_odom_target_msg;
        pub_enemy_target.publish(enemy_odom_target_msg);
        return result_enemy_target;
    }
    if (msg.red_num >= 2)
    {
        ROS_INFO("red_num >= 2");

        // 判断之前有没有打击过敌人, 如果之前没有打击过敌人, 选择距离近的敌人, 否则选择之前打击过的
        for (int i = 0; i < msg.num; i++)
        {
            if (msg.object[i].team.data == "red" + to_string(i))
            {
                enemy_index.push_back(i);
                enemy_self_distance.push_back(msg.object[i].basepose.position.x);
                enemy_last_distance.push_back(pow(msg.object[i].globalpose.position.x - last_enemy_target_msg.object[0].globalpose.position.x, 2) + pow(msg.object[i].globalpose.position.y - last_enemy_target_msg.object[0].globalpose.position.y, 2));
            }
        }

        if (last_enemy_target_msg.num == 0)
        {
            // 没有打击过敌人, 选择相对距离近的敌人
            vector<float>::iterator smallest = min_element(enemy_self_distance.begin(), enemy_self_distance.end());
            select_idx = distance(enemy_self_distance.begin(), smallest);
        }
        else
        {
            // 之前打击过敌人, 选择离之前的选择近的敌人
            vector<float>::iterator smallest = min_element(enemy_last_distance.begin(), enemy_last_distance.end());
            select_idx = distance(enemy_last_distance.begin(), smallest);
        }
        enemy_odom_target_msg.num = 1;
        enemy_odom_target_msg.red_num = 1;
        enemy_odom_target_msg.object.push_back(msg.object[select_idx]);
        enemy_odom_target_msg.object[0].team.data = "red0";
        enemy_odom_target_msg.object[0].basepose.orientation.w = 1;
        enemy_odom_target_msg.object[0].globalpose.orientation.w = 1;
        result_enemy_target = enemy_odom_target_msg;
        pub_enemy_target.publish(enemy_odom_target_msg);
        return result_enemy_target;
    }
    return result_enemy_target;
}

// 打击函数
GambalInfo RoboControl::ctl_stack_enemy(bool enable_chassis_rotate)
{
    /*************************************************************************
    *  ctl_stack_enemy()
    *  功能说明：控制云台转动和打击
    *  参数说明：无输入参数, 一直执行本函数
    *  函数返回：云台控制模式和角度 result -> mode, yaw, pitch, global_z
    *  TODO: 1. 添加摇头功能, 2. 考虑打击标志位如何放置, 理论上 armor mode = 3 时就应该开枪, 优先级高于一切
    *************************************************************************/
    // 2. realsense和armor都没有看到的时候, 并且丢帧数量小于 400, 维持云台角度
    ROS_INFO("armor_lost_counter: %d", armor_lost_counter);
    
    // delete realsense
    enemy_information.red_num = 0;

    if (armor_info_msg.mode == 3)
    {
        ROS_INFO("mode = 3, stacking enemy");
        if (SELF_ENEMY_TARGET_DISTANCE > LOW_SHOT_SPEED_DISTANCE || SELF_ENEMY_TARGET_DISTANCE == 0)
        {
            sent_mcu_gimbal_result.mode = 6; // 低速 6
        }
        else if (SELF_ENEMY_TARGET_DISTANCE > HIGH_SHOT_SPEED_DISTANCE)
        {
            sent_mcu_gimbal_result.mode = 7; // 中速 7
        }
        else
        {
            sent_mcu_gimbal_result.mode = 8; // 高速 8
        }
        if (enable_chassis_rotate)
        {
            sent_mcu_vel_msg.mode = 4;
        }

        sent_mcu_gimbal_result.yaw = armor_info_msg.yaw + robo_ukf_enemy_information.orientation.w * 100.0;
        sent_mcu_gimbal_result.pitch = armor_info_msg.pitch + robo_ukf_enemy_information.orientation.x * 100.0;
        sent_mcu_gimbal_result.global_z = SELF_ENEMY_TARGET_DISTANCE * 100;
        return sent_mcu_gimbal_result;
    }
    else
    {
        // sent_mcu_vel_msg.mode = 1;        
    }

    if (enemy_information.red_num == 0)
    {
        // realsense 没有检测到, 计数
        realsense_lost_counter++;
    }
    else
    {
        // 一旦 realsense 检测到, 清零
        realsense_lost_counter = 0;
    }
    if (armor_info_msg.mode == 1)
    {
        // realsense 没有检测到, 计数
        armor_around_lost_counter++;
    }
    else
    {
        // 一旦 realsense 检测到, 清零
        armor_around_lost_counter = 0;
    }
    if (armor_around_lost_counter > ARMOR_AROUND_MAX_LOST_NUM && realsense_lost_counter > REALSENSE_AROUND_MAX_LOST_NUM)
    {
        ROS_INFO("mode = 1");
        sent_mcu_gimbal_result.mode = 1;
        sent_mcu_gimbal_result.yaw = 0;
        sent_mcu_gimbal_result.pitch = 0;
        sent_mcu_gimbal_result.global_z = 0;
    }

    if (armor_lost_counter > ARMOR_MAX_LOST_NUM && armor_info_msg.mode == 1)
    {
        if (enemy_information.red_num > 0 && first_in_realsense_flag == false)
        {
            realsense_first_in_lose_count++;
            // ROS_INFO("realsense_first_in_lose_count: %d", realsense_first_in_lose_count);
            if (realsense_first_in_lose_count > 100)
            {
                realsense_first_in_lose_count = 0;
                first_in_realsense_flag = true;
            }
        }
        if (enemy_information.red_num > 0 && first_in_realsense_flag == true)
        {
            // realsense看到, armor没看到, 并且丢帧数量大于400帧, realsense引导云台转动
            ROS_INFO("realsense detected");
            realsense_first_in_lose_count = 0;
            first_in_realsense_flag = false;
            int enemy_realsense_angle = 0;
            if (robo_ukf_enemy_information.orientation.z != 999)
            {
                enemy_realsense_angle = -robo_ukf_enemy_information.orientation.z * 180.0 / PI;
            }

            first_in_gimbal_angle = game_msg.gimbalAngleYaw;
            target_gimbal_angle = enemy_realsense_angle;

            if (robo_ukf_enemy_information.orientation.z == 999)
            {
                target_gimbal_angle = 1000;
                first_in_realsense_flag = true;
            }

            sent_mcu_gimbal_result.mode = 3;
            sent_mcu_gimbal_result.yaw = enemy_realsense_angle * 100;
            sent_mcu_gimbal_result.pitch = 0;
            sent_mcu_gimbal_result.global_z = 0;
        }
        ROS_INFO("armor_lost_counter > max_lost_counter");
        current_gimbal_angle = game_msg.gimbalAngleYaw;
        ROS_INFO("first_in: %f, target: %f, current: %f", first_in_gimbal_angle, target_gimbal_angle, current_gimbal_angle);
        if (abs(abs(current_gimbal_angle - first_in_gimbal_angle) - abs(target_gimbal_angle)) < 5)
        {
            // mode = 3 的时候, 云台位置环
            ROS_INFO("realsense finish");
            first_in_armor_flag = true;
            detected_armor_flag = false;
            armor_lost_counter = 0;
        }
    }
    if (armor_lost_counter <= ARMOR_MAX_LOST_NUM || armor_info_msg.mode > 1)
    {
        // 4. realsense看到, armor没看到, 并且丢帧数量小于400帧, realsense不管
        if (armor_info_msg.mode == 1)
        {
            ROS_INFO("lose armor");
            armor_lost_counter++;
            if (first_in_armor_flag == true)
            {
                sent_mcu_gimbal_result.mode = 2;
                sent_mcu_gimbal_result.yaw = 0;
                sent_mcu_gimbal_result.pitch = 32760;
                sent_mcu_gimbal_result.global_z = 0;
            }
            if (detected_armor_flag == true)
            {
                sent_mcu_gimbal_result.mode = 2;
                sent_mcu_gimbal_result.yaw = 0;
                sent_mcu_gimbal_result.pitch = -ARMOR_LOST_PITCH;
                sent_mcu_gimbal_result.global_z = 0;
            }
        }

        // 只要 armor 检测到, 云台就转,j, 没有商量的余地, armor 丢帧的话需要商量一下
        // 3. realsense和armor都看到, 以armor的转动为主
        if (armor_info_msg.mode > 1)
        {
            ROS_INFO("detected armor");
            sent_mcu_gimbal_result.mode = 2;
            sent_mcu_gimbal_result.yaw = armor_info_msg.yaw + robo_ukf_enemy_information.orientation.w * 100.0;
            // sent_mcu_gimbal_result.pitch = armor_info_msg.pitch + robo_ukf_enemy_information.orientation.x * 100.0;
            // current_pitch = game_msg.gimbalAnglePitch;
            error = (armor_info_msg.pitch + robo_ukf_enemy_information.orientation.x * 100.0) / 100.0;
            // output_pitch = ctl_pid(P_pitch, I_pitch, D_pitch, error, last_error);
            last_error = error;
            sent_mcu_gimbal_result.pitch = output_pitch * 100;
            sent_mcu_gimbal_result.pitch = armor_info_msg.pitch + robo_ukf_enemy_information.orientation.x * 100.0;
            sent_mcu_gimbal_result.global_z = armor_info_msg.global_z * 100;

            armor_lost_counter = 0;
            realsense_lost_counter = 0;
            detected_armor_flag = true;
            first_in_armor_flag = false;
        }
        target_gimbal_angle = 1000;
        first_in_realsense_flag = true;
    }
    sent_mcu_gimbal_result.global_z = SELF_ENEMY_TARGET_DISTANCE * 100.0;
    return sent_mcu_gimbal_result;
}

float RoboControl::ctl_pid(float P, float I, float D, float error, float last_error)
{
    float output = 0;
    sum_error = sum_error + error;
    if (sum_error > MAX_SUM_ERROR)
    {
        sum_error = MAX_SUM_ERROR;
    }
    output = P * error + I * (sum_error) + D * (last_error - error);
    return output;
}

VelInfo RoboControl::ctl_chassis(int xy_mode, int yaw_mode, float goal_x, float goal_y, float goal_yaw)
{
    /*************************************************************************
    * ctl_chassis()
    * 功能说明：控制底盘硬件
    * 参数说明：xy_mode: 控制底盘的位置, yaw_mode: 控制地盘的角度, (goal_x, goal_y, goal_yaw): 控制地盘的位置和角度
    * 函数返回：返回底盘的转动模式, 位置和角度
    *************************************************************************/
    // 定义publish的值
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0;
    target_pose.position.y = 0;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    float yaw = ctl_yaw(yaw_mode, goal_yaw);
    PointInfo point = ctl_go_to_point(xy_mode, goal_x, goal_y);

    // 发送并返回
    target_pose.position.x = point.x;
    target_pose.position.y = point.y;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);

    sendNavGoal(target_pose);

    sent_mcu_vel_result.mode = point.mode;
    sent_mcu_vel_result.v_x = cmd_vel_msg.v_x;
    sent_mcu_vel_result.v_y = cmd_vel_msg.v_y;
    sent_mcu_vel_result.v_yaw = cmd_vel_msg.v_yaw;

    current_yaw = tf::getYaw(robo_ukf_pose.orientation);
    if (enable_direct_control_yaw == true)
    {
        sent_mcu_vel_result.v_x = 0;
        sent_mcu_vel_result.v_y = 0;
        // ROS_INFO("yaw:%f, current_yaw: %f", yaw, current_yaw);
        sent_mcu_vel_result.v_yaw = ctl_v_yaw(yaw, current_yaw);
    }
    return sent_mcu_vel_result;
}

float RoboControl::ctl_v_yaw(float yaw, float current_yaw)
{
    float error_v_yaw = calyaw(yaw, current_yaw);
    ROS_INFO("error_v_yaw: %f", error_v_yaw);
    float v_yaw = ctl_pid_v_yaw(P_v_yaw, I_v_yaw, D_v_yaw, error_v_yaw, last_error_v_yaw);
    ROS_INFO("v_yaw: %f", v_yaw);
    last_error_v_yaw = error_v_yaw;
    return v_yaw;
}

double RoboControl::calyaw(double set_yaw, double cur_yaw)
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

float RoboControl::ctl_pid_v_yaw(float P, float I, float D, float error, float last_error_v_yaw)
{
    float output = 0;
    ROS_INFO("PID: %f, %f, %f, %f, %f", P, I, D, error, last_error_v_yaw);
    sum_error_v_yaw = sum_error_v_yaw + error;
    if (sum_error_v_yaw > MAX_SUM_ERROR_V_YAW)
    {
        sum_error_v_yaw = MAX_SUM_ERROR_V_YAW;
    }
    output = P * error + D * (last_error_v_yaw - error);
    return output;
}


float RoboControl::ctl_yaw(int mode, float goal_yaw)
{
    /*************************************************************************
    * ctl_yaw()
    * 功能说明：控制底盘 yaw
    * 参数说明：mode: 1 -> 进攻模式 2 -> 防御模式
    * 函数返回：返回底盘转角
    * TODO: 1. 测试, 2. 改回原来的返回值
    *************************************************************************/
    float yaw = 0;
    float fish_yaw = 0;
    float yaw_index = 0;
    if (mode == 1)
    {
        // 正常模式
        ROS_INFO("death_area: %f, %f, %f, %f", yaw, tf::getYaw(robo_ukf_pose.orientation), yaw - tf::getYaw(robo_ukf_pose.orientation), DEATH_AREA);

        if (robo_ukf_enemy_information.orientation.y != 999)
        {
            // 有目标的时候才转
            yaw = robo_ukf_enemy_information.orientation.y * 180.0 / PI;
            if (robo_ukf_enemy_information.position.z > 2.0)
            {
                DEATH_AREA = 10;
            }
            else if (robo_ukf_enemy_information.position.z > 1.5)
            {
                DEATH_AREA = 15;
            }
            else if (robo_ukf_enemy_information.position.z > 1.0)
            {
                DEATH_AREA = 18;
            }
            else
            {
                DEATH_AREA = 35;
            }

            if (abs(yaw - tf::getYaw(robo_ukf_pose.orientation) * 180 / PI) > DEATH_AREA)
            {
                first_in_fishcam_yaw = true;
                yaw = yaw * PI / 180.0;
                last_yaw = yaw;
                return yaw;
            }
            return tf::getYaw(robo_ukf_pose.orientation);
        }
        ros::Duration timeout(3);
        if (fishcam_msg.size > 0 && ros::Time::now() - fishCamRotateStart > timeout)
        {
            if (fishcam_msg.size == 2)
            {
                if (fishcam_msg.target[0].z > fishcam_msg.target[1].z)
                {
                    yaw_index = 0;
                }
                else
                {
                    yaw_index = 1;
                }
            }
            else
            {
                yaw_index = 0;
            }
            // 鱼眼相机发现目标
            fish_yaw = fishcam_msg.target[yaw_index].x * PI / 180;
            fish_yaw = tf::getYaw(robo_ukf_pose.orientation) + fish_yaw;
            if (fish_yaw < -PI)
            {
                fish_yaw = fish_yaw + 2 * PI;
            }

            if (fish_yaw > PI)
            {
                fish_yaw = fish_yaw - 2 * PI;
            }
            ROS_INFO("fish_camera:%f", fish_yaw * 180.0 / PI);
            first_in_realsense_yaw = true;
            // first_in_fishcam_yaw = false;
            last_yaw = fish_yaw;
            return fish_yaw;
        }
    }
    if (mode == 2)
    {
        // 没有子弹. 永不转身
        last_yaw = goal_yaw;
        return goal_yaw;        // 直接转向设定的角度
    }
    return last_yaw;
    // return goal_yaw;
}

PointInfo RoboControl::ctl_go_to_point(int mode, float goal_x, float goal_y)
{
    /*************************************************************************
    * ctl_go_to_point()
    * 功能说明：去某个点
    * 参数说明：mode: 1 -> 根据给定点去某个点, 2 -> 根据地方位置去某个点(不使用realsense信息), 3 -> 根据地方位置去某个点(使用realsense信息)
    * 函数返回：返回下次去的位置
    * 底盘只存在两种控制模式, 1. 人工给定确定点的控制模式, 2. 由敌人位置确定的控制模式，敌人位置确定的控制模式优先级最高, 然后才是人工给定的点的控制模式
    * TODO: 1. 测试转动底盘保持目标在 realsense 的中心
    *************************************************************************/

    PointInfo point;
    geometry_msgs::Point track_enemy_point;
    if (mode == 1)
    {
        point.x = goal_x;
        point.y = goal_y;
        point.mode = 1;
    }
    if (mode == 2)
    {
        track_enemy_point = ctl_track_enemy(goal_x, goal_y);
        point.x = track_enemy_point.x;
        point.y = track_enemy_point.y;
        point.mode = 1;
    }
    if (mode == 3)
    {

    }

    return point;
}

geometry_msgs::Point RoboControl::ctl_track_enemy(double enemy_x, double enemy_y)
{
    /*************************************************************************
    *  ctl_track_enemy()
    *  功能说明：跟踪敌军
    *  参数说明：enemy_{x,y} 敌军全局位置, yaw: 自身姿态
    *  函数返回：返回下次去的位置
    *  TODO: 测试本函数, 5月12日
    *************************************************************************/
    float x_coefficient = 0.4, y_coefficient = 0.6;
    float self_coefficient = 0.4, enemy_coefficient = 0.6;

    float self_x = robo_ukf_pose.position.x;
    float self_y = robo_ukf_pose.position.y;

    geometry_msgs::Point target_pose;

    // 小于 0.7m, 后退, 大于 0.7m 前进

    if (robo_ukf_enemy_information.position.z < SWITCH_FORWARD_BACKWARD_DIS)
    {
        // 如果 enemy_target 的距离小于 0.8m, 将 enemy 映射到以自身为中心的对称点上, 计算得到相同直线上的反向最小距离点
        // self_coefficient = 0.6, enemy_coefficient = 0.4; // 修改系数大小, 这时候应该离自身较近
        // enemy_y = self_y - (enemy_y - self_y);
        // enemy_x = self_x - (enemy_x - self_x);
        if (self_x > 4.0)
        {
            target_pose.x = self_x - 0.5;
            target_pose.y = self_y;
        }
        else
        {
            target_pose.x = self_x - 0.5;
            if (self_y < 1.5)
            {
                self_y = 0.7;
            }
            else if (self_y < 2.7)
            {
                self_y = self_y;
            }
            else if (self_y < 3.7)
            {
                self_y = 3.1;
            }
            else
            {
                self_y = 4.5;
            }
            target_pose.y = self_y;
        }
        return target_pose;
    }
    else
    {
        vector<float> dis_list;
        for (int i = 0; i < point_list.rows; i++)
        {
            // ROS_INFO("find_enemy_self_closest_point_start: %d", i);
            float d_enemy_y = enemy_y - point_list.at<double>(i, 0) * 1.0 / 100;
            float d_enemy_x = enemy_x - point_list.at<double>(i, 1) * 1.0 / 100;

            float d_self_y = self_y - point_list.at<double>(i, 0) * 1.0 / 100;
            float d_self_x = self_x - point_list.at<double>(i, 1) * 1.0 / 100;

            float distance_enemy = sqrt(x_coefficient * d_enemy_x * d_enemy_x + y_coefficient * d_enemy_y * d_enemy_y);
            float distance_self = sqrt(x_coefficient * d_self_x * d_self_x + y_coefficient * d_self_y * d_self_y);

            if (distance_enemy < MIN_TRACK_ENEMY_DIS || distance_enemy > MAX_TRACK_ENEMY_DIS)
            {
                distance_enemy = 1000.0;
            }

            dis_list.push_back(self_coefficient * distance_self + enemy_coefficient * distance_enemy);
            ROS_INFO("find_enemy_self_closest_point_end: %d", i);
        }
        vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());
        int n = distance(dis_list.begin(), smallest);
        target_pose.x = point_list.at<double>(n, 1) / 100.0;
        target_pose.y = point_list.at<double>(n, 0) / 100.0;
        return target_pose;
    }
}

void RoboControl::mustRunInWhile(ros::NodeHandle private_nh)
{
    readMCUData();
    get_param(private_nh);
    last_enemy_target = sendEnemyTarget(enemy_information, last_enemy_target); // 目标会一直发送, 通过 'Nothing' 来判断有没有敌人
    last_enemy_target_pose.position.x = last_enemy_target.object[0].globalpose.position.x;
    last_enemy_target_pose.position.y = last_enemy_target.object[0].globalpose.position.y;
    sent_mcu_gimbal_msg = ctl_stack_enemy(); // 云台一直转动, 无论干什么都是一直转动
    sendMCUMsg(1, sent_mcu_gimbal_msg.mode,
               sent_mcu_vel_msg.v_x, sent_mcu_vel_msg.v_y, sent_mcu_vel_msg.v_yaw,
               sent_mcu_gimbal_msg.yaw, sent_mcu_gimbal_msg.pitch, sent_mcu_gimbal_msg.global_z);
}

void RoboControl::get_param(ros::NodeHandle private_nh)
{
    private_nh.getParam("DEATH_AREA", DEATH_AREA);

    private_nh.getParam("SWITCH_FORWARD_BACKWARD_DIS", SWITCH_FORWARD_BACKWARD_DIS);

    private_nh.getParam("FORWARD_DIS", FORWARD_DIS);
    private_nh.getParam("BACKWARD_DIS", BACKWARD_DIS);   

    private_nh.getParam("MIN_TRACK_ENEMY_DIS", MIN_TRACK_ENEMY_DIS);
    private_nh.getParam("MAX_TRACK_ENEMY_DIS", MAX_TRACK_ENEMY_DIS);

    private_nh.getParam("ARMOR_MAX_LOST_NUM", ARMOR_MAX_LOST_NUM);
    private_nh.getParam("ARMOR_AROUND_MAX_LOST_NUM", ARMOR_AROUND_MAX_LOST_NUM);
    private_nh.getParam("REALSENSE_AROUND_MAX_LOST_NUM", REALSENSE_AROUND_MAX_LOST_NUM);

    private_nh.getParam("LOW_SHOT_SPEED_DISTANCE", LOW_SHOT_SPEED_DISTANCE);
    private_nh.getParam("HIGH_SHOT_SPEED_DISTANCE", HIGH_SHOT_SPEED_DISTANCE);

    private_nh.getParam("ARMOR_LOST_PITCH", ARMOR_LOST_PITCH);

    private_nh.getParam("P_pitch", P_pitch);
    private_nh.getParam("I_pitch", I_pitch);
    private_nh.getParam("D_pitch", D_pitch);
    private_nh.getParam("MAX_SUM_ERROR", MAX_SUM_ERROR);
    
}
