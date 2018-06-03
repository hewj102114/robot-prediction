#include <iostream>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include "robo_vision/AngleSolver.hpp"
#include "robo_vision/ArmorDetector.hpp"
#include "robo_vision/ArmorInfo.h"
#include "robo_vision/Predictor.hpp"
#include "robo_vision/RMVideoCapture.hpp"
#include "robo_vision/RuneResFilter.hpp"
#include "robo_vision/Settings.hpp"
using namespace std;
using namespace cv;

Mat img_recv;
int pre_seq = 0, cur_seq = 0;

void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat img_temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    //ROS_INFO("recv");
    if (img_temp.empty())
    {
        ROS_ERROR("Could't not get image");
    }
    else
    {
        //cv::imshow("view", img_temp);
        img_recv = img_temp.clone();
        cur_seq = msg->header.seq;
    }
}

int main(int argc, char *argv[])
{
    // ros init
    ros::init(argc, argv, "robo_vision");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    // ros publisher
    ros::Publisher pub_armor_info =
        nh.advertise<robo_vision::ArmorInfo>("base/armor_info", 1);
    ros::Publisher pub_armor_pose =
        nh.advertise<geometry_msgs::PoseStamped>("base/armor_pose", 1);
    tf2_ros::TransformBroadcaster enemy_pnp_tf;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("usbcamera/image", 1, &image_callback);
    // load setting from fil
    string config_file_name = "/home/ubuntu/robot/src/robo_vision/param/param_config.xml";
    private_nh.getParam("config_file_name", config_file_name);
    Settings setting(config_file_name);

    // angle offset
    double offset_anlge_x = 0;
    double offset_anlge_y = 0;
    private_nh.getParam("offset_anlge_x", offset_anlge_x);
    private_nh.getParam("offset_anlge_y", offset_anlge_y);

    // get robot id & choose the calibration file
    string intrinsic_file;
    private_nh.getParam("intrinsic_file", intrinsic_file);
    cout << intrinsic_file << endl;
    FileStorage fs(intrinsic_file, FileStorage::READ);
    if (!fs.isOpened())
    {
        ROS_ERROR("Could not open the configuration file: %s",
                  intrinsic_file);
        return 0;
    }
    Mat cam_matrix_480, distortion_coeff_480;
    fs["Camera_Matrix"] >> cam_matrix_480;
    fs["Distortion_Coefficients"] >> distortion_coeff_480;

    // camera and gimbal transform
    const double ptz_camera_y = 5;
    const double ptz_camera_z = -15;
    double r_data[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double t_data[] = {
        0, ptz_camera_y,
        ptz_camera_z}; // ptz org position in camera coodinate system
    Mat t_camera_ptz(3, 1, CV_64FC1, t_data);
    Mat r_camera_ptz(3, 3, CV_64FC1, r_data);

    AngleSolver angle_solver(cam_matrix_480, distortion_coeff_480, t_camera_ptz,
                             r_camera_ptz, 13.5, 5.8);
    Point2f image_center = Point2f(cam_matrix_480.at<double>(0, 2),
                                   cam_matrix_480.at<double>(1, 2));

    double pre_angle_x = 0.0, pre_angle_y = 0.0;

    // load armor detector setting
    ArmorDetector armor_detector(setting.armor);

    FilterZ filter_z(0.1);
    ArmorFilter armor_filter(7);

    Mat src, src_csm;
    int miss_detection_cnt = 0;

    VideoWriter writer("/home/ubuntu/VideoTest.avi",
                       CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(640, 480));

    ros::Rate rate(150);
    ROS_INFO("Image Consumer Start!");

    //armor_detector_flag
    int armor_detector_flag = 0;
    while (ros::ok())
    {
        //ROS_INFO("loop");
        if (img_recv.empty() || pre_seq == cur_seq)
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        pre_seq = cur_seq;
        img_recv.copyTo(src);
        if (setting.show_image)
        {
            src.copyTo(src_csm);
        }

        //msg init
        robo_vision::ArmorInfo msg_armor_info;
        msg_armor_info.header.stamp = ros::Time::now();
        msg_armor_info.header.frame_id = "base_link";

        ArmorTarget armor_target;
        double angle_x = 0.0, angle_y = 0.0;
        ros::Time start = ros::Time::now();
        armor_target = armor_detector.getTargetAera(src);
        ros::Time end = ros::Time::now();
        ROS_INFO("Detect Time %f", (end - start).toSec());
        if (angle_solver.getAngle(armor_target, angle_x, angle_y) == true)
        {
            miss_detection_cnt = 0;

            double z = angle_solver.position_in_camera.at<double>(2, 0);
            double y = angle_solver.position_in_camera.at<double>(1, 0);
            double x = angle_solver.position_in_camera.at<double>(0, 0);

            // publish armor info data
            if (abs(angle_x) < 10 && abs(angle_y) < 10)
            {
                msg_armor_info.mode = 3;
                armor_detector_flag = 3;
            }
            else
            {
                msg_armor_info.mode = 2;
                armor_detector_flag = 2;
            }
            msg_armor_info.pose_image.x = armor_target.center.x;
            msg_armor_info.pose_image.y = armor_target.center.y;
            msg_armor_info.pose_camera.x = x * 1.0 / 100;
            msg_armor_info.pose_camera.y = y * 1.0 / 100;
            msg_armor_info.pose_camera.z = z * 1.0 / 100;

            int const_max = 2000;

            msg_armor_info.angle.x = (angle_x + offset_anlge_x) * 100;
            ; // yaw
            msg_armor_info.angle.y = (angle_y + offset_anlge_y) * 100;
            ; // pitch
            if (msg_armor_info.angle.x > const_max)
            {
                msg_armor_info.angle.x = const_max;
            }
            if (msg_armor_info.angle.x < -const_max)
            {
                msg_armor_info.angle.x = -const_max;
            }
            pub_armor_info.publish(msg_armor_info);

            // publish global pose data
            geometry_msgs::PoseStamped armor_pose_msg;
            armor_pose_msg.header.stamp = ros::Time::now();
            armor_pose_msg.header.frame_id = "usb_camera_link";
            armor_pose_msg.pose.position.x = msg_armor_info.pose_camera.x;
            armor_pose_msg.pose.position.y = msg_armor_info.pose_camera.y;
            pub_armor_pose.publish(armor_pose_msg);

            // publish enemy tf transform
            geometry_msgs::TransformStamped enemy_trans;
            enemy_trans.header.stamp = ros::Time::now();
            enemy_trans.header.frame_id = "usb_camera_link";
            enemy_trans.child_frame_id = "enemy_pnp_link";
            enemy_trans.transform.translation.x = x * 1.0 / 100;
            enemy_trans.transform.translation.y = y * 1.0 / 100;
            enemy_trans.transform.translation.z = z * 1.0 / 100;
            enemy_trans.transform.rotation =
                tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
            enemy_pnp_tf.sendTransform(enemy_trans);

            pre_angle_x = angle_x;
            pre_angle_y = angle_y;

            cout << "armor_detector_flag = " << armor_detector_flag << endl;
            ROS_INFO("FIND ENERMY ARMOR");
        }
        else
        {
            // publish zero global pose data
            geometry_msgs::PoseStamped armor_pose_msg;
            armor_pose_msg.header.stamp = ros::Time::now();
            armor_pose_msg.header.frame_id = "usb_camera_link";
            armor_pose_msg.pose.position.x = 0;
            armor_pose_msg.pose.position.y = 0;
            armor_pose_msg.pose.position.z = 0;
            armor_pose_msg.pose.orientation.x = 0;
            armor_pose_msg.pose.orientation.y = 0;
            armor_pose_msg.pose.orientation.z = 0;
            armor_pose_msg.pose.orientation.w = 1;
            pub_armor_pose.publish(armor_pose_msg);

            // miss <5 use history data
            if (miss_detection_cnt < 5)
            {
                msg_armor_info.mode = miss_detection_cnt;
                // msg_armor_info.angle.x = (pre_angle_x + offset_anlge_x) * 100;  // yaw
                msg_armor_info.angle.y = (pre_angle_y + offset_anlge_y) * 100;
            }
            else
            {
                msg_armor_info.mode = 0;
            }

            msg_armor_info.mode = 1;
            armor_detector_flag = 1;
            pub_armor_info.publish(msg_armor_info);

            cout << "armor_detector_flag = " << armor_detector_flag << endl;
            ROS_INFO(" Miss the detctor ");
            ++miss_detection_cnt;
        }
        ros::Time end2=ros::Time::now();
        ROS_INFO("PNP Time %f",(end2-end).toSec());
        ROS_INFO("angle_x = %f , angle_y = %f", msg_armor_info.angle.x, msg_armor_info.angle.y);

        // draw result
        if (setting.show_image > 0)
        {
            circle(src_csm, image_center, 3, CV_RGB(0, 255, 0), 2);

            circle(src_csm, armor_target.ld, 2, CV_RGB(255, 255, 0), 2);
            circle(src_csm, armor_target.lu, 2, CV_RGB(255, 255, 0), 2);
            circle(src_csm, armor_target.ru, 2, CV_RGB(255, 255, 0), 2);
            circle(src_csm, armor_target.rd, 2, CV_RGB(255, 255, 0), 2);
            line(src_csm, armor_target.ld, armor_target.lu, CV_RGB(0, 255, 0),
                 1);
            line(src_csm, armor_target.lu, armor_target.ru, CV_RGB(0, 255, 0),
                 1);
            line(src_csm, armor_target.ru, armor_target.rd, CV_RGB(0, 255, 0),
                 1);
            line(src_csm, armor_target.rd, armor_target.ld, CV_RGB(0, 255, 0),
                 1);

            Mat xyz = angle_solver.position_in_camera;

            if (!xyz.empty())
            {
                char str[30];
                sprintf(str, "%.1f, %.1f", msg_armor_info.angle.x,
                        msg_armor_info.angle.y);
                putText(src_csm, str, Point(10, 40),
                        CV_FONT_HERSHEY_COMPLEX_SMALL, 1.3, CV_RGB(128, 255, 0),
                        1);
                char str2[30];
                sprintf(str2, "%.1f, %.1f, %.1f", xyz.at<double>(0, 0),
                        xyz.at<double>(1, 0), xyz.at<double>(2, 0));
                putText(src_csm, str2, Point(10, 80),
                        CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(128, 255, 0),
                        1);
                Mat s = angle_solver.position_in_ptz;
                sprintf(str2, "%.1f, %.1f, %.1f", s.at<double>(0, 0),
                        s.at<double>(0, 1), s.at<double>(0, 2));
                putText(src_csm, str2, Point(10, 120),
                        CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(128, 255, 0),
                        1);
            }

            writer << src_csm;
            imshow("result", src_csm);
            waitKey(1);
        }
        ros::spinOnce();
        rate.sleep();
    }
}
