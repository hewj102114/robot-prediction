#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "robo_vision/RMVideoCapture.hpp"
#include "robo_vision/FishCamInfo.h"

using namespace std;
using namespace cv;
// #define SHOW_IMAGE
#define TH_RATIO 0.3
Mat img_recv_left, img_recv_right;

void image_left_callback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat img_temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    //ROS_INFO("recv");
    if (img_temp.empty())
    {
        ROS_ERROR("Could't not get image");
    }
    else
    {
#ifdef SHOW_IMAGE
        cv::imshow("view_left", img_temp);
        waitKey(1);
#endif
        img_recv_left = img_temp.clone();
    }
}

void image_right_callback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat img_temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    // ROS_INFO("recv");
    if (img_temp.empty())
    {
        ROS_ERROR("Could't not get image");
    }
    else
    {
#ifdef SHOW_IMAGE
        cv::imshow("view_right", img_temp);
#endif
        img_recv_right = img_temp.clone();
    }
}
RotatedRect adjustRRect(const RotatedRect &rect)
{
    const Size2f &s = rect.size;
    if (s.width < s.height)
        return rect;
    return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle + 90.0);
}

void prosess(Mat &img, vector<Vec3f> &pt_center)
{
    int img_size = img.cols * img.rows;
    const uchar *ptr_begin = img.data;
    const uchar *ptr_src = img.data;
    const uchar *ptr_src_end = img.data + img_size * 3;
    Mat img_rb(img.rows, img.cols, CV_8UC1);
    Mat img_gray(img.rows, img.cols, CV_8UC1);
    uchar *ptr_img_rb = img_rb.data;
    uchar *ptr_img_gray = img_gray.data;
    vector<Point2f> pt;
    for (; ptr_src != ptr_src_end; ++ptr_src)
    {
        uchar b = *ptr_src;
        uchar g = *(++ptr_src);
        uchar r = *(++ptr_src);
        if (r > 160)
            *ptr_img_gray = r;
        else
            *ptr_img_gray = 0;
        if ((r - b) > 60)
        {
            *ptr_img_rb = r - b;
        }
        else
        {
            *ptr_img_rb = 0;
        }
        ptr_img_rb++;
        ptr_img_gray++;
    }

    vector<vector<Point2i>> contours_br;
    vector<Vec4i> hierarchy;
    findContours(img_gray, contours_br, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    Mat img_contour(img.rows, img.cols, CV_8UC1);
    img_contour.setTo(Scalar(0));
    vector<vector<Point2i>>::const_iterator it = contours_br.begin();
    for (int i = 0; i < contours_br.size(); i++)
    {
        if (contours_br[i].size() < 8)
            continue;
        RotatedRect rect = minAreaRect(contours_br[i]);
        rect = adjustRRect(rect);
        //cout<<rect.angle<<endl;
        if (abs(rect.angle) < 50)
        {
            drawContours(img_contour, contours_br, i, Scalar(255, 255, 255), CV_FILLED, 8);
        }
    }
#ifdef SHOW_IMAGE
    imshow("con", img_contour);
    imshow("gray", img_gray);
    imshow("rb", img_rb);
#endif

    const uchar *ptr_con_begin = img_contour.data;
    const uchar *ptr_con_src =  img_contour.data;
    const uchar *ptr_con_end = img_contour.data + img_contour.cols * img_contour.rows;
    for (; ptr_con_src != ptr_con_end; ++ptr_con_src)
    {

        if (*ptr_con_src > 150)
        {
            int pt_y = (ptr_con_src - ptr_con_begin) / img_contour.cols;
            int pt_x = ((ptr_con_src - ptr_con_begin)) % img_contour.cols;
            if (pt_x < 10 || pt_y < 10 || pt_x > img_contour.cols - 10 || pt_y > img_contour.rows - 10)
                continue;
            Mat small_roi;
            img_rb(Rect(Point(pt_x - 10, pt_y - 10), Point(pt_x + 10, pt_y + 10))).copyTo(small_roi);
            // cout<<"rr"<<countNonZero(small_roi)<<endl;
            if (countNonZero(small_roi) > 10)
            {
                pt.push_back(Point2f(pt_x, pt_y));
            }
        }
    }

    if (pt.size() < 5)
        return;
    Mat points(pt);
    Mat labels;
    Mat centers;
    kmeans(points, 2, labels,
           TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 3,
           KMEANS_PP_CENTERS, centers);
     cout << centers << "   " << countNonZero(labels) << endl;
    if (abs(centers.at<float>(0, 0) - centers.at<float>(1, 0)) < 130 && centers.at<float>(0, 1) < 210 && centers.at<float>(1, 1) < 210)
    {
        cout<<"asd"<<endl;       
        float cx = (centers.at<float>(0, 0) + centers.at<float>(1, 0)) / 2.0;
        float cy = (centers.at<float>(0, 1) + centers.at<float>(1, 1)) / 2.0;

        pt_center.push_back(Vec3f(cx, cy, labels.rows));
    }
    else if (centers.at<float>(0, 1) > 210 || centers.at<float>(1, 1) > 210)
    {
        if (centers.at<float>(0, 1) > centers.at<float>(1, 1))
        {
            pt_center.push_back(Vec3f(centers.at<float>(0, 0), centers.at<float>(0, 1), labels.rows));
        }
        else
        {
            pt_center.push_back(Vec3f(centers.at<float>(1, 0), centers.at<float>(1, 1), labels.rows));
        }
    }
    else
    {
        pt_center.push_back(Vec3f(centers.at<float>(0, 0), centers.at<float>(0, 1), labels.rows - countNonZero(labels)));
        pt_center.push_back(Vec3f(centers.at<float>(1, 0), centers.at<float>(1, 1), countNonZero(labels)));
    }
}

int fisheye_pose_estimate(int pt_x, int pt_y, int center)
{
    if (abs(pt_x - center) < center * TH_RATIO)
        return 0;
    if (pt_x < center - center * TH_RATIO) //left
        return 1;
    if (pt_x > center + center * TH_RATIO) //right
        return -1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_fishcam");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pub_fisheye_list = nh.advertise<robo_vision::FishCamInfo>("/base/fishcam_info", 1);

    int cam_left_center, cam_left_radius, cam_left_up, cam_left_down;
    int cam_right_center, cam_right_radius, cam_right_up, cam_right_down;
    int edge_width;
    private_nh.getParam("cam_left_center", cam_left_center);
    private_nh.getParam("cam_left_radius", cam_left_radius);
    private_nh.getParam("cam_left_up", cam_left_up);
    private_nh.getParam("cam_left_down", cam_left_down);
    private_nh.getParam("cam_right_center", cam_right_center);
    private_nh.getParam("cam_right_radius", cam_right_radius);
    private_nh.getParam("cam_right_up", cam_right_up);
    private_nh.getParam("cam_right_down", cam_right_down);
    private_nh.getParam("edge_width", edge_width);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub_left =
        it.subscribe("fishcamera/left/image", 1, &image_left_callback);
    image_transport::Subscriber image_sub_right =
        it.subscribe("fishcamera/right/image", 1, &image_right_callback);

    Mat src_left, src_right, src_csm_left, src_csm_right;
    int th_ratio = 0.25;
    ros::Rate rate(10);
    ROS_INFO("Image FishEye Start!");
    while (ros::ok())
    {
        if (img_recv_left.empty() || img_recv_right.empty())
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        Rect roi_left(Point(cam_left_center - cam_left_radius, cam_left_up), Point(cam_left_center + cam_left_radius, cam_left_down));
        Rect roi_right(Point(cam_right_center - cam_right_radius, cam_right_up), Point(cam_right_center + cam_right_radius, cam_right_down));
        // cout<<img_recv_right.cols<<"   "<<cam_right_center - cam_right_radius<<"   "<<cam_right_center + cam_right_radius<<endl;
        img_recv_left(roi_left).copyTo(src_left);
        img_recv_right(roi_right).copyTo(src_right);
        rectangle(src_left, Point(0, 0), Point(edge_width, src_left.rows), Scalar(0), -1);
        rectangle(src_right, Point(src_right.cols - edge_width, 0), Point(src_right.cols, src_right.rows), Scalar(0), -1);

        vector<Vec3f> pt_center_left;
        vector<Vec3f> pt_center_right;
        prosess(src_left, pt_center_left);
        prosess(src_right, pt_center_right);
#ifdef SHOW_IMAGE
        for (int i = 0; i < pt_center_left.size(); i++)
        {
            circle(src_left, Point2f(pt_center_left[i][0], pt_center_left[i][1]), 10,
                   Scalar(255, 255, 255), 10);
        }
        for (int i = 0; i < pt_center_right.size(); i++)
        {
            circle(src_right, Point2f(pt_center_right[i][0], pt_center_right[i][1]), 10,
                   Scalar(255, 255, 255), 10);
        }
        line(src_left, Point(cam_left_radius - cam_left_radius * TH_RATIO, 0), Point(cam_left_radius - cam_left_radius * TH_RATIO, cam_left_down - cam_left_up), Scalar(255), 3);
        line(src_left, Point(cam_left_radius + cam_left_radius * TH_RATIO, 0), Point(cam_left_radius + cam_left_radius * TH_RATIO, cam_left_down - cam_left_up), Scalar(255), 3);
        line(src_right, Point(cam_right_radius - cam_right_radius * TH_RATIO, 0), Point(cam_right_radius - cam_right_radius * TH_RATIO, cam_right_down - cam_right_up), Scalar(255), 3);
        line(src_right, Point(cam_right_radius + cam_right_radius * TH_RATIO, 0), Point(cam_right_radius + cam_right_radius * TH_RATIO, cam_right_down - cam_right_up), Scalar(255), 3);

        imshow("l", src_left);
        imshow("r", src_right);
        waitKey(1);
#endif
        robo_vision::FishCamInfo fishcam_msg;
        geometry_msgs::Vector3 vec3_msg;
        fishcam_msg.header.stamp = ros::Time::now();
        fishcam_msg.header.frame_id = "base_link";

        int pose_matrix[5] = {0}; //0-90 1-135 2-180 3--135 4--90
        for (int i = 0; i < pt_center_left.size(); i++)
        {
            int ret = fisheye_pose_estimate(pt_center_left[i][0], pt_center_left[i][1], cam_left_radius);
            if (pt_center_left[i][2]<10)
                continue;
            switch (ret)
            {
            case 0: //135
            {
                pose_matrix[1] += pt_center_left[i][2];
                break;
            }
            case -1: //90
            {
                pose_matrix[0] += pt_center_left[i][2];
                break;
            }
            case 1: //180
            {
                pose_matrix[2] += pt_center_left[i][2];
                break;
            }
            }
        }
        for (int i = 0; i < pt_center_right.size(); i++)
        {
            int ret = fisheye_pose_estimate(pt_center_right[i][0], pt_center_right[i][1], cam_right_radius);
            if (pt_center_right[i][2]<10)
                continue;
            switch (ret)
            {
            case 0: //-135
            {
                pose_matrix[3] += pt_center_right[i][2];
                break;
            }
            case -1: //180
            {
                pose_matrix[2] += pt_center_right[i][2];
                break;
            }
            case 1: //-90
            {
                pose_matrix[4] += pt_center_right[i][2];
                break;
            }
            }
        }
        cout << pose_matrix[0] << "  " << pose_matrix[1] << "  " << pose_matrix[2] << "  " << pose_matrix[3] << "  " << pose_matrix[4] << endl;
        fishcam_msg.size = 0;
        for (int i = 0; i < 5; i++)
        {
            if (pose_matrix[i] > 0)
            {
                fishcam_msg.size += 1;
                geometry_msgs::Vector3 vecmsg;
                vecmsg.x = 90 + i * 45;
                if (vecmsg.x > 180)
                    vecmsg.x = vecmsg.x - 360;
                vecmsg.z = pose_matrix[i];
                fishcam_msg.target.push_back(vecmsg);
            }
        }
        pub_fisheye_list.publish(fishcam_msg);
        //         if (pt_center_left.size() + pt_center_right.size() == 1)
        //         {
        //             if (pt_center_left.size())
        //             {
        //                 int ret = fisheye_pose_estimate(pt_center_left[0][0], pt_center_left[0][1], cam_left_radius);

        //                 geometry_msgs::Vector3 vecmsg;
        //                 fishcam_msg.size = 1;
        //                 vecmsg.x = 135 + ret * 45;
        //                 vecmsg.z = pt_center_left[0][2];
        //                 fishcam_msg.target.push_back(vecmsg);
        // #ifdef COUT_PRINT
        //                 ROS_INFO("Fisheye Camera Left 1 %f", vecmsg.x);
        // #endif
        //             }
        //             else
        //             {
        //                 int ret = fisheye_pose_estimate(pt_center_right[0][0], pt_center_right[0][1], cam_right_radius);
        //                 geometry_msgs::Vector3 vecmsg;
        //                 vecmsg.x = -135 + ret * 45;
        //                 vecmsg.z = pt_center_right[0][2];
        //                 fishcam_msg.size = 1;
        //                 fishcam_msg.target.push_back(vecmsg);
        // #ifdef COUT_PRINT
        //                 ROS_INFO("Fisheye Camera Right 1 %f", vecmsg.x);
        // #endif
        //             }
        //         }
        //         else if (pt_center_left.size() + pt_center_right.size() == 2)
        //         {
        //             if (pt_center_left.size() == 2)
        //             {
        //                 int ret1 = fisheye_pose_estimate(pt_center_left[0][0], pt_center_left[0][1], cam_left_radius);
        //                 int ret2 = fisheye_pose_estimate(pt_center_left[1][0], pt_center_left[1][1], cam_left_radius);
        //                 if (ret1 == ret2)
        //                 {
        //                     geometry_msgs::Vector3 vecmsg;
        //                     vecmsg.x = 135 + ret1 * 45;
        //                     vecmsg.z = pt_center_left[0][2] + pt_center_left[1][2];
        //                     fishcam_msg.size = 1;
        //                     fishcam_msg.target.push_back(vecmsg);
        // #ifdef COUT_PRINT
        //                     ROS_INFO("Fisheye Camera Left 1 %f", vecmsg.x);
        // #endif
        //                 }
        //                 else
        //                 {
        //                     fishcam_msg.size = 2;
        //                     geometry_msgs::Vector3 vecmsg;
        //                     vecmsg.x = 135 + ret1 * 45;
        //                     vecmsg.z = pt_center_left[0][2];
        //                     fishcam_msg.target.push_back(vecmsg);
        //                     vecmsg.x = 135 + ret2 * 45;
        //                     vecmsg.z = pt_center_left[1][2];
        //                     fishcam_msg.target.push_back(vecmsg);
        // #ifdef COUT_PRINT
        //                     ROS_INFO("Fisheye Camera Left 1 %f Left 2 %f ", 135 + ret1 * 45, 135 + ret2 * 45);
        // #endif
        //                 }
        //             }
        //             else if (pt_center_right.size() == 2)
        //             {
        //                 int ret1 = fisheye_pose_estimate(pt_center_right[0][0], pt_center_right[0][1], cam_right_radius);
        //                 int ret2 = fisheye_pose_estimate(pt_center_right[1][0], pt_center_right[1][1], cam_right_radius);
        //                 if (ret1 == ret2)
        //                 {
        //                     fishcam_msg.size = 1;
        //                     geometry_msgs::Vector3 vecmsg;
        //                     vecmsg.x = -135 + ret1 * 45;
        //                     vecmsg.z = pt_center_right[0][2] + pt_center_right[1][2];
        //                     fishcam_msg.target.push_back(vecmsg);
        // #ifdef COUT_PRINT
        //                     ROS_INFO("Fisheye Camera Right 1 %f", vecmsg.x);
        // #endif
        //                 }
        //                 else
        //                 {
        //                     fishcam_msg.size = 2;
        //                     geometry_msgs::Vector3 vecmsg;
        //                     vecmsg.x = -135 + ret1 * 45;
        //                     vecmsg.z = pt_center_right[0][2];
        //                     fishcam_msg.target.push_back(vecmsg);
        //                     vecmsg.x = -135 + ret2 * 45;
        //                     vecmsg.z = pt_center_right[1][2];
        //                     fishcam_msg.target.push_back(vecmsg);
        //                 }
        //             }
        //             else
        //             {
        //                 int ret1 = fisheye_pose_estimate(pt_center_left[0][0], pt_center_left[0][1], cam_left_radius);
        //                 int ret2 = fisheye_pose_estimate(pt_center_right[0][0], pt_center_right[0][1], cam_right_radius);
        //                 if (ret1 == 1 && ret2 == -1)
        //                 {
        //                     fishcam_msg.size = 1;
        //                     geometry_msgs::Vector3 vecmsg;
        //                     vecmsg.x = 180;
        //                     vecmsg.z = pt_center_left[0][2] + pt_center_right[0][2];
        //                     fishcam_msg.target.push_back(vecmsg);
        //                 }
        //                 else
        //                 {
        //                     fishcam_msg.size = 2;
        //                     geometry_msgs::Vector3 vecmsg;
        //                     vecmsg.x = 135 + ret1 * 45;
        //                     vecmsg.z = pt_center_left[0][2];
        //                     fishcam_msg.target.push_back(vecmsg);
        //                     vecmsg.x = -135 + ret2 * 45;
        //                     vecmsg.z = pt_center_right[0][2];
        //                     fishcam_msg.target.push_back(vecmsg);
        //                 }
        //             }
        //         }

        ros::spinOnce();
        rate.sleep();
    }
}
