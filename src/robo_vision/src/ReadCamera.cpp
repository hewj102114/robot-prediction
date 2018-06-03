#include <iostream>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "robo_vision/RMVideoCapture.hpp"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "robo_vision_readcam");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string image_topic="image";
    string dev_name="/dev/video0";
    int image_width=640,image_height=480,exposure_time=64;

    private_nh.getParam("image_topic",image_topic);
    private_nh.getParam("dev_name",dev_name);

    private_nh.getParam("image_width",image_width);
    private_nh.getParam("image_height",image_height);
    private_nh.getParam("exposure_time",exposure_time);
    cout<<image_topic<<endl;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise(image_topic, 1);

    RMVideoCapture cap(dev_name.c_str(),3);
    cap.info();
    cap.setVideoFormat(image_width, image_height, 1);
    cap.getCurrentSetting();
    cap.setExposureTime(0, exposure_time);  // settings->exposure_time);
    cap.startStream();
    ROS_INFO("[robo_vision_readcam -- %s ]Image Producer Start!",dev_name.c_str());
    cv::Mat img;
    while (ros::ok()) {
        ros::Time start=ros::Time::now();
        cap >> img;
        ros::Time end =ros::Time::now();
        ros::Duration dua=end-start;
       // ROS_INFO("TTT  %f",dua.toSec());
        cv_bridge::CvImage img_msg;
        img_msg.header.stamp = ros::Time::now();
        img_msg.header.frame_id = "image";
        img_msg.image = img;
        img_msg.encoding = sensor_msgs::image_encodings::BGR8;
        pub_image.publish(img_msg.toImageMsg());
    }
    cap.closeStream();
}
