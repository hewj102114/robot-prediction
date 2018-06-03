#include "robo_vision/RMVideoCapture.hpp"
#include "robo_vision/Settings.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
using namespace cv;
using namespace std;

#define SHOW_DEBUG_IMG
RMVideoCapture::CAM_PARA campara;

Mat img;
RMVideoCapture *pcap2;

static void setpara(int para,void *p)
{
    if (p==NULL)
        return;
    RMVideoCapture *pcap=(RMVideoCapture *)p;
    pcap->cam_para=campara;
    pcap->setpara();
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "test_vision");
    ros::NodeHandle nh;
    RMVideoCapture cap2("/dev/ttyVideo0");
    pcap2 = &cap2;
    cap2.info();
    cout<<"\n  Pre Parameter  \n"<<endl;
    cap2.changeVideoFormat(640, 480, 0);
    cap2.getCurrentSetting();
    
    campara=cap2.cam_para;
    
    namedWindow("img", 1);  
    createTrackbar("gain", "img",&campara.gain,100,setpara);  
    createTrackbar("exposure", "img",&campara.exposure,100,setpara,pcap2);  
    createTrackbar("brightness", "img",&campara.brightness,128,setpara,pcap2);  
    createTrackbar("whiteness", "img",&campara.whiteness,500,setpara,pcap2);  
    createTrackbar("saturation", "img",&campara.saturation,128,setpara,pcap2);  
    createTrackbar("contrast", "img",&campara.contrast,64,setpara,pcap2); 
    
    setTrackbarPos("gain", "img", cap2.cam_para.gain);
    setTrackbarPos("exposure", "img", cap2.cam_para.exposure);
    setTrackbarPos("brightness", "img", cap2.cam_para.brightness);
    setTrackbarPos("whiteness", "img", cap2.cam_para.whiteness);
    setTrackbarPos("saturation", "img", cap2.cam_para.saturation);
    setTrackbarPos("contrast", "img", cap2.cam_para.contrast);
    
    
    
    setpara(0,NULL);
    cap2.startStream();
    char c = '1';
    
    char * config_file_name = "/home/ubuntu/robot/src/robo_vision/param/param_config.xml";
    Settings setting(config_file_name);
    ArmorDetector armor_detector(setting.armor);
    
    
    while(1)
    {
        cap2>>img;


        vector<ArmorTarget> armor_target;
        armor_detector.getAllTargetAera(img, armor_target);
        for (int i = 0; i < armor_target.size(); i++)
            {
                circle(img, armor_target[i].ld, 2, CV_RGB(255, 255, 0), 2);
                circle(img, armor_target[i].lu, 2, CV_RGB(255, 255, 0), 2);
                circle(img, armor_target[i].ru, 2, CV_RGB(255, 255, 0), 2);
                circle(img, armor_target[i].rd, 2, CV_RGB(255, 255, 0), 2);
                line(img, armor_target[i].ld, armor_target[i].lu,
                     CV_RGB(0, 255, 0), 1);
                line(img, armor_target[i].lu, armor_target[i].ru,
                     CV_RGB(0, 255, 0), 1);
                line(img, armor_target[i].ru, armor_target[i].rd,
                     CV_RGB(0, 255, 0), 1);
                line(img, armor_target[i].rd, armor_target[i].ld,
                     CV_RGB(0, 255, 0), 1);
            }
        imshow("img",img);
        c = waitKey(1);
        if (c=='q')
            break;
    }
    cap2.closeStream();
    cout<<"\n  Current Parameter  \n"<<endl;
    cap2.getCurrentSetting();
}
