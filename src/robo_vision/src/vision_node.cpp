#include "robo_vision/ImageConsProd.hpp"
#include "robo_vision/Settings.hpp"

#include <thread>
#include <unistd.h>
#include "robo_vision/RMVideoCapture.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <ros/ros.h>

using namespace cv;
using namespace std;


int main(int argc, char * argv[]){


    ros::init(argc,argv,"robo_vision");
    ros::NodeHandle nh;
    char * config_file_name = "/home/ubuntu/Documents/whuRobot2/src/robo_vision/param/param_config.xml";
    Settings setting(config_file_name);

    // start threads
    ImageConsProd image_cons_prod(&setting);
    std::thread t1(&ImageConsProd::ImageProducer, image_cons_prod); // pass by reference
    std::thread t2(&ImageConsProd::ImageConsumer, image_cons_prod);

	ros::spin();
    
 //   while(ros::ok())
//	ros::spinOnce();
}
