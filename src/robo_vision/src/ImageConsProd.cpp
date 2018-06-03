/*******************************************************************************************************************
 * Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions : 
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 * the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *******************************************************************************************************************/

#include "robo_vision/ImageConsProd.hpp"
#include "robo_vision/Predictor.hpp"
#include "robo_vision/AngleSolver.hpp"
#include "robo_vision/RMVideoCapture.hpp"
#include "robo_vision/RuneResFilter.hpp"
#include <sstream>

using namespace std;


#define VIDEO_WIDTH  1280
#define VIDEO_HEIGHT 720
#define BUFFER_SIZE 1

volatile unsigned int prdIdx;
volatile unsigned int csmIdx;
ros::Time start;
struct ImageData {
    Mat img;
    double frametime;
};

ImageData data[BUFFER_SIZE];

enum Video_Mode {
    FRAME_NONE = 0,
    FRAME_720_60 = 1,
    FRAME_480_30 = 2,
    FRAME_480_120 = 3
};
volatile Video_Mode cur_video_mode = VIDEO_HEIGHT == 720 ? FRAME_720_60 : FRAME_480_120;

void cvtRect(const cv::RotatedRect & rect, const Point2f & center1, cv::RotatedRect & cvted_rect, const Point2f & center2, double scale){
    cv::Size s(rect.size.width * scale, rect.size.height * scale);
    cv::Point2f c = (rect.center - center1) * scale + center2;
    cvted_rect = cv::RotatedRect(c, s, rect.angle);
}

ImageConsProd::ImageConsProd(Settings* _settings)
{
    pnh=new ros::NodeHandle("");
    settings = _settings;
    
    pub_armor_info=pnh->advertise<robo_vision::ArmorInfo>("base/armor_info",1);
    pub_armor_pose =
        pnh->advertise<geometry_msgs::PoseStamped>("base/armor_pose", 1);
    image_transport::ImageTransport it(*pnh);
    pub_image=it.advertise("image",5);

	robot_num=0;
	pnh->getParam("robot_num",robot_num);
}

void ImageConsProd::ImageProducer(){
    // set input source and image size
    
    RMVideoCapture cap("/dev/ttyVideo0", 1);
	cap.info();
    cap.setVideoFormat(640, 480, 1);
    cap.getCurrentSetting();
    cap.setExposureTime(0, 64);//settings->exposure_time);
    cap.startStream();

    
    start=ros::Time::now();
    ros::Duration delta_time;
    Video_Mode last_video_mode = FRAME_NONE;
	ROS_INFO("Image Producer Start!");
    while(1){
	//int t1 = cv::getTickCount();
	//cout << "resolution720: " << resolution720 << "\tmode: " << settings->mode << endl;
	ROS_INFO("%d",cur_video_mode);
	if (cur_video_mode != last_video_mode){
	    last_video_mode = cur_video_mode;
	    if(cur_video_mode == FRAME_720_60){
		cap.changeVideoFormat(1280, 720, 1);
        //cap.changeVideoFormat(640, 480, 1);
		cap.setExposureTime(0, 64);//76);
		cap.info();
	    }
	    else if (cur_video_mode == FRAME_480_30){
		cap.changeVideoFormat(640, 480, 0);
		cap.setExposureTime(0, 64);
		cap.info();
	    }
	    else if (cur_video_mode == FRAME_480_120){
		cap.changeVideoFormat(640, 480, 1);
		cap.setExposureTime(0, 64);
		//cap.setExposureTime(0, 96);
		cap.info();
	    }
	}
	
	while(prdIdx - csmIdx >= BUFFER_SIZE);
	//int t1 = cv::getTickCount();
	cap >> data[prdIdx % BUFFER_SIZE].img;
	delta_time=ros::Time::now()-start;
	data[prdIdx % BUFFER_SIZE].frametime =delta_time.toSec();
	
	cv_bridge::CvImage img_msg;
	img_msg.header.stamp=ros::Time::now();
	img_msg.header.frame_id="image";
	img_msg.image=data[prdIdx % BUFFER_SIZE].img;
	img_msg.encoding=sensor_msgs::image_encodings::BGR8;
	//pub_image.publish(img_msg.toImageMsg());
	//int t2 = cv::getTickCount();
	//cout << "Producer-Time: " << (t2 - t1) * 1000.0 / cv::getTickFrequency() << "ms\n";
	++prdIdx;
	
    }
}

void ImageConsProd::ImageConsumer(){
    Settings & setting = *settings;
    
    // load calibration parameter
	string intrinsic_file_480;
	if (robot_num==0||robot_num>2){
		ROS_ERROR("cannot get robot num!");
	}
	else if (robot_num==1){
		intrinsic_file_480=string("/home/ubuntu/Documents/whuRobot2/src/robo_vision/param/camera-01-640.xml");
	}
	else if (robot_num==2){
		intrinsic_file_480=string("/home/ubuntu/Documents/whuRobot2/src/robo_vision/param/camera-02-640.xml");
	
	}
    FileStorage fs(intrinsic_file_480, FileStorage::READ);
    if (!fs.isOpened())	{
	cout << "Could not open the configuration file: \"" << intrinsic_file_480 << "\"" << endl;
	return ;
    }
    Mat cam_matrix_480, distortion_coeff_480;
    fs["Camera_Matrix"] >> cam_matrix_480;
    fs["Distortion_Coefficients"] >> distortion_coeff_480;
    
    FileStorage fs1(setting.intrinsic_file_720, FileStorage::READ);
    if (!fs1.isOpened())	{
	cout << "Could not open the configuration file: \"" << setting.intrinsic_file_720 << "\"" << endl;
	return ;
    }
    Mat cam_matrix_720, distortion_coeff_720;
    fs1["Camera_Matrix"] >> cam_matrix_720;
    fs1["Distortion_Coefficients"] >> distortion_coeff_720;
    
    
    const double ptz_camera_y = 5;
    const double ptz_camera_z = -1;
    double r_data[] = {1,0,0,0,1,0, 0, 0,1};
    double t_data[] = {0, ptz_camera_y, ptz_camera_z}; // ptz org position in camera coodinate system
    Mat t_camera_ptz(3,1, CV_64FC1, t_data);
    Mat r_camera_ptz(3,3, CV_64FC1, r_data);
    AngleSolver solver_480(cam_matrix_480, distortion_coeff_480, t_camera_ptz,r_camera_ptz,13.5, 5.8);
    AngleSolver solver_720(cam_matrix_720, distortion_coeff_720,  t_camera_ptz,r_camera_ptz,13.5, 5.8);
    AngleSolver* angle_solver;
    
    Point2f image_center_480 = Point2f(cam_matrix_480.at<double>(0,2), cam_matrix_480.at<double>(1,2));
    Point2f image_center_720 = Point2f(cam_matrix_720.at<double>(0,2), cam_matrix_720.at<double>(1,2));
    
    
    Predictor predictor;
    // load armor detector template
    ArmorDetector armor_detector(setting.armor);
    
    FilterZ filter_z(0.1);
    ArmorFilter armor_filter(7);
    
    Mat src_csm;
    int t1 = 0, t2 = 0;
    
    
    // process loop
    double offset_anlge_x = 0.4;
    const double offset_anlge_y =0;
    double resolution_change_threshold = 160.0;
    
    Mat src;
    double frame_time = 0;
    int miss_detection_cnt = 0;
    bool flash_flag = false;
    
    
    //ros msg
    double pre_angle_x = 0.0, pre_angle_y = 0.0;
    ROS_INFO("Image Consumer Start!");
    
    VideoWriter writer("/home/ubuntu/VideoTest.avi",CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(640, 480));
    while(1){
	// waiting for image data ready
	while(prdIdx - csmIdx == 0);
	data[csmIdx % BUFFER_SIZE].img.copyTo(src);
	frame_time = data[csmIdx % BUFFER_SIZE].frametime;
	++csmIdx;
	
	if(setting.show_image){
	    t1 = cv::getTickCount();
	    src.copyTo(src_csm);
	}
	
	
	//msg init
	robo_vision::ArmorInfo msg_armor_info;
	msg_armor_info.header.stamp=ros::Time::now();
	msg_armor_info.header.frame_id="base_link";
	
	
	ArmorTarget armor_target;
	double angle_x = 0.0, angle_y = 0.0;
	double angle_x_predict=0;
	double send_data[4] = {0};
	
	//camera setting
	if (cur_video_mode == FRAME_480_30 || cur_video_mode == FRAME_NONE)
	    cur_video_mode = FRAME_480_120;
	if (!((cur_video_mode == FRAME_480_120 && src.rows == 480) || (cur_video_mode == FRAME_720_60 && src.rows == 720)))
	    continue;
	
	if (src.rows == 480){
	    armor_detector.setPara(setting.armor);
	    angle_solver=&solver_480;
	}
	else if(src.rows == 720){
	    ArmorParam armor_para_720 = setting.armor;
	    armor_para_720.max_light_delta_w = 700;
	    armor_para_720.min_light_height = 8;
	    armor_para_720.min_light_delta_w = 20;
	    armor_detector.setPara(armor_para_720);
	    angle_solver=&solver_720;
	}
	
	
	armor_target = armor_detector.getTargetAera(src);
	
	if (angle_solver->getAngle(armor_target,angle_x,angle_y) == true){
	    miss_detection_cnt = 0;
	    // using history data to predict the motion
	    
	    //predictor.setRecord(angle_x + offset_anlge_x, frame_time);
	    //angle_x_predict = predictor.predict((ros::Time::now()-start).toSec() + 0.40);
	    
	    
	    double z = angle_solver->position_in_camera.at<double>(2,0);
	    double y = angle_solver->position_in_camera.at<double>(1,0);	
	    double x = angle_solver->position_in_camera.at<double>(0,0);
	    
	    // send data to car
	    msg_armor_info.mode=1;
	    msg_armor_info.pose_image.x=armor_target.center.x;
	    msg_armor_info.pose_image.y=armor_target.center.y;
	    msg_armor_info.pose_global.position.x=x*1.0/100;
	    msg_armor_info.pose_global.position.y=y*1.0/100;
	    msg_armor_info.pose_global.position.z=z*1.0/100;
// 	    tf::Quaternion q;
// 	    double th_x=atan2( angle_solver->orientation_in_camera.at<double>(2,1),angle_solver->orientation_in_camera.at<double>(2,2));
// 	    double th_y=atan2(-angle_solver->orientation_in_camera.at<double>(2,0),sqrt(angle_solver->orientation_in_camera.at<double>(2,1)*angle_solver->orientation_in_camera.at<double>(2,1)+angle_solver->orientation_in_camera.at<double>(2,2)*angle_solver->orientation_in_camera.at<double>(2,2)));
// 	    double th_z=atan2( angle_solver->orientation_in_camera.at<double>(1,0),angle_solver->orientation_in_camera.at<double>(0,0));
        // 	    q.setRPY(th_x,th_y,th_z);
        // 	    msg_armor_info.pose_global.orientation=q;
        msg_armor_info.angle.x=(angle_x + offset_anlge_x) * 100;;//yaw
        msg_armor_info.angle.y=(angle_y + offset_anlge_y) * 100;;//pitch
        geometry_msgs::PoseStamped armor_pose_msg;
        armor_pose_msg.header.stamp=ros::Time::now();
        armor_pose_msg.header.frame_id="usb_camera_link";
        armor_pose_msg .pose= msg_armor_info.pose_global;
        pub_armor_pose.publish(armor_pose_msg);
        // cout<<"publish"<<ros::Time::now()<<endl;
        
        geometry_msgs::TransformStamped enemy_trans;
        enemy_trans.header.stamp = ros::Time::now();
        enemy_trans.header.frame_id = "usb_camera_link";
        enemy_trans.child_frame_id = "enemy_pnp_link";
        enemy_trans.transform.translation.x = x*1.0/100;
        enemy_trans.transform.translation.y = y*1.0/100;
        enemy_trans.transform.translation.z =z*1.0/100;
        enemy_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        enemy_pnp_tf.sendTransform(enemy_trans);
        
                double avg_z = filter_z.getResult(sqrt(x * x + y * y + z * z));
            /*		if (cur_video_mode == FRAME_480_120 && avg_z >
               resolution_change_threshold){ cur_video_mode = FRAME_720_60;
                                resolution_change_threshold =120;
                                offset_anlge_x=0.4;
                                const RotatedRect & last_rect =
               armor_detector.getLastResult(); RotatedRect s_last_rect;
                                cvtRect(last_rect, cv::Point2f(320, 240),
               s_last_rect, cv::Point2f(640, 360), 2.0);
                                armor_detector.setLastResult(s_last_rect);
                                cout << "capture mode changed : FRAME_720_60
               threshold: " << resolution_change_threshold
                                << "\t avg z: " << avg_z << "\n";
                            }
                            else */
            if (cur_video_mode == FRAME_720_60 &&
                avg_z < resolution_change_threshold) {
                cur_video_mode = FRAME_480_120;
                resolution_change_threshold = 180;
                offset_anlge_x = 1.2;
                const RotatedRect& last_rect = armor_detector.getLastResult();
                RotatedRect s_last_rect;
                cvtRect(last_rect, cv::Point2f(640, 360), s_last_rect,
                        cv::Point2f(320, 240), 0.5);
                armor_detector.setLastResult(s_last_rect);
                cout << "capture mode changed : FRAME_480_120   threshold: "
                     << resolution_change_threshold << "\t avg z: " << avg_z
                     << "\n";
		}
		else{
            pub_armor_info.publish(msg_armor_info);
        }
        
        pub_armor_info.publish(msg_armor_info);
	    pre_angle_x=angle_x;
        pre_angle_y=angle_y;
        ROS_INFO("FIND");
	}
	else {
	    
	    //cout<<"publish"<<ros::Time::now()<<endl;
                geometry_msgs::PoseStamped armor_pose_msg;
        armor_pose_msg.header.stamp=ros::Time::now();
        armor_pose_msg.header.frame_id="usb_camera_link";
        armor_pose_msg.pose.position.x=0;
        armor_pose_msg .pose.position.y=0;
        armor_pose_msg.pose.position.z=0;
        armor_pose_msg .pose.orientation.x=0;
                armor_pose_msg.pose.orientation.y=0;
                        armor_pose_msg .pose.orientation.z=0;
                                armor_pose_msg .pose.orientation.w=1;
        pub_armor_pose.publish(armor_pose_msg);
	    
	    if (miss_detection_cnt <5){
		msg_armor_info.mode=miss_detection_cnt;	
		msg_armor_info.angle.x=(pre_angle_x + offset_anlge_x) * 100;//yaw
		msg_armor_info.angle.y=(pre_angle_y + offset_anlge_y) * 100;
        ROS_INFO("UUUUUUUUUUUUNFIND");
        cout<<pre_angle_x<<"   "<<pre_angle_y<<endl;
	    }else{
		msg_armor_info.mode=0;
		}
	    pub_armor_info.publish(msg_armor_info);
	    
	    ++miss_detection_cnt;
	    if (miss_detection_cnt > 10){
		filter_z.clear();
	    }
	    if(miss_detection_cnt > 120 && cur_video_mode == FRAME_480_120){
		//cur_video_mode = FRAME_720_60;
		resolution_change_threshold = 130.0;
	    }
	    
	}
	t2 = cv::getTickCount();
	cout << "Consumer-Time: " << (t2 - t1) * 1000.0 / cv::getTickFrequency() << "ms   frame No.:" << frame_time << endl;
	
	
	// draw result
	if(setting.show_image > 0){
	    // show center and result
	    cv::Point2f & image_center = src_csm.rows == 720 ? image_center_720 : image_center_480;
	    circle(src_csm, image_center, 3, CV_RGB(0, 255, 0), 2);

	    circle(src_csm,armor_target.ld,2,CV_RGB(255,255,0),2);
	    circle(src_csm,armor_target.lu,2,CV_RGB(255,255,0),2);
	    circle(src_csm,armor_target.ru,2,CV_RGB(255,255,0),2);
	    circle(src_csm,armor_target.rd,2,CV_RGB(255,255,0),2);
	    line(src_csm, armor_target.ld, armor_target.lu, CV_RGB(0, 255, 0), 2);
	    line(src_csm, armor_target.lu, armor_target.ru, CV_RGB(0, 255, 0), 2);
	    line(src_csm, armor_target.ru, armor_target.rd, CV_RGB(0, 255, 0), 2);
	    line(src_csm, armor_target.rd, armor_target.ld, CV_RGB(0, 255, 0), 2);
	    
	    Mat xyz=angle_solver->position_in_camera;
	    
	    if (!xyz.empty())
	    {
		char str[30];
		sprintf(str, "%.1f, %.1f", msg_armor_info.angle.x,msg_armor_info.angle.y);
		putText(src_csm, str, Point(10, 40), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.3, CV_RGB(128, 255, 0), 1);
		char str2[30];
		sprintf(str2, "%.1f, %.1f, %.1f", xyz.at<double>(0,0),xyz.at<double>(1,0), xyz.at<double>(2,0));
		putText(src_csm, str2, Point(10, 80), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(128, 255, 0), 1);
		Mat s=angle_solver->position_in_ptz;
		sprintf(str2, "%.1f, %.1f, %.1f", s.at<double>(0,0),s.at<double>(0,1), s.at<double>(0,2));
		putText(src_csm, str2, Point(10, 120), CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(128, 255, 0), 1);
		
	    }    
	    
	    
	    Mat src_show = src_csm;
	    if (src_csm.rows == 720)
		resize(src_csm, src_show, Size(640,360));
        if (src_csm.rows == 480)
            writer << src_show;
        //imshow("result", src_show);
	    waitKey(1);
	}
	

    }
}

