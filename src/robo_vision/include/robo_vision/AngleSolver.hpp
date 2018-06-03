/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#pragma once
#include "opencv2/core/core.hpp"
#include "ArmorDetector.hpp"


class AngleSolver{
public:
    AngleSolver(const cv::Mat & camera_matrix, const cv::Mat & dist_coeff,
		 const cv::Mat & t_camera2ptz, const cv::Mat & r_camera2ptz,double target_width = 0, double target_height = 0){
	camera_matrix.copyTo(cam_matrix);
	dist_coeff.copyTo(distortion_coeff);
	t_camera2ptz.copyTo(trans_camera2ptz);
	r_camera2ptz.copyTo(rot_camera2ptz);
	armor_width=target_width;
	armor_height=target_height;
    };
    void solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);
    bool getAngle(struct ArmorTarget armor_target,double & angle_x, double & angle_y);
    void tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos);
    void adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed = 0.0);
    cv::Mat position_in_camera;
    cv::Mat position_in_ptz;
    cv::Mat rotation_in_camera;
private:
    double armor_width;
    double armor_height;
    cv::Mat cam_matrix,distortion_coeff;

    cv::Mat trans_camera2ptz;
    cv::Mat rot_camera2ptz;
};

