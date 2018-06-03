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

#include "robo_vision/AngleSolver.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

bool AngleSolver::getAngle(struct ArmorTarget armor_target,double & angle_x, double & angle_y){
    if (armor_target.center.x ==0)
	return false;
    
    vector<Point2f> target2d;
    target2d.push_back(armor_target.lu);
    target2d.push_back(armor_target.ru);
    target2d.push_back(armor_target.rd);
    target2d.push_back(armor_target.ld);
    
    solvePnP4Points(target2d,rotation_in_camera,position_in_camera);
    
//     if (position_in_camera.at<double>(2, 0) < 10 || position_in_camera.at<double>(2, 0) > 800){
// 	cout << "out of range"<<endl;
// 	return false;
//     }
    tranformationCamera2PTZ(position_in_camera,position_in_ptz);
    
    adjustPTZ2Barrel(position_in_ptz,angle_x,angle_y,17);
    return true;
}

void AngleSolver::tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos){
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}
void AngleSolver::solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans){
    double half_x = armor_width / 2.0;
    double half_y = armor_height / 2.0;
    
    
    std::vector<cv::Point3f> point3d;
    point3d.push_back(Point3f(-half_x, -half_y,0));//lu
    point3d.push_back(Point3f(half_x, -half_y, 0));//ru
    point3d.push_back(Point3f(half_x, half_y, 0));//rd
    point3d.push_back(Point3f(-half_x, half_y, 0));//ld
    
    cv::Mat r;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);
    Rodrigues(r, rot);
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed){
    const double *_xyz = (const double *)pos_in_ptz.data;
    double down_t = 0.0;
    if (bullet_speed > 10e-3)
	down_t = _xyz[2] / 100.0 / bullet_speed;
    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 100;
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};
    double alpha = 0.0, theta = 0.0;
    
    angle_y=atan2(xyz[1],xyz[2]);
    angle_x = atan2(xyz[0], xyz[2]);
    //cout << "angle_x: " << angle_x << "\tangle_y: " << angle_y <<  "\talpha: " << alpha << "\ttheta: " << theta << endl;
    angle_x = angle_x * 180 / 3.1415926;
    angle_y = angle_y * 180 / 3.1415926;
}




