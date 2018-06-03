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

#include "robo_vision/ArmorDetector.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <queue>
#include <vector>
#include <ros/ros.h>

#ifndef SHOW_DEBUG_IMG
//#define SHOW_DEBUG_IMG
#endif

#ifndef COUT_LOG
#define COUT_LOG
#endif

using namespace cv;
using namespace std;

void ArmorDetector::setImage(const cv::Mat &src)
{
	_size = src.size();
	const cv::Point &last_result = _res_last.center;
	if (last_result.x == 0 || last_result.y == 0)
	{
		_src = src;
		_dect_rect = Rect(0, 0, src.cols, src.rows);
	}
	else
	{
		Rect rect = _res_last.boundingRect();
		int max_half_w = src.rows == 480 ? 400 : 650;
		;
		int max_half_h = 300;
		double scale = src.rows == 480 ? 2 : 2.8;

		int exp_half_w = min(max_half_w / 2, int(rect.width * scale));
		int exp_half_h = min(max_half_h / 2, int(rect.height * 2));

		int w = std::min(max_half_w, exp_half_w);
		int h = std::min(max_half_h, exp_half_h);
		Point center = last_result;
		int x = std::max(center.x - w, 0);
		int y = std::max(center.y - h, 0);
		Point lu = Point(x, y);
		x = std::min(center.x + w, src.cols);
		y = std::min(center.y + h, src.rows);
		Point rd = Point(x, y);

		_dect_rect = Rect(lu, rd);
		if (makeRectSafe(_dect_rect, src.size()) == false)
		{
			_res_last = cv::RotatedRect();
			_dect_rect = Rect(0, 0, src.cols, src.rows);
			_src = src;
		}
		else
			src(_dect_rect).copyTo(_src);
	}

	int total_pixel = _src.cols * _src.rows;
	const uchar *ptr_src = _src.data;
	const uchar *ptr_src_end = _src.data + total_pixel * 3;
	_br = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));
	_g.create(_src.size(), CV_8UC1);
	_ec.create(_src.size(), CV_8UC1);
	_max_color = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));
	uchar *ptr_g = _g.data, *ptr_ec = _ec.data, *ptr_max_color = _max_color.data, *ptr_br = _br.data;
	if (_para.enemy_color == RED)
	{
		for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec, ++ptr_br)
		{
			uchar b = *ptr_src;
			uchar g = *(++ptr_src);
			uchar r = *(++ptr_src);
			*ptr_g = (g > 150) ? 255 : 0;
			*ptr_ec = r;
			//*ptr_g = b;
			if (r > _para.min_light_gray)
				*ptr_max_color = 255;
			if (r - b > _para.br_threshold && r >= g)
				*ptr_br = 255;
		}
	}
	else
	{
		for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec, ++ptr_br)
		{
			uchar b = *ptr_src;
			uchar g = *(++ptr_src);
			uchar r = *(++ptr_src);
			*ptr_g = (g > _para.min_light_gray) ? 255 : 0;
			*ptr_ec = b;
			//*ptr_g = r;
			if (b > _para.min_light_gray)
				*ptr_max_color = 255;
			if (b - r > _para.br_threshold && b >= g)
				*ptr_br = 255;
		}
	}

#ifdef SHOW_DEBUG_IMG
	cv::imshow("g", _g);
	cv::imshow("_max_color", _max_color);
	cv::imshow("_br", _br);
#endif
}

void ArmorDetector::findContourInEnemyColor(vector<RotatedRect> &contours_rect)
{
	vector<vector<Point2i>> contours_br;
	vector<Vec4i> hierarchy;
	findContours(_max_color, contours_br, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<vector<Point2i>>::const_iterator it = contours_br.begin();
#ifdef SHOW_DEBUG_IMG
	Mat tt = _src.clone();
#endif
	while (it != contours_br.end())
	{

		RotatedRect rect = minAreaRect(*it);

		rect = adjustRRect(rect);
		//cout<<"contour : h:"<<rect.size.height<< "w:"<<rect.size.width<<endl;
		//The smallest height of light bar
		if (rect.size.height < _para.min_light_height || rect.size.width < _para.min_light_width)
		{
			++it;
#ifdef SHOW_DEBUG_IMG
			Point2f vertices[4];
			rect.points(vertices);
			for (int i = 0; i < 4; i++)
			{
				line(tt, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255, 0), 2);//green
			}
			cout<<"contour refused 0: h:"<<rect.size.height<< "w:"<<rect.size.width<<endl;
#endif
			continue;
		}
		// _para.min_light_height
		if (rect.size.height / rect.size.width < _para.min_light_ratio)
		{
			++it;
#ifdef SHOW_DEBUG_IMG
			Point2f vertices[4];
			rect.points(vertices);
			for (int i = 0; i < 4; i++)
			{
				line(tt, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 255, 0), 2);//yellow
			}
			cout<<"contour refused 1:ratio : "<<rect.size.height/rect.size.width<<endl;
#endif
			continue;
		}
		//angle
		if (abs(rect.angle) > _para.max_light_delta_angle)
		{
			++it;
#ifdef SHOW_DEBUG_IMG
			Point2f vertices[4];
			rect.points(vertices);
			for (int i = 0; i < 4; i++)
			{
				line(tt, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255, 255), 2);//light blue
			}
			cout<<"contour refused 2:  angle: "<<rect.angle<<endl;
#endif
			continue;
		}

		int sum_color = 0;

		uchar *ptr_br = _br.data;
		vector<Point2i>::const_iterator it_pt = it->begin();
		while (it_pt != it->end())
		{
			for (int margin = 0; margin < 5; margin++)
			{
				if (it_pt->x - margin < 0 || it_pt->x + margin > _br.cols)
					break;
				if (it_pt->x < rect.center.x)
					sum_color += *(ptr_br + it_pt->x - margin + it_pt->y * _br.cols);
				else
					sum_color += *(ptr_br + it_pt->x + margin + it_pt->y * _br.cols);
			}

			it_pt++;
		}

		//color
		if (1.0 * sum_color / it->size() / 5 < _para.color_threshold)
		{
			cout<<"contour refused color  "<<sum_color<<"    "<<it->size()<<"     "<<1.0*sum_color/it->size()/5<<endl;
			++it;
			continue;
		}
  
		Rect rect2 = rect.boundingRect();
		makeRectSafe(rect2, _src.size());
		vector<vector<Point2i>> contours;
		findContours(_g(rect2), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		if (contours.size() == 1)
		{
			RotatedRect rect_contour = minAreaRect(contours[0]);

			rect_contour = adjustRRect(rect_contour);
			contours_rect.push_back(RotatedRect(rect_contour.center + Point2f(rect2.x, rect2.y), rect_contour.size, rect_contour.angle));
		}
		else if (contours.size() > 1)
		{
			int size = 0, max_id = 0;
			for (int i = 0; i < contours.size(); i++)
			{
				if (contours[i].size() > size)
				{
					size = contours[i].size();
					max_id = i;
				}
			}
			RotatedRect rect_contour = minAreaRect(contours[max_id]);
			rect_contour = adjustRRect(rect_contour);
			contours_rect.push_back(RotatedRect(rect_contour.center + Point2f(rect2.x, rect2.y), rect_contour.size, rect_contour.angle));
		}

#ifdef SHOW_DEBUG_IMG
		for (int i = 0; i < contours_rect.size(); i++)
		{
			Point2f vertices[4];
			contours_rect[i].points(vertices);
			for (int i = 0; i < 4; i++)
			{
				line(tt, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0), 2);
			}
			char str[30];
			sprintf(str, "%.1f, %.1f, %.1f",  contours_rect[i].size.width, contours_rect[i].size.height, contours_rect[i].angle);
			putText(tt, str, contours_rect[i].center, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, CV_RGB(255, 255, 255), 1);
		}
#endif
		++it;
	}

#ifdef SHOW_DEBUG_IMG
	imshow("contour", tt);
#endif
}

cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect &left, const cv::RotatedRect &right)
{
	const Point &pl = left.center, &pr = right.center;
	Point2f center = (pl + pr) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.height, wh_r.height);
	//float height = (wh_l.height + wh_r.height) / 2.0;
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

RotatedRect ArmorDetector::adjustRRect(const RotatedRect &rect)
{
	const Size2f &s = rect.size;
	if (s.width < s.height)
		return rect;
	return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle + 90.0);
}
void ArmorDetector::getRotatedRectPoint(RotatedRect rect, Point2f &lu, Point2f &ld, Point2f &ru, Point2f &rd)
{
	Point2f vertices[4];
	rect.points(vertices);
	int idx = -1;
	float sum = 10000;
	for (int i = 0; i < 4; i++)
	{
		if (vertices[i].x + vertices[i].y < sum)
		{
			sum = vertices[i].x + vertices[i].y;
			idx = i;
		}
	}
	lu = vertices[idx];
	ru = vertices[(idx + 1) % 4];
	rd = vertices[(idx + 2) % 4];
	ld = vertices[(idx + 3) % 4];
}
void ArmorDetector::findTargetInContours(const vector<RotatedRect> &contours_rect, vector<RotatedRect> &left_rects, vector<RotatedRect> &right_rects)
{
	if (contours_rect.size() < 2)
		return;
	Mat img_show = _src.clone();
	for (int i = 0; i < contours_rect.size() - 1; i++)
	{
		for (int j = i + 1; j < contours_rect.size(); j++)
		{
			double delta_angle = contours_rect[i].angle - contours_rect[j].angle;
			double delta_height = contours_rect[i].size.height - contours_rect[j].size.height;
			double delta_width = contours_rect[i].center.x - contours_rect[j].center.x;
			double delta_center = contours_rect[i].center.y - contours_rect[j].center.y;

			if (abs(delta_angle) > _para.max_light_delta_angle)
			{
				//cout<<"angle  "<<delta_angle<<endl;
				continue;
			}
			if (abs(delta_height) > min(contours_rect[i].size.height, contours_rect[j].size.height))
			{
				//cout<<"delta_height:"<<delta_height<<"  i:"<<contours_rect[i].size.height<<"  j:"<<contours_rect[j].size.height<<endl;
				continue;
			}
			if (abs(delta_width) > _para.max_light_delta_w || abs(delta_width) < _para.min_light_delta_w)
			{
				//cout<<"delta_width  "<<delta_width<<endl;
				continue;
			}
			if (abs(delta_center) > min(contours_rect[i].size.height, contours_rect[j].size.height))
			{
				//cout<<"delta_center  "<<delta_center<<endl;
				continue;
			}

			if (delta_width < 0)
			{
				left_rects.push_back(contours_rect[i]);
				right_rects.push_back(contours_rect[j]);
#ifdef SHOW_DEBUG_IMG

				Point2f vertices[4];
				contours_rect[i].points(vertices);
				for (int i = 0; i < 4; i++)
				{
					line(img_show, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0), 2);
				}
				contours_rect[j].points(vertices);
				for (int i = 0; i < 4; i++)
				{
					line(img_show, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 0, 255), 2);
				}

#endif
			}
			else
			{
				left_rects.push_back(contours_rect[j]);
				right_rects.push_back(contours_rect[i]);
#ifdef SHOW_DEBUG_IMG

				Point2f vertices[4];
				contours_rect[j].points(vertices);
				for (int i = 0; i < 4; i++)
				{
					line(img_show, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0), 2);
				}
				contours_rect[i].points(vertices);
				for (int i = 0; i < 4; i++)
				{
					line(img_show, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 0, 255), 2);
				}

#endif
			}
		}
	}
#ifdef SHOW_DEBUG_IMG
	imshow("pair", img_show);
#endif
}

void ArmorDetector::chooseAllTarget(std::vector<cv::RotatedRect> &left_rects,
                    std::vector<cv::RotatedRect> &right_rects,
                    vector<ArmorTarget> &armor_target){
	if (left_rects.size() < 1)
	{
		_is_lost = true;
		return;
	}
	for (int i = 0; i < left_rects.size(); i++)
	{
		RotatedRect rect = boundingRRect(left_rects[i], right_rects[i]);

		double wh_ratio = rect.size.width / rect.size.height;
		if (wh_ratio > 3 || wh_ratio < 1)
			continue;
		ArmorTarget armor;
		Point2f lu, ld, ru, rd;
		getRotatedRectPoint(left_rects[i], lu, ld, ru, rd);
		armor.lu = (lu+ru)/2.0;
		armor.ld = (ld+rd)/2.0;
		getRotatedRectPoint(right_rects[i], lu, ld, ru, rd);
		armor.ru =  (lu+ru)/2.0;
		armor.rd =  (ld+rd)/2.0;
		armor.center = (left_rects[i].center + right_rects[i].center) / 2;
		armor.rect = boundingRRect(left_rects[i], right_rects[i]);
		armor_target.push_back(armor);
	}

}

void ArmorDetector::chooseTarget(vector<RotatedRect> &left_rects, vector<RotatedRect> &right_rects, struct ArmorTarget &armor_target)
{

	if (left_rects.size() < 1)
	{
		_is_lost = true;
		return;
	}
	int id = -1;
	double score = 0;
	for (int i = 0; i < left_rects.size(); i++)
	{
		RotatedRect rect = boundingRRect(left_rects[i], right_rects[i]);

		double wh_ratio = rect.size.width / rect.size.height;
		// cout<<"WWWWWWWWWWWW  :  "<<wh_ratio<<endl;
		if (wh_ratio > 3 || wh_ratio < 1)
			continue;

		if (_is_lost == false && _res_last.size.width > _para.min_light_delta_w)
		{
			if (abs(rect.center.x - _res_last.center.x) >rect.size.width)
			{
				//#ifdef COUT_LOG
				//cout << "refused 0 : size_last.width: " << size_last.width << "\tcur width: "  << rect.size.width << endl;
				//#endif
				continue;
			}
		}
		if (score < rect.size.width * rect.size.height - (left_rects[i].angle - right_rects[i].angle) * 10)
		{
			score = rect.size.width * rect.size.height - (left_rects[i].angle - right_rects[i].angle) * 10;
			id = i;
		}
	}
	if (id == -1)
	{
		_is_lost = true;
		return;
	}
	else
	{
		_is_lost = false;
		Point2f lu, ld, ru, rd;
		getRotatedRectPoint(left_rects[id], lu, ld, ru, rd);
		armor_target.lu = (lu+ru)/2.0;
		armor_target.ld = (ld+rd)/2.0;
		getRotatedRectPoint(right_rects[id], lu, ld, ru, rd);
		armor_target.ru =  (lu+ru)/2.0;
		armor_target.rd =  (ld+rd)/2.0;
		armor_target.center = (left_rects[id].center + right_rects[id].center) / 2;
		armor_target.rect = boundingRRect(left_rects[id], right_rects[id]);
	}

#ifdef SHOW_DEBUG_IMG
	Mat img_show = _src.clone();
	circle(img_show, armor_target.ld, 2, CV_RGB(255, 0, 0), 2);
	circle(img_show, armor_target.lu, 2, CV_RGB(255, 0, 0), 2);
	circle(img_show, armor_target.ru, 2, CV_RGB(255, 0, 0), 2);
	circle(img_show, armor_target.rd, 2, CV_RGB(255, 0, 0), 2);
	Point2f vertices[4];
	armor_target.rect.points(vertices);
	for (int i = 0; i < 4; i++)
	{
		line(img_show, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 255, 0), 1);
	}
//imshow("pair2",img_show);
#endif
}

struct ArmorTarget ArmorDetector::getTargetAera(const cv::Mat &src)
{
	setImage(src);
	ArmorTarget armor_target;
	vector<RotatedRect> contours_rect;

	findContourInEnemyColor(contours_rect);
	vector<RotatedRect> left_rects, right_rects;
	findTargetInContours(contours_rect, left_rects, right_rects);
	//cout<<"Size "<<contours_rect.size()<<"   "<<left_rects.size()<<"   "<<right_rects.size()<<endl;

	chooseTarget(left_rects, right_rects, armor_target); //hhhhhhh

	if (armor_target.center.x != 0)
	{
		armor_target.center = armor_target.center + Point2f(_dect_rect.x, _dect_rect.y);
		armor_target.lu = armor_target.lu + Point2f(_dect_rect.x, _dect_rect.y);
		armor_target.ld = armor_target.ld + Point2f(_dect_rect.x, _dect_rect.y);
		armor_target.ru = armor_target.ru + Point2f(_dect_rect.x, _dect_rect.y);
		armor_target.rd = armor_target.rd + Point2f(_dect_rect.x, _dect_rect.y);

		//cout<<_res_last.center.x<<"     "<<armor_target.rect.center.x+_dect_rect.x<<"    "<<armor_target.rect.size.width<<endl;
		int flag = 0;
		if ((_res_last.center.x - (armor_target.rect.center.x + _dect_rect.x)) < armor_target.rect.size.width / 2)
			flag = 1;
		_res_last = RotatedRect(armor_target.rect.center + Point2f(_dect_rect.x, _dect_rect.y), armor_target.rect.size, armor_target.rect.angle);
		_res_last.size.width += _res_last.size.width * 0.5 * flag;
		_lost_cnt = 0;
	}
	else
	{
		++_lost_cnt;
		if (_lost_cnt < 2)
			_res_last.size = Size2f(_res_last.size.width * 2, _res_last.size.height * 1.5);
		else if (_lost_cnt == 3)
			_res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
		else if (_lost_cnt == 6)
			_res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
		else if (_lost_cnt == 10)
			_res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
		else if (_lost_cnt > 20)
			_res_last = RotatedRect();
	}
	return armor_target;
}


void ArmorDetector::getAllTargetAera(const cv::Mat &src,vector<ArmorTarget>& armor_target){
	ros::Time t0=ros::Time::now();
	setImage(src);
	vector<RotatedRect> contours_rect;
ros::Time t1=ros::Time::now();
	findContourInEnemyColor(contours_rect);
	ros::Time t2=ros::Time::now();
        
	vector<RotatedRect> left_rects, right_rects;
	findTargetInContours(contours_rect, left_rects, right_rects);
	//cout<<"Size "<<contours_rect.size()<<"   "<<left_rects.size()<<"   "<<right_rects.size()<<endl;
ros::Time t3=ros::Time::now();
	chooseAllTarget(left_rects, right_rects, armor_target);
	ros::Time t4=ros::Time::now();
		for(int i=0;i<armor_target.size();i++){
		armor_target[i].center = armor_target[i].center + Point2f(_dect_rect.x, _dect_rect.y);
		armor_target[i].lu = armor_target[i].lu + Point2f(_dect_rect.x, _dect_rect.y);
		armor_target[i].ld = armor_target[i].ld + Point2f(_dect_rect.x, _dect_rect.y);
		armor_target[i].ru = armor_target[i].ru + Point2f(_dect_rect.x, _dect_rect.y);
		armor_target[i].rd = armor_target[i].rd + Point2f(_dect_rect.x, _dect_rect.y);
		}
		//ROS_INFO("Time %f  %f %f %f",(t1-t0).toSec(),(t2-t1).toSec(),(t3-t2).toSec(),(t4-t3).toSec());
	
}