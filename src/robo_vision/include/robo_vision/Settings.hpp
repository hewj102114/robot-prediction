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
#include <string>
#include "ArmorDetector.hpp"

using namespace cv;

#define ARMOR_MODE 0
#define RUNE_MODE 1

struct OtherParam {
    OtherParam():angle_pitch(0.0){}
    double angle_pitch;
};

class Settings {
public:

    Settings(const std::string & filename){
        FileStorage setting_fs(filename, FileStorage::READ);
        read(setting_fs);
        setting_fs.release();
    }

	void read(const FileStorage& fs)  {
        // for debug image
        fs["show_image"] >> show_image;

		// for armor system
		fs["min_light_gray"] >> armor.min_light_gray;
		fs["min_light_height"] >> armor.min_light_height;
		fs["min_light_width"] >> armor.min_light_width;
		fs["min_light_ratio"] >> armor.min_light_ratio;
		fs["color_threshold"] >> armor.color_threshold;
		fs["max_light_delta_w"] >> armor.max_light_delta_w;
		fs["min_light_delta_w"] >> armor.min_light_delta_w;
		fs["max_light_delta_v"] >> armor.max_light_delta_v;
		fs["max_light_delta_angle"] >> armor.max_light_delta_angle;
		fs["br_threshold"] >> armor.br_threshold;
        fs["enemy_color"] >> armor.enemy_color;

        fs["min_detect_distance"] >> min_detect_distance;
        fs["max_detect_distance"] >> max_detect_distance;

        // for camerar
        fs["intrinsic_file_480"] >> intrinsic_file_480;
        fs["intrinsic_file_720"] >> intrinsic_file_720;
        fs["exposure_time"] >> exposure_time;



        fs["bullet_speed"] >> bullet_speed;
	}


public:
    int show_image;
    ArmorParam armor;
    std::string intrinsic_file_480;
    std::string intrinsic_file_720;
    int exposure_time;

    double min_detect_distance;
    double max_detect_distance;
    double bullet_speed;

};

