/*******************************************************************************************************************
 * Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files(the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions :
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED
 *"AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 *LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 *AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 *LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *******************************************************************************************************************/

#pragma once

#include "AngleSolver.hpp"
#include "opencv2/highgui/highgui.hpp"

#define TRUNC_ABS(a) ((a) > 0 ? (a) : 0);
#define POINT_DIST(p1, p2) \
  std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))

enum EnemyColor { RED = 0, BLUE = 1 };

struct ArmorParam {
  float min_light_gray;    // 灯条最小灰度值 g
  float min_light_height;  // 板灯最小高度值 g
  float min_light_width;
  float min_light_ratio;
  uchar color_threshold;  // g
  int max_light_delta_w;  // 左右灯柱在水平位置上的最大差值，像素单位 g
  uchar min_light_delta_w;  // 左右灯柱在水平位置上的最小差值，像素单位 g
  uchar max_light_delta_v;  // 左右灯柱在垂直位置上的最大差值，像素单位 g
  uchar max_light_delta_angle;  // 左右灯柱在斜率最大差值，单位度 g
  uchar br_threshold;           // 红蓝通道相减后的阈值 g
  uchar enemy_color;            // 0 for red, otherwise blue

  ArmorParam() {
    min_light_gray = 210;
    min_light_height = 8;
    min_light_width = 3;
    color_threshold = 110;
    max_light_delta_w = 450;
    min_light_delta_w = 12;
    max_light_delta_v = 50;
    max_light_delta_angle = 30;
    br_threshold = 30;
    enemy_color = 0;
  }
};

struct ArmorTarget {
  cv::Point2f center;
  cv::Point2f lu, ld, ru, rd;
  cv::RotatedRect rect;
  ArmorTarget() {
    center = cv::Point2f();
    lu = cv::Point2f();
    ld = cv::Point2f();
    ru = cv::Point2f();
    rd = cv::Point2f();
    rect = cv::RotatedRect();
  }
};

class ArmorDetector {
 public:
  ArmorDetector(const ArmorParam &para = ArmorParam()) {
    _para = para;
    _res_last = cv::RotatedRect();
    _dect_rect = cv::Rect();
    _lost_cnt = 0;
    _is_lost = true;
  }
  void setPara(const ArmorParam &para) { _para = para; }

  void reset() {
    _res_last = cv::RotatedRect();
    _dect_rect = cv::Rect();
    _lost_cnt = 0;
    _is_lost = true;
  }

  struct ArmorTarget getTargetAera(const cv::Mat &src);
  void getAllTargetAera(const cv::Mat &src,std::vector<ArmorTarget>& armor_target);
  void setLastResult(const cv::RotatedRect &rect) { _res_last = rect; }
  const cv::RotatedRect &getLastResult() const { return _res_last; }

 private:
  /**
   * @brief setImage Pocess the input (set the green component and sub of blue
   * and red component)
   * @param src
   */
  void setImage(const cv::Mat &src);

  /**
   * @brief findContourInEnemyColor Find contour in _max_color
   * @param left output left contour image (probably left lamp of armor)
   * @param right output righe edge (probably right lamp of armor)
   * @param contours_left output left contour
   * @param contours_right output right contour
   */

  void findContourInEnemyColor(std::vector<cv::RotatedRect> &contours_rect);

  /**
   * @brief findTargetInContours Find target rectangles
   * @param contours_left input left contour
   * @param contours_right input right contour
   * @param rects target rectangles (contains wrong area)
   */
  void findTargetInContours(const std::vector<cv::RotatedRect> &contours_rect,
                            std::vector<cv::RotatedRect> &left_rects,
                            std::vector<cv::RotatedRect> &right_rects);
  /**
   * @brief chooseTarget Choose the most possible rectangle among all the
   * rectangles
   * @param rects candidate rectangles
   * @return the most likely armor (RotatedRect() returned if no proper one)
   */
  void chooseTarget(std::vector<cv::RotatedRect> &left_rects,
                    std::vector<cv::RotatedRect> &right_rects,
                    struct ArmorTarget &armor_target);

  void chooseAllTarget(std::vector<cv::RotatedRect> &left_rects,
                    std::vector<cv::RotatedRect> &right_rects,
                    std::vector<ArmorTarget> &armor_target);
  /**
   * @brief boundingRRect Bounding of two ratate rectangle (minumum area that
   * contacts two inputs)
   * @param left left RotatedRect
   * @param right right RotatedRect
   * @return minumum area that contacts two inputs
   */
  cv::RotatedRect boundingRRect(const cv::RotatedRect &left,
                                const cv::RotatedRect &right);

  /**
   * @brief adjustRRect Adjust input angle
   * @param rect input
   * @return adjusted rotate rectangle
   */
  cv::RotatedRect adjustRRect(const cv::RotatedRect &rect);
  void getRotatedRectPoint(cv::RotatedRect rect, cv::Point2f &lu,
                           cv::Point2f &ld, cv::Point2f &ru, cv::Point2f &rd);
  bool makeRectSafe(cv::Rect &rect, cv::Size size) {
    if (rect.x < 0) rect.x = 0;
    if (rect.x + rect.width > size.width) rect.width = size.width - rect.x;
    if (rect.y < 0) rect.y = 0;
    if (rect.y + rect.height > size.height) rect.height = size.height - rect.y;
    if (rect.width <= 0 || rect.height <= 0) return false;
    return true;
  }

  bool broadenRect(cv::Rect &rect, int width_added, int height_added,
                   cv::Size size) {
    rect.x -= width_added;
    rect.width += width_added * 2;
    rect.y -= height_added;
    rect.height += height_added * 2;
    return makeRectSafe(rect, size);
  }

 private:
  bool _is_lost;
  int _lost_cnt;
  cv::RotatedRect _res_last;  // last detect result
  cv::Rect _dect_rect;        // detect roi of original image
  ArmorParam _para;           // parameter of alg
  cv::Mat _src;               // source image
  cv::Mat _g;                 // green component of source image
  cv::Mat _ec;                // enemy color
  cv::Mat _max_color;  // binary image of sub between blue and red component
  cv::Mat _br;
  cv::Size _size;
  //    cv::Mat _gray;
  //    cv::Mat _b;
  //    cv::Mat _r;
};
