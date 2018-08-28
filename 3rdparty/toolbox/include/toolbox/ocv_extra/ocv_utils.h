/**
* @Copyright (c) 2018 by JpHu
* @Date 2018-5-15
* @Company CuiZhou
* @Email hujp@cuizhouai.com
* @Function
*/

#ifndef CZ_TOOLS_UTILS_H
#define CZ_TOOLS_UTILS_H

#include <opencv2/core.hpp>
#include "shape/shape.h"

namespace cz {
namespace lt {
/// Origin OpenCV
//获取代表矩形框的点，根据提供的三种模式进行选择，头/中心/脚
enum PointOfRectMode {Head, Center, Feet};
cv::Point pointOfRect(const cv::Rect& r, PointOfRectMode mode = PointOfRectMode::Feet);
//确保矩形框在图片内部
bool edgeCheck(const cv::Rect& src_r, cv::Rect& dst_r, const int& w, const int& h);

/// Shape defined by myself
//获取代表矩形框的点，根据提供的三种模式进行选择，头/中心/脚
Point pointOfRect(const Rect& r, PointOfRectMode mode = PointOfRectMode::Feet);
//确保矩形框在图片内部
bool edgeCheck(const Rect& src_r, Rect& dst_r, const int& w, const int& h);
}
}


#endif //CZ_TOOLS_UTILS_H
