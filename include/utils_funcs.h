/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-28
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_VIDEO_ANALYSIS_UTILS_FUNCS_H
#define CZ_VIDEO_ANALYSIS_UTILS_FUNCS_H
#include <opencv2/core.hpp>
#include <czml/detectors/IDetector.h>
namespace cz {
namespace va {
float getIOU(const cv::Rect &rect1, const cv::Rect &rect2);
void nmsDiffClasses(cz::ml::RectVec &dets);
}
}
#endif //CZ_VIDEO_ANALYSIS_UTILS_FUNCS_H
