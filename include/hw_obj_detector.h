/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_VIDEO_ANALYSIS_HW_OBJ_DETECTOR_H
#define CZ_VIDEO_ANALYSIS_HW_OBJ_DETECTOR_H

#include <common/utils/cz_para_reader.h>
#include <czml/detectors/IDetector.h>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>
#include "hw_info.h"
namespace cz {
namespace va {
class HWObjDetectorPrivate;
class HWObjDetector {
public:
    typedef std::shared_ptr<HWObjDetector> SharedPtr;
    HWObjDetector(const cz::ParaReader::Ptr& reader_ptr);
    ~HWObjDetector();
    void setDetectRegion(const std::vector<REGION_DESC>& region_desc);
    void detect(const cv::Mat& src, std::vector<cz::ml::Rect>& hw_dets);
    void drawRects(cv::Mat& src, std::vector<cz::ml::Rect>& hw_dets);
private:
    std::shared_ptr<HWObjDetectorPrivate> _private_ptr;
};
}
}


#endif //CZ_VIDEO_ANALYSIS_HW_OBJ_DETECTOR_H
