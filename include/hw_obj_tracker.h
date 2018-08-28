/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_VIDEO_ANALYSIS_HW_OBJ_TRACKER_H
#define CZ_VIDEO_ANALYSIS_HW_OBJ_TRACKER_H

#include <common/utils/cz_para_reader.h>
#include <czml/detectors/IDetector.h>
#include <memory>
#include <tracker/mot.hpp>
#include "hw_info.h"

namespace cz {
namespace va {
struct HWObjTrack {
    int id;
    bool curr_update; //当前更新
    cv::Rect curr_rect; //当前矩形框
    // 短期轨迹
    std::vector<std::pair<Time::UINT, cv::Rect> > short_term_locations;
    // 长期轨迹
    std::vector<std::pair<Time::UINT, cv::Rect> > long_term_locations;
    enum StayState {confirmed, tensitive, deleted};
};

class HWObjTrackerPrivate;
class HWObjTracker {
public:
    typedef std::shared_ptr<HWObjTracker> SharedPtr;
    HWObjTracker(const cz::ParaReader::Ptr& reader_ptr);
    ~HWObjTracker();
    void setTrackRegion(const std::vector<REGION_DESC>& region_desc);
    void track(const cv::Mat& frame, const Time::UINT& time_stamp,
               const std::vector<cz::ml::Rect>& obj_dets, std::vector<HWObjTrack>& obj_tracks);
private:
    std::shared_ptr<HWObjTrackerPrivate> _private_ptr;
};
}
}


#endif //CZ_VIDEO_ANALYSIS_HW_OBJ_TRACKER_H
