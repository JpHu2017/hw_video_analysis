//
// Created by cvrsg1604 on 18-7-27.
//

#ifndef CZ_VIDEO_ANALYSIS_HW_EVENT_MONITOR_H
#define CZ_VIDEO_ANALYSIS_HW_EVENT_MONITOR_H

#include <common/utils/cz_para_reader.h>
#include <memory>
#include "hw_obj_tracker.h"
namespace cz {
namespace va {
class HWEventMonitorPrivate;
class HWEventMonitor {
public:
    typedef std::shared_ptr<HWEventMonitor> SharedPtr;
    HWEventMonitor(const cz::ParaReader::Ptr& reader_ptr);
    ~HWEventMonitor();
    // 更新监测过程中的目标轨迹映射图
    void updateMonitorMap(const std::vector<HWObjTrack>& obj_tracks);
    void setMonitorRegion(const std::vector<REGION_DESC>& region_desc);
    void setSpeedReferLine(const std::vector<SPEED_REFER_LINE>& speed_refer_line);
    void setFlowReferLine(const std::map<int, std::vector<cv::Point> >& flow_refer_lines);
    void deviceEventMonitor(const double& time_stamp);
    void pedestrianEventMonitor(const double& time_stamp,const HWObjTrack& obj_track);
    void vehicleEventMonitor(const cv::Mat& frame, const double& time_stamp,const HWObjTrack& obj_track);
    std::set<int> eventAlarm();
    double vehicleSpeed() const;
    std::string vehicleType() const;
    std::pair<std::string, double> vehicleType_v2() const;
    std::map<int, int> vehicleFlow() const; //车流量
private:
    std::shared_ptr<HWEventMonitorPrivate> _private_ptr;
};
}
}

#endif //CZ_VIDEO_ANALYSIS_HW_EVENT_MONITOR_H
