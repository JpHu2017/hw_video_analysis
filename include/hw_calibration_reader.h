/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_VIDEO_ANALYSIS_HW_CALIBRATION_READER_H
#define CZ_VIDEO_ANALYSIS_HW_CALIBRATION_READER_H
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include "hw_info.h"
#include "contrib/tinyxml2.h"

namespace cz {
namespace va {
class HWCalibrationReader {
public:
    HWCalibrationReader(){}
    HWCalibrationReader(const std::string& xml_path, const std::string& camera_IP);

    std::string place()
    {
        return findNodeValue("路名");
    }
    std::string cameraIP()
    {
        return findNodeValue("相机ip");
    }
    std::string portNum()
    {
        return findNodeValue("相机端口");
    }
    std::string userName()
    {
        return findNodeValue("用户名");
    }
    std::string password()
    {
        return findNodeValue("密码");
    }
    std::string cameraBrand()
    {
        return findNodeValue("相机品牌类型");
    }
    std::pair<int, std::string> channelNumber()
    {
        std::string channel_number = findNodeValue("通道号");
        int index = channel_number.find(',');
        return std::make_pair(atoi(channel_number.substr(0, index).c_str()), channel_number.substr(index + 1).c_str());
    }
    void getMonitorRegion(std::vector<cv::Point>& monitor_region);
    void getDescs(std::vector<REGION_DESC>& region_descs, std::vector<SPEED_REFER_LINE>& speed_refer_lines);
    void getFlowReferLines(std::map<int, std::vector<cv::Point> >& flow_refer_lines);
private:
    // 获取速度参考线
    void getSpeedReferLines(std::vector<SPEED_REFER_LINE>& speed_refer_lines);
    // 获得区域描述信息
    void getRegionDescs(std::vector<REGION_DESC>& region_descs);

    std::string findNodeValue(std::string node_name);
    void parseSpeedReferLineInfo(const std::string& info, SPEED_REFER_LINE& speed_refer_line);
    void parseRegionDescInfo(const std::string& info, REGION_DESC& region_desc);
    void parseFlowReferInfo(const std::string& info, std::pair<int, std::vector<cv::Point> >& flow_refer_line);
private:
    //xml读取工具
    tinyxml2::XMLDocument* doc_;
    tinyxml2::XMLElement* camera_root_; //根节点
    std::string camera_IP_;

    std::pair<POINT, POINT> _directions[2];
};
}
}

#endif //CZ_VIDEO_ANALYSIS_HW_CALIBRATION_READER_H
