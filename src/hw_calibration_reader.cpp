/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */
#include "hw_calibration_reader.h"
#include <iostream>
#include <fstream>
#include <glog/logging.h>
namespace cz {
namespace va {
using namespace std;
using namespace tinyxml2;

HWCalibrationReader::HWCalibrationReader(const std::string& xml_path, const std::string& camera_IP)
        :camera_IP_(camera_IP)
{
    //获取根节点，即persons节点
    doc_ = new tinyxml2::XMLDocument();
    if (doc_->LoadFile(xml_path.c_str()) != 0)
    {
        string err_info = "读取XML文件出现错误!";
        cout << err_info << endl;
    }
    XMLElement* root = doc_->RootElement();
    XMLElement* item = root->FirstChildElement("相机");

    camera_root_ = NULL;
    while (item)
    {
        string ip(item->FirstChildElement("相机ip")->GetText());
        if (ip == camera_IP_)
        {
            camera_root_ = item;
            break;
        }
        item = item->NextSiblingElement();
    }
    if (!camera_root_)
    {
        string err_info = "配置文件中不存在IP为" + camera_IP_ + "的相机";
        cout << err_info << endl;
    }
}


string HWCalibrationReader::findNodeValue(string node_name)
{
    XMLElement* elem = camera_root_->FirstChildElement(node_name.c_str());
    if (!elem)
    {
        string err_info = "配置文件中" + camera_IP_ + "的相机" + "没有节点*" + node_name + "*";
        cout << err_info << endl;
    }
    return elem->GetText();
}
//获取监控区域
void HWCalibrationReader::getMonitorRegion(std::vector<cv::Point>& monitor_region) {
    monitor_region.clear();
    //监测区域
    XMLElement* item = camera_root_->FirstChildElement("监测区域");
    if(!item)
    {
        string err_info = "相机" + camera_IP_ + "未设置监测区域信息";
        cout << err_info << endl;
    }
    string info_tmp = item->GetText();
    size_t pos = info_tmp.find(';');
    info_tmp = info_tmp.substr(pos + 1);
    pos = info_tmp.find(';');
    for(; pos != info_tmp.npos; pos = info_tmp.find(';'))
    {
        string pt_str = info_tmp.substr(0, pos);
        size_t pos_d = pt_str.find(',');
        int x = atoi(pt_str.substr(0, pos_d).c_str());
        int y = atoi(pt_str.substr(pos_d + 1).c_str());
        monitor_region.emplace_back(cv::Point(x,y));
        info_tmp = info_tmp.substr(pos + 1);
    }
    size_t pos_d = info_tmp.find(',');
    int x = atoi(info_tmp.substr(0, pos_d).c_str());
    int y = atoi(info_tmp.substr(pos_d + 1).c_str());
    monitor_region.emplace_back(cv::Point(x,y));
}
void HWCalibrationReader::getDescs(std::vector<REGION_DESC>& region_descs, std::vector<SPEED_REFER_LINE>& speed_refer_lines) {
    getSpeedReferLines(speed_refer_lines);
    getRegionDescs(region_descs);
    //设置区域的方向信息
    for(int i=0; i<region_descs.size(); ++i) {
        region_descs[i].s = _directions[region_descs[i].direct].first;
        region_descs[i].e = _directions[region_descs[i].direct].second;
    }
}
void HWCalibrationReader::getFlowReferLines(std::map<int, std::vector<cv::Point> >& flow_refer_lines) {
    XMLElement* item = camera_root_->FirstChildElement("车流量统计线");
    if (!item)
    {
        string err_info = "相机" + camera_IP_ + "未设置车流量统计线";
        cout << err_info << endl;
    }
    flow_refer_lines.clear();
    XMLElement* flow_refer_line_item = item->FirstChildElement("车流量统计线属性与位置");
    while(flow_refer_line_item)
    {
        string info = flow_refer_line_item->GetText();
        std::pair<int, std::vector<cv::Point> > flow_refer_line;
        parseFlowReferInfo(info, flow_refer_line);
        double refer_distance = 0.0;
        flow_refer_lines.insert(flow_refer_line);
        flow_refer_line_item = flow_refer_line_item->NextSiblingElement("车流量统计线属性与位置");
    }
}
// 获取速度参考线
void HWCalibrationReader::getSpeedReferLines(std::vector<SPEED_REFER_LINE>& speed_refer_lines) {
    XMLElement* item = camera_root_->FirstChildElement("速度参考线");
    if (!item)
    {
        string err_info = "相机" + camera_IP_ + "未设置速度参考线";
        cout << err_info << endl;
    }
    speed_refer_lines.clear();
    XMLElement* speed_referency_line_cell_length = item->FirstChildElement("速度参考线单位距离");
    double cell_length = atof(speed_referency_line_cell_length->GetText());
    XMLElement* speed_referency_line_item = item->FirstChildElement("速度参考线属性与位置");
    while(speed_referency_line_item)
    {
        string info = speed_referency_line_item->GetText();
        SPEED_REFER_LINE speed_refer_line;
        parseSpeedReferLineInfo(info, speed_refer_line);
        double refer_distance = 0.0;
        for(int i=0; i<speed_refer_line.nPointCount; ++i) {
            speed_refer_line.referDistance[i] = refer_distance;
            refer_distance += cell_length;
        }
        speed_refer_lines.emplace_back(speed_refer_line);
        speed_referency_line_item = speed_referency_line_item->NextSiblingElement("速度参考线属性与位置");
    }
}
// 获得区域描述信息
void HWCalibrationReader::getRegionDescs(std::vector<REGION_DESC>& region_descs) {
    //行车道
    XMLElement* item = camera_root_->FirstChildElement("行车道");
    if(!item)
    {
        string err_info = "相机" + camera_IP_ + "未设置行车道信息";
        cout << err_info << endl;
    }

    XMLElement* lanes_item = item->FirstChildElement("行车道属性及位置");
    while(lanes_item)
    {

        string info = lanes_item->GetText();
        REGION_DESC region_desc;
        parseRegionDescInfo(info, region_desc);
        region_desc.regionProp = 0; //行车道
        region_descs.emplace_back(region_desc);
        lanes_item = lanes_item->NextSiblingElement("行车道属性及位置");
    }
    //应急车道
    XMLElement* item2 = camera_root_->FirstChildElement("应急车道");
    if(!item2)
    {
        string err_info = "相机" + camera_IP_ + "未设置应急车道信息";
        cout << err_info << endl;
    }
    XMLElement* emergency_lanes_item = item2->FirstChildElement("应急车道属性及位置");
    while(emergency_lanes_item)
    {
        string info = emergency_lanes_item->GetText();
        REGION_DESC region_desc;
        parseRegionDescInfo(info, region_desc);
        region_desc.regionProp = 1; //行车道
        region_descs.emplace_back(region_desc);
        emergency_lanes_item = emergency_lanes_item->NextSiblingElement("应急车道属性及位置");
    }
}

void HWCalibrationReader::parseSpeedReferLineInfo(const std::string& info, SPEED_REFER_LINE& speed_refer_line) {
    string info_tmp = info;
    size_t pos = info_tmp.find(';');
    int property = atoi(info_tmp.substr(0, pos).c_str());
    speed_refer_line.direct = property;
//    if(property == 0 | property == 1) {
//        speed_refer_line.direct = 0;
//    } else if(property == 2 | property == 3) {
//        speed_refer_line.direct = 1;
//    } else {
//        LOG(FATAL) << "Speed refer line no the property.";
//    }
    speed_refer_line.nPointCount = 0;
    info_tmp = info_tmp.substr(pos + 1);
    pos = info_tmp.find(';');
    for(; pos != info_tmp.npos; pos = info_tmp.find(';'))
    {
        ++speed_refer_line.nPointCount;
        string pt_str = info_tmp.substr(0, pos);
        size_t pos_d = pt_str.find(',');
        int x = atoi(pt_str.substr(0, pos_d).c_str());
        int y = atoi(pt_str.substr(pos_d + 1).c_str());
        speed_refer_line.pt[speed_refer_line.nPointCount-1] = POINT(x,y);

        info_tmp = info_tmp.substr(pos + 1);
    }
    ++speed_refer_line.nPointCount;
    size_t pos_d = info_tmp.find(',');
    int x = atoi(info_tmp.substr(0, pos_d).c_str());
    int y = atoi(info_tmp.substr(pos_d + 1).c_str());
    speed_refer_line.pt[speed_refer_line.nPointCount-1] = POINT(x,y);
    // 方向信息
    speed_refer_line.s = speed_refer_line.pt[0];
    speed_refer_line.e = speed_refer_line.pt[speed_refer_line.nPointCount-1];
    _directions[speed_refer_line.direct] = std::pair<POINT,POINT>(speed_refer_line.s, speed_refer_line.e);
}
void HWCalibrationReader::parseRegionDescInfo(const std::string& info, REGION_DESC& region_desc) {
    string info_tmp = info;
    size_t pos = info_tmp.find(';');
    int property = atoi(info_tmp.substr(0, pos).c_str());
    if(property == 0 | property == 1) {
        region_desc.direct = property;
    }  else {
        LOG(FATAL) << "Region desc no the property.";
    }
    region_desc.nPointCount = 0;
    info_tmp = info_tmp.substr(pos + 1);
    pos = info_tmp.find(';');
    for(; pos != info_tmp.npos; pos = info_tmp.find(';'))
    {
        ++region_desc.nPointCount;
        string pt_str = info_tmp.substr(0, pos);
        size_t pos_d = pt_str.find(',');
        int x = atoi(pt_str.substr(0, pos_d).c_str());
        int y = atoi(pt_str.substr(pos_d + 1).c_str());
        region_desc.pt[region_desc.nPointCount-1] = POINT(x,y);
        info_tmp = info_tmp.substr(pos + 1);
    }
    ++region_desc.nPointCount;
    size_t pos_d = info_tmp.find(',');
    int x = atoi(info_tmp.substr(0, pos_d).c_str());
    int y = atoi(info_tmp.substr(pos_d + 1).c_str());
    region_desc.pt[region_desc.nPointCount-1] = POINT(x,y);
}
void HWCalibrationReader::parseFlowReferInfo(const std::string& info,
                                             std::pair<int, std::vector<cv::Point> >& flow_refer_line) {
    string info_tmp = info;
    size_t pos = info_tmp.find(';');
    int property = atoi(info_tmp.substr(0, pos).c_str());
    if(property == 0 | property == 1) {
        /*no operation*/
    }  else {
        LOG(FATAL) << "Region desc no the property.";
    }
    std::vector<cv::Point> line;
    info_tmp = info_tmp.substr(pos + 1);
    pos = info_tmp.find(';');
    for(; pos != info_tmp.npos; pos = info_tmp.find(';'))
    {
        string pt_str = info_tmp.substr(0, pos);
        size_t pos_d = pt_str.find(',');
        int x = atoi(pt_str.substr(0, pos_d).c_str());
        int y = atoi(pt_str.substr(pos_d + 1).c_str());
        line.emplace_back(cv::Point(x,y));
        info_tmp = info_tmp.substr(pos + 1);
    }
    size_t pos_d = info_tmp.find(',');
    int x = atoi(info_tmp.substr(0, pos_d).c_str());
    int y = atoi(info_tmp.substr(pos_d + 1).c_str());
    line.emplace_back(cv::Point(x,y));
    flow_refer_line = std::make_pair(property, line);
}
}
}
