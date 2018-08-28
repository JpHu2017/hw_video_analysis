/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-28
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#include <toolbox/ocv_extra.hpp>
#include <toolbox/shape.hpp>
#include <czml/classifiers/IClassifier.h>
#include "hw_event_monitor.h"
namespace cz {
namespace va {
class HWEventMonitorPrivate {
public:
    typedef std::shared_ptr<HWEventMonitorPrivate> SharedPtr;
    HWEventMonitorPrivate(const cz::ParaReader::Ptr& reader_ptr);
    ~HWEventMonitorPrivate();
    void updateMonitorMap(const std::vector<HWObjTrack>& obj_tracks);
    void setMonitorRegion(const std::vector<REGION_DESC>& region_desc);
    void setSpeedReferLine(const std::vector<SPEED_REFER_LINE>& speed_refer_line);
    void setFlowReferLine(const std::map<int, std::vector<cv::Point> >& flow_refer_lines);
    void deviceEventMonitor(const double& time_stamp);
    void pedestrianEventMonitor(const double& time_stamp,const HWObjTrack& obj_track);
    void vehicleEventMonitor(const cv::Mat& frame, const double& time_stamp, const HWObjTrack& obj_track);
    std::set<int> eventAlarm();
    double vehicleSpeed() const;
    std::string vehicleType() const;
    std::pair<std::string, double> vehicleType_v2() const;
    std::map<int, int> vehicleFlow() const; //车流量
private:
    void init();
    void setParameter();
    void updateVehicleTypeMonitorMap(const std::set<int> obj_track_ids,
                                     const std::vector<HWObjTrack>& obj_tracks);
    void vehicleTypeMonitor(const cv::Mat& frame, const HWObjTrack& obj_track);
    void updateVehicleFlowMonitorMap(const std::set<int> obj_track_ids,
                                     const std::vector<HWObjTrack>& obj_tracks);
    void getMotorFlowStartSign();
    void getMotorFlowStartSignAux(const std::vector<cv::Point>& flow_refer_line, const int& direct, int& start_sign);
    void vehicleFlowMonitor(const HWObjTrack& obj_track, const int& direct);
    void speedMonitor(const double& time_stamp,const HWObjTrack& obj_track, const int& direct); //速度
    void illegalParkMonitor(const double& time_stamp, const HWObjTrack& obj_track, const int& region_prop); //违章停车
    void retrogradeMonitor(const double& time_stamp,const HWObjTrack& obj_track, const int& direct, const int& region_prop); //逆行
    void abandonMonitor(const double& time_stamp,const HWObjTrack& obj_track); //遗留物
    void sprinklingMonitor(const double& time_stamp,const HWObjTrack& obj_track); //抛洒物
    void illegalPassMonitor(const double& time_stamp,const HWObjTrack& obj_track);
    void deviceExceptionMonitor(const double& time_stamp);
    bool getLocLane(const cz::lt::Point& pt, int& direct, int& region_prop, const int& dist_from_board);
    bool computeSpeed(const cz::lt::Point& curr_pt, const cz::lt::Point& last_pt,
                      const double& time_interval, const int& direct, double& speed);
    std::set<int> _event_alarms;
    double _speed;
    std::string _vehicle_type;
    std::pair<std::string, float> _vehicle_type_v2;
    std::map<int, int> _vehicle_flow;
    
    std::map<int, cz::lt::Polygon > _motor_lane_poly_map;
    std::map<int, cz::lt::Polygon > _eme_lane_poly_map;
    std::map<int, std::vector<cz::lt::Segment> > _speed_refer_line_map;
    std::map<int, std::vector<double> > _speed_refer_distance_map;
    std::map<int, std::pair<cv::Point, cv::Point> > _drive_direction_map;

    bool _bIllegalParking;
    bool _bAbandonedDetect;
    bool _bSprinklingDetect;
    bool _bPedestrianDetect;
    bool _bRetrograde;
    bool _bDeviceException;
    // motor type
    std::string _motor_type_pb;
    bool _motor_type_use_gpu;
    int _motor_type_gpu_id;
    double _motor_type_score_thresh;
    std::map<int, std::pair<std::string, double> > _motor_type_map;
    cz::ml::IClassifier::Ptr _motor_type_classifier_ptr;
    // flow
    std::map<int ,std::vector<cv::Point> > _flow_refer_lines;
    struct ObjFlowInfo {
        bool start_count;
        bool end_count;
        bool is_count;
        ObjFlowInfo() : start_count(false), end_count(false), is_count(false) {}
    };
    std::map<int, ObjFlowInfo> _motor_flow_map;
    std::map<int, int> _motor_flow_start_sign; //车流量统计开始一侧的符号

    // 速度测量时间间隔
    double _measure_speed_interval;
    // 行车道允许车辆停留时间(0 : 表明只要有停留，就应获取)
    double _motor_lane_stay_thresh;
    // 应急车道允许车辆停留时间
    double _eme_lane_stay_thresh;
    int _admit_motor_dist_from_board;
    int _admit_pede_dist_from_board;
    int _illegal_pass_nums;
    int _retrograde_nums;
    int _retrograde_dist_thresh;
    double _time_interval_thresh;
    double _show_type_side_thresh;

    cz::ParaReader::Ptr _reader_ptr;

    // output ofstream
    std::map<int, std::string> _out_put_type;
};
/// HWEventMonitorPrivate public
HWEventMonitorPrivate::HWEventMonitorPrivate(const cz::ParaReader::Ptr& reader_ptr) {
    _reader_ptr = reader_ptr;
    init();
}
HWEventMonitorPrivate::~HWEventMonitorPrivate() {
}

void HWEventMonitorPrivate::updateMonitorMap(const std::vector<HWObjTrack>& obj_tracks) {
    std::set<int> obj_track_ids;
    for(int i=0; i<obj_tracks.size(); ++i) {
        obj_track_ids.insert(obj_tracks.at(i).id);
    }
    updateVehicleTypeMonitorMap(obj_track_ids,obj_tracks);
    updateVehicleFlowMonitorMap(obj_track_ids,obj_tracks);
}
void HWEventMonitorPrivate::setFlowReferLine(const std::map<int, std::vector<cv::Point> >& flow_refer_lines) {
    _flow_refer_lines = flow_refer_lines;
    getMotorFlowStartSign(); //获取开始一侧的符号
}
void HWEventMonitorPrivate::setMonitorRegion(const std::vector<REGION_DESC>& region_desc) {
    for(int i=0; i<region_desc.size(); ++i) {
        std::vector<cz::lt::Point> pts;
        for(int j=0; j<region_desc.at(i).nPointCount; ++j) {
            pts.emplace_back(cz::lt::Point(region_desc.at(i).pt[j].x,region_desc.at(i).pt[j].y));
        }
        cz::lt::Polyline pl(pts);
        cz::lt::Polygon pg(pl);
        if(region_desc.at(i).regionProp == 0) {
            _motor_lane_poly_map.insert(std::make_pair(region_desc.at(i).direct, pg));
        } else {
            _eme_lane_poly_map.insert(std::make_pair(region_desc.at(i).direct, pg));
        }
    }
}
void HWEventMonitorPrivate::setSpeedReferLine(const std::vector<SPEED_REFER_LINE>& speed_refer_line) {
    for(int i=0; i<speed_refer_line.size(); ++i) {
        std::vector<cz::lt::Segment> segments;
        std::vector<double> distances;
        for(int j=0; j<speed_refer_line.at(i).nPointCount-1; ++j) {
            cz::lt::Point pt1, pt2;
            cz::lt::ShapeConverter<int>::convert(cv::Point(speed_refer_line.at(i).pt[j].x,
                                                           speed_refer_line.at(i).pt[j].y), pt1);
            cz::lt::ShapeConverter<int>::convert(cv::Point(speed_refer_line.at(i).pt[j+1].x,
                                                           speed_refer_line.at(i).pt[j+1].y), pt2);
            segments.emplace_back(cz::lt::Segment(pt1, pt2));
        }
        for(int j=0; j<speed_refer_line.at(i).nPointCount; ++j) {
            distances.emplace_back(speed_refer_line.at(i).referDistance[j]);
        }
        _speed_refer_line_map.insert(std::make_pair(speed_refer_line.at(i).direct, segments));
        _speed_refer_distance_map.insert(std::make_pair(speed_refer_line.at(i).direct, distances));
        std::pair<cv::Point, cv::Point> drive_direction;
        drive_direction.first = cv::Point(speed_refer_line.at(i).s.x,speed_refer_line.at(i).s.y);
        drive_direction.second = cv::Point(speed_refer_line.at(i).e.x,speed_refer_line.at(i).e.y);
        _drive_direction_map.insert(std::make_pair(speed_refer_line.at(i).direct, drive_direction));
    }
}
void HWEventMonitorPrivate::deviceEventMonitor(const double& time_stamp) {
    _event_alarms.clear();
    if(_bDeviceException) {
        deviceExceptionMonitor(time_stamp);
    }
}
void HWEventMonitorPrivate::pedestrianEventMonitor(const double& time_stamp,const HWObjTrack& obj_track) {
    _event_alarms.clear();
    if(_bPedestrianDetect) {
        illegalPassMonitor(time_stamp,obj_track);
    }
}
void HWEventMonitorPrivate::vehicleEventMonitor(const cv::Mat& frame, const double& time_stamp,
                                                const HWObjTrack& obj_track) {
    _event_alarms.clear();
    if(obj_track.short_term_locations.size()==0) {
        return;
    } else { /*do something below*/ }
    cv::Point pt = cz::lt::pointOfRect(obj_track.short_term_locations.rbegin()->second, cz::lt::Feet);
    cz::lt::Point cl_pt;
    cz::lt::ShapeConverter<int>::convert(pt, cl_pt);
    int direct;
    int region_prop;
    bool in_lane = getLocLane(cl_pt, direct, region_prop, _admit_motor_dist_from_board);
    if(!in_lane) {
        return;
    }else { /*do something below*/ }
    // 速度
    speedMonitor(time_stamp,obj_track, direct);
    // 车型
    vehicleTypeMonitor(frame, obj_track);
    // 流量
    vehicleFlowMonitor(obj_track, direct);
    if(_bIllegalParking) {
        illegalParkMonitor(time_stamp,obj_track, region_prop);
    }
    if(_bAbandonedDetect) {
        abandonMonitor(time_stamp,obj_track);
    }
    if(_bSprinklingDetect) {
        sprinklingMonitor(time_stamp,obj_track);
    }
    if(_bRetrograde) {
        retrogradeMonitor(time_stamp,obj_track, direct, region_prop);
    }
}
std::set<int> HWEventMonitorPrivate::eventAlarm() {
    return _event_alarms;
}
double HWEventMonitorPrivate::vehicleSpeed() const {
    return _speed;
}
std::string HWEventMonitorPrivate::vehicleType() const {
    return _vehicle_type;
}

std::pair<std::string, double> HWEventMonitorPrivate::vehicleType_v2() const {
    return _vehicle_type_v2;
};

std::map<int, int> HWEventMonitorPrivate::vehicleFlow() const {
    return _vehicle_flow;
}
/// HWEventMonitorPrivate private
void setUIParaInfo(const cz::ParaReader::Ptr& reader_ptr, const std::string& key, bool& value) {
    std::string flag;
    bool get_flag = reader_ptr->getValue(key,flag);
    if(!get_flag) {
        flag = "false";
    } else {
        // no operation
    }
    value = (flag=="true")?true:false;
}
void HWEventMonitorPrivate::setParameter() {
    std::string UIParamInfo_ini;
    _reader_ptr->getValue("UIParamInfo_ini", UIParamInfo_ini);
    cz::ParaReader::Ptr ui_param_reader_ptr;
    ui_param_reader_ptr.reset(new cz::ParaReader(UIParamInfo_ini));
    setUIParaInfo(ui_param_reader_ptr,"bIllegalParking", _bIllegalParking);
    setUIParaInfo(ui_param_reader_ptr,"bAbandonedDetect", _bAbandonedDetect);
    setUIParaInfo(ui_param_reader_ptr,"bSprinklingDetect", _bSprinklingDetect);
    setUIParaInfo(ui_param_reader_ptr,"bPedestrianDetect", _bPedestrianDetect);
    setUIParaInfo(ui_param_reader_ptr,"bRetrograde", _bRetrograde);
    setUIParaInfo(ui_param_reader_ptr,"bDeviceException", _bDeviceException);
    _reader_ptr->getValue("measure_speed_interval", _measure_speed_interval);
    _reader_ptr->getValue("motor_lane_stay_thresh", _motor_lane_stay_thresh);
    _reader_ptr->getValue("eme_lane_stay_thresh", _eme_lane_stay_thresh);
    _reader_ptr->getValue("admit_motor_dist_from_board", _admit_motor_dist_from_board);
    _reader_ptr->getValue("admit_pede_dist_from_board", _admit_pede_dist_from_board);
    _reader_ptr->getValue("illegal_pass_nums", _illegal_pass_nums);
    _reader_ptr->getValue("retrograde_nums", _retrograde_nums);
    _reader_ptr->getValue("retrograde_dist_thresh", _retrograde_dist_thresh);
    _reader_ptr->getValue("time_interval_thresh", _time_interval_thresh);
    _reader_ptr->getValue("motor_type_pb",_motor_type_pb);
    std::string use_gpu_str;
    _reader_ptr->getValue("motor_type_compute_mode",use_gpu_str);
    _motor_type_use_gpu = (use_gpu_str=="gpu"?true:false);
    _reader_ptr->getValue("motor_type_gpu_id",_motor_type_gpu_id);
    _reader_ptr->getValue("motor_type_score_thresh",_motor_type_score_thresh);

}
void HWEventMonitorPrivate::init() {
    setParameter();
    cz::ml::ModelParameter motor_type_parameter;
    motor_type_parameter.setParam(_motor_type_pb, _motor_type_use_gpu, _motor_type_gpu_id);
    _motor_type_classifier_ptr = cz::ml::IClassifier::createFromName("caffe");
    _motor_type_classifier_ptr->loadModel(motor_type_parameter);
    //流量初始化
    _vehicle_flow[0] = 0;
    _vehicle_flow[1] = 0;

}

void HWEventMonitorPrivate::updateVehicleTypeMonitorMap(const std::set<int> obj_track_ids,
                                                        const std::vector<HWObjTrack>& obj_tracks) {
    //清理场景中已经消失的目标
    for(std::map<int, std::pair<std::string, double> >::iterator iter = _motor_type_map.begin();
        iter!=_motor_type_map.end();) {
        //经过流量统计线，且离开场景的目标
        if(obj_track_ids.count(iter->first) == 0
           && _motor_flow_map.count(iter->first) != 0) {
           //&& _motor_flow_map.at(iter->first).is_count) {
            // txt输出
            // ofile init
            _out_put_type.insert(std::make_pair(iter->first, _motor_type_map.at(iter->first).first));
            std::ofstream ofile;
            ofile.open("../data/res.txt");
            for(auto e : _out_put_type) {
                ofile << e.first << " " << e.second << std::endl;
            }
            ofile.close();
            // end output

            //注: map删除时，一定要写iter=，不然会运行报错
            iter = _motor_type_map.erase(iter);
        } else {
            ++iter;
        }
    }
}
void HWEventMonitorPrivate::updateVehicleFlowMonitorMap(const std::set<int> obj_track_ids,
                                                        const std::vector<HWObjTrack>& obj_tracks) {
    //清理场景中已经消失的目标
    for(std::map<int, ObjFlowInfo >::iterator iter = _motor_flow_map.begin();
        iter!=_motor_flow_map.end();) {
        if(obj_track_ids.count(iter->first) == 0) {
            //注: map删除时，一定要写iter=，不然会运行报错
            iter = _motor_flow_map.erase(iter);
        } else {
            ++iter;
        }
    }
}

void HWEventMonitorPrivate::getMotorFlowStartSign() {
    for(std::map<int, std::vector<cv::Point> >::iterator iter=_flow_refer_lines.begin();
        iter!=_flow_refer_lines.end(); ++iter) {
        int start_sign;
        getMotorFlowStartSignAux(iter->second, iter->first, start_sign);
        _motor_flow_start_sign.insert(std::make_pair(iter->first, start_sign));
    }
}
void HWEventMonitorPrivate::getMotorFlowStartSignAux(const std::vector<cv::Point>& flow_refer_line,
                                                     const int& direct, int& start_sign) {
    std::pair<cv::Point, cv::Point> direct_line = _drive_direction_map[direct];
    cz::lt::Point flow_refer_s_pt, flow_refer_e_pt;
    cz::lt::ShapeConverter<int>::convert(flow_refer_line.at(0), flow_refer_s_pt);
    cz::lt::ShapeConverter<int>::convert(flow_refer_line.at(1), flow_refer_e_pt);
    cz::lt::Segment flow_refer_ct_segment(flow_refer_s_pt, flow_refer_e_pt);
    cz::lt::Point direct_s_pt, direct_e_pt;
    cz::lt::ShapeConverter<int>::convert(direct_line.first, direct_s_pt);
    cz::lt::ShapeConverter<int>::convert(direct_line.second, direct_e_pt);
    cz::lt::Segment direct_ct_segment(direct_s_pt, direct_e_pt);
    cz::lt::Point crs_pt;
    cz::lt::lineCross(flow_refer_ct_segment.line(), direct_ct_segment.line(), crs_pt);
    // 交点位于线段上
    if(cz::lt::pointSegmentTest(crs_pt, direct_ct_segment, false) < CT_EPS) {
        start_sign = int(cz::lt::pointLineTest(direct_s_pt, flow_refer_ct_segment.line(), false));
    } else if(cz::lt::computeDist(crs_pt, direct_s_pt) > cz::lt::computeDist(crs_pt, direct_e_pt)) {
        // 交点距离终点更近， 可知起点和终点均在开始侧
        start_sign = int(cz::lt::pointLineTest(direct_s_pt, flow_refer_ct_segment.line(), false));
    } else {
        // 交点距离起点更近， 可知起点和终点均在终止侧
        start_sign = -1 * int(cz::lt::pointLineTest(direct_s_pt, flow_refer_ct_segment.line(), false));
    }
}
void HWEventMonitorPrivate::vehicleTypeMonitor(const cv::Mat& frame, const HWObjTrack& obj_track) {
    if(obj_track.curr_update) {
        cv::Rect rect = obj_track.curr_rect;
        //不处理超出边界的目标
        cv::Point st_pt(rect.x + rect.width, rect.y);
        if (rect.width > _show_type_side_thresh && rect.height >= _show_type_side_thresh) {
            cz::ml::Classification clsfy;
            _motor_type_classifier_ptr->classify(frame(rect), clsfy);
            //更新处理
            if (clsfy.getPredictions().at(0).score > _motor_type_score_thresh) {
                if(_motor_type_map.count(obj_track.id)!=0
                   && clsfy.getPredictions().at(0).score >_motor_type_map.at(obj_track.id).second) {
//                   && _motor_flow_map.at(obj_track.id).is_count==false) { //直至更新到流量线
                    _motor_type_map.at(obj_track.id).second = clsfy.getPredictions().at(0).score;
                    _motor_type_map.at(obj_track.id).first = clsfy.getPredictions().at(0).type;
                } else {
                    _motor_type_map.insert(std::make_pair(obj_track.id,
                                                          std::make_pair(clsfy.getPredictions().at(0).type,
                                                                         clsfy.getPredictions().at(0).score)));
                }
            }
        }
        if(_motor_type_map.count(obj_track.id)!=0) {
//            _vehicle_type = _motor_type_map.at(obj_track.id).first;
            _vehicle_type_v2 = _motor_type_map.at(obj_track.id);
        } else {
//            _vehicle_type = "";
            _vehicle_type_v2 = std::make_pair("", 0);
        }
    }
}
// 车流量统计
void HWEventMonitorPrivate::vehicleFlowMonitor(const HWObjTrack& obj_track, const int& direct) {
    std::pair<cv::Point, cv::Point> direct_line = _drive_direction_map[direct];
    cz::lt::Point flow_refer_s_pt, flow_refer_e_pt;
    cz::lt::ShapeConverter<int>::convert(_flow_refer_lines.at(direct).at(0), flow_refer_s_pt);
    cz::lt::ShapeConverter<int>::convert(_flow_refer_lines.at(direct).at(1), flow_refer_e_pt);
    cz::lt::Segment flow_refer_ct_segment(flow_refer_s_pt, flow_refer_e_pt);
    if(obj_track.curr_update && _motor_flow_map.count(obj_track.id)!=0)
    {
        cv::Point pt = cz::lt::pointOfRect(obj_track.curr_rect, cz::lt::Center);
        cz::lt::Point ct_pt;
        cz::lt::ShapeConverter<int>::convert(pt, ct_pt);
        if(!_motor_flow_map.at(obj_track.id).start_count) {
            if(cz::lt::pointLineTest(ct_pt, flow_refer_ct_segment.line(), false)
               * _motor_flow_start_sign.at(direct) > 0) {
                _motor_flow_map.at(obj_track.id).start_count = true;
            } else { /*no operation*/}
        } else if(_motor_flow_map.at(obj_track.id).start_count && !_motor_flow_map.at(obj_track.id).end_count) {
            if(cz::lt::pointLineTest(ct_pt, flow_refer_ct_segment.line(), false)
               * _motor_flow_start_sign.at(direct) < 0) {
                _motor_flow_map.at(obj_track.id).end_count = true;
            } else { /*no operation*/}
        } else if(_motor_flow_map.at(obj_track.id).start_count
           && _motor_flow_map.at(obj_track.id).end_count
           && !_motor_flow_map.at(obj_track.id).is_count) {
            _vehicle_flow[direct]++;
            _motor_flow_map.at(obj_track.id).is_count = true;
        } else {/*no operation*/}
    } else {
        ObjFlowInfo flow_info;
        _motor_flow_map.insert(std::make_pair(obj_track.id, flow_info));
    }
}
void HWEventMonitorPrivate::speedMonitor(const double& time_stamp, const HWObjTrack& obj_track, const int& direct) { //速度
    if(obj_track.short_term_locations.size() < 2) {
        _speed = -1.0;
        return;
    } else { /*do something below*/ }
    double curr_time = obj_track.short_term_locations.rbegin()->first;
    cv::Point curr_pt = cz::lt::pointOfRect(obj_track.short_term_locations.rbegin()->second, cz::lt::Feet);
    // 当前时间如果与短期轨迹的最后一个目标时间差在_time_interval_thresh以上时，不计算目标速度
    if(time_stamp - curr_time > _time_interval_thresh) {
        _speed = -1.0;
        return;
    }else { /*do something below*/ }
    int last_pt_id = -1;
    for(int i=obj_track.short_term_locations.size()-2; i>=0; --i) {
        if(curr_time - obj_track.short_term_locations.at(i).first >= _measure_speed_interval) {
            last_pt_id = i;
            break;
        }
    }
    if(last_pt_id == -1) {
        _speed = -1.0;
        return;
    } else { /*do something below*/ }
    double last_time = obj_track.short_term_locations.at(last_pt_id).first;
    cv::Point last_pt = cz::lt::pointOfRect(obj_track.short_term_locations.at(last_pt_id).second, cz::lt::Feet);
    cz::lt::Point cl_curr_pt, cl_last_pt;
    cz::lt::ShapeConverter<int>::convert(curr_pt, cl_curr_pt);
    cz::lt::ShapeConverter<int>::convert(last_pt, cl_last_pt);
    bool get_speed = computeSpeed(cl_curr_pt, cl_last_pt, curr_time-last_time, direct,_speed);
    if(get_speed == false) {
        _speed = -1.0;
    }else { /*do something below*/ }
}
void HWEventMonitorPrivate::illegalParkMonitor(const double& time_stamp, const HWObjTrack& obj_track, const int& region_prop) { //违章停车
    if(obj_track.long_term_locations.size() == 0) {
        return;
    } else { /*do something below*/ }
    //
    if(obj_track.short_term_locations.size() == 0)
    {
        return;
    } else { /*do someting below*/ }
    if(time_stamp - obj_track.short_term_locations.rbegin()->first > _time_interval_thresh) {
        return;
    } else { /*do someting below*/ }
    // 从检测到位置的时候，一直往前推时间
    if(region_prop == 0) {
        if(obj_track.short_term_locations.rbegin()->first - obj_track.long_term_locations.begin()->first
           > _motor_lane_stay_thresh) {
            _event_alarms.insert(5);

        } else { /*do someting below*/ }
    } else {
        if(obj_track.short_term_locations.rbegin()->first - obj_track.long_term_locations.begin()->first
           > _eme_lane_stay_thresh) {
            _event_alarms.insert(3);

        } else { /*do someting below*/ }
    }
}

// 计算夹角余弦值
double computeIntersectAngleCos(const cz::lt::Line& l1, const cz::lt::Line& l2) {
    double a1, b1, c1;
    l1.coef(a1,b1,c1);
    double a2, b2, c2;
    l2.coef(a2, b2, c2);
    return (a1*a2+b1*b2)/(sqrt(a1*a1+b1*b1)+sqrt(a2*a2+b2*b2));
}
void HWEventMonitorPrivate::retrogradeMonitor(const double& time_stamp,const HWObjTrack& obj_track,
                                              const int& direct, const int& region_prop) { //逆行
    //
    //直线夹角公式 ： cosθ=(a1a2+b1b2)/[√(a1^2+b1^2)*√(a2^2+b2^2)]
    cz::lt::Point pt1, pt2;
    cz::lt::ShapeConverter<int>::convert(_drive_direction_map.at(direct).first, pt1);
    cz::lt::ShapeConverter<int>::convert(_drive_direction_map.at(direct).second, pt2);
    cz::lt::Line right_line(pt1, pt2);

    if(obj_track.short_term_locations.size() <= _retrograde_nums) {
        return;
    } else { /*do something below*/ }
    if(time_stamp - obj_track.short_term_locations.rbegin()->first > _time_interval_thresh) {
        return;
    } else { /*do someting below*/ }
    cv::Point curr_pt = cz::lt::pointOfRect(obj_track.short_term_locations.rbegin()->second, cz::lt::Feet);
    cz::lt::Point cl_curr_pt;
    cz::lt::ShapeConverter<int>::convert(curr_pt, cl_curr_pt);

    int curr_direct, curr_region_prop;
    bool in_lane = getLocLane(cl_curr_pt, curr_direct, curr_region_prop, _admit_motor_dist_from_board);
    if(!in_lane) {
        return;
    } else { /*do something below*/ }
    cv::Point curr_cen_loc = cz::lt::pointOfRect(obj_track.short_term_locations.rbegin()->second,cz::lt::Center);
    cz::lt::Point cl_curr_cen_loc;
    cz::lt::ShapeConverter<int>::convert(curr_cen_loc, cl_curr_cen_loc);
    int retrograde_count = 0;
    for(int i=0; i<obj_track.short_term_locations.size()-1; ++i) {
        cv::Point prev_loc = cz::lt::pointOfRect(obj_track.short_term_locations.at(i).second,cz::lt::Feet);
        cz::lt::Point cl_prev_loc;
        cz::lt::ShapeConverter<int>::convert(prev_loc, cl_prev_loc);
        int prev_direct, prev_region_prop;
        bool prev_in_lane = getLocLane(cl_prev_loc, prev_direct, prev_region_prop, _admit_motor_dist_from_board);
        if(!prev_in_lane) {
            continue;
        }else { /*do something below*/ }
        cv::Point prev_cen_loc = cz::lt::pointOfRect(obj_track.short_term_locations.at(i).second,cz::lt::Center);
        cz::lt::Point cl_prev_cen_loc;
        cz::lt::ShapeConverter<int>::convert(prev_cen_loc, cl_prev_cen_loc);
        cz::lt::Line line(cl_prev_cen_loc, cl_curr_cen_loc);
        double distance = std::sqrt(std::pow(curr_cen_loc.x-prev_cen_loc.x,2)
                                    +std::pow(curr_cen_loc.y-prev_cen_loc.y,2));
        if(distance >= _retrograde_dist_thresh // 距离应足够大，以排除停车时的随机扰动
           && computeIntersectAngleCos(right_line, line) < 0 //余弦值小于0,说明夹角大于90度
           && curr_direct == prev_direct) { //须同方向车道上，避免应跟踪的isw的问题引起的目标易道问题，导致逆行误判
            ++retrograde_count;
        }
    }
    if(retrograde_count >= _retrograde_nums) {
        _event_alarms.insert(7);
    } else {/*no operation*/}
}
void HWEventMonitorPrivate::abandonMonitor(const double& time_stamp,const HWObjTrack& obj_track) { //遗留物
    // TODO : Not Implemented
}
void HWEventMonitorPrivate::sprinklingMonitor(const double& time_stamp,const HWObjTrack& obj_track) { //抛洒物
    // TODO : Not Implemented
}
void HWEventMonitorPrivate::illegalPassMonitor(const double& time_stamp,const HWObjTrack& obj_track) { //异常穿越
    //
    if(obj_track.short_term_locations.size() < _illegal_pass_nums) {
        return;
    } else { /*do something below*/ }
    if(time_stamp - obj_track.short_term_locations.rbegin()->first > _time_interval_thresh) {
        return;
    } else { /*do someting below*/ }
    int in_lane_nums = 0;
    for(int i=0; i<obj_track.short_term_locations.size(); ++i) {
        cv::Point pt = cz::lt::pointOfRect(obj_track.short_term_locations.at(i).second, cz::lt::Feet);
        cz::lt::Point cl_pt;
        cz::lt::ShapeConverter<int>::convert(pt, cl_pt);
        int region_prop, direct;
        if(getLocLane(cl_pt, direct, region_prop, _admit_pede_dist_from_board)) {
            ++in_lane_nums;
        }
    }
    if(in_lane_nums > _illegal_pass_nums) {
        _event_alarms.insert(6);
    } else { /*do something below*/ }
}
void HWEventMonitorPrivate::deviceExceptionMonitor(const double& time_stamp) {
    // TODO : Not Implemented
}
bool HWEventMonitorPrivate::getLocLane(const cz::lt::Point& pt, int& direct, int& region_prop, const int& dist_from_board) {
    if(_motor_lane_poly_map.size() == 0 && _eme_lane_poly_map.size() == 0) {
        return false;
    } else { /*do something below*/ }
    double max_dist_from_board = -100;
    // 两次检查，判定方向
    for(std::map<int, cz::lt::Polygon>::iterator iter = _motor_lane_poly_map.begin();
        iter!=_motor_lane_poly_map.end(); ++iter) {
        double dist = cz::lt::pointPolygonTest(pt, iter->second, true);
        if( dist > max_dist_from_board) {
            max_dist_from_board = dist;
            region_prop = 0;
            direct = iter->first;
        }
    }
    for(std::map<int, cz::lt::Polygon>::iterator iter = _eme_lane_poly_map.begin();
        iter!=_eme_lane_poly_map.end(); ++iter) {
        double dist = cz::lt::pointPolygonTest(pt, iter->second, true);
        if( dist > max_dist_from_board) {
            max_dist_from_board = dist;
            region_prop = 1;
            direct = iter->first;
        }
    }
    if(max_dist_from_board < dist_from_board) {
        return false;
    } else {
        return true;
    }
}
// 根据速度参考线计算车辆速度
bool HWEventMonitorPrivate::computeSpeed(const cz::lt::Point& curr_pt, const cz::lt::Point& last_pt,
                                         const double& time_interval, const int& direct, double& speed) {
    std::vector<cz::lt::Segment> speed_reference_segment = _speed_refer_line_map[direct];
    double curr_value = -1.0, last_value = -1.0;
    for(int i=0; i<speed_reference_segment.size(); ++i) {
        cz::lt::Point curr_dst_pt;
        cz::lt::pointProjInLine(curr_pt, speed_reference_segment.at(i).line(), curr_dst_pt);
        if(cz::lt::pointSegmentTest(curr_dst_pt, speed_reference_segment.at(i), false) == 0) {
            curr_value = cz::lt::computeDist(curr_dst_pt,speed_reference_segment.at(i).startPt())/
                         cz::lt::computeDist(speed_reference_segment.at(i).startPt(),speed_reference_segment.at(i).endPt()) *
                         (_speed_refer_distance_map.at(direct).at(i+1)-_speed_refer_distance_map.at(direct).at(i))
                         + _speed_refer_distance_map.at(direct).at(i);
        }
        cz::lt::Point last_dst_pt;
        cz::lt::pointProjInLine(last_pt, speed_reference_segment.at(i).line(), last_dst_pt);
        if(cz::lt::pointSegmentTest(last_dst_pt, speed_reference_segment.at(i), false) == 0) {
            last_value = cz::lt::computeDist(last_dst_pt,speed_reference_segment.at(i).startPt())/
                         cz::lt::computeDist(speed_reference_segment.at(i).startPt(),speed_reference_segment.at(i).endPt()) *
                         (_speed_refer_distance_map.at(direct).at(i+1)-_speed_refer_distance_map.at(direct).at(i))
                         + _speed_refer_distance_map.at(direct).at(i);;
        }
    }
    if(curr_value!=-1.0 && last_value!=-1.0) {
        speed = std::round(fabs(curr_value - last_value) / (time_interval*1e-3) * 3.6); //3.6为单位转换
        return true;
    } else {
        return false;
    }
}


/// HWEventMonitor public
HWEventMonitor::HWEventMonitor(const cz::ParaReader::Ptr& reader_ptr) {
    _private_ptr.reset(new HWEventMonitorPrivate(reader_ptr));
}
HWEventMonitor::~HWEventMonitor() {

}
void HWEventMonitor::setMonitorRegion(const std::vector<REGION_DESC>& region_desc) {
    _private_ptr->setMonitorRegion(region_desc);
}
void HWEventMonitor::setSpeedReferLine(const std::vector<SPEED_REFER_LINE>& speed_refer_line) {
    _private_ptr->setSpeedReferLine(speed_refer_line);
}
void HWEventMonitor::setFlowReferLine(const std::map<int, std::vector<cv::Point> >& flow_refer_lines) {
    _private_ptr->setFlowReferLine(flow_refer_lines);
}
void HWEventMonitor::deviceEventMonitor(const double& time_stamp) {
    _private_ptr->deviceEventMonitor(time_stamp);
}
void HWEventMonitor::pedestrianEventMonitor(const double& time_stamp, const HWObjTrack& obj_track) {
    _private_ptr->pedestrianEventMonitor(time_stamp, obj_track);
}
void HWEventMonitor::vehicleEventMonitor(const cv::Mat& frame, const double& time_stamp, const HWObjTrack& obj_track) {
    _private_ptr->vehicleEventMonitor(frame, time_stamp, obj_track);
}
std::set<int> HWEventMonitor::eventAlarm() {
    return _private_ptr->eventAlarm();
}
double HWEventMonitor::vehicleSpeed() const {
    return _private_ptr->vehicleSpeed();
}
void HWEventMonitor::updateMonitorMap(const std::vector<HWObjTrack>& obj_tracks) {
    _private_ptr->updateMonitorMap(obj_tracks);
}
std::string HWEventMonitor::vehicleType() const {
    return _private_ptr->vehicleType();
}
std::pair<std::string, double> HWEventMonitor::vehicleType_v2() const {
 return _private_ptr->vehicleType_v2();
}
std::map<int, int> HWEventMonitor::vehicleFlow() const { //车流量
    return _private_ptr->vehicleFlow();
}
}
}
