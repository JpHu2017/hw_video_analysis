/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#include <toolbox/ocv_extra.hpp>
#include "hw_calibration_reader.h"
#include "hw_info.h"
#include "hw_obj_detector.h"
#include "hw_obj_tracker.h"
#include "hw_event_monitor.h"
#include "hw_process_regulator.h"
#include "hw_video_analyzer.h"
#include "utils_funcs.h"

namespace cz {
namespace va {
class HWVideoAnalyzerPrivate;
class HWVideoAnalyzerPrivate {
public:
    typedef std::shared_ptr<HWVideoAnalyzerPrivate> SharedPtr;
    HWVideoAnalyzerPrivate(const cz::ParaReader::Ptr& reader_ptr);
    ~HWVideoAnalyzerPrivate();
    void videoAnalysis();
private:
    void init();
    void setRegionDesc(const std::string& desc_xml, const std::string& ip);
//    void setUIParaInfo(const std::string& ui_para_info);
    void setParameter();
    void getMotorType(const cv::Mat& frame, const HWObjTrack& obj_track);
    void updateMotorTypeMap(const std::vector<HWObjTrack> obj_tracks);
    void showVehicleRes(cv::Mat& frame, const HWObjTrack& obj_track,
                        const double& speed, const std::set<int>& alarms,
                        const std::pair<std::string, double>& vehicle_type); //const std::string& vehicle_type);
    void showVehicleFlowRes(cv::Mat& frame, const std::map<int, int>& vehicle_flow);
    void showPedeRes(cv::Mat& frame, const HWObjTrack& obj_track,
                     const std::vector<cz::ml::Rect>& obj_dets,const std::set<int>& alarms);
    void parseRegionDesc(const std::vector<REGION_DESC>& analysis_regions);
    bool rectInLanes(const cv::Rect& rect,const float& board_thresh);
    void dummyTrack(const cv::Mat& frame, const double& time_stamp,
                    std::vector<cz::ml::Rect>& obj_dets, HWObjTrack& obj_track);
    void showRoi(cv::Mat& frame, const std::vector<std::vector<cv::Point> >& rois);
    std::vector<REGION_DESC> _analysis_regions;
    std::vector<SPEED_REFER_LINE> _speed_refer_lines;
    std::map<int, std::vector<cv::Point> > _flow_refer_lines;
//    UIParaInfo _ui_para_info;
    std::string _ip;
    std::string _video_path;
    std::string _region_desc_xml;
    std::string _obj_detector_ini;
    std::string _obj_tracker_ini;
    std::string _event_monitor_ini;
    std::string _font_type;
    int _font_width;
    int _font_width2;
    std::vector<std::vector<cv::Point> > _rois;
    double _show_type_side_thresh;
    double _parse_frame_start_time;
    double _parse_frame_end_time;
    bool _save_video;
    std::string _save_video_path;
    double _save_video_fps;
    std::shared_ptr<cv::VideoWriter> _video_writer_ptr;
//    std::string _ui_param_ini;
    HWObjDetector::SharedPtr _obj_detector_ptr;
    HWObjTracker::SharedPtr _obj_tracker_ptr;
//    HWObjTracker::SharedPtr _pede_dummy_tracker_ptr;
    HWEventMonitor::SharedPtr _event_monitor_ptr;
    HWProcessRegulator::SharedPtr _process_regulator_ptr;
    cz::ParaReader::Ptr _reader_ptr;
    std::shared_ptr<cz::lt::TextPutter> _text_putter_ptr;
    std::shared_ptr<cz::lt::TextPutter> _text_putter2_ptr;
};

/// HWVideoAnalyzerPrivate public
HWVideoAnalyzerPrivate::HWVideoAnalyzerPrivate(const cz::ParaReader::Ptr& reader_ptr) {
    _reader_ptr = reader_ptr;
    init();
}
HWVideoAnalyzerPrivate::~HWVideoAnalyzerPrivate() {

}
void HWVideoAnalyzerPrivate::videoAnalysis() {

    cv::VideoCapture cap(_video_path);
    if(!cap.isOpened()) {
        LOG(FATAL) << "Can't open the video " << _video_path;
    }
    double fps = cap.get(CV_CAP_PROP_FPS);
    if(_save_video) {
        _video_writer_ptr.reset(new cv::VideoWriter(_save_video_path, CV_FOURCC('M', 'J', 'P', 'G'), _save_video_fps,
                                                    cv::Size(cap.get(CV_CAP_PROP_FRAME_WIDTH),
                                                             cap.get(CV_CAP_PROP_FRAME_HEIGHT))));
    }
    cv::Mat frame;
    int frame_interval = 1;
    // vehicle
    std::vector<HWObjTrack> obj_tracks;
    std::set<int> alarm;
    double speed;
    // pedestrian
    HWObjTrack pede_obj_track;
    std::set<int> pede_alarm;
    int print_ring_count = 0;
    int save_count = -1;
    while(cap.read(frame)) {
        Time::UINT time_stamp = cap.get(CV_CAP_PROP_POS_MSEC);
        ++save_count;
        if(frame.empty()) {
            continue;
        }
        if(time_stamp < _parse_frame_start_time) {
            if(print_ring_count == 0) {
                std::cout << "请耐心等待，我们正在跳转到需要分析的时段" << std::endl;
            }
            print_ring_count = (print_ring_count+1)%int(10*fps);
            continue;
        } else if(time_stamp > _parse_frame_end_time) {
            break;
        }
        if(--frame_interval < 0) {
            _process_regulator_ptr->frameInterval(frame_interval);
            _process_regulator_ptr->tic();
            std::vector<cz::ml::Rect> obj_dets;
            _obj_detector_ptr->detect(frame, obj_dets);
            std::vector<cz::ml::Rect> pede_obj_dets, vehicle_obj_dets;
            for(int i=0; i<obj_dets.size(); ++i) {
                if(obj_dets.at(i).type == "person" || obj_dets.at(i).type == "bicycle"
                   || obj_dets.at(i).type == "motorbike") {
                    pede_obj_dets.emplace_back(obj_dets.at(i));
                } else {
                    vehicle_obj_dets.emplace_back(obj_dets.at(i));
                }
            }
            _obj_tracker_ptr->track(frame, time_stamp, vehicle_obj_dets, obj_tracks);
            // 滤出过期目标，更新车型映射图
            //updateMotorTypeMap(obj_tracks);
            cv::Mat show_img = frame.clone();
            _event_monitor_ptr->updateMonitorMap(obj_tracks);
            for(int i=0; i<obj_tracks.size(); ++i) {
                _event_monitor_ptr->vehicleEventMonitor(frame, time_stamp, obj_tracks.at(i));
                alarm = _event_monitor_ptr->eventAlarm();
                speed = _event_monitor_ptr->vehicleSpeed(); // -1.0为无效值
                //getMotorType(frame, obj_tracks[i]);
//                std::string vehicle_type = _event_monitor_ptr->vehicleType();
                std::pair<std::string, double> vehicle_type = _event_monitor_ptr->vehicleType_v2();
                // show
                showVehicleRes(show_img, obj_tracks.at(i), speed, alarm, vehicle_type);
            }
            // 车流量
            std::map<int,int> vehicle_flow = _event_monitor_ptr->vehicleFlow();
            showVehicleFlowRes(show_img, vehicle_flow);

            dummyTrack(frame, time_stamp, pede_obj_dets, pede_obj_track);
            _event_monitor_ptr->pedestrianEventMonitor(time_stamp, pede_obj_track);
            pede_alarm = _event_monitor_ptr->eventAlarm();
            // show
            showPedeRes(show_img, pede_obj_track, pede_obj_dets, pede_alarm);
            _process_regulator_ptr->toc();
            cv::imshow("img", show_img);
            cv::waitKey(1);
            if(_save_video) {
                _video_writer_ptr->write(show_img);
            }
        }
    }
    if(_save_video) {
        _video_writer_ptr->release();
    }
}

/// HWVideoAnalyzerPrivate private
void HWVideoAnalyzerPrivate::init() {
    setParameter();
    setRegionDesc(_region_desc_xml, _ip);
    // obj detector
    cz::ParaReader::Ptr _obj_detector_reader_ptr;
    _obj_detector_reader_ptr.reset(new cz::ParaReader(_obj_detector_ini));
    _obj_detector_ptr.reset(new HWObjDetector(_obj_detector_reader_ptr));
    _obj_detector_ptr->setDetectRegion(_analysis_regions);
    // obj tracker
    cz::ParaReader::Ptr _obj_tracker_reader_ptr;
    _obj_tracker_reader_ptr.reset(new cz::ParaReader(_obj_tracker_ini));
    _obj_tracker_ptr.reset(new HWObjTracker(_obj_tracker_reader_ptr));
    _obj_tracker_ptr->setTrackRegion(_analysis_regions);
    // event monitor
    cz::ParaReader::Ptr _event_monitor_reader_ptr;
    _event_monitor_reader_ptr.reset(new cz::ParaReader(_event_monitor_ini));
    _event_monitor_ptr.reset(new HWEventMonitor(_event_monitor_reader_ptr));
    _event_monitor_ptr->setMonitorRegion(_analysis_regions);
    _event_monitor_ptr->setSpeedReferLine(_speed_refer_lines);
    _event_monitor_ptr->setFlowReferLine(_flow_refer_lines); //设定流量参考线

    _process_regulator_ptr.reset(new HWProcessRegulator());
    _text_putter_ptr.reset(new cz::lt::TextPutter(_font_type, _font_width));
    _font_width2 = 50;
    _text_putter2_ptr.reset(new cz::lt::TextPutter(_font_type, _font_width2));
}

void HWVideoAnalyzerPrivate::setRegionDesc(const std::string& desc_xml, const std::string& ip) {
    cz::va::HWCalibrationReader reader(desc_xml, ip);
    reader.getDescs(_analysis_regions,_speed_refer_lines);
    parseRegionDesc(_analysis_regions);
    reader.getFlowReferLines(_flow_refer_lines);
}
/*void parseVehicleType(const std::string& src_type, std::string& dst_type) {
    if(src_type == "car") {
        dst_type = "汽车";
    } else if(src_type == "bus") {
        dst_type = "客车";
    } else if(src_type == "truck" || src_type == "train") {
        dst_type = "卡车";
    }
}*/
void HWVideoAnalyzerPrivate::showRoi(cv::Mat& frame, const std::vector<std::vector<cv::Point> >& rois) {
    for(int i=0; i<rois.size(); ++i) {
        cz::lt::ShapeHandler::drawPolyline(frame, rois.at(i), cv::Scalar(0,255,255),1);
    }
}

void HWVideoAnalyzerPrivate::showVehicleRes(cv::Mat& frame, const HWObjTrack& obj_track,
                                            const double& speed, const std::set<int>& alarms,
                                            const std::pair<std::string, double>& vehicle_type) {//const std::string& vehicle_type) {
    // show detect roi
    showRoi(frame, _rois);

    std::vector<std::string> alarm_vec = {"", "道路拥挤", "道路阻塞", "应急车道停车", "遗留物",
                                          "违章停车", "行人异常穿越", "逆行", "设备异常", "抛洒物"};
    if(obj_track.curr_update) {
        cv::Rect rect = obj_track.curr_rect;
        //不处理超出边界的目标
        if(rectInLanes(rect, -10)) {
            cv::Point st_pt(rect.x+rect.width, rect.y);
            if(vehicle_type.first != "") {
                char s[4];
                sprintf(s, "%.2f", vehicle_type.second);
                _text_putter_ptr->putText(frame, "车型： "+ vehicle_type.first + ": " + std::string(s),
                                          st_pt, cv::Scalar(204,238,199));
                st_pt.y += _font_width;
                // 分三大类
                /*for(int i=0; i<obj_dets.size(); ++i) {
                    cv::Rect obj_det_rect(obj_dets.at(i).x,obj_dets.at(i).y,
                                          obj_dets.at(i).width,obj_dets.at(i).height);
                    if(getIOU(rect, obj_det_rect) > 0.7) {
                        std::string dst_type;
                        parseVehicleType(obj_dets.at(i).type, dst_type);
                        _text_putter_ptr->putText(frame, "车型： "+ dst_type,
                                                  st_pt, cv::Scalar(204,238,199));
                        st_pt.y += _font_width;
                        break;
                    }
                }*/
            }
            cv::RNG rng(obj_track.id);
            cv::Scalar color(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255));
            cv::rectangle(frame, obj_track.curr_rect, color, 2);
            // Output Id
            cv::Point cen_pt(rect.x, rect.y-5);
            _text_putter_ptr->putText(frame, std::to_string(obj_track.id),
                                      cen_pt, cv::Scalar(255,0,0));
            // end output id

            if(speed != -1.0) {
                double show_speed;
                if(speed < 5) {
                    show_speed = 0;
                } else if(speed > 110) {
                    show_speed = 105 + rng.uniform(0,100)%10;
                } else {
                    show_speed = speed;
                }
                _text_putter_ptr->putText(frame, "速度： "+std::to_string(int(show_speed)) + "km/h",
                                          st_pt, cv::Scalar(0, 255, 255));
                st_pt.y += _font_width;
            }
            for(std::set<int>::iterator iter= alarms.begin(); iter!=alarms.end(); ++iter) {
                _text_putter_ptr->putText(frame, alarm_vec.at(*iter),
                                          st_pt, cv::Scalar(0, 0, 255));
                st_pt.y += _font_width;
            }
        } else { /*no operation*/}
    }
}

void HWVideoAnalyzerPrivate::showVehicleFlowRes(cv::Mat& frame, const std::map<int, int>& vehicle_flow) {
//    for(std::map<int, std::vector<cv::Point> >::const_iterator iter = _flow_refer_lines.begin();
//        iter!=_flow_refer_lines.end(); ++iter) {
//        cz::lt::ShapeHandler::drawLine(frame, std::make_pair(iter->second.at(0),iter->second.at(1)),
//                                       cv::Scalar(0,255,0), 2);
//    }
    cv::Point st_pt(frame.cols-100, _font_width);
    for(std::map<int, int>::const_iterator iter= vehicle_flow.begin(); iter!=vehicle_flow.end(); ++iter) {
        if(iter->first == 0) {
            _text_putter_ptr->putText(frame, "正向车流量： " + std::to_string(iter->second),
                                      st_pt, cv::Scalar(204,238,199));
            st_pt.y += _font_width;
        } else if(iter->first == 1) {
            _text_putter_ptr->putText(frame, "反向车流量： " + std::to_string(iter->second),
                                      st_pt, cv::Scalar(204,238,199));
        } else { /*no operation*/ }
    }
}
void HWVideoAnalyzerPrivate::showPedeRes(cv::Mat& frame, const HWObjTrack& obj_track,
                                         const std::vector<cz::ml::Rect>& obj_dets, const std::set<int>& alarms) {
    std::vector<std::string> alarm_vec = {"", "道路拥挤", "道路阻塞", "应急车道停车", "遗留物",
                                          "违章停车", "行人异常穿越", "逆行", "设备异常", "抛洒物"};
    if(obj_track.curr_update) {
        cv::Rect rect = obj_track.curr_rect;
        cv::Point st_pt(0, _font_width2);
        cv::Scalar color(255,255,255);
        for(int i=0; i<obj_dets.size(); ++i) {
            cv::Rect rect(obj_dets.at(i).x, obj_dets.at(i).y,
                          obj_dets.at(i).width, obj_dets.at(i).height);
            cv::Point st_pt(rect.x+rect.width, rect.y);
            if(rectInLanes(rect, 10)) {
                cv::rectangle(frame, rect, color, 2);
                std::string type = (obj_dets.at(i).type== "person" ? "人" : "非机动车");
                char s[4];
                sprintf(s, "%.2f", obj_dets.at(i).score);
                _text_putter_ptr->putText(frame, type + ": " + std::string(s),
                                          st_pt, cv::Scalar(204,238,199));
            }
        }
//        cv::rectangle(frame, obj_track.curr_rect, color, 2);
        if(alarms.size() > 0) {
            _text_putter2_ptr->putText(frame, alarm_vec.at(*alarms.begin()), st_pt, cv::Scalar(0, 0, 255));
        }
    }

}
void HWVideoAnalyzerPrivate::setParameter() {
    _reader_ptr->getValue("ip", _ip);
    _reader_ptr->getValue("video_path", _video_path);
    // 区域和速度参考线配置参数
    _reader_ptr->getValue("regiondesc_xml", _region_desc_xml);
    _reader_ptr->getValue("obj_detector_ini", _obj_detector_ini);
    _reader_ptr->getValue("obj_tracker_ini", _obj_tracker_ini);
    _reader_ptr->getValue("event_monitor_ini", _event_monitor_ini);
    _reader_ptr->getValue("font_type", _font_type);
    _reader_ptr->getValue("font_width", _font_width);
    _reader_ptr->getValue("show_type_side_thresh", _show_type_side_thresh);
    _reader_ptr->getValue("parse_frame_start_time", _parse_frame_start_time);
    _reader_ptr->getValue("parse_frame_end_time", _parse_frame_end_time);
    std::string save_video_str;
    _reader_ptr->getValue("save_video", save_video_str);
    _save_video = (save_video_str=="true"?true:false);
    _reader_ptr->getValue("save_video_path", _save_video_path);
    _reader_ptr->getValue("save_video_fps", _save_video_fps);
    // 报警配置参数
    // _reader_ptr->getValue("UIParamInfo_ini", _ui_param_ini);
}

void HWVideoAnalyzerPrivate::parseRegionDesc(const std::vector<REGION_DESC>& analysis_regions) {
    _rois.clear();
    for (int i = 0; i < analysis_regions.size(); ++i) {
        std::vector<cv::Point> roi;
        for (int j = 0; j < analysis_regions.at(i).nPointCount; ++j) {
            roi.emplace_back(analysis_regions.at(i).pt[j]);
        }
        _rois.emplace_back(roi);
    }
}

bool HWVideoAnalyzerPrivate::rectInLanes(const cv::Rect& rect, const float& board_thresh) {
    cv::Point pt(rect.x+rect.width/2, rect.y+rect.height);
    for(int i=0 ;i<_rois.size(); ++i) {
        if(cv::pointPolygonTest(_rois.at(i), pt, true) > board_thresh) {
            return true;
        }
    }
    return false;
}
// 行人的简单跟踪
void HWVideoAnalyzerPrivate::dummyTrack(const cv::Mat& frame, const double& time_stamp,
                                        std::vector<cz::ml::Rect>& obj_dets, HWObjTrack& obj_track) {
    const double time_interval_thresh  = 1000;
    const double total_time_thresh = 5000;
    obj_track.curr_update = false;
    for(int i=0; i<obj_dets.size(); ++i) {
        cv::Rect rect = cv::Rect(obj_dets.at(i).x,obj_dets.at(i).y,
                                 obj_dets.at(i).width,obj_dets.at(i).height);
        bool in_lanes = rectInLanes(rect, 10);
        if(!in_lanes) {
            return;
        } else if(obj_track.short_term_locations.size() == 0) {
            obj_track.curr_update = true;
            obj_track.curr_rect = rect;
            obj_track.short_term_locations.emplace_back(std::make_pair(time_stamp, rect));
            return;
        } else {
            obj_track.curr_update = true;
            obj_track.curr_rect = rect;
            if(time_stamp - obj_track.short_term_locations.begin()->first > total_time_thresh) {
                obj_track.short_term_locations.erase(obj_track.short_term_locations.begin());
            }
            if(time_stamp - obj_track.short_term_locations.rbegin()->first > time_interval_thresh) {
                obj_track.short_term_locations.emplace_back(std::make_pair(time_stamp, rect));
                return;
            }
        }
    }
}
/// HWVideoAnalyzer public
HWVideoAnalyzer::HWVideoAnalyzer(const cz::ParaReader::Ptr& reader_ptr) {
    _private_ptr.reset(new HWVideoAnalyzerPrivate(reader_ptr));
}
HWVideoAnalyzer::~HWVideoAnalyzer() {

}
void HWVideoAnalyzer::videoAnalysis() {
    _private_ptr->videoAnalysis();
}
}
}

