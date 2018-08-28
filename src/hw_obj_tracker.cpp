/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */
#include <toolbox/ocv_extra.hpp>
#include <toolbox/shape.hpp>
#include "hw_obj_tracker.h"
#include "utils_funcs.h"

namespace cz {
namespace va {

class HWObjTrackerPrivate {
public:
    typedef std::shared_ptr<HWObjTracker> SharedPtr;
    HWObjTrackerPrivate(const cz::ParaReader::Ptr& reader_ptr);
    ~HWObjTrackerPrivate();
    void setTrackRegion(const std::vector<REGION_DESC>& region_desc);
    std::vector<HWObjTrack>& track(const cv::Mat& frame, const Time::UINT& time_stamp,
                                   const std::vector<cz::ml::Rect>& obj_dets);
private:
    void setParameter();
    void initTracker();
    void init();
    void setWinSize(const int& width, const int& height);
    void postprocess(const Time::UINT& time_stamp); //轨迹后处理操作
    void longTermLocsProcess(HWObjTrack& obj_track); //长期轨迹处理
    // 根据车长宽的先验知识，对车辆宽度进行限制
    // (主要是因为目标刚进入监控区域时，由于kalman filter预测的特性，得到的矩形框都特别夸张)
    void correctRect(cv::Rect& rect, const double& ratio=1.5);
    // feature generator
    bool _use_feature;
    std::string _feature_generator_model_path;
    std::string _feature_generator_proto_path;
    std::string _compute_mode;
    int _gpu_id;
    // nn matching
    std::string _nn_matching_metric;
    float _nn_matching_threshold;
    int _nn_matching_budget;
    // multi-obj tracker
    float _max_iou_distance;
    int _max_age;
    int _n_init;
    // short long term params
    double _short_term_interval;
    double _short_term_total_time;
    double _long_term_interval;
    //
    float _judge_stay_ratio;
    float _judge_short_stay_iou;
    float _judge_long_stay_iou;
    // win size
    int _win_width;
    int _win_height;
    bool _get_win_size_flag;
    //
    float _normal_rect_ratio;
    float _currect_rect_ratio;
    // tracker roi
    std::vector<std::vector<cv::Point> > _left_rois;
    std::vector<std::vector<cv::Point> > _right_rois;
    std::vector<int> _track_ids;
    int _id_count;
    std::vector<HWObjTrack> _obj_tracks;
    // smart pointer
    cz::ht::DsFeatureGenerator::SharedPtr _feature_generator_ptr;
    cz::ht::DsTracker::SharedPtr _tracker_ptr;
    cz::ParaReader::Ptr _reader_ptr;
};

/// HWObjTrackerPrivate public
HWObjTrackerPrivate::HWObjTrackerPrivate(const cz::ParaReader::Ptr& reader_ptr) {
    _reader_ptr = reader_ptr;
    _get_win_size_flag = false;
    _id_count = 0;
    init();
}
HWObjTrackerPrivate::~HWObjTrackerPrivate() {

}
void HWObjTrackerPrivate::setTrackRegion(const std::vector<REGION_DESC>& region_desc) {
    _right_rois.clear(); //正向
    _left_rois.clear(); //反向
    for(int i=0; i<region_desc.size(); ++i) {
        if(region_desc.at(i).direct == 0) {
            std::vector<cv::Point> roi;
            for(int j=0; j<region_desc.at(i).nPointCount; ++j) {
                roi.emplace_back(region_desc.at(i).pt[j]);
            }
            _right_rois.emplace_back(roi);
        } else if(region_desc.at(i).direct == 1) {
            std::vector<cv::Point> roi;
            for(int j=0; j<region_desc.at(i).nPointCount; ++j) {
                roi.emplace_back(region_desc.at(i).pt[j]);
            }
            _left_rois.emplace_back(roi);
        } else { /*no operation*/ }
    }
}
void HWObjTrackerPrivate::setWinSize(const int& width, const int& height) {
    _win_width = width;
    _win_height = height;
}
std::vector<HWObjTrack>& HWObjTrackerPrivate::track(const cv::Mat& frame, const Time::UINT& time_stamp,
                                                    const std::vector<cz::ml::Rect>& obj_dets) {
    if(_get_win_size_flag == false) {
        setWinSize(frame.cols, frame.rows);
        _get_win_size_flag = true;
    }
    // feature generate
    std::vector<cz::ht::DsDetection> ds_dets;
    for(int i=0; i<obj_dets.size(); ++i) {
        cz::ht::DsDetection ds_det;
        Eigen::RowVector4f tlwh;
        tlwh << obj_dets.at(i).x,obj_dets.at(i).y,obj_dets.at(i).width,obj_dets.at(i).height;
        float conf = obj_dets.at(i).score;
        Eigen::RowVectorXf feature;
        if(_use_feature) {
            cv::Rect rect(obj_dets.at(i).x,obj_dets.at(i).y,obj_dets.at(i).width,obj_dets.at(i).height);
            cv::Rect checked_rect;
            cz::lt::edgeCheck(rect, checked_rect, _win_width, _win_height);
            _feature_generator_ptr->generateFeature(frame(checked_rect), feature);
        } else {
            feature = Eigen::RowVectorXf();
        }
        ds_det.setDetetion(tlwh, conf, feature);
        ds_dets.emplace_back(ds_det);
    }
    // multi tracking
    _tracker_ptr->predict();
    _tracker_ptr->update(ds_dets);
    // postprocess
    postprocess(time_stamp);
    return _obj_tracks;
}

/// HWObjTrackerPrivate private
void HWObjTrackerPrivate::setParameter() {
    std::string use_feature;
    _reader_ptr->getValue("use_feature", use_feature);
    _use_feature = (use_feature=="true"?true:false);
    _reader_ptr->getValue("feature_generator_model_path", _feature_generator_model_path);
    _reader_ptr->getValue("feature_generator_proto_path", _feature_generator_proto_path);
    _reader_ptr->getValue("compute_mode", _compute_mode);
    _reader_ptr->getValue("gpu_id", _gpu_id);
    _reader_ptr->getValue("nn_matching_metric", _nn_matching_metric);
    _reader_ptr->getValue("nn_matching_threshold",_nn_matching_threshold);
    _reader_ptr->getValue("nn_matching_budget",_nn_matching_budget);
    _reader_ptr->getValue("max_iou_distance",_max_iou_distance);
    _reader_ptr->getValue("max_age",_max_age);
    _reader_ptr->getValue("n_init",_n_init);
    _reader_ptr->getValue("short_term_interval",_short_term_interval);
    _reader_ptr->getValue("short_term_total_time",_short_term_total_time);
    _reader_ptr->getValue("long_term_interval",_long_term_interval);
    _reader_ptr->getValue("judge_stay_ratio",_judge_stay_ratio);
    _reader_ptr->getValue("judge_short_stay_iou",_judge_short_stay_iou);
    _reader_ptr->getValue("judge_long_stay_iou",_judge_long_stay_iou);
    _reader_ptr->getValue("normal_rect_ratio", _normal_rect_ratio);
    _reader_ptr->getValue("currect_rect_ratio",_currect_rect_ratio);
}
void HWObjTrackerPrivate::initTracker() {
    //特征提取器
    _feature_generator_ptr.reset(new cz::ht::DsFeatureGenerator());
    _feature_generator_ptr->setComputeMode(_compute_mode, _gpu_id);
    _feature_generator_ptr->init(_feature_generator_proto_path, _feature_generator_model_path);
    //nn distance metric
    cz::ht::NearestNeighborDistanceMetric nn_distance_metric(_nn_matching_metric, _nn_matching_threshold,
                                                             _nn_matching_budget);
    //跟踪器
    _tracker_ptr.reset(new cz::ht::DsTracker(nn_distance_metric, _max_iou_distance, _max_age, _n_init));
}
void HWObjTrackerPrivate::init() {
    setParameter();
    initTracker();
}

void HWObjTrackerPrivate::postprocess(const Time::UINT& time_stamp) {
    // 删除不在图中的目标
    std::set<int> alive_ids;
    for(int i=0; i< _tracker_ptr->tracks().size(); ++i) {
        alive_ids.insert(_tracker_ptr->tracks().at(i).trackId());
    }
    for(int i=0; i<_track_ids.size(); ) {
        if(alive_ids.count(_track_ids.at(i))==0) {
            _track_ids.erase(_track_ids.begin()+i);
            _obj_tracks.erase(_obj_tracks.begin()+i);
        } else {
            ++i;
        }
    }
    // 更新重置
    for(int i=0; i<_obj_tracks.size(); ++i) {
        _obj_tracks[i].curr_update = false;
    }
    for(int i=0; i<_tracker_ptr->tracks().size(); ++i) {
        cz::ht::DsTrack track = _tracker_ptr->tracks().at(i);
        if(!track.isConfirmed() || track.timeSinceUpdate()>1 ) { //之后调整值试一试
            continue;
        }else { /* do something below */ }
        cv::Rect rect = cv::Rect(cv::Point(track.toTlbr()(0),track.toTlbr()(1)),
                                 cv::Point(track.toTlbr()(2),track.toTlbr()(3)));
        cv::Rect checked_rect;
        cz::lt::edgeCheck(rect, checked_rect, _win_width, _win_height);
        //调整矩形框
        std::vector<int>::iterator id_iter = std::find(_track_ids.begin(),_track_ids.end(), track.trackId());
        if(id_iter == _track_ids.end()) { // initialize
            //初始化时，仅加入形状合理的矩形框
            if(checked_rect.width * 1.0 / checked_rect.height <= _normal_rect_ratio) {
                _track_ids.emplace_back(track.trackId());
                HWObjTrack obj_track;
                obj_track.curr_update = true;
                obj_track.curr_rect = checked_rect;
                obj_track.id = ++_id_count;
                obj_track.short_term_locations.emplace_back(std::make_pair(time_stamp, checked_rect));
                _obj_tracks.emplace_back(obj_track);
            } else {
                // no operation
            }
        } else {
            // 当并非第一个矩形框时，根据长宽比限制，对车辆矩形框做出调整
            correctRect(checked_rect, _currect_rect_ratio);
            int id_pos = id_iter - _track_ids.begin();
            _obj_tracks.at(id_pos).curr_update = true;
            _obj_tracks.at(id_pos).curr_rect = checked_rect;
            // 短期轨迹判断
            if(time_stamp - _obj_tracks.at(id_pos).short_term_locations.rbegin()->first >= _short_term_interval) {
                _obj_tracks[id_pos].short_term_locations.emplace_back(std::make_pair(time_stamp, checked_rect));
                if(time_stamp - _obj_tracks.at(id_pos).short_term_locations.begin()->first > _short_term_total_time) {
                    _obj_tracks[id_pos].short_term_locations.erase(_obj_tracks.at(id_pos).short_term_locations.begin());
                }
                longTermLocsProcess(_obj_tracks[id_pos]);
            }
        }
    }
    // TODO : Not Implemented
    /* 处理错误跟踪导致横跨车道问题 */
}
void HWObjTrackerPrivate::longTermLocsProcess(HWObjTrack& obj_track) {
    // 长期轨迹删除操作
    // 最近长期轨迹和所有短期轨迹的iou均小于_judge_long_stay_iou
    if(obj_track.long_term_locations.size()!=0) {
        bool erase_flag = true;
        for(int i=0; i<obj_track.short_term_locations.size(); ++i) {
            if(getIOU(obj_track.long_term_locations.rbegin()->second,
                      obj_track.short_term_locations.at(i).second) > _judge_long_stay_iou) {
                erase_flag = false;
            }
        }
        //目标位置改变或者直接消失，都将长期轨迹删除
        if(obj_track.short_term_locations.size() == 0 || erase_flag) {
            obj_track.long_term_locations.clear();
            return;
        }
    }
    // 长期轨迹添加操作
    // 未到时间间隔时，长期轨迹不加入目标
    if(obj_track.long_term_locations.size()!= 0
       && obj_track.short_term_locations.rbegin()->first
          - obj_track.long_term_locations.rbegin()->first < _long_term_interval) {
        return;
    } else if(obj_track.short_term_locations.size()
       <= std::max(int(_short_term_total_time/_short_term_interval*_judge_stay_ratio),1)) { // 没有足够数量进行判断
        return ;
    } else { /*do something below*/ }
    int max_nums = std::max(int(_short_term_total_time/_short_term_interval*_judge_stay_ratio),1);
    int stay_nums = 0;
    for(int i=0; i<obj_track.short_term_locations.size()-1; ++i) {
        if(getIOU(obj_track.short_term_locations.at(i).second,
                  obj_track.short_term_locations.rbegin()->second) >= _judge_short_stay_iou) {
            ++stay_nums;
        }
    }
    if(stay_nums >= max_nums) {
        if(obj_track.long_term_locations.size() == 0) {
            obj_track.long_term_locations.emplace_back(*obj_track.short_term_locations.rbegin());
        } else if(getIOU( obj_track.short_term_locations.rbegin()->second,
                          obj_track.long_term_locations.rbegin()->second) >= _judge_long_stay_iou) {
            obj_track.long_term_locations.emplace_back(*obj_track.short_term_locations.rbegin());
        } else { /*no operation*/ }
    }
}

void HWObjTrackerPrivate::correctRect(cv::Rect& rect, const double& ratio) {
    if(rect.width*1.0/rect.height > 2) {
        cv::Rect tmp_rect = rect;
        rect.height = tmp_rect.height;
        rect.width = tmp_rect.height * ratio;
        rect.x = tmp_rect.x + tmp_rect.width/2 - rect.width/2;
        rect.y = tmp_rect.y;
    } else { /*no operation*/ }
}
/// HWObjTracker public
HWObjTracker::HWObjTracker(const cz::ParaReader::Ptr& reader_ptr) {
    _private_ptr.reset(new HWObjTrackerPrivate(reader_ptr));
}
HWObjTracker::~HWObjTracker() {

}
void HWObjTracker::setTrackRegion(const std::vector<REGION_DESC>& region_desc) {
    _private_ptr->setTrackRegion(region_desc);
}

void HWObjTracker::track(const cv::Mat& frame, const Time::UINT& time_stamp,
                         const std::vector<cz::ml::Rect>& obj_dets, std::vector<HWObjTrack>& obj_tracks) {
    obj_tracks = _private_ptr->track(frame, time_stamp, obj_dets);
}

}
}
