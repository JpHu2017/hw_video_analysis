/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */
#include <toolbox/ocv_extra.hpp>
#include "hw_obj_detector.h"
#include "utils_funcs.h"
namespace cz {
namespace va {
class HWObjDetectorPrivate;
class HWObjDetectorPrivate {
public:
    typedef std::shared_ptr<HWObjDetectorPrivate> SharedPtr;
    HWObjDetectorPrivate(const cz::ParaReader::Ptr& reader_ptr);
    ~HWObjDetectorPrivate();
    void setDetectRegion(const std::vector<REGION_DESC>& region_desc);
    void detect(const cv::Mat& src, std::vector<cz::ml::Rect>& hw_dets);
    void drawRects(cv::Mat& src, std::vector<cz::ml::Rect>& hw_dets);
private:
    void init();
    void initDetector();
    void setParameter();
    void padImage(const cv::Mat& src, cv::Mat& dst);
    void eraseAnormalDets(std::vector<cz::ml::Rect>& obj_dets, const double& normal_rect_max_thresh,
                          const double& normal_rect_min_thresh);
    std::string _detector_type;
    std::string _detector_path;
    bool _gpu_mode;
    int _gpu_id;
    float _conf_thresh;
    int _region_clip_thresh;
    std::vector<cv::Rect> _detect_rects;
    std::set<std::string> _filter_mask;
    bool _rects_edge_check;
    float _normal_rect_max_thresh;
    float _normal_rect_min_thresh;
    cz::ml::IDetector::Ptr _detector_ptr;
    cz::ParaReader::Ptr _reader_ptr;
};
/// HWObjDetectorPrivate pbulic
HWObjDetectorPrivate::HWObjDetectorPrivate(const cz::ParaReader::Ptr& reader_ptr) {
    _reader_ptr = reader_ptr;
    _rects_edge_check = false; //检测区域是否做过边缘检查
    init();
}
HWObjDetectorPrivate::~HWObjDetectorPrivate() {

}
void HWObjDetectorPrivate::setDetectRegion(const std::vector<REGION_DESC>& region_desc) {
    std::map<int, std::vector<cv::Point> > detect_regions;
    std::vector<cv::Point> total_region;
    for(int i=0; i<region_desc.size(); ++i) {
        if(detect_regions.count(region_desc.at(i).direct) == 0) {
            detect_regions.insert(std::make_pair(region_desc.at(i).direct, std::vector<cv::Point>()));
        } else { /*no operation*/ }
        for(int j=0; j<region_desc.at(i).nPointCount; ++j) {
            detect_regions[region_desc.at(i).direct].emplace_back(cv::Point(region_desc.at(i).pt[j].x,
                                                                            region_desc.at(i).pt[j].y));
            total_region.emplace_back(cv::Point(region_desc.at(i).pt[j].x,
                                                region_desc.at(i).pt[j].y));
        }
    }
    _detect_rects.clear();
    int width = 0, height = 0;
    width = cv::boundingRect(total_region).width;
    height = cv::boundingRect(total_region).height;
    //设定检测区域
    if(width <= _region_clip_thresh && height <= _region_clip_thresh) {
        _detect_rects.emplace_back(cv::boundingRect(total_region));
    } else {
        for(std::map<int, std::vector<cv::Point> >::iterator iter=detect_regions.begin();
            iter!=detect_regions.end(); ++iter) {
            _detect_rects.emplace_back(cv::boundingRect(iter->second));
        }
    }
}
void HWObjDetectorPrivate::detect(const cv::Mat& src, std::vector<cz::ml::Rect>& hw_dets) {
    //仅在初始检测时，做边缘检查
    if(_rects_edge_check == false) {
        for(int i=0; i<_detect_rects.size(); ++i) {
            cv::Rect rect;
            cz::lt::edgeCheck(_detect_rects.at(i), rect, src.cols, src.rows);
            _detect_rects[i] = rect;
        }
        _rects_edge_check = true;
    }
    cz::ml::Detection det;
    hw_dets.clear();
    for(int i=0; i<_detect_rects.size(); ++i) {
        cv::Mat roi;
        padImage(src(_detect_rects.at(i)), roi);
        cz::ml::Detection det;
        _detector_ptr->detect(roi, det, _conf_thresh);
        std::vector<cz::ml::Rect> rects = det.filter(_filter_mask);
        for(int j=0; j<rects.size(); ++j) {
            rects[j].x += _detect_rects.at(i).x;
            rects[j].y += _detect_rects.at(i).y;
            hw_dets.emplace_back(rects.at(j));
        }
    }
    // 重叠框滤除
    nmsDiffClasses(hw_dets);
    // 去除异常框
    eraseAnormalDets(hw_dets, _normal_rect_max_thresh, _normal_rect_min_thresh);
}
void HWObjDetectorPrivate::drawRects(cv::Mat& src, std::vector<cz::ml::Rect>& hw_dets) {
    cz::ml::Detection::drawRects(src, hw_dets);
}
void HWObjDetectorPrivate::eraseAnormalDets(std::vector<cz::ml::Rect>& obj_dets, const double& normal_rect_max_thresh,
                                            const double& normal_rect_min_thresh) {
    for(std::vector<cz::ml::Rect>::iterator iter=obj_dets.begin(); iter!=obj_dets.end(); ) {
        if(iter->width > normal_rect_max_thresh || iter->height > normal_rect_max_thresh) {
            iter = obj_dets.erase(iter);
        } else if(iter->width < normal_rect_min_thresh || iter->height < normal_rect_min_thresh) {
            iter = obj_dets.erase(iter);
        } else {
            ++iter;
        }
    }
}
/// HWObjDetectorPrivate private
void HWObjDetectorPrivate::init() {
    setParameter();
    initDetector();
}
void HWObjDetectorPrivate::initDetector() {
    cz::ml::ModelParameter parameter;
    parameter.setParam(_detector_path, _gpu_mode, _gpu_id);
    _detector_ptr = cz::ml::IDetector::createFromName(_detector_type);
    _detector_ptr->loadModel(parameter);
    _filter_mask.clear();
    _filter_mask.insert("person");
    _filter_mask.insert("bicycle");
    _filter_mask.insert("motorbike");
    _filter_mask.insert("car");
    _filter_mask.insert("bus");
    _filter_mask.insert("truck");
//    _filter_mask.insert("train");
}

void HWObjDetectorPrivate::setParameter() {
    _reader_ptr->getValue("detector_type", _detector_type);
    _reader_ptr->getValue("detector_path", _detector_path);
    std::string compute_mode;
    _reader_ptr->getValue("compute_mode", compute_mode);
    _gpu_mode = (compute_mode == "gpu" ? true : false);
    _reader_ptr->getValue("gpu_id", _gpu_id);
    _reader_ptr->getValue("conf_thresh", _conf_thresh);
    _reader_ptr->getValue("region_clip_thresh", _region_clip_thresh);
    _reader_ptr->getValue("normal_rect_max_thresh", _normal_rect_max_thresh);
    _reader_ptr->getValue("normal_rect_min_thresh", _normal_rect_min_thresh);
}

void HWObjDetectorPrivate::padImage(const cv::Mat& src, cv::Mat& dst) {
    //当长宽比超过2时
    float max_scale = double(std::max(src.rows, src.cols)) / double(std::min(src.rows, src.cols));
    if(max_scale > 2) {
        int side = std::max(src.rows, src.cols);
        dst = cv::Mat(side, side, CV_8UC3, cv::Scalar(0,0,0));
        src.copyTo(dst(cv::Rect(0,0,src.cols, src.rows))); //一次复制
    } else {
        dst = src;
    }
}
/// HWObjDetector pbulic
HWObjDetector::HWObjDetector(const cz::ParaReader::Ptr& reader_ptr) {
    _private_ptr.reset(new HWObjDetectorPrivate(reader_ptr));
}
HWObjDetector::~HWObjDetector() {

}
void HWObjDetector::setDetectRegion(const std::vector<REGION_DESC>& region_desc) {
    _private_ptr->setDetectRegion(region_desc);
}
void HWObjDetector::detect(const cv::Mat& src, std::vector<cz::ml::Rect>& hw_dets) {
    _private_ptr->detect(src, hw_dets);
}
void HWObjDetector::drawRects(cv::Mat& src, std::vector<cz::ml::Rect>& hw_dets) {
    _private_ptr->drawRects(src, hw_dets);
}
}
}