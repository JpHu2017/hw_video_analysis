#ifndef _CZ_ML_DETECTUTIL_H
#define _CZ_ML_DETECTUTIL_H
#include <common/core.h>
#include <opencv2/opencv.hpp>

namespace cz {
namespace ml {

struct Rect {
    float x;
    float y;
    float width;
    float height;
    float score;
    std::string type;
};
typedef std::vector<Rect> RectVec;

class CZ_EXPORTS Detection {
public:
    Detection();
    ~Detection();
    
    bool isGood() {return !_rect_vec.empty();}

    std::vector<Rect>& getRects() {return _rect_vec;}
    const std::vector<Rect>& getRects() const {return _rect_vec;}

    std::vector<Rect> filter(const std::string& type_mask, const cv::Rect& roi_rect = cv::Rect(0,0,0,0));
    std::vector<Rect> filter(const std::set<std::string>& type_masks, const cv::Rect& roi_rect = cv::Rect(0,0,0,0));

    enum SortRectPolicy{ SORT_BY_X, SORT_BY_Y, SORT_BY_AREA, SORT_BY_SCORE };
    void sortRects(SortRectPolicy sort_policy);
        
    static void drawRects(cv::Mat& image, const RectVec& rects,
                bool draw_type = true, bool draw_score = true,
                cv::Scalar rect_clr = cv::Scalar(0, 255, 0), 
                cv::Scalar text_clr = cv::Scalar(255, 0, 0),
                cv::Point2f offset = cv::Point2f(0.0, 0.0)); 
    static void drawRect(cv::Mat& image, const Rect& rect,
                bool draw_type = true, bool draw_score = true,
                cv::Scalar rect_clr = cv::Scalar(0, 255, 0), 
                cv::Scalar text_clr = cv::Scalar(255, 0, 0),
                cv::Point2f offset = cv::Point2f(0.0, 0.0));
    
    void checkRectEdge(int img_width, int img_height, int marrin = 0);

private:
    std::vector<Rect> _rect_vec;
};
}
}
#endif