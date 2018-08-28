/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-1
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_TOOLS_OCV_SHAPE_HANDLER_H
#define CZ_TOOLS_OCV_SHAPE_HANDLER_H

#include <memory>
#include <opencv2/core.hpp>
#include "ocv_extra/ocv_shape_converter.hpp"
#include "shape/shape.h"

namespace cz {
namespace lt {

class ShapeHandler {
public:
    typedef std::shared_ptr<ShapeHandler> SharedPtr;
    typedef cv::Mat CImage;
    typedef cv::Scalar CColor;
    ShapeHandler();

    /// Origin OpenCV
    // 图像绘制
    static void drawLine(cv::Mat& img, const std::pair<cv::Point, cv::Point>& ocv_l, const cv::Scalar& c, const int& w);
    static void drawPolyline(cv::Mat& img, const std::vector<cv::Point>& ocv_pl, const cv::Scalar& c, const int& w);
    static void drawRect(cv::Mat& img, const cv::Rect& r, const cv::Scalar& c, const int& w);
    static void drawCircle(cv::Mat& img, const cv::Point& cen, const double& r, const cv::Scalar& c, const int& w);
    // 图形选择
    static void selectLine(const cv::Mat& img, std::pair<cv::Point, cv::Point>& ocv_l, const double& scale = 1.0);
    static void selectPolyline(const cv::Mat& img, std::vector<cv::Point>& pl, const double& scale = 1.0);
    static void selectRect(const cv::Mat& img, cv::Rect& r, const double& scale = 1.0);
    static void selectCircle(const cv::Mat& img, cv::Point& cen, double& r, const double& scale = 1.0);
    // 图形兴趣区
    static void polygonRoi(const cv::Mat& src, cv::Mat& dst, const std::vector<cv::Point>& outer,
                           const std::vector<cv::Point>& inner = std::vector<cv::Point>());
    static void rectRoi(const cv::Mat& src, cv::Mat& dst, const cv::Rect& r);

    /// Shape Defined by myself
    // 图形选择
    static void selectLine(const CImage& img, Line& l, const double& scale = 1.0);
    static void selectPolyline(const CImage& img, Polyline& pl, const double& scale = 1.0);
    static void selectRect(const CImage& img, Rect& r, const double& scale = 1.0);
    static void selectCircle(const CImage& img, Circle& c, const double& scale = 1.0);
    // 图像绘制
    static void drawLine(CImage& img, const Line& l, const CColor& c, const int& w);
    static void drawPolyline(CImage& img, const Polyline& pl, const CColor& c, const int& w);
    static void drawPolygon(CImage& img, const Polygon& pg, const CColor& c, const int& w);
    static void drawRect(CImage& img, const Rect& r, const CColor& c, const int& w);
    static void drawCircle(CImage& img, const Circle& cc, const CColor& c, const int& w);
    static void drawCircle(CImage& img, const Point& pt, const Circle::T& r, const CColor& c, const int& w);
    // 图形兴趣区
    static void polygonRoi(const CImage& src, CImage& dst, const Polygon& pg);
    static void rectRoi(const CImage& src, CImage& dst, const Rect& r);

private:
    static ShapeConverter<int>::SharedPtr _converter_ptr;
};

}
}

#endif //CZ_TOOLS_OCV_SHAPE_HANDLER_H
