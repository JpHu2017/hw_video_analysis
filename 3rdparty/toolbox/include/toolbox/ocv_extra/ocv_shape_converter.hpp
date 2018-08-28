/**
* @Copyright (c) 2018 by JpHu
* @Date 2018-5-31
* @Company CuiZhou
* @Email hujp@cuizhouai.com
* @Function : opencv与cz的图形库转换
*/

#ifndef CZ_TOOLS_OCV_SHAPE_CONVERTER_H
#define CZ_TOOLS_OCV_SHAPE_CONVERTER_H

#include <memory>
#include <opencv2/core.hpp>
#include "shape/shape.h"
namespace cz {
namespace lt {

template <class T>
class ShapeConverter {
public:
    typedef std::shared_ptr<ShapeConverter> SharedPtr;
    explicit ShapeConverter(){}
    static void convert(const cv::Point_<T>& ocv_pt, Point& pt) {}
    static void convert(const std::pair<cv::Point_<T>,cv::Point_<T> >& ocv_l,
                         Line& l) {}
    static void convert(const std::pair<cv::Point_<T>,cv::Point_<T> >& ocv_s,
                         Segment& s) {}
    static void convert(const std::vector<cv::Point_<T>>& ocv_pl,
                         Polyline& pl) {}
    static void convert(const std::vector<cv::Point_<T> >& ocv_outer,
                         Polygon& pg,
                         const std::vector<cv::Point_<T> >& ocv_inner = std::vector<cv::Point_<int> >()) {}
    static void convert(const std::vector<cv::Point_<T> >& ocv_outer,
                        const std::vector<cv::Point_<T> >& ocv_inner,
                        Polygon& pg) {}
    static void convert(const cv::Rect_<T>& ocv_rect,
                         Rect& r) {}
    static void convert(const cv::Point_<T>& ocv_cen, const T& ocv_r,
                         Circle& c) {}

    static void convertInv(const Point& pt, cv::Point_<T>& ocv_pt) {}
    static void convertInv(const Line& l,
                            std::pair<cv::Point_<T>,cv::Point_<T> >& ocv_l) {}
    static void convertInv(const Segment& s,
                            std::pair<cv::Point_<T>,cv::Point_<T> >& ocv_s) {}
    static void convertInv(const Polyline& pl,
                         std::vector<cv::Point_<T>>& ocv_pl) {}
    static void convertInv(const Polygon& pg,
                            std::vector<cv::Point_<T> >& ocv_outer,
                            std::vector<cv::Point_<T> >& ocv_inner) {}
    static void convertInv(const Rect& r,
                            cv::Rect_<T>& ocv_rect) {}
    static void convertInv(const Circle& c,
                         cv::Point_<T>& ocv_cen,
                         T& ocv_r) {}
private:
};

template <>
class ShapeConverter<int> {
public:
    typedef std::shared_ptr<ShapeConverter> SharedPtr;
    explicit ShapeConverter(){}
    static void convert(const cv::Point_<int>& ocv_pt, Point& pt) {
        pt.setPoint(ocv_pt.x, ocv_pt.y);
    }
    static void convert(const std::pair<cv::Point_<int>,cv::Point_<int> >& ocv_l,
                         Line& l) {
        Point pt1, pt2;
        convert(ocv_l.first, pt1);
        convert(ocv_l.second, pt2);
        l.setLine(pt1, pt2);
    }
    static void convert(const std::pair<cv::Point_<int>,cv::Point_<int> >& ocv_s,
                         Segment& s) {
        Point pt1, pt2;
        convert(ocv_s.first, pt1);
        convert(ocv_s.second, pt2);
        s.setSegment(pt1, pt2);
    }
    static void convert(const std::vector<cv::Point_<int> >& ocv_pl,
                         Polyline& pl) {
        std::vector<Point> pts;
        for(auto e : ocv_pl) {
            Point pt;
            convert(e, pt);
            pts.emplace_back(pt);
        }
        pl.setPolyline(pts);
    }
    static void convert(const std::vector<cv::Point_<int> >& ocv_outer,
                         Polygon& pg,
                         const std::vector<cv::Point_<int> >& ocv_inner = std::vector<cv::Point_<int> >()) {
        Polyline outer, inner;
        convert(ocv_outer, outer);
        convert(ocv_inner, inner);
        pg.setPolygon(outer, inner);
    }
    static void convert(const std::vector<cv::Point_<int> >& ocv_outer,
                        const std::vector<cv::Point_<int> >& ocv_inner,
                        Polygon& pg) {
        convert(ocv_outer, ocv_inner, pg);
    }
    static void convert(const cv::Rect_<int>& ocv_rect,
                         Rect& r) {
        Point tl,br;
        convert(ocv_rect.tl(), tl);
        convert(ocv_rect.br(), br);
        r.setRect(tl, br);
    }
    static void convert(const cv::Point_<int>& ocv_cen, const int& ocv_r,
                         Circle& c) {
        Point cen;
        convert(ocv_cen, cen);
        c.setCircle(cen, ocv_r);
    }

    static void convertInv(const Point& pt, cv::Point_<int>& ocv_pt) {
        ocv_pt = cv::Point(pt.x(), pt.y());
    }
    static void convertInv(const Line& l,
                            std::pair<cv::Point_<int>,cv::Point_<int> >& ocv_l) {
        convertInv(l.pt1(), ocv_l.first);
        convertInv(l.pt2(), ocv_l.second);
    }
    static void convertInv(const Segment& s,
                            std::pair<cv::Point_<int>,cv::Point_<int> >& ocv_s) {
        convertInv(s.startPt(), ocv_s.first);
        convertInv(s.endPt(), ocv_s.second);
    }
    static void convertInv(const Polyline& pl,
                            std::vector<cv::Point_<int> >& ocv_pl) {
        ocv_pl.clear();
        std::vector<Point> pts;
        pl.pts(pts);
        for(auto e : pts) {
            cv::Point ocv_pt;
            convertInv(e, ocv_pt);
            ocv_pl.push_back(ocv_pt);
        }
    }
    static void convertInv(const Polygon& pg,
                            std::vector<cv::Point_<int> >& ocv_outer,
                            std::vector<cv::Point_<int> >& ocv_inner) {
        ocv_inner.clear();
        ocv_outer.clear();
        convertInv(pg.outer(), ocv_outer);
        convertInv(pg.inner(), ocv_inner);
    }
    static void convertInv(const Rect& r,
                            cv::Rect_<int>& ocv_rect) {
        cv::Point tl, br;
        convertInv(r.tl(), tl);
        convertInv(r.br(), br);
        ocv_rect = cv::Rect(tl, br);
    }
    static void convertInv(const Circle& c,
                            cv::Point_<int>& ocv_cen,
                            int& ocv_r) {
        convertInv(c.center(), ocv_cen);
        ocv_r = c.radius();
    }

private:
};

}
}


#endif //CZ_TOOLS_OCV_SHAPE_CONVERTER_H
