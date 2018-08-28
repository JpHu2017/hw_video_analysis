/**
* @Copyright (c) 2018 by JpHu
* @Date 2018-5-31
* @Company CuiZhou
* @Email hujp@cuizhouai.com
* @Function
*/

#ifndef CZ_TOOLS_SHAPE_HPP
#define CZ_TOOLS_SHAPE_HPP

#include <memory>
#include <vector>
namespace cz {
namespace lt {
//
#define CT_EPS 0.000001
class Point {
public:
    typedef double T;
    typedef std::shared_ptr<Point> SharedPtr;
    explicit Point() {}
    explicit Point(const T& x, const T& y);
    ~Point() {}
    void setPoint(const T& x, const T& y);
    void setX(const T& x);
    void setY(const T& y);
    T x() const;
    T y() const;
private:
    T _x;
    T _y;
};

class Line {
public:
    typedef std::shared_ptr<Line> SharedPtr;
    explicit Line() {}
    explicit Line(const Point& pt1, const Point& pt2);
    explicit Line(const double& a, const double& b, const double& c);
    ~Line() {}
    void setLine(const Point& pt1, const Point& pt2);
    void setCoef(const double& a, const double& b, const double& c);
    Point pt1() const;
    Point pt2() const;
    void coef(double& a, double& b, double& c) const;

private:
    void coefOfLine(const Point& pt1, const Point& pt2,
                    double& a, double& b, double& c);
    double _coef_a;
    double _coef_b;
    double _coef_c;
    Point _pt1;
    Point _pt2;
};

// 线段
class Segment : public Line {
public:
    typedef std::shared_ptr<Segment> SharedPtr;
    explicit Segment() {}
    explicit Segment(const Point& s_pt, const Point& e_pt);
    ~Segment() {}
    void setSegment(const Point& s_pt, const Point& e_pt);
    Line line() const;
    Point startPt() const;
    Point endPt() const;
private:
    Point _s_pt;
    Point _e_pt;
};

// 多线
/*
 * 注：若多边形首，尾点相同，则闭合
 */
class Polyline {
public :
    typedef std::shared_ptr<Polyline> SharedPtr;
    explicit Polyline() {
        _pts.clear(); //点清空，size=0
    }
    explicit Polyline(const std::vector<Point>& pts);
    ~Polyline() {}
    void setPolyline(const std::vector<Point>& pts);
    void close();
    //是否闭合
    bool isClosed() const;
    void pts(std::vector<Point>& pts_) const;
    std::vector<Point> pts() const;
private:
    std::vector<Point> _pts;
};
// 多边形
/*
 * 为了处理环形多边形，故包含内，外轮廓线
 * 注：多边形的边线有重合点
 */
class Polygon {
public:
    typedef std::shared_ptr<Polygon> SharedPtr;
    explicit Polygon() {}
    //inner为空，普通多边形；inner不为空，环形多边形
    explicit Polygon(const Polyline& outer, const Polyline& inner = Polyline());
    ~Polygon() {}
    void setPolygon(const Polyline& outer, const Polyline& inner = Polyline());
    bool isAnnular();
    Polyline inner() const;
    Polyline outer() const;
private:
    Polyline _inner; //内轮廓线
    Polyline _outer; //外轮廓线
};

class Rect {
public:
    typedef std::shared_ptr<Rect> SharedPtr;
    typedef Point::T T;
    explicit Rect(){}
    explicit Rect(const Point& tl, const Point& br);
    explicit Rect(const T& x, const T& y, const T& width, const T& height);
    ~Rect() {}
    void setRect(const Point& tl, const Point& br);
    void setRect(const T& x, const T& y, const T& width, const T& height);
    void setTl(const Point& tl);
    void setBr(const Point& br);
    Point tl() const;
    Point br() const;
    T x() const;
    T y() const;
    T width() const;
    T height() const;
private:
    void setRectAux(const Point& tl, const Point& br);
    void setRectAux(const T& x, const T& y, const T& width, const T& height);
    Point _tl; // top left corner
    Point _br; // bottom right corner
    T _x;
    T _y;
    T _width;
    T _height;
};
class Circle {
public:
    typedef Point::T T;
    explicit Circle() {}
    explicit Circle(const Point& cen_pt, const T& r);
    ~Circle() {}
    void setCenter(const Point& cen_pt);
    void setRadius(const T& r);
    void setCircle(const Point& cen_pt, const T& r);
    Point center() const;
    T radius() const;
private:
    Point _cen_pt;
    T _r;
};

}
}

#endif //CZ_TOOLS_SHAPE_HPP
