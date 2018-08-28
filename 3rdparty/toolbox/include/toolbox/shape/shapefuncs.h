/**
* @Copyright (c) 2018 by JpHu
* @Date 2018-5-31
* @Company CuiZhou
* @Email hujp@cuizhouai.com
* @Function
*/

#ifndef CZ_TOOLS_SHAPEFUNCS_H
#define CZ_TOOLS_SHAPEFUNCS_H

#include "shape/shape.h"
namespace cz {
namespace lt {

// 距离计算（非负值）
double computeDist(const Point& pt1, const Point& pt2);

// 距离计算（非负值）
double computeDist(const Point& pt, const Line& l);

// 距离计算
/* 返回值为非负值，线段上，返回0；不在线段上，返回与线段上点的最小距离
 * */
double computeDist(const Point& pt, const Segment& s);

// 判断点是否在直线上
/* measure_dist=false，返回-1，0，1，分别表示直线左侧，直线上，直线右侧
 * measure_dist=true，返回自然数，小于0表示在直线左侧，等于0表示在直线上，大于0表示在直线右侧
 * 直线的左右侧与直线方向相关，直线方向为直线的起点到终点方向
 * */
double pointLineTest(const Point& pt, const Line& l, bool measure_dist = false);

// 判断点是否在线段上
/* measure_dist=false，返回0，1，分别表示线段上，不在线段上
 * measure_dist=true，返回自然数，等于0表示在线段上，大于0表示不在线段上
 * 注意：measure_dist=false时， 该处返回值与bool型（若设定为）返回值正好相反，勿取错
 * */
double pointSegmentTest(const Point& pt, const Segment& s, bool measure_dist = false);

// 判断点是否在多线上
/* 分两种情况：多线不闭合，多线闭合
 * 第一种情况：多线不闭合
 * measure_dist=false，返回0，1，分别表示在多线上，不在多线上
 * measure_dist=true，返回自然数，等于0表示在多线上，大于0表示不在多线上
 * 注意：measure_dist=false时， 该处返回值与bool型（若设定为）返回值正好相反，勿取错
 * 第二种情况：多线闭合
 * measure_dist=false，返回-1，0，1，分别表示闭合多线外，闭合多线上，闭合多线内
 * measure_dist=false，返回自然数，分别表示闭合多线外(<0)，闭合多线上(=0)，闭合多线内(>0)
 * */
double pointPolylineTest(const Point& pt, const Polyline& pl, bool measure_dist = false);

// 判断点是否在多变形内
/* measure_dist=false，返回-1，0，1，分别表示多边形外，多边形上，多边形内
 * measure_dist=true，返回自然数，等于0表示多边形上，小于0表示多边形外，大于0表示多边形内
 * */
double pointPolygonTest(const Point& pt, const Polygon& pg, bool measure_dist = false);
// 判断点是否在矩形内
/* measure_dist=false，返回-1，0，1，分别表示矩形外，矩形上，矩形内
 * measure_dist=true，返回自然数，等于0表示矩形上，小于0表示矩形外，大于0表示矩形内
 * */
double pointRectTest(const Point& pt, const Rect& r, bool measure_dist = false);
// 判断点是否在圆形内
/* measure_dist=false，返回-1，0，1，分别表示圆形外，圆形上，圆形内
 * measure_dist=true，返回自然数，等于0表示圆形上，小于0表示圆形外，大于0表示圆形内
 * */
double pointCircleTest(const Point& pt, const Circle& c, bool measure_dist = false);
// 判断矩形是否在多边形内
double rectPolygonTest(const Rect& r, const Polygon& pg, bool measure_dist = false);

// 点在直线上的投影
void pointProjInLine(const Point& src_pt, const Line& l, Point& dst_pt);
// 直线交点
bool lineCross(const Line& l1, const Line& l2, Point& cross_pt);
// 线段交点
/*
 * （当交点在两个线段上时，返回true；没有交点，或者交点在线段延长线上返回false）
 * */
bool segmentCorss(const Segment& s1, const Segment& s2, Point& cross_pt);

void parallelLine(const Line& l, const double& dist, Line& para_l);
// 过点pt的l1的平行线
void parallelLine(const Line& l, const Point& pt, Line& para_l);
// 获取平行多线
void parallelPolyline(const Polyline& pl, const double& dist, Polyline& para_pl);
// 距离多线dist的平行线围成的多边形
void polygonAroundPolyline(const Polyline& pl, const double& dist, Polygon& pg);

}
}

#endif //CZ_TOOLS_SHAPEFUNCS_H
