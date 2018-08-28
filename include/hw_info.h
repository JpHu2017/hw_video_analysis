/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_VIDEO_ANALYSIS_HW_INFO_H
#define CZ_VIDEO_ANALYSIS_HW_INFO_H
#include <chrono>
#include <opencv2/core.hpp>

namespace cz {
namespace va {

typedef cv::Point POINT;
//计时器
class Time {
public:
    typedef double UINT;
    //求时间差t2-t1，单位：ms
    static double timeInterval(const UINT& t2, const UINT& t1) {
        return (t2-t1);
    }
};

//程序运行配置信息
typedef struct _UIParaInfo {
    int channel;
    int width;
    int height;
    int ipCamId;
    char ip[15];

    bool bIllegalParking; //违章停车
    bool bAbandonedDetect; //遗留物检测
    bool bSprinklingDetect; //抛洒物检测
    bool bPedestrianDetect; //行人检测
    bool bRetrograde; //逆行检测
    bool bDeviceException; //设备异常
} UIParaInfo;

//检测区域描述信息
typedef struct _REGION_DESC {
    const static int MAX_POINT_COUNT = 20; //最大点数
    int nPointCount; // 检测区域的点数
    POINT pt[MAX_POINT_COUNT]; //检测区域点的像素坐标
    int direct; //当前区域方向
    POINT s,e; //车辆行驶正方向
    int regionProp; //区域属性（行车道，应急车道）
} REGION_DESC;

typedef struct _SPEED_REFER_LINE {
    const static int MAX_POINT_COUNT = 20;
    int nPointCount;
    POINT pt[MAX_POINT_COUNT]; //速度参考点数组
    double referDistance[MAX_POINT_COUNT]; //速度参考点分别对应的距离
    int direct; //当前区域方向（右区域，左区域->正方向，反方向）
    POINT s,e; //车辆行驶正方向
} SPEED_REFER_LINE;

//物件位置信息
typedef struct _MONITOR_OBJECT_POS {
    POINT posNow, posLast; //当前位置和上一次位置
    int dist; //两次位置之间的距离
    int speed; //速度
    char direct; //物件所在区域方向
    int objectId; //物件ID
    int type; //物件类型(如客车，行人等)
} MONITOR_OBJECT_POS;

//////////////////////////////
// alarmType字段含义说明
// 1. 代表道路拥挤
// 2. 代表道路阻塞
// 3. 代表应急车道停车
// 4. 代表遗留物
// 5. 代表违章停车
// 6. 代表行人异常穿越
// 7. 代表逆行检测
// 8. 代表设备异常
// 9. 代表抛洒物
//////////////////////////////
//告警结果信息
typedef struct _MONITOR_RESULT_ALARM {
    int alarmIndex; //告警编号
    int alarmType; //告警类型(如违章停车，逆行等)
    int width; //告警物件的宽度
    int height; //告警物件的高度
    int objectId; //告警物件ID

    Time::UINT tsEvent; //告警时间发生时间
    int eventTime; //告警事件持续时间
    int eventType; //告警事件类型
    POINT objStartPos; //发生事件时，物件的起始位置
    POINT objCurPos;  //物件的当前位置
    int direct; //物件所在方向

} MONITOR_RESULT_ALARM;
//结果信息
typedef struct _MONITOR_RESULT_INFO {
    Time::UINT tsNow; //当前时间
    Time::UINT tsLast; //上一次时间
    int leftCarInFrame; //当前帧左区域车辆数
    int rightCarInFrame; //当前帧右区域车辆数
    int monitorCarInFrame; //当前帧总区域车辆数
    int leftCarStat; //左区域车辆总数
    int rightCarStat; //右区域车辆总数
    int monitorCarStat; //当前帧总区域车辆总数
    int ipCamId; //摄像头ID
    int alarmCount; //当前的告警事件数
    int diveceState; //设备状态
    MONITOR_OBJECT_POS *objPos; //当前帧内检测出的物件信息
    MONITOR_RESULT_ALARM *alarmInfo; //当前帧内检测出的告警信息
} MONITOR_RESULT_INFO;
}
}
#endif //CZ_VIDEO_ANALYSIS_HW_INFO_H
