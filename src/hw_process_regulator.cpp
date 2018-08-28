/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */
#include <algorithm>
#include <chrono>
#include <glog/logging.h>
#include <vector>
#include "hw_process_regulator.h"
namespace cz {
namespace va {
class HWProcessRegulatorPrivate;
class HWProcessRegulatorPrivate {
public:
    typedef std::shared_ptr<HWProcessRegulatorPrivate> SharedPtr;
    HWProcessRegulatorPrivate();
    ~HWProcessRegulatorPrivate();
    void setFps(const double& fps);
    void tic();
    void toc();
    void frameInterval(int& frame_interval);
private:
    std::chrono::high_resolution_clock::time_point _tic_time;
    std::chrono::high_resolution_clock::time_point _toc_time;
    double _fps;
    std::vector<double> _spend_times;
    static int MAX_SIZE;
};
int HWProcessRegulatorPrivate::MAX_SIZE = 5;

HWProcessRegulatorPrivate::HWProcessRegulatorPrivate() {

}
HWProcessRegulatorPrivate::~HWProcessRegulatorPrivate() {

}
void HWProcessRegulatorPrivate::setFps(const double& fps) {
    _fps = fps;
}
void HWProcessRegulatorPrivate::tic() {
    _tic_time = std::chrono::high_resolution_clock::now();
}
void HWProcessRegulatorPrivate::toc() {
    _toc_time = std::chrono::high_resolution_clock::now();
}
void HWProcessRegulatorPrivate::frameInterval(int& frame_interval) {
    if(_spend_times.size() == 0) {
        _spend_times.emplace_back(33); // initialize
        frame_interval = 1;
    } else {
        _spend_times.emplace_back((_toc_time-_tic_time).count()*1e-6); // ms
//        LOG(ERROR) << "Total time : " << (_toc_time-_tic_time).count()*1e-6;
        if(_spend_times.size() > MAX_SIZE) {
            _spend_times.erase(_spend_times.begin());
        }
        double sum = std::accumulate(_spend_times.begin(), _spend_times.end(), 0.0);
        double mean_time = sum / _spend_times.size();
        frame_interval  = int(30/(1000/mean_time));
    }
}
/// HWProcessRegulator public
HWProcessRegulator::HWProcessRegulator() {
    _private_ptr.reset(new HWProcessRegulatorPrivate());
}
HWProcessRegulator::~HWProcessRegulator() {

}
void HWProcessRegulator::setFps(const double& fps) {
     _private_ptr->setFps(fps);
}
void HWProcessRegulator::tic() {
     _private_ptr->tic();
}
void HWProcessRegulator::toc() {
    _private_ptr->toc();
}
void HWProcessRegulator::frameInterval(int& frame_interval) {
    _private_ptr->frameInterval(frame_interval);
}
}
}
