/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_VIDEO_ANALYSIS_HW_PROCESS_REGULATOR_H
#define CZ_VIDEO_ANALYSIS_HW_PROCESS_REGULATOR_H

#include <memory>

namespace cz {
namespace va {
class HWProcessRegulatorPrivate;
class HWProcessRegulator {
public:
    typedef std::shared_ptr<HWProcessRegulator> SharedPtr;
    HWProcessRegulator();
    ~HWProcessRegulator();
    void setFps(const double& fps);
    void tic();
    void toc();
    void frameInterval(int& frame_interval);
private:
    std::shared_ptr<HWProcessRegulatorPrivate> _private_ptr;
};
}
}

#endif //CZ_VIDEO_ANALYSIS_HW_PROCESS_REGULATOR_H
