/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_VIDEO_ANALYSIS_HW_VIDEO_ANALYZER_H
#define CZ_VIDEO_ANALYSIS_HW_VIDEO_ANALYZER_H

#include <common/utils/cz_para_reader.h>
#include <memory>
namespace cz {
namespace va {
class HWVideoAnalyzerPrivate;
class HWVideoAnalyzer {
public:
    typedef std::shared_ptr<HWVideoAnalyzer> SharedPtr;
    HWVideoAnalyzer(const cz::ParaReader::Ptr& reader_ptr);
    ~HWVideoAnalyzer();
    void videoAnalysis();
private:
    std::shared_ptr<HWVideoAnalyzerPrivate> _private_ptr;
};
}
}



#endif //CZ_VIDEO_ANALYSIS_HW_VIDEO_ANALYZER_H
