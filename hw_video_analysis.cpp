/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-27
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */
#include <glog/logging.h>
#include <iostream>
#include <opencv2/core.hpp>
#include "hw_video_analyzer.h"
#include <czml/detectors/IDetector.h>
int main(int argc, char* argv[]) {
    FLAGS_alsologtostderr = false;
    google::InitGoogleLogging(argv[0]);
    std::string video_analyzer_ini = "../data/params/video_analyzer.ini";
    cz::ParaReader::Ptr reader_ptr;
    reader_ptr.reset(new cz::ParaReader(video_analyzer_ini));
    cz::va::HWVideoAnalyzer::SharedPtr video_analyzer_ptr;
    video_analyzer_ptr.reset(new cz::va::HWVideoAnalyzer(reader_ptr));
    video_analyzer_ptr->videoAnalysis();
    return 0;
}

