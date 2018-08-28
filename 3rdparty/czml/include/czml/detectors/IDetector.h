#ifndef __CZ_ML_IDETECTOR_H_
#define __CZ_ML_IDETECTOR_H_
#include <string>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include "common/core.h"
#include "czml/utils/ModelParameter.h"
#include "Detection.h"
namespace cz {
namespace ml {
class CZ_EXPORTS IDetector {
public:
    typedef boost::shared_ptr<IDetector> Ptr;
    IDetector() {}
    virtual ~IDetector() {}
    
    virtual std::string getNetName() const = 0;
    
    virtual bool loadModel(const ModelParameter& param) = 0;
    virtual bool detect(const cv::Mat& image_in, Detection& detection_out, float score_threshold) = 0;
    virtual bool multiDetect(const std::vector<cv::Mat>& image_vec_in, std::vector<Detection>& detections_out, float score_threshold) = 0;

    static IDetector::Ptr createFromName(const std::string& netName);
};

class czBaseDetectorFactory{
public:
    typedef boost::shared_ptr<czBaseDetectorFactory> Ptr;
    czBaseDetectorFactory(){}
	virtual ~czBaseDetectorFactory(){}
    virtual IDetector::Ptr createDetector() = 0;
};
class CZ_EXPORTS czDetectorRegisterAction{
public:
    czDetectorRegisterAction(const std::string& className, czBaseDetectorFactory* factory);
};

#define CZ_DETECTOR_REGISTER(className) class czDetectorFactory##className : public czBaseDetectorFactory {public: virtual IDetector::Ptr createDetector() {IDetector::Ptr detector_ptr(new className());return detector_ptr;}};czDetectorRegisterAction g_detectorFacotryRegister##className(#className, new czDetectorFactory##className);

}
}

#endif