#ifndef __CZ_ML_ICLASSIFIER_H_
#define __CZ_ML_ICLASSIFIER_H_
#include <string>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include "common/core.h"
#include "czml/utils/ModelParameter.h"
#include "Classification.h"

namespace cz {
namespace ml {

class CZ_EXPORTS IClassifier {
public:
    typedef boost::shared_ptr<IClassifier> Ptr;
    IClassifier() {}
    virtual ~IClassifier() {}
    
    virtual std::string getNetName() const = 0;
    virtual bool loadModel(const ModelParameter& param) = 0;
    virtual bool classify(const cv::Mat& image_in, Classification& classification_out, int top_n = 1) = 0;
    virtual bool multiClassify(const std::vector<cv::Mat>& image_vec_in, std::vector<Classification>& classifications_out, int top_n = 1) = 0;

    static IClassifier::Ptr createFromName(const std::string& netName);
};

class czBaseClassifierFactory{
public:
    typedef boost::shared_ptr<czBaseClassifierFactory> Ptr;
    czBaseClassifierFactory(){}
	virtual ~czBaseClassifierFactory(){}
    virtual IClassifier::Ptr createClassifier() = 0;
};
class CZ_EXPORTS czClassifierRegisterAction{
public:
    czClassifierRegisterAction(const std::string& className, czBaseClassifierFactory* factory);
};

#define CZ_CLASSIFIER_REGISTER(className) class czClassifierFactory##className : public czBaseClassifierFactory {public: virtual IClassifier::Ptr createClassifier() {IClassifier::Ptr classifier_ptr(new className());return classifier_ptr;}};czClassifierRegisterAction g_classifierFacotryRegister##className(#className, new czClassifierFactory##className);

}
}

#endif