/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-17
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef CZ_HEAVY_TOOLS_DS_FEATURE_GENERATOR_H
#define CZ_HEAVY_TOOLS_DS_FEATURE_GENERATOR_H

//#ifdef USE_TENSORFLOW
#include <memory>
#include <opencv2/core.hpp>
#include <string>
//#include <tensorflow/c/c_api.h>
#include "ds_detection.h"
namespace cz {
namespace ht {
class DsFeatureGeneratorPrivate;
class DsFeatureGenerator {
public:
    typedef std::shared_ptr<DsFeatureGenerator> SharedPtr;
    DsFeatureGenerator();
    ~DsFeatureGenerator();
    void init(const std::string& deploy_path, const std::string& model_path);
    void setComputeMode(const std::string& mode, const int& id);
    void generateFeature(const cv::Mat& image, Eigen::RowVectorXf& feature);
    void generateFeatureBatches(const std::vector<cv::Mat>& image_batches,
                                std::vector<Eigen::RowVectorXf>& feature_batches);
private:
    std::shared_ptr<DsFeatureGeneratorPrivate> _private_ptr;
};
}
}
//#endif

#endif //CZ_HEAVY_TOOLS_DS_FEATURE_GENERATOR_H
