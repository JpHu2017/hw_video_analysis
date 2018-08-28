/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-6-13
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
  """
 *  This class represents a bounding box detection in a single image.
 *
 *  Parameters
 *  ----------
 *  _tlwh : array_like
 *      Bounding box in format `(x, y, w, h)`.
 *  _confidence : float
 *      Detector confidence score.
 *  _feature : array_like
 *      A feature vector that describes the object contained in this image.
 *
 *  Attributes
 *  ----------
 *  tlwh : Eigen::Matrix<float, 1, 4, Eigen::RowMajor>
 *      Bounding box in format `(top left x, top left y, width, height)`.
 *  confidence : float
 *      Detector confidence score.
 *  feature : Eigen::Matrix<float, 1, 4, Eigen::RowMajor> | NULL
 *      A feature vector that describes the object contained in this image.
 *
   """
 * */

#ifndef DEEPSORT_DETECTION_H
#define DEEPSORT_DETECTION_H

#include <glog/logging.h>
#include <vector>
#include <Eigen/Eigen>
namespace cz {
namespace ht {

class DsDetection {
public:
    DsDetection() {}
    DsDetection(const Eigen::RowVector4f& tlwh, const float& conf, const Eigen::RowVectorXf& feature = Eigen::RowVectorXf())
            : _tlwh(tlwh), _conf(conf), _feature(feature) {}
    ~DsDetection() {}
    void setDetetion(const Eigen::RowVector4f& tlwh, const float& conf, const Eigen::RowVectorXf& feature = Eigen::RowVectorXf()) {
        _tlwh = tlwh;
        _conf = conf;
        _feature = feature;
    }
    Eigen::RowVector4f toTlbr() const {
        Eigen::RowVector4f ret = _tlwh;
        ret(0,2) += ret(0,0);
        ret(0,3) += ret(0,1);
        return ret;
    }
    Eigen::RowVector4f toXyah() const {
        Eigen::RowVector4f ret = _tlwh;
        ret(0, 0) += ret(0, 2) / 2;
        ret(0, 1) += ret(0, 3) / 2;
        if(ret(0,3) == 0) {
            LOG(FATAL) << "Detection height can't be 0.";
        }
        ret(0, 2) /= ret(0, 3);
        return ret;
    }
    Eigen::RowVector4f toTlwh() const {
        return _tlwh;
    }
    Eigen::RowVectorXf feature() const {
        return _feature;
    }
private:
    Eigen::RowVector4f _tlwh;
    float _conf;
    Eigen::RowVectorXf _feature;
};

}
}


#endif //DEEPSORT_DETECTION_H
