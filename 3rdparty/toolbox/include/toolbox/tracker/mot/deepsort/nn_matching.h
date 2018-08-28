/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-6-14
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */
#ifndef DEEPSORT_NN_MATCHING_H
#define DEEPSORT_NN_MATCHING_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <glog/logging.h>
#include <map>
#include <sys/time.h>
#include <vector>
#include "ds_detection.h"

namespace cz {
namespace ht {

// 求a,b中每组特征之间的距离
/* 输入：
 * a : NxM矩阵，N组特征，一组特征中含M个值
 * b : LxM矩阵，L组特征，一组特征中含M个值
 * 输出：
 * NxL矩阵
 * */
Eigen::MatrixXf _pdist(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b);
// consine距离
Eigen::MatrixXf _consine_distance(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b,
                       const bool data_is_normalized=false);
// 求y与x比较的中最小欧式距离
/* 输入：
 * a : NxM矩阵，N组特征，一组特征中含M个值
 * b : LxM矩阵，L组特征，一组特征中含M个值
 * 输出：
 * 长度为L的向量
 * */
Eigen::VectorXf _nn_euclidean_distance(const Eigen::MatrixXf& x, const Eigen::MatrixXf& y);
// 求y与x比较的中最小cosine距离
/* 输入：
 * a : NxM矩阵，N组特征，一组特征中含M个值
 * b : LxM矩阵，L组特征，一组特征中含M个值
 * 输出：
 * 长度为L的向量
 * */
Eigen::VectorXf _nn_consine_distance(const Eigen::MatrixXf& x, const Eigen::MatrixXf& y);

typedef Eigen::VectorXf (*Metric)(const Eigen::MatrixXf& x, const Eigen::MatrixXf& y);

// 最近邻距离准则
class NearestNeighborDistanceMetric {
public:
    typedef std::shared_ptr<NearestNeighborDistanceMetric> SharedPtr;
    NearestNeighborDistanceMetric(const std::string& metric = "cosine", const float matching_threshold = 0.2, const int& budget = 100);
    void partial_fit(const std::vector<Eigen::RowVectorXf>& features,
                     const std::vector<int>& targets,
                     const std::vector<int>& active_targets);
    Eigen::MatrixXf distance(const std::vector<Eigen::RowVectorXf>& features, const std::vector<int>& targets);
    float matchingThreshold() const {
        return _matching_threshold;
    }
    int samplesSizeTest() const {
        return _samples.size();
    }
private:
    Metric _metric;
    float _matching_threshold;
    int _budget;
    std::map<int, std::vector<Eigen::RowVectorXf> > _samples;
};

}
}

#endif //DEEPSORT_NN_MATCHING_H
