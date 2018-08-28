/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-6-15
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef DEEPSORT_KALMAN_FILTER_H
#define DEEPSORT_KALMAN_FILTER_H

#include <Eigen/Eigen>
#include <map>
#include <memory>
//#include <alias.h>

namespace cz {
namespace ht {
//std::map<int, float> chi2inv95;
class KalmanFilter {
public:
    typedef std::shared_ptr<KalmanFilter> SharedPtr;

    KalmanFilter();
    ~KalmanFilter();
    void initiate(const Eigen::RowVectorXf& measurement, Eigen::RowVectorXf& mean, Eigen::MatrixXf& covariance);
    void predict(const Eigen::RowVectorXf& src_mean, const Eigen::MatrixXf& src_covariance,
                 Eigen::RowVectorXf& dst_mean, Eigen::MatrixXf& dst_covariance);
    void project(const Eigen::RowVectorXf& src_mean, const Eigen::MatrixXf& src_covariance,
                 Eigen::RowVectorXf& dst_mean, Eigen::MatrixXf& dst_covariance);
    void update(const Eigen::RowVectorXf& src_mean, const Eigen::MatrixXf& src_covariance,
                const Eigen::RowVectorXf& measurement,
                 Eigen::RowVectorXf& dst_mean, Eigen::MatrixXf& dst_covariance);
    void gating_distance(const Eigen::RowVectorXf& mean, const Eigen::MatrixXf& convariance,
                         const Eigen::MatrixXf& measurements,
                         Eigen::RowVectorXf& squared_maha, const bool& only_position=false);
private:
    Eigen::MatrixXf _motion_mat;
    Eigen::MatrixXf _update_mat;
    float _std_weight_position;
    float _std_weight_velocity;
    static int _ndim;
};


}
}


#endif //DEEPSORT_KALMAN_FILTER_H
