/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-6-15
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef DEEPSORT_TRACK_H
#define DEEPSORT_TRACK_H

#include <Eigen/Eigen>
#include "ds_detection.h"
#include "kalman_filter.h"

namespace cz {
namespace ht {

class DsTrack {
public:
    enum TrackState {Tentative, Confirmed, Deleted};
    DsTrack(const Eigen::RowVectorXf& mean, const Eigen::MatrixXf& covariance,
            const int& track_id, const int& n_init, const int& max_age,
            const Eigen::RowVectorXf& feature= Eigen::RowVectorXf());
    ~DsTrack() {}
    Eigen::RowVector4f toTlwh() const;
    Eigen::RowVector4f toTlbr() const;
    void predict();
    void update(const DsDetection& det);
    TrackState markMissed();
    bool isTentative();
    bool isConfirmed();
    bool isDeleted();
    int timeSinceUpdate() const;
    Eigen::RowVectorXf mean();
    Eigen::MatrixXf covariance();
    int trackId() const;
    std::vector<Eigen::RowVectorXf>& features();

private:
    Eigen::RowVectorXf _mean;
    Eigen::MatrixXf _covariance;
    int _track_id;
    int _hits;
    int _age;
    int _time_since_update;
    TrackState _state;
    // 用std::vector，而不用DsFeatureVector，是为了能够动态增长
    std::vector<Eigen::RowVectorXf> _features;
    int _n_init;
    int _max_age;
    KalmanFilter::SharedPtr _kf_ptr;
};

}
}

#endif //DEEPSORT_TRACK_H
