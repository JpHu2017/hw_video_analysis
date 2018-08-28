/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-6-15
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef DEEPSORT_TRACKER_H
#define DEEPSORT_TRACKER_H

#include <Eigen/Eigen>
#include <memory>
#include "ds_detection.h"
#include "ds_track.h"
#include "iou_matching.h"
#include "kalman_filter.h"
#include "linear_assignment.h"
#include "nn_matching.h"


namespace cz {
namespace ht {

class DsTracker {
public:
    typedef std::shared_ptr<DsTracker> SharedPtr;
    DsTracker (const NearestNeighborDistanceMetric& metric, const float& max_iou_distance = 0.7,
             const int& max_age=30, const int& n_init = 3);
    ~DsTracker();
    void predict();
    std::vector<DsTrack> tracks() const;
    void update(const std::vector<DsDetection>& detections);
private:
    void match(const std::vector<DsDetection>& detections, std::vector<std::pair<int,int> >& matches,
               std::vector<int>& unmatched_tracks, std::vector<int>& unmatched_detections);
    Eigen::MatrixXf gated_metric(const std::vector<DsTrack>& tracks, const std::vector<DsDetection>& dets,
                                 const std::vector<int>& track_indices, const std::vector<int>& detection_indices);

    void initiate_track(const DsDetection& detection);
    NearestNeighborDistanceMetric::SharedPtr _metric_ptr;
    double _max_iou_distance;
    int _max_age;
    int _n_init;
    static KalmanFilter::SharedPtr _kf_ptr;
    int _next_id;
    std::vector<DsTrack> _tracks;
};

}
}

#endif //DEEPSORT_TRACKER_H
