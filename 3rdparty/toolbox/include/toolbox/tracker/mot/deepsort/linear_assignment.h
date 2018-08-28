/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-6-20
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef DEEPSORT_LINEAR_ASSIGNMENT_H
#define DEEPSORT_LINEAR_ASSIGNMENT_H

#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "ds_detection.h"
#include "ds_track.h"
#include "tracker/mot/hungarian/hungarian.h"
#include "nn_matching.h"


namespace cz {
namespace ht {
extern float INFIY_COST;

// 定义函数指针
typedef boost::function<Eigen::MatrixXf (const std::vector<DsTrack>&, const std::vector<DsDetection>&,
                                         const std::vector<int>&, const std::vector<int>&)> DistanceMetric;
// 该种方式无法用于boost::bind
//typedef Eigen::MatrixXf (*DistanceMetric)(const std::vector<Track>& tracks, const std::vector<Detection>& detections,
//                                          const std::vector<int>& track_indices, const std::vector<int>& tracks_indices);
void min_cost_matching(const DistanceMetric& distance_metric, const float& max_distance,
                       const std::vector<DsTrack>& tracks, const std::vector<DsDetection>& detections,
                       std::vector<std::pair<int,int> >& matches,
                       std::vector<int>& unmatched_tracks,
                       std::vector<int>& unmatched_detections,
                       const std::vector<int>& track_indices=std::vector<int>(),
                       const std::vector<int>& detection_indices=std::vector<int>());
void matching_cascade(const DistanceMetric& distance_metric, const float& max_distance, const int& cascade_depth,
                      const std::vector<DsTrack>& tracks, const std::vector<DsDetection>& detections,
                      std::vector<std::pair<int,int> >& matches,
                      std::vector<int>& unmatched_tracks,
                      std::vector<int>& unmatched_detections,
                      const std::vector<int>& track_indices=std::vector<int>(),
                      const std::vector<int>& detection_indices=std::vector<int>());
Eigen::MatrixXf gate_cost_matrix(KalmanFilter& kf, const Eigen::MatrixXf& cost_matrix,
                                 const std::vector<DsTrack> &tracks,
                                 const std::vector<DsDetection> &detections,
                                 const std::vector<int>& track_indices,
                                 const std::vector<int>& detection_indices,
                                 float gated_cost = INFIY_COST,
                                 bool only_position=false);

}
}

#endif //DEEPSORT_LINEAR_ASSIGNMENT_H
