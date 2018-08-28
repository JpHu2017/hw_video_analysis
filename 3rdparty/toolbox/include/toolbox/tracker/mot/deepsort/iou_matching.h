/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-6-13
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#ifndef DEEPSORT_IOU_MATCHING_H
#define DEEPSORT_IOU_MATCHING_H
#include <Eigen/Eigen>
#include <vector>
#include "ds_detection.h"
#include "ds_track.h"
//#include "alias.h"

namespace cz {
namespace ht {

Eigen::VectorXf iou(const Eigen::RowVector4f& bbox, const std::vector<Eigen::RowVector4f>& candidates);
Eigen::MatrixXf iou_cost(const std::vector<DsTrack>& tracks, const std::vector<DsDetection>& detections,
                         const std::vector<int>& track_indices = std::vector<int>(),
                         const std::vector<int>& detection_indices = std::vector<int>());

}
}


#endif //DEEPSORT_IOU_MATCHING_H
