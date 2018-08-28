/* *
 * @Copyright (c) 2018 by JpHu
 * @Date 2018-7-28
 * @Company CuiZhou
 * @Email hujp@cuizhouai.com
 * @Function
 * */

#include "utils_funcs.h"
namespace cz {
namespace va {
float getIOU(const cv::Rect &rect1, const cv::Rect &rect2) {
    float a0, a1, a2, a3, b0, b1, b2, b3;

    a0 = rect1.x;
    a1 = rect1.y;
    a2 = a0 + rect1.width;
    a3 = a1 + rect1.height;

    b0 = rect2.x;
    b1 = rect2.y;
    b2 = b0 + rect2.width;
    b3 = b1 + rect2.height;

    if (a0 > b2 || a1 > b3 || a2 < b0 || a3 < b1) {
        return 0;
    }

    // overlapped region (= box)
    const float x1 = std::max(a0, b0);
    const float y1 = std::max(a1, b1);
    const float x2 = std::min(a2, b2);
    const float y2 = std::min(a3, b3);

    // intersection area
    const float width = std::max((float) 0, x2 - x1 + (float) 1);
    const float height = std::max((float) 0, y2 - y1 + (float) 1);
    const float area = width * height;

    // area of A, B
    const float A_area = (a2 - a0 + (float) 1) * (a3 - a1 + (float) 1);
    const float B_area = (b2 - b0 + (float) 1) * (b3 - b1 + (float) 1);

    // IoU
    return area / (A_area + B_area - area);
}

void nmsDiffClasses(cz::ml::RectVec &dets) {
    int dets_size = dets.size();
    bool need_delete = false;
    std::vector<bool> valid_flags(dets_size, true);
    for (int i = 0; i < dets_size; ++i) {
        if (valid_flags[i]) {
            for (int j = i + 1; j < dets_size; ++j) {
                if (valid_flags[i] && valid_flags[j]) {
//                    if (dets[i].type != dets[j].type) {
                        if (getIOU(cv::Rect(dets[i].x, dets[i].y, dets[i].width, dets[i].height),
                                   cv::Rect(dets[j].x, dets[j].y, dets[j].width, dets[j].height)) > 0.3) {
                            if (!need_delete) {
                                need_delete = true;
                            }
                            if (dets[i].score > dets[j].score) {
                                valid_flags[j] = false;
                            } else {
                                valid_flags[i] = false;
                            }
                        }
//                    }
                }
            }
        }
    }
    if (need_delete) {
        int index = 0;
        for (auto itr = dets.begin(); itr != dets.end();) {
            if (!valid_flags[index]) {
                itr = dets.erase(itr);
            } else {
                ++itr;
            }
            ++index;
        }
    }
}
}
}