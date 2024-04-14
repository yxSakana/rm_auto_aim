#ifndef ARMOR_AUTO_AIM_BUFF_DETECTOR_BUFF_PNP_SOLVER_H
#define ARMOR_AUTO_AIM_BUFF_DETECTOR_BUFF_PNP_SOLVER_H

#include <opencv2/opencv.hpp>

#include <buff_detector/buff_detector.h>

namespace armor_auto_aim {
class BuffPnP {
public:
    BuffPnP(const std::array<double, 9>& intrinsic_matrix,
            const std::vector<double>& distortion_vector) {

    }

    bool solver(const FanBoard& fan, cv::Mat& rvec, cv::Mat& tvec);
private:
    // Unit: mm
    static constexpr float SMALL_ARMOR_WIDTH = 135;
    static constexpr float SMALL_ARMOR_HEIGHT = 55;
    static constexpr float LARGE_ARMOR_WIDTH = 225;
    static constexpr float LARGE_ARMOR_HEIGHT = 55;

    cv::Mat m_intrinsic_matrix;
    cv::Mat m_distortion_vector;
    std::vector<cv::Point3f> m_small_armor_point3d;
};
}

#endif
