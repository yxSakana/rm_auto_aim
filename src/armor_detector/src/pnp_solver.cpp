/**
 * @projectName armor_auto_aim
 * @file pnp_solver.cpp
 * @brief 
 * 
 * @author yx 
 * @date 2023-10-22 16:39
 */

#include <armor_detector/pnp_solver.h>

#include <Eigen/Dense>
#include <cassert>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

namespace armor_auto_aim {
PnPSolver::PnPSolver(const std::array<double, 9>& intrinsic_matrix,
                     const std::vector<double>& distortion_vector)
                     : m_intrinsic_matrix(cv::Mat(3, 3, CV_64F, const_cast<double*>(intrinsic_matrix.data())).clone()),
                       m_distortion_vector(cv::Mat(1, 5, CV_64F, const_cast<double*>(distortion_vector.data())).clone())
{
    // Unit: m
    constexpr double small_half_x = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double small_half_y = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
    constexpr double large_half_x = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double large_half_y = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;
    // 3d Points(lt lb rb rt)
    m_small_armor_point3d.push_back(cv::Point3f(0, small_half_x, small_half_y));
    m_small_armor_point3d.push_back(cv::Point3f(0, small_half_x, -small_half_y));
    m_small_armor_point3d.push_back(cv::Point3f(0, -small_half_x, -small_half_y));
    m_small_armor_point3d.push_back(cv::Point3f(0, -small_half_x, small_half_y));

    m_large_armor_point3d.push_back(cv::Point3f(0, large_half_x, large_half_y));
    m_large_armor_point3d.push_back(cv::Point3f(0, large_half_x, -large_half_y));
    m_large_armor_point3d.push_back(cv::Point3f(0, -large_half_x, -large_half_y));
    m_large_armor_point3d.push_back(cv::Point3f(0, -large_half_x, large_half_y));
}

bool PnPSolver::pnpSolver(const auto_aim_interfaces::msg::Armor& armor,
        const std::vector<cv::Point2f>& img_points, cv::Mat& rvec, cv::Mat& tvec) {
    auto point3d = armor.type == "SMALL"? m_small_armor_point3d: m_large_armor_point3d;
    try {
        return cv::solvePnP(point3d, img_points, m_intrinsic_matrix, m_distortion_vector,
                            rvec, tvec, false, cv::SOLVEPNP_IPPE);  // cv::SOLVEPNP_ITERATIVE ?
    } catch (const cv::Exception& e) {
        for (int i = 0; i < 4; ++i) {
            auto item = img_points[i];
            RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "%d: %f, %f;", i, item.x, item.y);
        }
        RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "-----");
        for (int i = 0; i < 4; ++i) {
            auto item = point3d[i];
            RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "%d: %f, %f;", i, item.x, item.y);
        }
        RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "======");
        return false;
    }
}
} // armor_auto_aim