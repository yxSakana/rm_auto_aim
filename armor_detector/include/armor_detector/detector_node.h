#ifndef ARMOR_AUTO_AIM_ARMOR_DETECTOR_NODE_H
#define ARMOR_AUTO_AIM_ARMOR_DETECTOR_NODE_H

#include <memory>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <auto_aim_interfaces/msg/armors.hpp>
#include <armor_detector/inference.h>
#include <armor_detector/pnp_solver.h>

namespace armor_auto_aim {
class ArmorDetectorNode: public rclcpp::Node {
public:
    explicit ArmorDetectorNode(const rclcpp::NodeOptions& options);
private:
    std::unique_ptr<Inference> m_inference;
    std::unique_ptr<PnPSolver> m_pnp_solver;

    auto_aim_interfaces::msg::Armors m_armors_msg;
    cv::Point2f m_cam_center;
    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cam_info_sub;
    // Publisher
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr m_armors_pub;
    // Debug Publisher
    bool m_is_debug;
    image_transport::Publisher m_result_img_pub;
    std::shared_ptr<cv::VideoWriter> m_raw_img_writer;
    std::shared_ptr<cv::VideoWriter> m_result_img_writer;

    // Function
    void declareParams();

    void subCamInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info);

    void subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    void orientationFromRvec(const cv::Mat& rvec, geometry_msgs::msg::Quaternion& q);
};
}

#endif
