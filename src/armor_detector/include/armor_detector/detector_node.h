#ifndef ARMOR_AUTO_AIM_ARMOR_DETECTOR_NODE_H
#define ARMOR_AUTO_AIM_ARMOR_DETECTOR_NODE_H

#include <auto_aim_interfaces/msg/detail/armors__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <image_transport/publisher.hpp>
#include <memory>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
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
    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cam_info_sub;
    // Publisher
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr m_armors_pub;
    // Debug Publisher
    image_transport::Publisher m_result_img_pub;
    // Marker
    visualization_msgs::msg::Marker m_armor_marker;
    visualization_msgs::msg::Marker m_armor_info_marker;
    visualization_msgs::msg::MarkerArray m_marker_array;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub;

    // Function
    void declareParams();

    void subCamInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info);

    void subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    void orientationFromRvec(const cv::Mat& rvec, geometry_msgs::msg::Quaternion& q);
};
}

#endif
