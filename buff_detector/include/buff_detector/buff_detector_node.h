#ifndef ARMOR_AUTO_AIM_BUFF_DETECTOR_NODE_H
#define ARMOR_AUTO_AIM_BUFF_DETECTOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <buff_detector/buff_detector_.h>

namespace armor_auto_aim {
class BuffDetectorNode: public rclcpp::Node {
public:
    BuffDetectorNode(const rclcpp::NodeOptions& options);
private:
    BuffDetector::Parameters m_param;
    std::shared_ptr<BuffDetector> m_detector;
    // Subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cam_info_sub;
    // Publisher
    // rclcpp::Publisher<>::SharedPtr m_armors_pub;
    // Debug
    image_transport::Publisher m_binray_img_pub;
    image_transport::Publisher m_circle_img_pub;
    image_transport::Publisher m_result_img_pub;

    void subCamInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info);

    void subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    // Debug function
    void drawBuff(cv::Mat& img, const Buff& buff);
};
}

#endif