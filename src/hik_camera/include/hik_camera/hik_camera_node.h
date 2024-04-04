#ifndef ARMOR_AUTO_AIM_HIK_CAMERA_NODE_H
#define ARMOR_AUTO_AIM_HIK_CAMERA_NODE_H

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <hik_camera/hik_driver.h>

namespace hik_camera {
class HikCameraNode: public rclcpp::Node {
public:
    HikCameraNode(const rclcpp::NodeOptions& options);
private:
    HikDriver m_hik_driver;
    image_transport::CameraPublisher m_camera_pub;
    std::unique_ptr<camera_info_manager::CameraInfoManager> m_camera_info_manager;
    sensor_msgs::msg::CameraInfo m_camera_info_msg;
    sensor_msgs::msg::Image m_image_msg;
    MV_IMAGE_BASIC_INFO m_hik_img_info;
    std::thread m_core_thread;

    void initHikCamera();

    void core();
};
}

#endif
