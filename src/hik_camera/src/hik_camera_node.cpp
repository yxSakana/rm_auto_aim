#include <hik_camera/hik_camera_node.h>

#include <opencv2/imgproc.hpp>
// ROS2
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

namespace hik_camera {
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions& options)
    : Node("hik_camera", options),
        m_hik_driver(0) {
    m_camera_pub = image_transport::create_camera_publisher(
        this, "image_raw", rmw_qos_profile_sensor_data);
    // Paraments
    std::string camera_name = this->declare_parameter("camera_name", "narrow_stereo");
    std::string camera_info_url = this->declare_parameter(
        "camera_info_url", "package://hik_camera/config/camera_info.yaml");
    this->declare_parameter<float>("exposure_time", 4000.0);
    this->declare_parameter<float>("gain", 15.0);

    m_camera_info_manager = std::make_unique<
        camera_info_manager::CameraInfoManager>(this, camera_name);

    if (m_camera_info_manager->validateURL(camera_info_url)) {
        m_camera_info_manager->loadCameraInfo(camera_info_url);
        m_camera_info_msg = m_camera_info_manager->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid acmera info url: %s", camera_info_url.c_str());
    }

    initHikCamera();
    m_core_thread = std::thread([this](){ this->core(); });
}

void HikCameraNode::initHikCamera() {
    if (m_hik_driver.isConnected()) {
        m_hik_driver.setExposureTime(this->get_parameter("exposure_time").as_double());
        m_hik_driver.setGain(this->get_parameter("gain").as_double());
        m_hik_driver.showParamInfo();
        // m_hik_driver.startReadThread();
        // return;
        MV_CC_GetImageInfo(m_hik_driver.getHandle(), &m_hik_img_info);
        m_image_msg.data.reserve(m_hik_img_info.nHeightMax * m_hik_img_info.nWidthMax * 3);
        RCLCPP_INFO(this->get_logger(), "Hik Camera Connected!");
    } else {
        RCLCPP_WARN(this->get_logger(), "Hik Camera unable to connect!");
    }
}

void HikCameraNode::core() {
    while (!m_hik_driver.isConnected()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        m_hik_driver.connectDriver(0);
        initHikCamera();
    }
    RCLCPP_INFO(this->get_logger(), "Pushing camera msg...");
 
    rclcpp::Time last_t = this->now();
    // HikFrame hik_frame;
    MV_CC_PIXEL_CONVERT_PARAM_EX m_convert_param;
    MV_FRAME_OUT out_frame;
    m_convert_param.nWidth = m_hik_img_info.nWidthValue;
    m_convert_param.nHeight = m_hik_img_info.nHeightValue;
    m_convert_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    m_camera_info_msg.header.frame_id = m_image_msg.header.frame_id = "camera_optical_frame";
    m_image_msg.encoding = "rgb8";
    void* handle = m_hik_driver.getHandle();
    while (rclcpp::ok()) {
        auto status = MV_CC_GetImageBuffer(handle, &out_frame, 1000);
        if (status == MV_OK) {
            // hik sdk
            m_convert_param.pDstBuffer = m_image_msg.data.data();
            m_convert_param.nDstBufferSize = m_image_msg.data.size();
            m_convert_param.pSrcData = out_frame.pBufAddr;
            m_convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            m_convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
            MV_CC_ConvertPixelTypeEx(handle, &m_convert_param);
            // img_msg
            m_camera_info_msg.header.stamp = m_image_msg.header.stamp = this->now();
            m_image_msg.height = out_frame.stFrameInfo.nHeight;
            m_image_msg.width = out_frame.stFrameInfo.nWidth;
            m_image_msg.step = out_frame.stFrameInfo.nWidth * 3;
            m_image_msg.data.resize(m_image_msg.width * m_image_msg.height * 3);
            // Pub
            m_camera_pub.publish(m_image_msg, m_camera_info_msg);
            // RCLCPP_WARN(this->get_logger(), "pub img latency: %f", (last_t - m_image_msg.header.stamp).seconds() * 1000);

            MV_CC_FreeImageBuffer(handle, &out_frame);
            // last_t = m_image_msg.header.stamp;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get hik out frame!");
        }
        // hik_frame = m_hik_driver.getFrame();
        // auto image_msg_ptr = cv_bridge::CvImage(
        //     std_msgs::msg::Header(),
        //     "bgr8",
        //     *hik_frame.getRgbFrame()).toImageMsg();
        // image_msg_ptr->header.stamp = this->now();
        // image_msg_ptr->header.frame_id = "camera_optical_frame";
        // m_camera_info_msg.header = image_msg_ptr->header;
        // m_camera_pub.publish(*image_msg_ptr, m_camera_info_msg);
    }
}
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)