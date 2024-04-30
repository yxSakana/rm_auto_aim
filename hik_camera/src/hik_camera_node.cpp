#include <hik_camera/hik_camera_node.h>

// ROS2
#include <image_transport/image_transport.hpp>

namespace hik_camera {
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions& options)
    : Node("hik_camera", options),
      m_hik_driver(this->declare_parameter("camera_index", 0)) {
    m_camera_pub = image_transport::create_camera_publisher(
        this, "image_raw", rmw_qos_profile_default/*rmw_qos_profile_sensor_data*/);
    // Paraments
    std::string camera_name = this->declare_parameter("camera_name", "narrow_stereo");
    std::string camera_info_url = this->declare_parameter(
        "camera_info_url", "package://hik_camera/config/camera_info.yaml");
    this->declare_parameter("camera_frame", "camera_optical_frame");
    this->declare_parameter<float>("exposure_time", 4000.0);
    this->declare_parameter<float>("gain", 15.0);
    // Service
    m_enable_srv = this->create_service<std_srvs::srv::SetBool>("hik_camera/enable", [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    )->void {
        m_enable = request->data;
        response->success = true;
        const char* mess = request->data? "disenable hik camera": "enable hik camera";
        response->message = mess;
        RCLCPP_INFO(this->get_logger(), "%s(%ld)", mess, this->get_parameter("camera_index").as_int());
    });
    // Camera manager
    m_camera_info_manager = std::make_unique<
        camera_info_manager::CameraInfoManager>(this, camera_name);
    // Camera info manager
    if (m_camera_info_manager->validateURL(camera_info_url)) {
        m_camera_info_manager->loadCameraInfo(camera_info_url);
        m_camera_info_msg = m_camera_info_manager->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid acmera info url: %s", camera_info_url.c_str());
    }

    initHikCamera();
    m_core_thread = std::thread([this]()->void { this->core(); });
}

void HikCameraNode::initHikCamera() {
    RCLCPP_INFO(this->get_logger(), "Hik device number: %d", m_hik_driver.getDeviceNumber());
    if (m_hik_driver.isConnected()) {
        m_hik_driver.setExposureTime(this->get_parameter("exposure_time").as_double());
        m_hik_driver.setGain(this->get_parameter("gain").as_double());
        std::string info = m_hik_driver.getDeviceParamInfo();
        MV_CC_GetImageInfo(m_hik_driver.getHandle(), &m_hik_img_info);
        m_image_msg.data.reserve(m_hik_img_info.nHeightMax * m_hik_img_info.nWidthMax * 3);

        RCLCPP_INFO_STREAM(this->get_logger(), info);
        RCLCPP_INFO(this->get_logger(), "Hik Camera Connected!");
    } else {
        RCLCPP_WARN(this->get_logger(), "Hik Camera unable to connect!");
    }
}

void HikCameraNode::core() {
    while (!m_hik_driver.isConnected()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        m_hik_driver.connectDevice(this->get_parameter("camera_index").as_int());
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

    m_camera_info_msg.header.frame_id = m_image_msg.header.frame_id =
        this->get_parameter("camera_frame").as_string();
    m_image_msg.encoding = "rgb8";
    void* handle = m_hik_driver.getHandle();
    while (rclcpp::ok()) {
        // check
        if (!m_enable) return;
        while (!m_hik_driver.isConnected()) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            m_hik_driver.connectDevice(this->get_parameter("camera_index").as_int());
            initHikCamera();
        }
        if (m_hik_driver.readImageData(out_frame, 1000)) {
            // hik sdk
            m_convert_param.pDstBuffer = m_image_msg.data.data();
            m_convert_param.nDstBufferSize = m_image_msg.data.size();
            m_convert_param.pSrcData = out_frame.pBufAddr;
            m_convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            m_convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
            MV_CC_ConvertPixelTypeEx(handle, &m_convert_param);
            // RCLCPP_INFO(this->get_logger(), "Convert Type: %f", (this->now() - s).seconds() * 1000);
            // img_msg
            m_image_msg.header.stamp = this->now();
            m_image_msg.height = out_frame.stFrameInfo.nHeight;
            m_image_msg.width = out_frame.stFrameInfo.nWidth;
            m_image_msg.step = out_frame.stFrameInfo.nWidth * 3;
            m_image_msg.data.resize(m_image_msg.width * m_image_msg.height * 3);
            // Pub
            m_camera_info_msg.header = m_image_msg.header;
            m_camera_pub.publish(m_image_msg, m_camera_info_msg);
            MV_CC_FreeImageBuffer(handle, &out_frame);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get hik out frame!");
        }
    }
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
