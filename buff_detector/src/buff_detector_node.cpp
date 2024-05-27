#include <buff_detector/buff_detector_node.h>

#include <cv_bridge/cv_bridge.h>

namespace armor_auto_aim {
BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("buff_detector_node", options) {
    // m_param. = this->declare_parameter("rel_distance", 1.0);
    m_param.preprocess.inner = this->declare_parameter("inner", 80.0);
    m_param.preprocess.outer = this->declare_parameter("outer", 150.0);
    // m_param.preprocess.r_threshold = this->declare_parameter<int>("r_threshold", 1);
    // m_param.preprocess.g_threshold = this->declare_parameter<int>("g_threshold", 234);
    // m_param.preprocess.b_threshold = this->declare_parameter<int>("b_threshold", 254);
    m_param.preprocess.r_threshold = this->declare_parameter<int>("r_threshold", 254);
    m_param.preprocess.g_threshold = this->declare_parameter<int>("g_threshold", 234);
    m_param.preprocess.b_threshold = this->declare_parameter<int>("b_threshold", 1);
    m_param.r.min_area = this->declare_parameter("r_min_area", 50.0);
    m_param.r.max_area = this->declare_parameter("r_max_area", 500.0);
    m_param.r.min_ratio = this->declare_parameter("r_min_ratio", 0.8);
    m_param.r.max_ratio = this->declare_parameter("r_max_ratio", 1.1);
    // .distance_from_center = this->declare_parameter("r_distance_from_center", 50.0)
    m_param.fan_board.min_area = this->declare_parameter("fan_min_area", 3000.0);
    m_param.fan_board.max_area = this->declare_parameter("fan_max_area", 17000.0);
    m_param.fan_board.min_ratio = this->declare_parameter("fan_min_ratio", 0.2);
    m_param.fan_board.max_ratio = this->declare_parameter("fan_max_ratio", 0.99);
    m_param.fan_board.distance_from_r = this->declare_parameter("fan_distance_from_r", 15000.0);

    // Detector
    m_detector = std::make_shared<BuffDetector>(m_param);
    // Subscript
    m_cam_info_sub = this->create_subscription<
        sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS(),
        std::bind(&BuffDetectorNode::subCamInfoCallback, this, std::placeholders::_1));
    m_img_sub = this->create_subscription<
        sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(),
        std::bind(&BuffDetectorNode::subImageCallback, this, std::placeholders::_1));
    // Debug
    m_binray_img_pub = image_transport::create_publisher(this, "buff_detector/binary_img");
    m_circle_img_pub = image_transport::create_publisher(this, "buff_detector/circle_img");
    m_result_img_pub = image_transport::create_publisher(this, "buff_detector/result_img");
}

void BuffDetectorNode::subCamInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info) {
    // m_pnp_solver = std::make_unique<PnPSolver>(cam_info->k, cam_info->d);
    // m_cam_center = cv::Point2f(cam_info->k[2], cam_info->k[5]);
    m_cam_info_sub.reset();
}

void BuffDetectorNode::subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    cv::Mat img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    cv::Mat binary_img = m_detector->preprocessImage(img);
    m_binray_img_pub.publish(cv_bridge::CvImage(img_msg->header, "mono8", binary_img).toImageMsg());
    Buff buff_obj = m_detector->detect(binary_img);
    auto target = m_detector->getTarget();
    drawBuff(img, buff_obj);
    // Debug
    m_circle_img_pub.publish(cv_bridge::CvImage(img_msg->header, "mono8", binary_img).toImageMsg());
    m_result_img_pub.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
}

void BuffDetectorNode::drawBuff(cv::Mat& img, const Buff& buff) {
    auto drawRotatedRect = [](
        const cv::Mat& src,      const cv::RotatedRect& rrect,
        const cv::Scalar& color, int thickness) -> void {
            cv::Point2f p[4];
            rrect.points(p);
            for (int i = 0; i < 4; i++) {
                cv::line(src, p[i], p[(i + 1) % 4], color, thickness);
                cv::putText(src, std::to_string(i), p[i], 0, 0.8, color, 2);
        }
    };
    drawRotatedRect(img, buff.r, cv::Scalar(255, 255, 0), 2);
    for (const auto& item: buff.fan_boards) {
        drawRotatedRect(img, item.region,
            item.state == FanBoardState::Target? cv::Scalar(255, 0, 255): cv::Scalar(255, 0, 0), 2);
    }
}
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::BuffDetectorNode)
