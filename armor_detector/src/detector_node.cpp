#include <armor_detector/detector_node.h>

// std
#include <sstream>
#include <algorithm>
#include <functional>

// 3rdlibs
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <fmt/format.h>
// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point32.hpp>

namespace armor_auto_aim {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options) {
    // declare parameter && initialization inference
    declareParams();
    // Publisher
    m_armors_pub = this->create_publisher<
        auto_aim_interfaces::msg::Armors>("armor_detector/armors", rclcpp::SensorDataQoS());
    // Subscription
    m_cam_info_sub = this->create_subscription<
        sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::subCamInfoCallback, this, std::placeholders::_1));
    m_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::subImageCallback, this, std::placeholders::_1));
    // Debug publisher
    m_result_img_pub = image_transport::create_publisher(this, "armor_detector/result_img");
}

void ArmorDetectorNode::declareParams() {
    auto model_path = this->declare_parameter("model_path", "opt-0527-001.onnx");
    model_path = ament_index_cpp::get_package_share_directory("armor_detector") + "/model/" + model_path;
    auto inference_driver = this->declare_parameter("inference_driver", "AUTO");
    auto c = this->declare_parameter("detect_color", "BLUE");
    RCLCPP_INFO(this->get_logger(), "model path_: %s", model_path.c_str());
    RCLCPP_INFO(this->get_logger(), "target color: %s", c.c_str());
    if (inference_driver == "AUTO")
        m_inference = std::make_unique<Inference>(model_path);
    else
        m_inference = std::make_unique<Inference>(model_path, inference_driver);
}

void ArmorDetectorNode::subCamInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info) {
    m_pnp_solver = std::make_unique<PnPSolver>(cam_info->k, cam_info->d);
    m_cam_center = cv::Point2f(cam_info->k[2], cam_info->k[5]);
    m_cam_info_sub.reset();
}

void ArmorDetectorNode::subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    if (m_pnp_solver == nullptr) return;
    // initialization
    m_armors_msg.armors.clear();
    m_armors_msg.header = img_msg->header;

    cv::Mat img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    if (img.empty()) return;

    auto s = this->now();
    std::vector<InferenceResult> results;
    if (m_inference->inference(img, &results)) {
        auto inf_latency = (this->now() - s).seconds() * 1000;
        // construct armors
        auto_aim_interfaces::msg::Armor armor;
        cv::Mat rvec, tvec;
        for (auto const& result: results) {
            // construct armor
            if ((result.color == 0 /*RED*/ && this->get_parameter("detect_color").as_string() == "BLUE") ||
                (result.color == 1 /*BLUE */ && this->get_parameter("detect_color").as_string() == "RED") ||
                 result.color == 2)
                continue;
            armor.number = result.classification;
            // armor.number = 6;
            armor.color = result.color? "BLUE": "RED";
            // armor apex
            std::vector<cv::Point2f> img_points(result.armor_apex, result.armor_apex + 4);
            // ratio
            cv::RotatedRect rrect = cv::minAreaRect(img_points);
            auto ratio = std::max(rrect.size.height, rrect.size.width) /
                                std::min(rrect.size.height, rrect.size.width);
//             RCLCPP_INFO(this->get_logger(), "ratio: %f", ratio);
            armor.type  = ratio < 3.0f?  "SMALL":"LARGE";
            if (!m_pnp_solver->pnpSolver(armor, img_points, rvec, tvec)) continue;
            armor.distance_to_center = m_pnp_solver->getDistance(rrect.center);
            // Draw armor in debug img
            cv::line(img, img_points[0], img_points[2], cv::Scalar(255, 0, 0), 2);
            cv::line(img, img_points[1], img_points[3], cv::Scalar(255, 0, 0), 2);
            std::stringstream txt;
            txt <<  "start latency: " 
                << std::fixed << std::setprecision(2) 
                << (this->now() - img_msg->header.stamp).seconds() * 1000
                << "ms; infer latency: "
                << std::fixed << std::setprecision(2) 
                << inf_latency << "ms";
            cv::putText(img, txt.str(), cv::Point(100, 150), 0, 1.0, cv::Scalar(255, 0, 0), 2);
            cv::circle(img, m_cam_center, 3, cv::Scalar(255, 0, 0), -1);
            // armor pose
            armor.pose.position.x = tvec.at<double>(0);
            armor.pose.position.y = tvec.at<double>(1);
            armor.pose.position.z = tvec.at<double>(2);
            orientationFromRvec(rvec, armor.pose.orientation);
            // tf2::Quaternion tfq;
            // tf2::fromMsg(armor.pose.orientation, tfq);
            // double r, p, y;
            // tf2::Matrix3x3(tfq).getRPY(r, p, y);
            // RCLCPP_INFO(this->get_logger(), "yaw: %f", y * 180.0 / M_PI);
            m_armors_msg.armors.emplace_back(armor);
        }
    }
    // Publish armors
    m_armors_pub->publish(m_armors_msg);
    // Publish debug img
    m_result_img_pub.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
}

void ArmorDetectorNode::orientationFromRvec(const cv::Mat& rvec, geometry_msgs::msg::Quaternion& q) {
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        tf2::Matrix3x3 tf2_rotation_matrix(
            rotation_matrix.at<double>(0, 0),rotation_matrix.at<double>(0, 1),rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0),rotation_matrix.at<double>(1, 1),rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0),rotation_matrix.at<double>(2, 1),rotation_matrix.at<double>(2, 2)
        );
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        q = tf2::toMsg(tf2_q);
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ArmorDetectorNode)
