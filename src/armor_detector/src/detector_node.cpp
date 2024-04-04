
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
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

namespace armor_auto_aim {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options) {
    // declare parameter && initialization inference
    declareParams();
    // Publisher
    m_armors_pub = this->create_publisher<
        auto_aim_interfaces::msg::Armors>("/armor_detector/armors", rclcpp::SensorDataQoS());
    // Debug publisher
    m_result_img_pub = image_transport::create_publisher(this, "/armor_detector/result_img");
    // Subscription
    m_cam_info_sub = this->create_subscription<
        sensor_msgs::msg::CameraInfo>("/camera_info", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::subCamInfoCallback, this, std::placeholders::_1));
    m_img_sub = this->create_subscription<
        sensor_msgs::msg::Image>("/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::subImageCallback, this, std::placeholders::_1));
    // Marker
    m_armor_marker.ns = "armors_marker";
    m_armor_marker.action = visualization_msgs::msg::Marker::ADD;
    m_armor_marker.type = visualization_msgs::msg::Marker::CUBE;
    m_armor_marker.scale.x = 0.05;
    m_armor_marker.scale.z = 0.125;
    m_armor_marker.color.a = 0.9;
    m_armor_marker.color.g = 0.5;
    m_armor_marker.color.b = 0.1;
    m_armor_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    m_armor_info_marker.ns = "armors_info_marker";
    m_armor_info_marker.action = visualization_msgs::msg::Marker::ADD;
    m_armor_info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m_armor_info_marker.scale.z = 0.125;
    m_armor_info_marker.color.a = 1.0;
    m_armor_info_marker.color.r = 1.0;
    m_armor_info_marker.color.g = 1.0;
    m_armor_info_marker.color.b = 1.0;
    m_armor_info_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    m_marker_pub = this->create_publisher<
        visualization_msgs::msg::MarkerArray>("/armor_detetor/marker", 10);
}

void ArmorDetectorNode::declareParams() {
    auto model_path = this->declare_parameter("model_path", "opt-0527-001.onnx");
    model_path = ament_index_cpp::get_package_share_directory("armor_detector") + "/model/" + model_path;
    auto inference_driver = this->declare_parameter("inference_driver", "AUTO");
    auto a = this->declare_parameter("detect_color", "BLUE");
    RCLCPP_INFO(this->get_logger(), "model path_: %s", model_path.c_str());
    RCLCPP_INFO(this->get_logger(), "target color: %s", a.c_str());
    if (inference_driver == "AUTO")
        m_inference = std::make_unique<Inference>(model_path);
    else
        m_inference = std::make_unique<Inference>(model_path, inference_driver);
}

void ArmorDetectorNode::subCamInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info) {
    m_pnp_solver = std::make_unique<PnPSolver>(cam_info->k, cam_info->d);
    m_cam_info_sub.reset();
}

void ArmorDetectorNode::subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    if (m_pnp_solver == nullptr) return;
    // initialization
    // RCLCPP_WARN(this->get_logger(), "start latency: %f", (this->now() - img_msg->header.stamp).seconds() * 1000);
    m_armors_msg.armors.clear();
    m_marker_array.markers.clear();
    m_armor_marker.id = 0;
    m_armor_info_marker.id = 0;
    m_armors_msg.header = m_armor_marker.header = m_armor_info_marker.header = img_msg->header;

    cv::Mat img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    if (img.empty()) return;

    auto s = this->now();
    std::vector<InferenceResult> results;
    if (m_inference->inference(img, &results)) {
        auto latency = (this->now() - img_msg->header.stamp).seconds() * 1000;
        auto inf_latency = (this->now() - s).seconds() * 1000;
        // RCLCPP_INFO(this->get_logger(), "latency: %f", latency);
        // RCLCPP_INFO(this->get_logger(), "inf_latency: %f", inf_latency);z
        // RCLCPP_INFO(this->get_logger(), "latency ratio: %f %%", inf_latency/latency * 100);
        // construct armors
        auto_aim_interfaces::msg::Armor armor;
        cv::Mat rvec, tvec;
        for (auto const& result: results) {
            // construct armor
            if ((result.color == 0 /*RED*/ && this->get_parameter("detect_color").as_string() == "BLUE") ||
                (result.color == 1 /*BLUE */ && this->get_parameter("detect_color").as_string() == "RED"))
                continue;
            armor.number = result.classification;
            armor.color = result.color? "BLUE": "RED";
            // armor apex
            std::vector<cv::Point2f> img_points(result.armor_apex, result.armor_apex + 4);
            cv::RotatedRect rrect = cv::minAreaRect(img_points);
            auto ratio = std::max(rrect.size.height, rrect.size.width) /
                                std::min(rrect.size.height, rrect.size.width);
            // RCLCPP_INFO(this->get_logger(), "ratio: %f", ratio);
            armor.type  = ratio < 2.8f?  "SMALL":"LARGE";
            if (!m_pnp_solver->pnpSolver(armor, img_points, rvec, tvec)) continue;
            // Draw armor in debug img
            cv::line(img, img_points[0], img_points[2], cv::Scalar(255, 0, 0), 2);
            cv::line(img, img_points[1], img_points[3], cv::Scalar(255, 0, 0), 2);
            std::stringstream txt;
            txt <<  "start latency: " 
                << std::fixed << std::setprecision(2) 
                << (this->now() - img_msg->header.stamp).seconds() 
                << "ms; infer latency: "
                << std::fixed << std::setprecision(2) 
                << inf_latency << "ms";
            auto t = fmt::format(
                    "start lantency: {:.2f} ms infer latency: {:.2f} ms",
                    (this->now() - img_msg->header.stamp).seconds() * 1000, inf_latency);
            // auto t = txt.str();
            cv::putText(img, t, cv::Point(30, 60), 0, 1, cv::Scalar(255, 255, 255));
            // armor pose
            armor.pose.position.x = tvec.at<double>(0);
            armor.pose.position.y = tvec.at<double>(1);
            armor.pose.position.z = tvec.at<double>(2);
            orientationFromRvec(rvec, armor.pose.orientation);
            m_armors_msg.armors.emplace_back(armor);

            // Marker
            m_armor_marker.id++;
            m_armor_marker.pose = armor.pose;
            m_marker_array.markers.emplace_back(m_armor_marker);
        }
    }
    // Publish armors && debug img
    m_armors_pub->publish(m_armors_msg);
    // Publish Marker && debug img
    m_marker_pub->publish(m_marker_array);
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
