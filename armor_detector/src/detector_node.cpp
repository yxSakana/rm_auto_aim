#include <armor_detector/detector_node.h>

// std
#include <filesystem>
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
    this->declare_parameter("is_capture_result_image", false);
    this->declare_parameter("is_capture_raw_image", false);
    m_is_debug = this->declare_parameter("is_debug", true);
    this->declare_parameter("is_outpost", false);
    RCLCPP_INFO(this->get_logger(), "model path: %s", model_path.c_str());
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

    // Video capture
    namespace fs = std::filesystem;
    using namespace std::chrono_literals;

    auto timestamp = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    auto np = (std::string(this->get_namespace()) == "/" ? "":std::string(this->get_namespace()));
    auto save_path = "runtime_log/video" + np + "/" + timestamp;
    RCLCPP_INFO_STREAM_EXPRESSION(this->get_logger(),
        this->get_parameter("is_capture_raw_image").as_bool() ||
        this->get_parameter("is_capture_result_image").as_bool(),
        "Save video path: " << save_path);

    m_save_timer = this->create_wall_timer(1min, [this, cam_info, save_path]() -> void {
        // raw image
        if (this->get_parameter("is_capture_raw_image").as_bool()) {
            if (!fs::exists(save_path)) fs::create_directories(save_path);
            m_raw_img_writer = std::make_shared<cv::VideoWriter>(
                save_path + "/raw_image" + std::to_string(m_save_count) + ".mp4",
                cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
                30, cv::Size(cam_info->width, cam_info->height));
        }
        // result image
        if (this->get_parameter("is_capture_result_image").as_bool()) {
            if (!fs::exists(save_path)) fs::create_directories(save_path);
            m_result_img_writer = std::make_shared<cv::VideoWriter>(
                save_path + "/result_image" + std::to_string(m_save_count) + ".mp4",
                cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
                30, cv::Size(cam_info->width, cam_info->height));
        }
        if (this->get_parameter("is_capture_raw_image").as_bool() ||
            this->get_parameter("is_capture_result_image").as_bool()) ++m_save_count;
    });
}

void ArmorDetectorNode::subImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    if (m_pnp_solver == nullptr) return;
    // initialization
    m_armors_msg.armors.clear();
    m_armors_msg.header = img_msg->header;

    cv::Mat img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    if (img.empty()) return m_armors_pub->publish(m_armors_msg);
    // Capture
    if (m_raw_img_writer) {
        cv::Mat img_bgr;
        cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);
        m_raw_img_writer->write(img_bgr);
    }

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
            // {{{
            if (!this->get_parameter("is_outpost").as_bool()) {
                armor.type  = ratio < 2.9f?  "SMALL":"LARGE";
                // Base purple armor {{{
                if (armor.type == "LARGE" &&
                    (armor.number != 1  &&  armor.number != 2 &&
                     armor.number != 3  &&  armor.number != 4 &&
                     armor.number != 5) && (armor.color == "BLUE")) continue;
                // }}}
                if      (armor.number == 1) armor.type = "LARGE";
                else if (armor.number == 2 || armor.number == 3 ||
                         armor.number == 4 || armor.number == 5)
                    armor.type = ratio > 3.5? "LARGE": "SMALL";
            } else {
                armor.type = "SMALL";
                armor.number = 10;
            }
            // Sentry robot -> 1 2 3 4 5 And outpost
            if (std::string(this->get_namespace()) != "/" && 
                armor.number != 10 && armor.number != 1 &&
                armor.number != 2  && armor.number != 3 &&
                armor.number != 4  && armor.number != 5) continue;
            // }}}

            if (!m_pnp_solver->pnpSolver(armor, img_points, rvec, tvec)) continue;
            armor.distance_to_center = m_pnp_solver->getDistance(rrect.center);
            // Draw armor in debug img
            if (m_is_debug) {
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
            }
            // armor pose
            armor.pose.position.x = tvec.at<double>(0);
            armor.pose.position.y = tvec.at<double>(1);
            armor.pose.position.z = tvec.at<double>(2);
            orientationFromRvec(rvec, armor.pose.orientation);
            m_armors_msg.armors.emplace_back(armor);
        }
    }
    // Publish armors
    m_armors_pub->publish(m_armors_msg);
    // Publish debug img
    if (m_is_debug)
        m_result_img_pub.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    if (m_result_img_writer) {
        cv::Mat img_bgr;
        cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);
        m_result_img_writer->write(img_bgr);
    }
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
