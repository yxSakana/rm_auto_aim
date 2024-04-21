
#include <buff_detector/buff_detector.h>

#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <buff_detector/iou_compute.h>

namespace armor_auto_aim {
BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("buff_detector", options) {
    // paraments
    m_real_distance = this->declare_parameter("rel_distance", 1.0);
    m_inner = this->declare_parameter("inner", 80.0);
    m_outer = this->declare_parameter("outer", 150.0);
    m_params = {
        .r_threshold = static_cast<int>(this->declare_parameter<int>("r_threshold", 1)),
        .g_threshold = static_cast<int>(this->declare_parameter<int>("g_threshold", 234)),
        .b_threshold = static_cast<int>(this->declare_parameter<int>("b_threshold", 254)) 
    };
    // m_r_params = {
    //     .min_area = this->declare_parameter("r_min_area", 50.0),
    //     .max_area = this->declare_parameter("r_max_area", 500.0),
    //     .min_ratio = this->declare_parameter("r_min_ratio", 0.8),
    //     .max_ratio = this->declare_parameter("r_max_ratio", 1.1),
    //     .distance_from_center = this->declare_parameter("r_distance_from_center", 50.0)
    // };
    m_r_params = {
        .min_area = this->declare_parameter("r_min_area", 1500.0),
        .max_area = this->declare_parameter("r_max_area", 4000.0),
        .min_ratio = this->declare_parameter("r_min_ratio", 0.8),
        .max_ratio = this->declare_parameter("r_max_ratio", 1.1),
        .distance_from_center = this->declare_parameter("r_distance_from_center", 500000.0)
    };
    m_fan_params = {
        .min_area = this->declare_parameter("fan_min_area", 10000.0),
        .max_area = this->declare_parameter("fan_max_area", 17000.0),
        .min_ratio = this->declare_parameter("fan_min_ratio", 0.2),
        .max_ratio = this->declare_parameter("fan_max_ratio", 0.99),
        .distance_from_r = this->declare_parameter("fan_distance_from_r", 15000.0)
    };
    // m_fan_params = {
    //     .min_area = this->declare_parameter("fan_min_area", 3000.0),
    //     .max_area = this->declare_parameter("fan_max_area", 17000.0),
    //     .min_ratio = this->declare_parameter("fan_min_ratio", 0.2),
    //     .max_ratio = this->declare_parameter("fan_max_ratio", 0.99),
    //     .distance_from_r = this->declare_parameter("fan_distance_from_r", 15000.0)
    // };
    m_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&BuffDetectorNode::reset, this));
    // sub
    m_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(), std::bind(
            &BuffDetectorNode::detect, this, std::placeholders::_1));
    // Debug publisher
    m_result_pub = image_transport::create_publisher(this, "/buff_detector/result_img");
    m_binary_pub = image_transport::create_publisher(this, "/buff_detector/binary_img");
    m_contour_pub = image_transport::create_publisher(this, "/buff_detector/contour_img");
}

void BuffDetectorNode::detect(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    cv::Mat img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    cv::Mat binary_img = preprocessImage(img);

    // find r
    auto r = matchR(binary_img);
    if (!r.size.empty()) {
        m_r_box = std::make_shared<cv::RotatedRect>(r);
        // Sec preprocess image
        // cv::circle(binary_img, m_r_box->center, m_inner, cv::Scalar(0), -1);
        // cv::circle(binary_img, m_r_box->center, m_outer, cv::Scalar(0), 5);
        cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, k);
        // Find fans
        matchFan(binary_img);
        // Draw
        if (m_r_box != nullptr) drawRotatedRect(img, *m_r_box);
        if (!m_last_fans.empty()) {
            for (const auto& item: m_last_fans) {
                drawRotatedRect(img, item.region, 3, cv::Scalar(255, 0, 200));
                cv::putText(img, std::to_string(item.id), item.region.center+cv::Point2f(40, 10), 0, 0.8, cv::Scalar(0, 255, 255));
                cv::putText(img, armor_auto_aim::to_string(item.state),
                    item.region.center+cv::Point2f(40, 30), 0, 0.5,
                    item.state == FanState::Target? cv::Scalar(255, 0, 0): cv::Scalar(0, 255, 255));
            }
        }
    }
    // Debug publish
    m_result_pub.publish(*(cv_bridge::CvImage(img_msg->header, "bgr8", img).toImageMsg()));
    m_binary_pub.publish(*(cv_bridge::CvImage(img_msg->header, "mono8", binary_img).toImageMsg()));
}

cv::Mat BuffDetectorNode::preprocessImageToR(const cv::Mat& src) {
    cv::Mat binary_img = cv::Mat::zeros(src.size(), CV_8UC1);
    uchar* dst_ptr = binary_img.data;
    uchar* src_ptr = src.data;
    for (; src_ptr != src.data + src.cols * src.rows * 3; dst_ptr++) {
        uchar* b = src_ptr++;
        uchar* g = src_ptr++;
        uchar* r = src_ptr++;
        if (*b >= 240 && *g >= 240 && *r >= 240)
            *dst_ptr = 255;
    }
    // cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    // cv::morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, k);
    return binary_img;
}

cv::Mat BuffDetectorNode::preprocessImage(const cv::Mat& src) {
    cv::Mat binary_img = cv::Mat::zeros(src.size(), CV_8UC1);
    uchar* dst_ptr = binary_img.data;
    uchar* src_ptr = src.data;
    for (; src_ptr != src.data + src.cols * src.rows * 3; dst_ptr++) {
        uchar* b = src_ptr++;
        uchar* g = src_ptr++;
        uchar* r = src_ptr++;
        if (*b >= m_params.b_threshold && *g >= m_params.g_threshold)
            *dst_ptr = 255;
    }
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    cv::morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, k);
    return binary_img;
}

cv::RotatedRect BuffDetectorNode::matchR(const cv::Mat& binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    cv::Mat draw = cv::Mat::zeros(binary_img.size(), CV_8UC1);
    cv::drawContours(draw, contours, -1, cv::Scalar(255));
    std_msgs::msg::Header header;
    header.stamp = this->now();
    m_contour_pub.publish(*(cv_bridge::CvImage(header, "mono8", draw).toImageMsg()));

    for (const auto& contour: contours) {
        cv::RotatedRect r = cv::minAreaRect(contour);
        if (isVaild(binary_img, r.boundingRect()) && isR(binary_img, r)) {
            RCLCPP_DEBUG_STREAM(this->get_logger(), cv::norm(r.center - cv::Point2f(
                binary_img.cols/2, binary_img.rows/2))); {
            float w = r.size.width,
                  h = r.size.height;
            if (w > h) std::swap(w,h);
            // RCLCPP_INFO(this->get_logger(), "area: %f; ratio: %f; angle: %f; dis: %f", r.size.area(), w/h, r.angle,
            //         cv::norm(r.center - cv::Point2f(binary_img.cols/2, binary_img.rows/2)));
            
            return r;
            }
        }
    }
    return {};
}

void BuffDetectorNode::matchFan(const cv::Mat& binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<FanBoard> detected_fans;
    for (const auto& contour: contours) {
        cv::RotatedRect r = cv::minAreaRect(contour);
        if (isVaild(binary_img, r.boundingRect()) && isFan(r)) {
            detected_fans.emplace_back(FanBoard{
                .id = static_cast<int>(detected_fans.size()),
                .region = r,
            });
        }
    }
    if (detected_fans.empty()) return;

    std::vector<FanBoard> current_fans;
    float max_iou = DBL_MIN;
    for (auto& detected_fan: detected_fans) {
        float max_iou = DBL_MIN;
        for (const auto& fan: m_last_fans) {
            auto val = iou::IoU(fan.region, detected_fan.region);
            if (val > 0.5 && val > max_iou) {
                max_iou = val;
                detected_fan.id = fan.id;
            }
        }
    }
    createFanBoard(*m_r_box, detected_fans[0], current_fans);
    if (m_last_fans.empty()) {
        current_fans[0].state = FanState::Target;
        m_last_fans = std::move(current_fans);
        m_last_light_num = 1;
    } else {
        if (detected_fans.size() == 5) {
            for (auto& fan: m_last_fans) {
                fan.state = FanState::Activated;
            }
            return;
        }
        if (detected_fans.size() == m_last_light_num + 1) {
            m_timer->reset();
            for (const auto& detected_fan: detected_fans) {
                for (const auto& fan: m_last_fans) {
                    if (m_last_fans[detected_fan.id].state == FanState::Unlight) {
                        current_fans[detected_fan.id].state = FanState::Target;
                    } else {
                        current_fans[detected_fan.id].state = FanState::Activated;
                    }
                }
            }

            m_last_fans = std::move(current_fans);
            m_last_light_num++;
        } else if (detected_fans.size() == m_last_light_num) {

            m_timer->reset();
            // for (const auto& detected_fan: detected_fans)
            //     for (const auto& fan: m_last_fans)
            //         current_fans[detected_fan.id].state = m_last_fans[detected_fan.id].state;
            for (int i = 0; i < 5; ++i)
                current_fans[i].state = m_last_fans[i].state;
            m_last_fans = std::move(current_fans);
        } else if (detected_fans.size() < m_last_light_num) {

            m_timer->reset();
            for (int i = 0; i < 5; ++i)
                current_fans[i].state = m_last_fans[i].state;
            m_last_fans = std::move(current_fans);
        } else {
            RCLCPP_WARN(this->get_logger(), "detected fans <==> last_light_num(%ld > %d)",
                detected_fans.size(), m_last_light_num);
        }
    }
}

void BuffDetectorNode::createFanBoard(const cv::RotatedRect& r_box, const FanBoard& fan_board, std::vector<FanBoard>& fans) {
    fans.clear();
    fans.resize(5);

    cv::Point2f fan_at_r_coordinate = fan_board.region.center - r_box.center;
    float r = cv::norm(fan_at_r_coordinate);
    float theta = std::atan2(fan_at_r_coordinate.y, fan_at_r_coordinate.x);
    cv::Size size = fan_board.region.size;
    float angle = fan_board.region.angle;
    for (int i = 0; i < 5; ++i) {
        float offset = i*2*M_PI/5;
        float tmp_theta = theta + offset;
        auto rect_center = cv::Point2f(
            r*cos(tmp_theta), r*sin(tmp_theta)) + r_box.center;
        fans[(fan_board.id + i) % 5] = FanBoard{
            .id = (fan_board.id + i) % 5,
            .region = cv::RotatedRect(rect_center, size, angle + offset*180.0/M_PI),
            .state = FanState::Unlight
        };
    }
}

bool BuffDetectorNode::isVaild(const cv::Mat& src, const cv::Rect& r) {
    return (0 <= r.x && 0 <= r.width && r.x + r.width <= src.cols &&
        0 <= r.y && 0 <= r.height && r.y + r.height <= src.rows);
}

bool BuffDetectorNode::isR(const cv::Mat& binary_img, const cv::RotatedRect& r) {
    float area = r.size.area();
    if (area <= 20) return false;
    float w = r.size.width, h = r.size.height;
    if (w > h) std::swap(w, h);
    float ratio = w / h; // 短/长
    // RCLCPP_DEBUG(this->get_logger(), "area: %f; ratio: %f; angle: %f", area, ratio, r.angle);
    auto rect = r.boundingRect();
    if (m_r_params.min_area < area && area < m_r_params.max_area &&
        m_r_params.min_ratio < ratio && ratio < m_r_params.max_ratio &&
        /*((r.angle - 90) < 5 || (r.angle - 0) < 5) &&*/
        cv::norm(r.center - cv::Point2f(
            binary_img.cols/2, binary_img.rows/2)) < m_r_params.distance_from_center)
            return true;

    return false;
}

bool BuffDetectorNode::isFan(const cv::RotatedRect& r) {
    float area = r.size.area();
    if (area <= 20) return false;
    float w = r.size.width, h = r.size.height;
    if (w > h) std::swap(w, h);
    float ratio = w / h; // 短/长
    if (m_fan_params.min_area < area && area < m_fan_params.max_area &&
        m_fan_params.min_ratio < ratio && ratio < m_fan_params.max_ratio &&
        cv::norm(m_r_box->center - r.center) < m_fan_params.distance_from_r) {
            if (!m_last_fans.empty()) {
                std::vector<float> iou_vals;
                std::transform(m_last_fans.begin(), m_last_fans.end(),
                    std::back_inserter(iou_vals), [r](const FanBoard& r1)->float {
                        return iou::IoU(r1.region, r);
                    });
                auto max_element = std::max_element(iou_vals.begin(), iou_vals.end());
                return max_element != iou_vals.end() && *max_element > 0.5;
            } else {
                RCLCPP_INFO(this->get_logger(), "fan area: %f; ratio: %f; dis: %f", area, ratio, cv::norm(m_r_box->center - r.center));
                return true;
            }
        }
    return false;
}

// std::vector<cv::Point2f> BuffDetectorNode::sortFeaturePoint(const std::vector<cv::Point2f>& ps) {
//     std::sort(ps.begin(), ps.end(), [this](const cv::Point2f& a, const cv::Point2f& b)->bool {
//         return cv::norm(a - m_r_box->center) < cv::norm(b - m_r_box->center);
//     });
//     return {};
// }

void BuffDetectorNode::reset() {
    RCLCPP_WARN(this->get_logger(), "reset");
    m_r_box = nullptr;
    m_last_fans.clear();
    m_last_light_num = 0;
}

void BuffDetectorNode::drawRotatedRect(cv::Mat& src, const cv::RotatedRect& r, int thickness, const cv::Scalar& color) {
    cv::Point2f p[4];
    r.points(p);
    for (int i = 0; i < 4; i++) {
        cv::line(src, p[i], p[(i + 1) % 4], color, thickness);
        cv::putText(src, std::to_string(i), p[i], 0, 0.8, color);
    }
}

std::string to_string(const FanState& state) {
    switch (state) {
        case FanState::Unlight:
            return "Unlight";
        case FanState::Target:
            return "Target";
        case FanState::Activated:
            return "Activated";
    }
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::BuffDetectorNode)
