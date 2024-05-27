// #ifndef ARMOR_AUTO_AIM_BUFF_DETECTOR_H
// #define ARMOR_AUTO_AIM_BUFF_DETECTOR_H

// #define USE_ARMOR

// #include <array>

// #include <opencv2/opencv.hpp>
// // ROS2
// #include <rclcpp/rclcpp.hpp>
// #include <image_transport/image_transport.hpp>

// namespace armor_auto_aim {
// enum class FanState {
//     Unlight,
//     Target,
//     Activated
// };

// struct BuffParams {
//     int r_threshold;
//     int g_threshold;
//     int b_threshold;
// };

// struct RParams {
//     double min_area;
//     double max_area;
//     double min_ratio;
//     double max_ratio;
//     double distance_from_center;
// };

// struct FanParams {
//     double min_area;
//     double max_area;
//     double min_ratio;
//     double max_ratio;
//     double distance_from_r;
// };

// struct FanBoard {
//     int id;
//     cv::RotatedRect region;
//     FanState state;
// };

// using BuffFanBoards = std::array<FanBoard, 5>;

// class StateUpgrader {
//     enum UpgradState {
//         Unknown,
//         Updating,
//         TmpLoss,
//         Confirmed
//     };
// public:
//     StateUpgrader();

//     void update(bool condition) {
//         if (m_state == Confirmed) return;

//         if (condition) {
//             ++m_accord_count, m_loss_count = 0;
//             if (m_state == Unknown || m_state == TmpLoss)
//                 m_state = Updating;
//             if (m_state == Updating)
//                 m_state = m_accord_count > m_confirme_threshold? Confirmed: Updating;
//         } else {
//             ++m_loss_count, m_accord_count = 0;
//             if (m_state == Updating)
//                 m_state = TmpLoss;
//             else if (m_state == TmpLoss)
//                 m_state = m_loss_count > m_abort_threshold ? Unknown : TmpLoss;
//         }
//     }

//     bool isConfirmed() const { return m_state == Confirmed; }
// private:
//     UpgradState m_state = Unknown;
//     int m_accord_count = 0;
//     int m_loss_count = 0;

//     int m_confirme_threshold;
//     int m_abort_threshold;
// };

// class BuffDetectorNode: public rclcpp::Node {
// public:
//     BuffDetectorNode(const rclcpp::NodeOptions& options);
// private:
//     // Subscript
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_img_sub;
//     // Timer
//     rclcpp::TimerBase::SharedPtr m_timer;
//     // Box
//     std::shared_ptr<cv::RotatedRect> m_r_box = nullptr;
//     std::vector<FanBoard> m_last_fans;
//     int m_last_light_num;
//     // params
//     float m_real_distance;
//     float m_inner;
//     float m_outer;
//     BuffParams m_params;
//     RParams m_r_params;
//     FanParams m_fan_params;

//     // Debug publisher
//     image_transport::Publisher m_result_pub;
//     image_transport::Publisher m_binary_pub;
//     image_transport::Publisher m_contour_pub;

//     void detect(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

//     cv::Mat preprocessImage(const cv::Mat& src);

//     cv::Mat preprocessImageToR(const cv::Mat& src);

//     cv::RotatedRect matchR(const cv::Mat& binary_img);

//     void matchFan(const cv::Mat& binary_img);

//     inline static bool isVaild(const cv::Mat& src, const cv::Rect& r);

//     bool isR(const cv::Mat& binary_img, const cv::RotatedRect& r);

//     bool isFan(const cv::RotatedRect& r);

//     void createBuffFanBoard(
//         const cv::RotatedRect& r_box,
//         const FanBoard& fan_board,
//         BuffFanBoards& fans);

//     void createFanBoard(const cv::RotatedRect& r_box, const FanBoard& fan_board, std::vector<FanBoard>& fans);

//     std::vector<cv::Point2f> sortFeaturePoint(const std::vector<cv::Point2f>& ps);

//     void reset();
    
//     void drawRotatedRect(cv::Mat& src, const cv::RotatedRect& r, int thickness=2, const cv::Scalar& color=cv::Scalar(0, 0, 255));
// };

// std::string to_string(const FanState& state);
// }
// #endif
