#ifndef ARMOR_AUTO_AIM_CONTROLLER_IO_H
#define ARMOR_AUTO_AIM_CONTROLLER_IO_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <custom_serial_interfaces/msg/receive.hpp>
#include <custom_serial_interfaces/srv/send_package.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <auto_aim_interfaces/msg/controller_aim.hpp>

namespace armor_auto_aim {
class ControllerIONode: public rclcpp::Node {
    using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    // IPC
    static constexpr int mPCId = 0;
    static constexpr int mTimestampPackte = 0;
    static constexpr int mAimPacket = 1;
    // Controller
    static constexpr int mControllerId = 10;
    static constexpr int mSlaveControllerId = 11;
    static constexpr int mNeedUpdateTimestamp = 0;
    static constexpr int mGimbalPose = 1;
    static constexpr int mSetTargetColor = 2;
public:
    ControllerIONode(const rclcpp::NodeOptions& options);
private:
    // tf
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_gimbal_tf_broad;
    // Subscription
    rclcpp::Subscription<custom_serial_interfaces::msg::Receive>::SharedPtr m_serial_sub;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr m_target_sub;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr m_target_slave_sub;
    // Client
    rclcpp::Client<custom_serial_interfaces::srv::SendPackage>::SharedPtr m_serial_cli;
    // Armor detector target color, Armro trakcer pattem client. And the reuslt future
    // The client and future are a pair.
    std::vector<rclcpp::AsyncParametersClient::SharedPtr> m_detect_color_clis;
    std::vector<ResultFuturePtr> m_set_param_futures;
    // 
    std::array<bool, 2> m_trakcer_state;
    uint8_t m_is_detector_configured;
    // bool m_is_sentry;
    bool m_is_outpost = false;
    bool m_is_outpost_flags = false;

    // visualization
    visualization_msgs::msg::Marker m_aim_marker;
    visualization_msgs::msg::MarkerArray m_marker_array;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub;
    // Debug publish
    rclcpp::Publisher<auto_aim_interfaces::msg::ControllerAim>::SharedPtr m_c_aim_pub;

    // Callback
    void serialHandle(const custom_serial_interfaces::msg::Receive::SharedPtr serial_msg);

    void trackerCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg);

    void setDetectorColor(const rclcpp::Parameter& param);

    void setOutpostMode(const rclcpp::Parameter& param);

    void setParam(int index,
                  const rclcpp::Parameter& param,
                  std::function<void(void)> callback);
    // void setParam(const rclcpp::Parameter& param);
};
}

#endif
