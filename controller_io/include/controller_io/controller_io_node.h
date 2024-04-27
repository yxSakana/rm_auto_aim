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
    static constexpr int mControllerId = 1;
    static constexpr int mSlaveControllerId = 2;
    static constexpr int mNeedUpdateTimestamp = 0;
    static constexpr int mGimbalPose = 1;
    static constexpr int mControllerAim = 2;
    static constexpr int mSetTargetColor = 3;
public:
    ControllerIONode(const rclcpp::NodeOptions& options);
private:
    // tf
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_gimbal_tf_broad;
    // Subscription
    rclcpp::Subscription<custom_serial_interfaces::msg::Receive>::SharedPtr m_serial_sub;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr m_target_sub;
    // Client
    rclcpp::Client<custom_serial_interfaces::srv::SendPackage>::SharedPtr m_serial_cli;
    // Armor detector target color client
    rclcpp::AsyncParametersClient::SharedPtr m_detect_color_cli;
    ResultFuturePtr m_set_param_future;
    
    // visualization
    visualization_msgs::msg::Marker m_aim_marker;
    visualization_msgs::msg::MarkerArray m_marker_array;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub;
    // Debug publish
    rclcpp::Publisher<auto_aim_interfaces::msg::ControllerAim>::SharedPtr m_c_aim_pub;

    // Callback
    void serialHandle(const custom_serial_interfaces::msg::Receive::SharedPtr serial_msg);

    void trackerCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg);

    void setParam(const rclcpp::Parameter& param);
};
}

#endif
