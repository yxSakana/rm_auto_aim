#ifndef ARMOR_AUTO_AIM_CONTROLLER_IO_H
#define ARMOR_AUTO_AIM_CONTROLLER_IO_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <custom_serial_interfaces/msg/receive.hpp>
#include <custom_serial_interfaces/srv/send_package.hpp>
#include <auto_aim_interfaces/msg/target.hpp>

namespace armor_auto_aim {
class ControllerIONode: public rclcpp::Node {
    // IPC
    static constexpr int mPCId = 0;
    static constexpr int mTimestampPackte = 0;
    static constexpr int mAimPacket = 1;
    // Controller
    static constexpr int mControllerId = 1;
    static constexpr int mNeedUpdateTimestamp = 0;
    static constexpr int mGimbalPose = 1;
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

    // Callback
    void serialHandle(const custom_serial_interfaces::msg::Receive::SharedPtr serial_msg);

    void trackerCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg);
};
}

#endif
