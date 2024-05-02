#include <armor_detector_filter/armor_detector_filter.h>

namespace armor_auto_aim {
ArmorDetectorFilterNode::ArmorDetectorFilterNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("armor_detector_filter", options) {
    m_armors_pub = this->create_publisher<
        auto_aim_interfaces::msg::Armors>("/armor_detector/armors", rclcpp::SensorDataQoS());
    m_armors_master_sub = this->create_subscription<
        auto_aim_interfaces::msg::Armors>("/sentry_master/armor_detector/armors", rclcpp::SensorDataQoS(), 
        std::bind(&ArmorDetectorFilterNode::subArmors, this, std::placeholders::_1));
    m_armors_slave_sub = this->create_subscription<
        auto_aim_interfaces::msg::Armors>("/sentry_slave/armor_detector/armors", rclcpp::SensorDataQoS(), 
        std::bind(&ArmorDetectorFilterNode::subArmors, this, std::placeholders::_1));
    m_cam_enable_cli = this->create_client<std_srvs::srv::SetBool>("/sentry_slave/hik_camera/enable");
}

void ArmorDetectorFilterNode::subArmors(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg) {
    std::lock_guard<std::mutex> lk(m_sub_mutex);
    m_armors_pub->publish(*armors_msg);
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(armor_auto_aim::ArmorDetectorFilterNode)
