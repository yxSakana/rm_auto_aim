#ifndef ARMOR_AUTO_AIM_ARMOR_DETECTOR_FILTER_H
#define ARMOR_AUTO_AIM_ARMOR_DETECTOR_FILTER_H 

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <auto_aim_interfaces/msg/armors.hpp>

namespace armor_auto_aim {
class ArmorDetectorFilterNode: public rclcpp::Node {
public:
    explicit ArmorDetectorFilterNode(const rclcpp::NodeOptions& options);
private:
    std::mutex m_sub_mutex;
    // Publisher
    rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr m_armors_pub;
    // Sublisher
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr m_armors_master_sub;
    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr m_armors_slave_sub;
    // Client
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_cam_enable_cli;

    void subArmors(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg);
};
}

#endif
