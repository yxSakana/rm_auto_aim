#ifndef ARMOR_AUTO_AIM_ARMOR_TRACKER_NODE_H
#define ARMOR_AUTO_AIM_ARMOR_TRACKER_NODE_H

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <armor_tracker/tracker.h>
#include <auto_aim_interfaces/msg/target.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>

namespace armor_auto_aim {
class ArmorTrackerNode: public rclcpp::Node {
public:
    ArmorTrackerNode(const rclcpp::NodeOptions& options);
private:
    Tracker m_tracker;
    float dt;
    rclcpp::Time m_last_stamp;
    // tf
    std::string m_odom_frame;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
    std::shared_ptr<tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>> m_tf_filter;
    // Publisher
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr m_target_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_armor_odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_armor_gimbal_pub;
    // Subscription
    message_filters::Subscriber<auto_aim_interfaces::msg::Armors> m_armors_sub;
    // Visualization marker
    visualization_msgs::msg::Marker m_center_marker;
    visualization_msgs::msg::Marker m_armors_marker;
    visualization_msgs::msg::Marker m_linear_v_marker;
    visualization_msgs::msg::Marker m_omega_marker;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub;
    void initEkf();
    
    void publishMarkers(const auto_aim_interfaces::msg::Target& target_msg);

    // Callback
    void subArmorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armos_msg);
};
}

#endif
